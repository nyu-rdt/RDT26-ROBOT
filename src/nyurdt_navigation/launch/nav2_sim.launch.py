"""
nav2_sim.launch.py

Full simulation stack:
  Gazebo (arena.sdf) → robot spawn → ROS↔Gz bridges (camera + drive + clock)
  → RTAB-Map SLAM → Nav2 → twist_mux → cmd_vel_bridge (no-op I2C in sim)
  → apriltag_ros (simulated RealSense) → apriltag_tf (robot pose corrections)

Launch with:
  ros2 launch nyurdt_navigation nav2_sim.launch.py
  ros2 launch nyurdt_navigation nav2_sim.launch.py use_rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir  = get_package_share_directory('nyurdt_navigation')
    desc_dir = get_package_share_directory('nyurdt_description')
    ctrl_dir = get_package_share_directory('nyurdt_control')
    loc_dir  = get_package_share_directory('nyurdt_localization')
    slam_dir = get_package_share_directory('nyurdt_slam')
    gz_dir   = get_package_share_directory('ros_gz_sim')

    world_file    = os.path.join(nav_dir, 'worlds', 'arena.sdf')
    models_dir    = os.path.join(nav_dir, 'worlds', 'models')
    nav2_params   = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    arena_map     = os.path.join(nav_dir, 'config', 'arena_map.yaml')
    apriltag_cfg  = os.path.join(loc_dir, 'config', 'tags_36h11.yaml')
    twist_topics  = os.path.join(ctrl_dir, 'config', 'twist_mux.yaml')
    twist_locks   = os.path.join(ctrl_dir, 'config', 'twist_mux_locks.yaml')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_dir, 'launch', 'gz_sim.launch.py')),
        # -s = server-only (no Gazebo GUI window).  Removing Gazebo's render loop
        # eliminates the physics/render competition that causes sim clock jumps.
        # Visualise via RViz instead.  rgbd_camera sensors still work headlessly.
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── Robot State Publisher ────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # ── Spawn robot in Gazebo ────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # -world: required in headless server mode (-s) so Gazebo knows which world
        # to spawn into.  Without it the create command silently fails.
        arguments=['-world', 'ucf_lunabotics_2026_arena',
                   '-topic', 'robot_description',
                   '-name',  'rdt26_robot',
                   '-x', '7.45', '-y', '1.9', '-z', '0.22',
                   '-R', '0', '-P', '0', '-Y', '3.14159'],
        output='screen',
    )

    # Gazebo Harmonic scopes sensor frames as <model>/<link>/<sensor>.
    # Alias it to the URDF camera_link so rtabmap can resolve the image frame_id.
    camera_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_link', 'rdt26_robot/base_link/realsense_d435'],
        parameters=[{'use_sim_time': True}],
    )

    # ── ROS ↔ Gazebo bridges ─────────────────────────────────────────────────
    core_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='core_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Nav2 outputs /cmd_vel_nav; remap ROS side so Gazebo diff drive sees /cmd_vel
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
        output='screen',
    )

    # Camera bridge: remap Gazebo topics directly to the names RTAB-Map/apriltag_ros expect.
    # This avoids requiring the topic_tools package.
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/realsense/image',       '/camera/camera/color/image_raw'),
            ('/realsense/depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/realsense/camera_info', '/camera/camera/color/camera_info'),
            ('/realsense/points',      '/camera/camera/depth/points'),
        ],
        output='screen',
    )

    # ── AprilTag detection — RealSense colour stream ─────────────────────────
    # Publishes to /sim_cam/detections → apriltag_tf._process_sim() uses camera_link
    # (RealSense frame) — correct geometry.
    # A SEPARATE node feeds /rtabmap/apriltag for RTAB-Map landmarks only so that
    # back-cam detections on that topic don't pollute apriltag_tf's pose calculation.
    apriltag_sim = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_sim',
        remappings=[
            ('image_rect',   '/camera/camera/color/image_raw'),
            ('camera_info',  '/camera/camera/color/camera_info'),
            ('detections',   '/sim_cam/detections'),
        ],
        parameters=[apriltag_cfg, {'use_sim_time': True}],
        output='screen',
    )
    apriltag_sim_rtabmap = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_sim_rtabmap',
        remappings=[
            ('image_rect',   '/camera/camera/color/image_raw'),
            ('camera_info',  '/camera/camera/color/camera_info'),
            ('detections',   '/rtabmap/apriltag'),
        ],
        parameters=[apriltag_cfg, {'use_sim_time': True}],
        output='screen',
    )

    # ── Back camera bridge (rgbd_camera, works with ogre) ───────────────────
    # Back camera: rpy="0 0 -π" → faces +X when robot faces -X.
    # Continuously sees the fiducial rail → AprilTag landmarks for RTAB-Map.
    webcam_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='webcam_bridge',
        arguments=[
            '/back_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/back_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Gazebo lumps fixed joints → back_camera (fixed child of base_link) becomes
    # base_link in the Gazebo model, so the actual sensor frame is
    # rdt26_robot/base_link/back_cam  (NOT rdt26_robot/back_camera/back_cam).
    # Same rule as the RealSense: rdt26_robot/base_link/realsense_d435.
    # Connect the actual Gazebo frame to back_camera so apriltag_tf can look up
    # back_camera_optical_frame_link → back_camera → rdt26_robot/base_link/back_cam → tag.
    back_cam_alias = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='back_cam_alias',
        arguments=['0', '0', '0', '0', '0', '0',
                   'back_camera', 'rdt26_robot/base_link/back_cam'],
        parameters=[{'use_sim_time': True}],
    )
    # Also anchor directly to base_link for robustness during RSP TF-buffer gaps
    back_cam_base_alias = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='back_cam_base_alias',
        arguments=['-0.2799208', '-0.195', '0.45', '-3.14159', '0', '0',
                   'base_link', 'rdt26_robot/base_link/back_cam'],
        parameters=[{'use_sim_time': True}],
    )

    # apriltag_ros on back camera → /back_cam/detections (for apriltag_tf pose corrections)
    # Single back-cam apriltag node → /back_cam/detections → apriltag_tf._process_back.
    # RTAB-Map landmarks come from apriltag_sim_rtabmap (RealSense) — running a second
    # apriltag_ros instance on the same back-cam image halved the controller rate (20→5 Hz).
    apriltag_back = Node(
        package='apriltag_ros', executable='apriltag_node', name='apriltag_back',
        remappings=[('image_rect', '/back_cam/image'),
                    ('camera_info', '/back_cam/camera_info'),
                    ('detections', '/back_cam/detections')],
        parameters=[apriltag_cfg, {'use_sim_time': True}], output='screen',
    )

    # apriltag_tf: converts detections → /apriltag/robot_pose
    apriltag_tf = Node(
        package='nyurdt_localization',
        executable='apriltag_tf',
        name='apriltag_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Static map→odom transform ────────────────────────────────────────────
    # Robot spawns at x=7.45, y=1.9, yaw=π (facing -X into arena).
    # Gazebo diff-drive initialises odom at the robot's spawn origin (odom≡base_link
    # at t=0), so map→odom = spawn pose.  This lets Nav2 know where it is
    # immediately, without waiting for rtabmap to process its first keyframe.
    # rtabmap TF publishing is disabled (publish_tf_map=false) to avoid conflict.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        # Map origin = centre of 2×2m starting zone (SDF world 7.10, 1.28).
        # odom starts at robot spawn (SDF 7.45, 1.9) → map (0.35, 0.62), dist≈0.71m.
        arguments=['--x', '0.35', '--y', '0.62', '--z', '0',
                   '--yaw', '3.14159', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # ── RTAB-Map SLAM ────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'main.launch.py')),
        launch_arguments={
            # Raise loop-closure error tolerance so AprilTag landmarks have time to
            # anchor the pose graph before tighter constraints kick in.
            'args':              '--delete_db_on_start --RGBD/OptimizeMaxError 5.0',
            'use_sim_time':      'true',
            'approx_sync':       'true',
            'frame_id':          'base_link',
            'odom_frame_id':     'odom',   # use Gazebo diff-drive TF, not visual odometry
            'wait_for_transform': '0.5',   # tolerate TF gaps after sim clock-jump buffer clears
            'visual_odometry':   'false',  # avoids dual odom→base_link TF conflict
            'publish_tf_map':    'false',  # static map_odom_tf node handles this
            'initial_pose':      '0.35 0.62 0 0 0 3.14159',
            'sync_queue_size':   '10',
            'topic_queue_size':  '10',
            'PyDetector/Cuda':   'false',
            'PyMatcher/Cuda':    'false',
            'SuperPoint/Cuda':   'false',
            'Grid/RayTracing':   'false',
            'OdomF2M/MaxSize':   '1000',
            'Vis/MinInliers':    '20',   # harder threshold → fewer false loop closures in low-texture arena
            'FAST/Threshold':    '10',
            'Rtabmap/DetectionRate': '0.5',  # 0.5 Hz — halves RTAB-Map CPU on each burst
            # With perfect Gazebo odometry there is no drift — wrong loop closures
            # hurt more than they help.  Raise the max graph error ratio so RTAB-Map
            # doesn't keep rejecting valid tag-based constraints.
            # RGBD/OptimizeMaxError set via args above (not a declared slam launch arg)
            'RGBD/CreateOccupancyGrid': 'true',
            # Grid/3D=false: 2D normals-based grid — generates the correct occupancy map
            # (walls/rocks as black cells, free interior as white).
            # Grid/3D=true was tried but causes an empty voxel grid because the flat
            # arena floor makes all normals point up → everything classified as ground.
            'Grid/3D':           'false',
            # Force a new keyframe every processed frame so cloud_obstacles and
            # rtabmap/map populate quickly even while the robot is stationary.
            'RGBD/LinearUpdate':  '0.0',
            'RGBD/AngularUpdate': '0.0',
            'Grid/FlatObstacleDetected': 'true',
            'Grid/DepthDecimation': '4',
            'Odom/ScanKeyFrameThr': '.6',
            # Separate floor from obstacles so cloud_obstacles doesn't block nav paths
            'Grid/MinGroundHeight': '-0.1',   # world z below this = ground
            'Grid/MaxGroundHeight': '0.05',   # world z above this = obstacle candidate
            'Grid/MaxObstacleHeight': '0.55', # ignore anything above walls/craters
            # Native AprilTag landmark integration into the RTAB-Map SLAM graph.
            # apriltag_sim_rtabmap publishes on /rtabmap/apriltag; route it here.
            'tag_topic':            '/rtabmap/apriltag',
            'tag_linear_variance':  '0.0001',
            'tag_angular_variance': '9999.0',
        }.items(),
    )

    # ── map_server (publishes arena_map.pgm as /static_map) ─────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': arena_map,
            'topic_name':    'static_map',
            'frame_id':      'map',
            'use_sim_time':  True,
        }],
    )
    lifecycle_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'],
                     'use_sim_time': True}],
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation_core.launch.py')),
        launch_arguments={
            'use_sim_time':    'true',
            'params_file':     nav2_params,
            'autostart':       'True',
            'use_composition': 'False',
            'namespace':       '',
        }.items(),
    )

    # ── twist_mux (joystick priority 100 > nav priority 10) ─────────────────
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[{'use_sim_time': True}, twist_locks, twist_topics],
        remappings=[('/cmd_vel_out', '/robot/cmd_vel')],
        output='screen',
    )

    # cmd_vel_bridge: FAKE_TEENSY=1 makes send_HEX a no-op instead of spamming
    # I2C errors at 20Hz (Gazebo diff-drive handles motion via core_bridge).
    cmd_vel_bridge = Node(
        package='nyurdt_control',
        executable='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        additional_env={'FAKE_TEENSY': '1'},
    )

    # ── Optional RViz ────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    autonomy = Node(
        package='nyurdt_autonomy',
        executable='autonomy_manager',
        name='autonomy_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Delay non-Gazebo nodes slightly so Gazebo has time to come up.
    # camera_frame_alias and webcam aliases are pure static TFs — they start
    # immediately (see return LaunchDescription below) so RTAB-Map never hits
    # a "frame does not exist" race when it processes its first images.
    # spawn is in a separate 5s timer so RSP has time to publish /robot_description
    # before ros_gz_sim create tries to read it.
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    delayed = TimerAction(period=3.0, actions=[
        rsp, joint_state_pub,
        camera_bridge,
        webcam_bridge,
        map_odom_tf,
        apriltag_sim, apriltag_sim_rtabmap,
        apriltag_back,
        apriltag_tf,
        slam, map_server, lifecycle_map, nav2,
        twist_mux, cmd_vel_bridge,
        rviz,
    ])

    # Give nav2 time to fully activate before autonomy starts issuing goals
    delayed_autonomy = TimerAction(period=15.0, actions=[autonomy])

    # Tell Gazebo Harmonic where to find the AprilTag model directories.
    # GZ_SIM_RESOURCE_PATH is the correct variable for gz-sim (Harmonic/Jazzy).
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2'),
        gz_resource_path,
        core_bridge,
        gazebo,
        # Static TF aliases start immediately — no delay — so RTAB-Map never
        # hits "frame does not exist" when processing its first images.
        camera_frame_alias,
        back_cam_alias,
        back_cam_base_alias,
        delayed,
        delayed_spawn,
        delayed_autonomy,
    ])
