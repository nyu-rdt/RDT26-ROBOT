import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = get_package_share_directory("nyurdt_navigation")
    pkg_nav2 = get_package_share_directory("nav2_bringup")
    pkg_gz = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("nyurdt_description")
    pkg_localization = get_package_share_directory("nyurdt_localization")
    pkg_slam = get_package_share_directory("nyurdt_slam")

    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    slam = LaunchConfiguration("slam")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    robot_name = LaunchConfiguration("robot_name")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_navigation, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for Nav2",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_navigation, "worlds", "arena.sdf"),
        description="Gazebo world file",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_navigation, "config", "arena_map.yaml"),
        description="Full path to map.yaml. Ignored when use_rtabmap:=True.",
    )

    declare_slam = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Run slam_toolbox (nav2_bringup). Ignored when use_rtabmap:=True.",
    )

    declare_use_rtabmap = DeclareLaunchArgument(
        "use_rtabmap",
        default_value="True",
        description=(
            "Run RTAB-Map SLAM. When True, map_server and static map→odom TF are "
            "disabled — RTAB-Map owns the /map topic and map→odom transform."
        ),
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="rdt26_robot",
        description="Entity name to spawn in Gazebo (must match bridge model name).",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_nav2, "rviz", "nav2_default_view.rviz"),
        description="Full path to an RViz config file",
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "rsp.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -------------------------
    # Gazebo model search path (AprilTag SDF models)
    # -------------------------
    _existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    _models_dir = os.path.join(pkg_navigation, 'worlds', 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(_existing_gz_path + ':' + _models_dir) if _existing_gz_path else _models_dir,
    )

    # -------------------------
    # Gazebo Sim
    # -------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PythonExpression(['"-r " + "', world, '"'])}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_name,
            "-x", "1.00", "-y", "3.47", "-z", "0.5",
            "-Y", "0.0",
        ],
        output="screen",
    )

    # -------------------------
    # Bridges
    # Split into two parameter_bridge nodes so a missing/bad Gazebo topic
    # for the back camera cannot stall the main bridge.
    # -------------------------

    # Main bridge: clock, odom, TF, cmd_vel, RealSense (RTAB-Map critical path).
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_main",
        parameters=[{
            "config_file": os.path.join(pkg_navigation, "config", "sim_bridge.yaml"),
        }],
        output="screen",
    )

    # Back camera bridge: isolated so the main bridge is never affected.
    back_cam_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_back_cam",
        parameters=[{
            "config_file": os.path.join(pkg_navigation, "config", "back_cam_bridge.yaml"),
        }],
        output="screen",
    )

    # -------------------------
    # RTAB-Map SLAM (via nyurdt_slam)
    # Active when use_rtabmap:=True (default).
    # visual_odometry=false: use Gazebo odom TF instead of rgbd_odometry.
    # idk man michael you might have to take a look at this one
    # -------------------------
    rtabmap_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "main.launch.py")
        ),
        condition=IfCondition(use_rtabmap),
        launch_arguments={
            "use_sim_time":           use_sim_time,
            "approx_sync":            "true",
            "visual_odometry":        "false",
            "odom_frame_id":          "odom",
            "rtabmap_args":           "--delete_db_on_start",
            "initial_pose":           "1.00 3.47 0.0 0 0 0",
            # the topic names are based on the sim_bridge.yaml, could rename to use naming from nyurdt_slam
            "rgb_topic":              "/camera/image_raw",
            "depth_topic":            "/camera/depth/image_rect_raw",
            "camera_info_topic":      "/camera/camera_info",
            "map_topic":              "/map",
            "Kp/MaxFeatures":         "400",
            "PyDetector/Cuda":        "false",
            "PyMatcher/Cuda":         "false",
            "SuperPoint/Cuda":        "false",
            "Grid/RayTracing":        "false",
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/3D":                "false",
            "Grid/MaxGroundHeight":   "0.15",
            "Grid/MinGroundHeight":   "-0.5",
            "Grid/MaxObstacleHeight": "1.5",
            "Grid/NormalsSegmentation": "true",
            "RGBD/LinearUpdate":      "0.3",
            "RGBD/AngularUpdate":     "0.2",
        }.items(),
    )

    # -------------------------
    # Nav2 bringup
    # -------------------------

    # slam_toolbox — only if slam:=True AND use_rtabmap:=False
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, "launch", "slam_launch.py")),
        condition=IfCondition(PythonExpression([
            '"', slam, '" == "True" and "', use_rtabmap, '" == "False"'
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"', slam, '" == "False" and "', use_rtabmap, '" == "False"'
        ])),
        parameters=[
            params_file,
            {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"', slam, '" == "False" and "', use_rtabmap, '" == "False"'
        ])),
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server"]},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Gazebo Harmonic names the RealSense sensor frame as:
    #   rdt26_robot/base_link/realsense_d435
    # It uses optical convention (z-forward along the lens axis, x-right, y-down).
    # camera_depth_optical_frame in the URDF TF tree uses the same optical convention
    # and represents the same physical point, so the identity transform is correct.
    # Using camera_link (body convention, x-forward) was wrong — it introduced a
    # (-pi/2, 0, -pi/2) rotation error that put floor points at wrong world-frame
    # heights, making the entire ground appear as obstacles.
    gz_camera_frame_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gz_camera_frame_bridge",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0",
                   "camera_depth_optical_frame",
                   "rdt26_robot/base_link/realsense_d435"],
    )

    # Back camera (Logitech C615 sim) — same optical-vs-body frame issue as the
    # RealSense.  back_camera is a fixed joint on base_link so Gazebo names its
    # sensor frame rdt26_robot/base_link/back_cam.  Connect it to the URDF optical
    # frame so apriltag_ros gets the correct camera→base_link TF.
    gz_back_cam_frame_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gz_back_cam_frame_bridge",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0",
                   "back_camera_optical_frame_link",
                   "rdt26_robot/base_link/back_cam"],
    )

    # -------------------------
    # AprilTag detection — back camera
    # Publishes AprilTagDetectionArray on /back_cam/detections.
    # apriltag_ros also broadcasts tag_N TF frames relative to the camera,
    # which apriltag_tf uses to compute the robot's world pose.
    # -------------------------
    apriltag_cfg = os.path.join(pkg_localization, "config", "tags_36h11.yaml")

    back_cam_apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace="back_cam",
        remappings=[
            ("image_rect", "/back_cam/image_raw"),
            ("camera_info", "/back_cam/camera_info"),
        ],
        parameters=[apriltag_cfg, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -------------------------
    # apriltag_tf — pose correction publisher
    # Converts tag detections → /apriltag/robot_pose (PoseWithCovarianceStamped)
    # and injects corrections into /rtabmap/initialpose for drift recovery.
    # -------------------------
    apriltag_tf_node = Node(
        package="nyurdt_localization",
        executable="apriltag_tf",
        name="apriltag_localization",
        parameters=[{
            "use_sim_time": use_sim_time,
            # slam_mode=False: let apriltag_tf inject /rtabmap/initialpose
            # corrections at startup so RTAB-Map's map frame anchors to arena
            # coordinates (1.00, 3.47) from the first tag detection.
            # Once the robot drives away from the tags (Stage 6+), corrections
            # stop automatically (tags out of back-camera view) and RTAB-Map
            # tracks on its own visual odometry.
            "slam_mode": False,
        }],
        output="screen",
    )

    # Static map→odom TF — only when RTAB-Map is NOT providing it
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"', use_sim_time, '" == "True" and "', use_rtabmap, '" == "False"'
        ])),
        arguments=["1.00", "3.47", "0", "0", "0", "0", "map", "odom"],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, "launch", "navigation_core.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
            "namespace": "",
        }.items(),
    )

    warn_no_map_no_slam = LogInfo(
        condition=IfCondition(PythonExpression([
            '"', slam, '" == "False" and "',
            use_rtabmap, '" == "False" and "',
            map_yaml, '" == ""'
        ])),
        msg="[sim_nav2] WARNING: slam:=False, use_rtabmap:=False, and map:='' — no map source.",
    )

    # -------------------------
    # RViz
    # -------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_world,
        declare_map,
        declare_slam,
        declare_use_rtabmap,
        declare_robot_name,
        declare_rviz_config,
        warn_no_map_no_slam,
        set_gz_resource_path,
        rsp,
        joint_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        back_cam_bridge,
        gz_camera_frame_bridge,
        gz_back_cam_frame_bridge,
        rtabmap_slam,
        back_cam_apriltag,
        apriltag_tf_node,
        map_server,
        lifecycle_manager_map,
        map_to_odom,
        slam_launch,
        navigation_launch,
        rviz_node,
    ])
