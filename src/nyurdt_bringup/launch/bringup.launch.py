import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    navigation   = get_package_share_directory('nyurdt_navigation')
    description  = get_package_share_directory('nyurdt_description')
    realsense    = get_package_share_directory('realsense2_camera')
    localization = get_package_share_directory('nyurdt_localization')
    slam         = get_package_share_directory('nyurdt_slam')
    rtabmap      = get_package_share_directory('rtabmap_launch')

    
    nav2_params = os.path.join(navigation, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(slam, 'config', 'rtabmap.yaml')
    realsense_params = os.path.join(slam, 'config', 'realsense.yaml')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ── Robot State Publisher (from nyurdt_description) ──────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── RealSense ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'config_file': realsense_params,
            }.items()
        ),

        # ── Apriltag ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization, 'launch', 'camera_apriltag.launch.py'),
            ),
            launch_arguments={'device2': '0'}.items() 
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(rtabmap, 'launch', 'rtabmap.launch.py')
        #     ),
        #     launch_arguments={
        #         'params_file':          slam_params,
        #         'use_sim_time':         use_sim_time,
        #         'depth_topic':         '/camera/camera/aligned_depth_to_color/image_raw',
        #         'rgb_topic':           '/camera/camera/color/image_raw',
        #         'camera_info_topic':   '/camera/camera/color/camera_info',
        #         'frame_id':            'camera_link',
        #         'approx_sync':         'true',
        #         'rtabmap_args':        '--delete_db_on_start',
        #     }.items()
        # ),

        # ── Nav2 (from nyurdt_navigation) ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation, 'launch', 'nav2_standalone.launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # ── Autonomy Nodes ───────────────────────────────────────────────
        Node(package='nyurdt_control', executable='i2c_reader', name='i2c_reader'),
        Node(package='nyurdt_autonomy', executable='telemetry_processor', name='telemetry_processor'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager'),

        # ── RTAB-Map SLAM ────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=[{
                'sync_queue_size':50,
                'subscribe_depth': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'use_sim_time': use_sim_time,
                'Mem/IncrementalMemory': 'true',
                'Kp/MaxFeatures': '400',
            }],
            remappings=[
                ('/rgb/image', '/camera/camera/image_raw'),
                ('/depth/image', '/camera/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera/camera_info'),
            ]
        ),

        # ── RTAB-Map Odometry ────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'link',
                'approx_sync': True,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/rgb/image', '/camera/camera/image_raw'),
                ('/depth/image', '/camera/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera/camera_info'),
            ]
        ),
    ])