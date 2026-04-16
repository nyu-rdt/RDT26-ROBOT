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
                'sync_queue_size': 100,
                'topic_queue_size': 100,
                'subscribe_depth': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'use_sim_time': use_sim_time,
                'Mem/IncrementalMemory': 'true',
                'Kp/MaxFeatures': '400',
                'PyDetector/Cuda': "false",   
                'PyMatcher/Cuda':  "false",   
                'SuperPoint/Cuda': "false",
                'Optimizer/GravitySigma': "0.3" ,  
                'Grid/RayTracing': "false",
                'wait_for_transform': 1.0,
                'Vis/DepthAsMask': 'false',
                'Mem/DepthAsMask': 'false',
                'OdomF2M/ValidDepthRatio': 0.2,
                'Vis/MinInliers': '10',
                'Odom/FillInfoData': 'true',
                'OdomF2M/BundleAdjustment': 0,
                'Vis/CorGuessWinSize': '20',
            }],
            remappings=[
                ('imu_topic', 'imu/data'),
                ('/rgb/image', '/camera/camera/color/image_raw'),
                ('/depth/image', '/camera/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera/color/camera_info'),
            ]
        ),

        # ── RTAB-Map Odometry ────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'sync_queue_size': 100,
                'topic_queue_size': 100,
                'frame_id': 'base_link',
                'approx_sync': True,
                'use_sim_time': use_sim_time,
                'approx_sync_max_interval': 0.019,
                'Vis/DepthAsMask': 'false',
                'Mem/DepthAsMask': 'false',
            }],
            remappings=[
                ('/rgb/image', '/camera/camera/color/image_raw'),
                ('/depth/image', '/camera/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera/color/camera_info'),
            ]
        ),
    ])