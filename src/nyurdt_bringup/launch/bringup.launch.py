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
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'enable_sync': 'true',
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                'serial_no': '244422300437',
            }.items()
        ),

        # ── Apriltag ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization, 'launch', 'camera_apriltag.launch.py'),
            ),
            launch_arguments={'device2': '0'}.items() 
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap, 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                    'params_file': slam_params,
                    'use_sim_time': use_sim_time,
                    'rgb_topic': '/camera/camera/color/image_raw',
                    'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                    'camera_info_topic': '/camera/camera/color/camera_info',
                    'frame_id': 'base_link',
                    'approx_sync': 'true',
                    'wait_imu_to_init': 'true',
                }.items()
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
    ])