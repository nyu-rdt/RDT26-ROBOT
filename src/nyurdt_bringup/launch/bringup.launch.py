import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nav2_dir    = get_package_share_directory('rdt_nav2_bringup')
    description_dir = get_package_share_directory('nyurdt_description')
    realsense_dir = get_package_share_directory('realsense2_camera')

    
    nav2_params = os.path.join(nav2_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ── Robot State Publisher (from nyurdt_description) ──────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── RealSense ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_dir, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'enable_sync': 'true',
            }.items()
        ),

        # ── RTAB-Map SLAM ────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
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
                ('rgb/image',        '/camera/color/image_raw'),
                ('rgb/camera_info',  '/camera/color/camera_info'),
                ('depth/image',      '/camera/depth/image_rect_raw'),
                ('initialpose',      '/apriltag/robot_pose'),
            ]
        ),

        # ── RTAB-Map Odometry ────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': True,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/depth/image_rect_raw'),
            ]
        ),

        # ── Nav2 (from rdt_nav2_bringup) ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'nav2_sim.launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # ── Autonomy Nodes ───────────────────────────────────────────────
        Node(package='nyurdt_localization', executable='apriltag_detector', name='apriltag_detector'),
        Node(package='nyurdt_command_handler', executable='i2c_reader', name='i2c_reader'),
        Node(package='nyurdt_autonomy', executable='telemetry_processor', name='telemetry_processor'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager'),
    ])