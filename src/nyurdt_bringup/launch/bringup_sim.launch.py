import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():


    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_dir    = get_package_share_directory('nyurdt_navigation')
    description_dir = get_package_share_directory('nyurdt_description')
    # realsense_dir = get_package_share_directory('realsense2_camera')
    control_dir = get_package_share_directory('nyurdt_control')

    nav2_params = os.path.join(nav2_dir, 'config', 'nav2_params.yaml')
    twist_mux_params = os.path.join(control_dir, 'config', 'twist_mux.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ── Robot State Publisher  ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
        ),

        # ── Joystick ──────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_dir, 'launch', 'control.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # # ── RealSense ────────────────────────────────────────────────────
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        #     ),
        #     launch_arguments={
        #         'enable_gyro': 'true',
        #         'enable_accel': 'true',
        #         'unite_imu_method': '2',
        #         'enable_sync': 'true',
        #     }.items()
        # ),


        # ── Twist Mux ────────────────────────────────────────────────
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
        ),

        # ── Controllers ──────────────────────────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
        ),

        # ── RTAB-Map SLAM ────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
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
                ('/rgb/image', '/camera/image_raw'),
                ('/depth/image', '/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera_info'),
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
                ('/rgb/image', '/camera/image_raw'),
                ('/depth/image', '/camera/depth/image_rect_raw'),
                ('/rgb/camera_info', '/camera/camera_info'),
            ]
        ),

        # ── Nav2 (from nyurdt_navigation) ─────────────────────────────────
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
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager'),
    ])
