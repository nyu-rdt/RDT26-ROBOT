# launch/bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_dir    = get_package_share_directory('nav2_bringup')
    my_pkg_dir  = get_package_share_directory('nyurdt_autonomy')
    nav2_params = os.path.join(my_pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # ── Robot state publisher (URDF → TF) ───────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(
                os.path.join(my_pkg_dir, 'urdf', 'robot.urdf')
            ).read()}]
        ),

        # ── RealSense ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py'
                )
            ),
            launch_arguments={
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'enable_sync': 'true',
            }.items()
        ),

        # ── RTAB-Map ─────────────────────────────────────────────────────
        Node(
            package='rtabmap_ros',
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
                'Mem/IncrementalMemory': 'true',
                'Kp/MaxFeatures': '400',
            }],
            remappings=[
                ('rgb/image',        '/camera/color/image_raw'),
                ('rgb/camera_info',  '/camera/color/camera_info'),
                ('depth/image',      '/camera/depth/image_rect_raw'),
                ('initialpose',      '/apriltag/robot_pose'),  # AprilTag corrections
            ]
        ),

        # ── RTAB-Map odometry ────────────────────────────────────────────
        Node(
            package='rtabmap_ros',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/depth/image_rect_raw'),
            ]
        ),

        # ── Nav2 ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
            }.items()
        ),

        # ── AprilTag localization node ────────────────────────────────────
        Node(
            package='nyurdt_autonomy',
            executable='apriltag_localization',
            name='apriltag_localization',
            output='screen',
        ),

        # ── I2C reader (Teensy) ───────────────────────────────────────────
        Node(
            package='nyurdt_autonomy',
            executable='i2c_reader',
            name='i2c_reader',
            output='screen',
        ),

        # ── Telemetry processor ───────────────────────────────────────────
        Node(
            package='nyurdt_autonomy',
            executable='telemetry_processor',
            name='telemetry_processor',
            output='screen',
        ),

        # ── Autonomy manager ──────────────────────────────────────────────
        Node(
            package='nyurdt_autonomy',
            executable='autonomy_manager',
            name='autonomy_manager',
            output='screen',
        ),

    ])