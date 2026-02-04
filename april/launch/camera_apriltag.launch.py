#launch file for camera and apriltag and configs as composable nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    device_arg = DeclareLaunchArgument('device', default_value='0')
    device = LaunchConfiguration('device')

    apriltag_cfg = os.path.join(get_package_share_directory('april'), 'config', 'tags_36h11.yaml')

    container = ComposableNodeContainer(
        name='apriltag_cam_nodes',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                namespace='camera',
                parameters=[
                    {'camera': device},
                    {'format': 'MJPEG'},
                    #adjust path if needed, this is the path from the NUC
                    {'camera_info_url': 'file:///home/nyu-rdt/april_test/src/RDT26-ROBOT/april/config/LogitechC615.yaml'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='apriltag',
                remappings=[
                    ('image_rect', '/camera/camera/image_raw'),
                    ('camera_info', '/camera/camera/camera_info'),
                ],
                parameters=[apriltag_cfg],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        container
    ])
