#launch file for camera and apriltag and configs as composable nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    device_arg1 = DeclareLaunchArgument('device1', default_value='0')
    device_arg2 = DeclareLaunchArgument('device2', default_value='1')
    device_arg3 = DeclareLaunchArgument('device3', default_value='2')

    device1 = LaunchConfiguration('device1')
    device2 = LaunchConfiguration('device2')
    device3 = LaunchConfiguration('device3')

    apriltag_cfg = os.path.join(get_package_share_directory('april'), 'config', 'tags_36h11.yaml')
    camera_cfg_a = os.path.join(get_package_share_directory('april'), 'config', 'LogitechC615.yaml')
    camera_cfg_b = os.path.join(get_package_share_directory('april'), 'config', 'LogitechC615b.yaml')
    camera_cfg_c = os.path.join(get_package_share_directory('april'), 'config', 'LogitechC615c.yaml')

    container1 = ComposableNodeContainer(
            name='apriltag_cam_nodes',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera1',
                namespace='camera1',
                parameters=[
                    {'camera': device1},
                    {'width': 352},
                    {'height': 288},
                    {'fps': 5},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_a},
                    {'frame_id': 'camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag1',
                namespace='apriltag',
                remappings=[
                    ('image_rect', '/camera1/camera1/image_raw'),
                    ('camera_info', '/camera1/camera1/camera_info'),
                ],
                parameters=[apriltag_cfg],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )
    container2 = ComposableNodeContainer(
            name='apriltag_cam_nodes2',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera2',
                namespace='camera2',
                parameters=[
                    {'camera': device2},
                    {'width': 352},
                    {'height': 288},
                    {'fps': 5},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_b},
                    {'frame_id': 'camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag2',
                namespace='apriltag2',
                remappings=[
                    ('image_rect', '/camera2/camera2/image_raw'),
                    ('camera_info', '/camera2/camera2/camera_info'),
                ],
                parameters=[apriltag_cfg],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )
    container3 = ComposableNodeContainer(
            name='apriltag_cam_nodes3',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera3',
                namespace='camera3',
                parameters=[
                    {'camera': device3},
                    {'width': 352},
                    {'height': 288},
                    {'fps': 5},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_c},
                    {'frame_id': 'camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag3',
                namespace='apriltag3',
                remappings=[
                    ('image_rect', '/camera3/camera3/image_raw'),
                    ('camera_info', '/camera3/camera3/camera_info'),
                ],
                parameters=[apriltag_cfg],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return LaunchDescription([
        device_arg1,
        device_arg2,
        device_arg3,
        container1,
        container2,
        container3
    ])
