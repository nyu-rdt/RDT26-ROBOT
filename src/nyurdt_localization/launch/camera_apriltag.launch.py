#launch file for camera and apriltag and configs as composable nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    device_arg1 = DeclareLaunchArgument('front_cam', default_value='0')
    device_arg2 = DeclareLaunchArgument('right_cam', default_value='2')
    device_arg3 = DeclareLaunchArgument('back_cam', default_value='1')

    device1 = LaunchConfiguration('front_cam')
    device2 = LaunchConfiguration('right_cam')
    device3 = LaunchConfiguration('back_cam')

    # USE LogitechC615c.yaml for all logitechs for now, most accurate
    apriltag_cfg = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'tags_36h11.yaml')
    camera_cfg_a = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'LogitechC615.yaml')
    camera_cfg_b = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'Innomaker.yaml')
    camera_cfg_c = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'LogitechC615c.yaml')

    container1 = ComposableNodeContainer(
            name='apriltag_cam_nodes',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                namespace='front_cam',
                parameters=[
                    {'camera': device1},
                    {'width': 640},
                    {'height': 480},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_c},
                    {'frame_id': 'front_camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='front_cam',
                remappings=[
                    ('image_rect', '/front_cam/camera/image_raw'),
                    ('camera_info', '/front_cam/camera/camera_info'),
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
                    {'width': 1920},
                    {'height': 1080},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_b},
                    {'frame_id': 'camera_optical_frame_link2'}
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
                name='camera',
                namespace='right_cam',
                parameters=[
                    {'camera': device3},
                    {'width': 1600},
                    {'height': 896},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_c},
                    {'frame_id': 'right_camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='right_cam',
                remappings=[
                    ('image_rect', '/right_cam/camera/image_raw'),
                    ('camera_info', '/right_cam/camera/camera_info'),
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
