#launch file for camera and apriltag and configs as composable nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #WAITING ON FOURTH CAMERA
    device_arg1 = DeclareLaunchArgument('front_cam', default_value='/base/bus@0/usb@3610000-2.4:1.2-046d:082c') #jetson bottom right dp logitech cam w/ velcro
    device_arg2 = DeclareLaunchArgument('left_cam', default_value='/base/bus@0/usb@3610000-1.1:1.0-0bda:5883') #top left port innomaker
    # device_arg3 = DeclareLaunchArgument('right_cam', default_value='\_SB_.PCI0.XHC_.RHUB.SS02-2:1.0-0bda:5883') #innomaker
    device_arg4 = DeclareLaunchArgument('back_cam', default_value='/base/bus@0/usb@3610000-2.2:1.2-046d:082c') #bottom left logitech camera plastic 

    device1 = LaunchConfiguration('front_cam')
    device2 = LaunchConfiguration('left_cam')
    # device3 = LaunchConfiguration('right_cam')
    device4 = LaunchConfiguration('back_cam')

    # USE LogitechC615c.yaml for all logitechs for now, most accurate
    apriltag_cfg = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'tags_36h11.yaml')
    # camera_cfg_a = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'LogitechC615.yaml')
    camera_cfg_b = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'Innomaker.yaml')
    camera_cfg_c = os.path.join(get_package_share_directory('nyurdt_localization'), 'config', 'LogitechC615c.yaml')


    container1 = ComposableNodeContainer(
            name='apriltag_cam_nodes3',
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
            ),
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
                namespace='left_cam',
                parameters=[
                    {'camera': device2},
                    {'width': 800},
                    {'height': 600},
                    {'format': 'MJPEG'},
                    {'camera_info_url': 'file://' + camera_cfg_b},
                    {'frame_id': 'left_camera_optical_frame_link'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag2',
                namespace='left_cam',
                remappings=[
                    ('image_rect', '/left_cam/camera2/image_raw'),
                    ('camera_info', '/left_cam/camera2/camera_info'),
                ],
                parameters=[apriltag_cfg],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )

    # container3 = ComposableNodeContainer(
    #         name='apriltag_cam_nodes',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #         ComposableNode(
    #             package='camera_ros',
    #             plugin='camera::CameraNode',
    #             name='camera3',
    #             namespace='right_cam',
    #             parameters=[
    #                 {'camera': device3},
    #                 {'width': 800},
    #                 {'height': 600},
    #                 {'format': 'MJPEG'},
    #                 {'camera_info_url': 'file://' + camera_cfg_b},
    #                 {'frame_id': 'right_camera_optical_frame_link'}
    #             ],
    #             extra_arguments=[{'use_intra_process_comms': True}],
    #         ),
    #         ComposableNode(
    #             package='apriltag_ros',
    #             plugin='AprilTagNode',
    #             name='apriltag',
    #             namespace='right_cam',
    #             remappings=[
    #                 ('image_rect', '/right_cam/camera/image_raw'),
    #                 ('camera_info', '/right_cam/camera/camera_info'),
    #             ],
    #             parameters=[apriltag_cfg],
    #             extra_arguments=[{'use_intra_process_comms': True}],
    #         )
    #     ],
    # )


    container4 = ComposableNodeContainer(
        name='apriltag_cam_nodes',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='camera4',
            namespace='back_cam',
            parameters=[
                {'camera': device4},
                {'width': 640},
                {'height': 480},
                {'format': 'MJPEG'},
                {'camera_info_url': 'file://' + camera_cfg_c},
                {'frame_id': 'back_camera_optical_frame_link'}
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode',
            name='apriltag',
            namespace='back_cam',
            remappings=[
                ('image_rect', '/back_cam/camera4/image_raw'),
                ('camera_info', '/back_cam/camera4/camera_info'),
            ],
            parameters=[apriltag_cfg],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
        ],
    )

    return LaunchDescription([
        device_arg1,
        device_arg2,
        # device_arg3,
        device_arg4,
        container1,
        container2,
        # container3,
        container4
    ])
