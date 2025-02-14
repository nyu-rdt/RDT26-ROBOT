from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',     # Dummy pointcloud subscriber to keep the pointcloud topic subscribed. This does not seem to work for some reason, thats why we subscribe to the pointcloud in RViz as well.
            remappings=[('cloud_in', '/camera/camera/depth/color/points')],
            parameters=[{'cloud_frame_id': '/camera/camera/depth/color/points', 'cloud_extent': 3.0, 'cloud_size': 100.0}],
            name='cloud_publisher'
        ),
        Node(
            package='tf2_ros',  # Start publishing a new frame. Have to test rotating this frame and using it as the target frame in pointcloud to laserscan node to align the laserscan after the camera is tilted downwards.
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '1', '--y', '1', '--z', '1',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'cloud'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',   # Start pointcloud to laserscan node
            remappings=[('/cloud_in', '/camera/camera/depth/color/points'),                 # Remap pointcloud topic to cloud_in for the pointcloud to laserscan node
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])], # Starts publishing the output scan to scanner/scan topic
            parameters=[{
                'target_frame': 'camera_link',  # Set the target frame to camera_link to properly align the laserscan.
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -0.76,  # -M_PI/2  Realsense -FOV/2
                'angle_max': 0.76,  # M_PI/2    Realsense FOV/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.5,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'queue_size' : 10
            }],
            name='pointcloud_to_laserscan'
        )
    ])
