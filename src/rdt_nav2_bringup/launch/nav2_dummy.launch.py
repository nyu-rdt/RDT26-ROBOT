from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Get path to existing Nav2 launch
    nav2_bringup_dir = get_package_share_directory('rdt_nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'nav2_sim.launch.py')
        )
    )

    # Dummy odometry node
    dummy_odom = Node(
        package='pathplanning',
        executable='dummy_odometry',
        name='dummy_odometry_node',
        output='screen',
        
        # uses sim_time
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'publish_rate': 50.0,
        }]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d',
            os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
        ]
    )

    return LaunchDescription([
        nav2_launch,
        dummy_odom,
        rviz,
    ])