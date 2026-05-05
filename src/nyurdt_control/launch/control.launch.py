# Author: Gad Major & Chris Jin

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = get_package_share_directory('nyurdt_control')
    twist_mux_pkg = get_package_share_directory('twist_mux')

    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_params = os.path.join(package_name, 'config', 'joy_config.yaml')
    twist_topics = os.path.join(package_name, 'config', 'twist_mux.yaml')
    twist_locks = os.path.join(package_name, 'config', 'twist_mux_locks.yaml')

    manual_control_node = Node(
        package='nyurdt_control',
        executable='manual_control_publisher',
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            {'use_sim_time': use_sim_time},
            twist_locks,
            twist_topics,
        ],
        remappings=[
            ('/cmd_vel_out', '/robot/cmd_vel')
        ],
        output='screen',
    )


    teensy_bridge_node = Node(
        package='nyurdt_control',
        executable='cmd_vel_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # manual_control_node,
        joy_node,
        # teleop_node,
        twist_mux_node,
        teensy_bridge_node,
    ])
