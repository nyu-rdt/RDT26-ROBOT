# Author: Gad Major & Chris Jin

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = get_package_share_directory('nyurdt_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_params = os.path.join(package_name, 'config', 'joy_config.yaml')
    twist_params = os.path.join(package_name, 'config', 'twist_mux.yaml')

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

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_params],
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
        teleop_node,
        twist_mux_node,
        teensy_bridge_node,
    ])