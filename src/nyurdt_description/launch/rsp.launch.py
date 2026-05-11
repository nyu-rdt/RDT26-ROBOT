'''
rsp.launch.py
publishes the static transforms of the robot
'''

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_path = get_package_share_directory('nyurdt_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
            'publish_frequency': 50.0,
        }],
        remappings=[('/joint_states', '/joint_states')],
        arguments=[
            '--ros-args',
            '--param', 'qos_overrides./joint_states.subscription.reliability:=reliable',
            '--param', 'qos_overrides./tf.publisher.reliability:=reliable',
            '--param', 'qos_overrides./tf_static.publisher.reliability:=reliable',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        node_robot_state_publisher
    ])