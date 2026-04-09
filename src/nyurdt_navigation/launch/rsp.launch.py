import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Path to your xacro file
    pkg_path = get_package_share_directory('nyurdt_navigation')
    xacro_file = os.path.join(pkg_path, 'config', 'robot.xacro')

    # 2. Process the xacro file into raw URDF (XML)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. Create the Robot State Publisher node
    params = {'robot_description': robot_desc, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])