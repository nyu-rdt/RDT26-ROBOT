#launch file for robot and camera models
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    robot_xacro = os.path.join(get_package_share_directory('april'), 'model', 'robot.xacro')
    robot_desc = xacro.process_file(robot_xacro).toxml()

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )
    
    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
