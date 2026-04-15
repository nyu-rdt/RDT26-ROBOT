import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = get_package_share_directory('nyurdt_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
    ])
