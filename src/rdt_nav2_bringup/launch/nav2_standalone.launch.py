import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rdt = get_package_share_directory('rdt_nav2_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Arguments
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_rdt, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    # Include the standard Nav2 bringup
    # This brings up the map_server, amcl, planner, controller, behavior trees, etc.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'map': '/home/nyu-rdt/RDT26-ROBOT/src/rdt_nav2_bringup/config/nav2_params.yaml', # Or pass as argument
            'use_sim_time': 'True', # Set to True if using Gazebo
            'autostart': 'True'
        }.items()
    )

    # RViz Node
    rviz_config = os.path.join(pkg_rdt, 'rviz', 'nav2_view.rviz') 
    # Note: You need to save an RViz config to this path later
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_params_file,
        nav2_launch,
        rviz_node
    ])