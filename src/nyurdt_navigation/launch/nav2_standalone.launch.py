import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rdt = get_package_share_directory('nyurdt_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Arguments
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_rdt, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False', # Change to 'True' ONLY if running Gazebo
        description='Use simulation (Gazebo) clock if true')

    # Include the standard Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'params_file': params_file,
            # POINT THIS TO A REAL MAP FILE (.yaml), NOT nav2_params.yaml
            # 'map': os.path.join(pkg_rdt, 'maps', 'your_map.yaml'), 
            'use_sim_time': use_sim_time,
            'autostart': 'True'
        }.items()
    )

    # RViz Node
    rviz_config = os.path.join(pkg_rdt, 'rviz', 'nav2_view.rviz') 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        nav2_launch,
        rviz_node
    ])