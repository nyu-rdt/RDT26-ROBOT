'''
gazebo.launch.py
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = 'nyurdt_navigation'
    pkg_description = 'nyurdt_description'
    

    # Path to your custom world file
    world_file = os.path.join(get_package_share_directory(pkg_navigation), 'worlds', 'my_world.sdf')

    # 0. Let Gazebo resolve model://nyurdt_description/... (our meshes) and model://realsense2_description/...
    #    gz looks these up under GZ_SIM_RESOURCE_PATH; ros_gz_sim's launch does not add overlay workspaces.
    resource_paths = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([
            os.path.dirname(get_package_share_directory(pkg_description)),
            os.path.dirname(get_package_share_directory('realsense2_description')),
        ]))

    # 1. Include existing Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_description),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo Sim launch file (UPDATED TO USE YOUR CUSTOM WORLD)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn the robot in Gazebo
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'rdt26_robot',
                                   '-z', '0.5'],
                        output='screen')

    # 4. The bridge. Topic list lives in nyurdt_description/config/bridge_parameters.yaml so it
    #    stays next to robot.gazebo, whose DiffDrive plugin publishes plain "odom" and "tf"
    #    (not the /model/<name>/... defaults). Bridges clock, joint_states, odom, tf, cmd_vel
    #    and the RealSense image/depth/points topics.
    bridge_params = os.path.join(get_package_share_directory(pkg_description), 'config', 'bridge_parameters.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    return LaunchDescription([
        resource_paths,
        rsp,
        gazebo,
        spawn_entity,
        bridge
    ])