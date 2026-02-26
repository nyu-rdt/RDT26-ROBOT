import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rdt_nav2_bringup'
    
    # 1. Include your existing Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo Sim launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn the robot in Gazebo
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'rdt26_robot'],
                        output='screen')

    # 4. The Bridge (connects /odom and /tf)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/rdt26_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/rdt26_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/rdt26_robot/tf', '/tf'),
            ('/model/rdt26_robot/odometry', '/odom'),
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge
    ])