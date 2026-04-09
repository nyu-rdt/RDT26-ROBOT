import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = 'nyurdt_navigation'
    pkg_description = 'nyurdt_description'
    

    # Path to your custom world file
    world_file = os.path.join(get_package_share_directory(pkg_navigation), 'worlds', 'my_world.sdf')

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

    # 4. The Bridge (connects /odom, /tf, and NOW /scan)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/model/rdt26_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/rdt26_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
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