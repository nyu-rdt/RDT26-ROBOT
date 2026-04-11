import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = get_package_share_directory("nyurdt_navigation")
    pkg_nav2 = get_package_share_directory("nav2_bringup")
    pkg_gz = get_package_share_directory("ros_gz_sim")

    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    slam = LaunchConfiguration("slam")
    robot_name = LaunchConfiguration("robot_name")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_navigation, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for Nav2",
    )

    declare_world = DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_navigation, "worlds", "my_world.sdf"),
            description="Gazebo world file",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_navigation, "config", "my_map.yaml"),
        description="Full path to map.yaml. Leave empty if using SLAM.",
    )

    declare_slam = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether to run SLAM (slam_toolbox) via nav2_bringup bringup_launch.py",
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="rdt26_robot",
        description="Entity name to spawn in Gazebo (must match bridge model name).",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_nav2, "rviz", "nav2_default_view.rviz"),
        description="Full path to an RViz config file",
    )

    # -------------------------
    # Robot State Publisher
    # -------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, "launch", "rsp.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # -------------------------
    # Joint State Publisher (FIX FOR WHEEL ERRORS)
    # -------------------------
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -------------------------
    # Gazebo Sim
    # -------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PythonExpression(['"-r " + "', world, '"'])}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", robot_name],
        output="screen",
    )

    # -------------------------
    # Bridges (Lidar + Odom + Clock)
    # -------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",


            # Odom and TF
            PythonExpression(['"/model/" + "', robot_name, '" + "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"']),
            PythonExpression(['"/model/" + "', robot_name, '" + "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"']),

            # Cmd Vel
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            (PythonExpression(['"/model/" + "', robot_name, '" + "/tf"']), "/tf"),
            (PythonExpression(['"/model/" + "', robot_name, '" + "/odometry"']), "/odom"),
        ],
        output="screen",
    )

    # -------------------------
    # Nav2 bringup
    # -------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, "launch", "bringup_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            "slam": slam,
            "map": map_yaml,
        }.items(),
    )

    warn_no_map_no_slam = LogInfo(
        condition=IfCondition(PythonExpression(['"', slam, '" == "False" and "', map_yaml, '" == ""'])),
        msg="[sim_nav2] WARNING: slam:=False and map:='' -> RViz will show only the robot (no /map).",
    )

    # -------------------------
    # RViz
    # -------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_world,
        declare_map,
        declare_slam,
        declare_robot_name,
        declare_rviz_config,
        warn_no_map_no_slam,
        rsp,
        joint_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        nav2_launch,
        rviz_node,
    ])