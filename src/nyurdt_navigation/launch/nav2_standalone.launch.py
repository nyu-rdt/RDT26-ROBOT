'''
nav2_standalone.launch.py

Hardware-free Nav2 test — no Gazebo, no RealSense, no I2C.

The robot appears stationary at the arena starting position (1.00, 3.47 m).
Use RViz2 to send a "2D Nav Goal" and confirm that:
  - the planner computes a path through the arena map
  - the controller publishes cmd_vel (visible in RViz2 or `ros2 topic echo /cmd_vel`)

What works:  path planning, controller output, costmap visualisation
What doesn't: robot actually moves (no real odom feedback loop)

Run:
  ros2 launch nyurdt_navigation nav2_standalone.launch.py
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('nyurdt_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_desc = get_package_share_directory('nyurdt_description')

    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_nav, 'config', 'arena_map.yaml')
    rviz_config = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    # ── Robot description + static TF tree ───────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'rsp.launch.py')
        ),
    )

    # Publishes wheel joint states so robot_state_publisher can compute FK.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # ── Static localisation TFs ───────────────────────────────────────────────
    # Robot sits at the arena starting position (matches nav2_sim.launch.py spawn).
    # map → odom: places the odom frame at the robot's start in the arena.
    # odom → base_link: zero offset so the robot is at the odom origin.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['1.00', '3.47', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    # ── Map server ────────────────────────────────────────────────────────────
    # Override yaml_filename so it resolves via ament rather than the hardcoded
    # Jetson absolute path baked into nav2_params.yaml.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            params_file,
            {'yaml_filename': map_file, 'use_sim_time': False},
        ],
        output='screen',
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── Nav2 navigation stack (planner + controller + BT navigator) ───────────
    # Uses navigation_core.launch.py — no AMCL, map server is handled above.
    nav2_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation_core.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'False',
            'autostart': 'True',
            'use_composition': 'False',
            'namespace': '',
        }.items(),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher,
        map_to_odom,
        odom_to_base,
        map_server,
        lifecycle_manager_map,
        nav2_core,
        rviz,
    ])
