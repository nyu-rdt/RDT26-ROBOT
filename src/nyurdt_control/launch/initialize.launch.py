"""
initialize.launch.py
"""

import os
import math

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    command_subscriber = Node(
        package='nyurdt_control',
        executable='command_subscriber',
        output='screen'
    )
    i2c_reader = Node(
        package='nyurdt_control',
        executable='i2c_reader',
        output='screen'
    )
    
    # sensor_service = Node(
    #     package='nyurdt_autonomy',
    #     executable='sensor_service',
    #     output='screen'
    # )
    
    # autonomy_destroyer = Node(
    #     package='nyurdt_autonomy',
    #     executable='autonomy_destroyer',
    #     output='screen'
    # )

    excavation = Node(
        package='nyurdt_autonomy',
        executable='excavation_node',
        output='screen'
    )

    return LaunchDescription([
        command_subscriber,
        i2c_reader,
        # sensor_service,
        # autonomy_destroyer,
        excavation
    ])