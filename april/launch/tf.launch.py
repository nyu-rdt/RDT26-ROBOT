#launch file for full tf
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    april_tf_node = Node(
            package='april',
            executable='april_tf',
            name='apriltag_tf',
            output='screen'
        )
    
    return LaunchDescription([
        april_tf_node
    ])