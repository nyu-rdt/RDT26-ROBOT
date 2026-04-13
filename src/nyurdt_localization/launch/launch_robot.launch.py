#top level robot launch file
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('nyurdt_localization'), 'launch'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'camera_tag.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
                ])
            ),
            launch_arguments={
                'align_depth.enable': 'true',
                'pointcloud.enable' : 'true',
            }.items()
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'tf.launch.py'])
        ),
    ])