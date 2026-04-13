from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='nyurdt_autonomy', executable='ExcavationNode', name='excavation'),
        Node(package='nyurdt_autonomy', executable='DepostionNode', name='deposition'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager')                                                                                                                                                                                                                                                                       
    ])


