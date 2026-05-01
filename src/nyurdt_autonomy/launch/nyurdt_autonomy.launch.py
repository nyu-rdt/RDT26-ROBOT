from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='nyurdt_autonomy', executable='excavation_node', name='excavation'),
        Node(package='nyurdt_autonomy', executable='deposition_node', name='deposition'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager')                                                                                                                                                                                                                                                                       
    ])


