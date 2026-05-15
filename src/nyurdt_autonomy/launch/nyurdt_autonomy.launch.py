from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='nyurdt_control', executable='command_subscriber', name='command_subscriber'),
        Node(package='nyurdt_autonomy', executable='telemetry_processor', name='telemetry_processor'),
        Node(package='nyurdt_autonomy', executable='excavation_node', name='excavation'),
        #Node(package='nyurdt_autonomy', executable='deposition_node', name='deposition'),
        #Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager')                                                                                                                                                                                                                                                                       
    ])


