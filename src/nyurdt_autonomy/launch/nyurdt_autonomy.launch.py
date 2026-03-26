from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='nyurdt_autonomy', executable='serial_bridge', name='serial_bridge_node'),
        Node(package='nyurdt_autonomy', executable='LocomotionNode', name='locomotion'),
        Node(package='nyurdt_autonomy', executable='ExcavationNode', name='excavation'),
        Node(package='nyurdt_autonomy', executable='ConstructionNode', name='construction'),
        Node(package='nyurdt_autonomy', executable='autonomy_destroyer', name='autonomy_destroyer'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager')                                                                                                                                                                                                                                                                       
    ])


