from launch import LaunchDescription 
from launch_ros.actions import Node 
# Gad26. 'Constants for last year it seems this

# ROS always look for this function when launch starts
def generate_launch_description():
    # Robot frames & sensor measurements
    # All measurements relative to ground level, at the CENTER of the robot
    ROBOT_FRAME = "robot"

    REALSENSE_OFFSET_X = "0.777"
    REALSENSE_OFFSET_Y = "-0.2375"
    REALSENSE_OFFSET_Z = "0.35"
    REALSENSE_FRAME = "realsense"

    WEBCAM_FRONT_OFFSET_X = "0.756"
    WEBCAM_FRONT_OFFSET_Y = "-0.247"
    WEBCAM_FRONT_OFFSET_Z = "0.502"
    WEBCAM_FRONT_FRAME = "webcam_front"

    WEBCAM_BACK_OFFSET_X = "-0.707"
    WEBCAM_BACK_OFFSET_Y = "0"
    WEBCAM_BACK_OFFSET_Z = "0.636"
    WEBCAM_BACK_FRAME = "webcam_back"

    # Use static_transform_publisher to publish frame transforms
    # Arguments: x, y, z, yaw, pitch, roll, parent, child
    #   Measurements in meters & radians
    #   Parent -- Origin frame
    #   Child -- Destination frame

    realsense_tf = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = [REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y, REALSENSE_OFFSET_Z, '0', '0', '0', ROBOT_FRAME, REALSENSE_FRAME],
        output = 'screen'
    )

    webcam_front_tf = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = [WEBCAM_FRONT_OFFSET_X, WEBCAM_FRONT_OFFSET_Y, WEBCAM_FRONT_OFFSET_Z, '0', '0', '0', ROBOT_FRAME, WEBCAM_FRONT_FRAME],
        output = 'screen'
    )

    webcam_back_tf = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = [WEBCAM_BACK_OFFSET_X, WEBCAM_BACK_OFFSET_Y, WEBCAM_BACK_OFFSET_Z, '3.14', '0', '0', ROBOT_FRAME, WEBCAM_BACK_FRAME],
        output = 'screen'
    )

    return LaunchDescription([
        realsense_tf, 
        # lidar_tf, 
        webcam_front_tf, 
        webcam_back_tf
    ])

    
