import rclpy
from rclpy.node import Node
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import Transform, TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagPose(Node):
    def __init__(self):
        super().__init__('april_posesub')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]
            detected_pose = detection.pose.pose.pose
            self.get_logger().info(f'AprilTag ID: {tag_id}')
            self.get_logger().info(f'Pose: {detected_pose}')


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagPose()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()


