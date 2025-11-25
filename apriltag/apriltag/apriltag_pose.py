import rclpy
from rclpy.node import Node
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('april_posesub')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagSubscriber()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()


