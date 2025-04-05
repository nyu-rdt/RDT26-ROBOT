import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

class OdometryToBaseLinkPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_tf_broadcaster')

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish transforms at a regular rate
        self.timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        # Create a TransformStamped message
        transform = TransformStamped()

        # Fill in the header information
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"  # Parent frame
        transform.child_frame_id = "base_link"  # Child frame

        # Define the translation (offset) between odom and base_link
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Define the rotation (identity quaternion in this case)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation (identity quaternion)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Publishing transform from odom to base_link')

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and spin the node
    node = OdometryToBaseLinkPublisher()
    rclpy.spin(node)

    # Shutdown ROS 2 when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
