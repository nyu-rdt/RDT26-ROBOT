import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Byte

class HexBridge(Node):
    def __init__(self):
        super().__init__('hex_to_twist_bridge')

        self.subscription = self.create_subscription(
            Byte,
            'robotCmds/manual',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel_joy',
            10
        )
    
    def callback(self, msg):
        twist = Twist()

        code = msg.data


        if code == 0x10:
            pass

        elif code == 0x20:
            twist.linear.x = 0.5

        elif code == 0x21:
            twist.linear.x = 1.0

        elif code == 0x22:
            twist.linear.x = 1.5

        elif code == 0x23:
            twist.linear.x = 2.0

        elif code == 0x30:
            twist.linear.x = -0.5

        elif code == 0x31:
            twist.linear.x = -1.0

        elif code == 0x40:
            twist.angular.z = 0.5

        elif code == 0x41:
            twist.angular.z = 1.0

        elif code == 0x50:
            twist.angular.z = -0.5

        elif code == 0x51:
            twist.angular.z = -1.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HexBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()