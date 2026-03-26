import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nyurdt_interfaces.msg import RobotStatus

class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')

        self.mode_pub = self.create_publisher(String, '/robot/current_mode', 10)

        # Subscriber for robot status updates
        self.status_subscription = self.create_subscription(
            RobotStatus,
            '/robot/telemtry',
            self.telemetry_callback,
            10
        )

        self.state = "NAV_TO_DIG_SITE"
        self.goal_mass = 1.5
        self.min_mass = 0.1


    def telemetry_callback(self, data):
        self.get_logger().info(f"Received RobotStatus: {msg}")

        msg = String()

        if self.state == "NAV_TO_DIG_SITE":
            if self.at_dig_site():
                self.state = "DIGGING"
            msg.data = "NAVIGATING"
        
        elif self.state == "DIGGING":
            if msg.bin_mass >= self.goal_mass:
                self.state = "NAV_TO_DUMP_SITE"
            msg.data = "DIGGING"
        
        elif self.state == "NAV_TO_DUMP_SITE":
            if self.at_dump_site():
                self.state = "DUMPING"
            msg.data = "NAVIGATING"

        elif self.state == "DUMPING":
            if msg.bin_mass <= self.min_mass:
                self.state = "NAV_TO_DIG_SITE"
            msg.data = "DUMPING"

        self.mode_pub.publish(msg)