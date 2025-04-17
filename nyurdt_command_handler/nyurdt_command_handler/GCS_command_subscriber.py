'''
GCS_command_subscriber.py

File for testing subscribe only, never used
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import Byte, Int16
from std_srvs.srv import Empty

import subprocess

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')

        # Subscibers for incoming commands
        self.subscription_manual = self.create_subscription(
            Byte, '/robotCmds/manual', self.manual_callback, 10)
        self.subscription_auto = self.create_subscription(
            Int16, '/robotCmds/autonomy', self.autonomy_callback, 10
        )

        # Services for beginning autonomies
        self.locomotion_srv = self.create_service(Empty, 'locmotion_autonomy', self.locomotion_auto)
        self.excavation_srv = self.create_service(Empty, 'excavation_autonomy', self.excavation_auto)


    def manual_callback(self, data: Byte) -> None:
        """
        Handle incoming manual control commands

        :param data: Byte message to be parsed & sent to Teensy
        :return: None
        """
        self.get_logger().info("Incoming manual control command: %s" % str(data.data))


    def autonomy_callback(self, data: Int16) -> None:
        """
        Handle incoming autonomy commands (via /robotCmds/autonomy topic)

        :param data: Int16 message representing which autonomy mode to start (1 = locomotion, 2 = excavation)
        :return: None
        """
        self.get_logger().info(f"Incoming autonomy command: {data.data}")

        if data.data == 1:
            self.get_logger().info("Launching Locomotion Autonomy...")
            self.locomotion_auto(None, None)

        elif data.data == 2:
            self.get_logger().info("Launching Excavation Autonomy...")
            self.excavation_auto(None, None)

        else:
            self.get_logger().warn(f"Unknown autonomy command received: {data.data}")


    def locomotion_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin locomotion autonomy

        :param request: Empty service request (not used)
        :param response: Empty service response to be returned
        :return: Empty service response
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'locomotion'])
        return response

    def excavation_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin dig and dump autonomy

        :param request: Empty service request (not used)
        :param response: Empty service response to be returned
        :return: Empty service response
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'excavation'])
        return response
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = CommandSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()