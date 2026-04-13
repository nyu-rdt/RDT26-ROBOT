"""
autonomy_destoyer.py

File for managing & destroying autonomy nodes by listening to any manual control commands.
Any manual control command stops autonomy files from running, regardless of what it is.
"""

import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import Byte

class AutonomyDestroyer(Node):

    def __init__(self):
        super().__init__('autonomy_destroyer')

        # Manual control interrupt subscribers
        self.manual_subscription = self.create_subscription(
            Byte,
            "/robotCmds/manual",
            self.manual_callback,
            10
        )
    

    def manual_callback(self, msg):
        """
        Method used to terminate the autonomy files if a manual control command
        is sent
        :return: None
        """
        # Kill deposition node
        process_one = subprocess.Popen(['killall', 'deposition'])
        process_one.wait()

        # Kill excavation node
        process_two = subprocess.Popen(['killall', 'excavation'])
        process_two.wait()

        # Log that autonomies were killed
        self.get_logger().info("Killed autonomies")


def main(args=None):
    rclpy.init(args=args)
    autonomy_destroyer = AutonomyDestroyer()
    rclpy.spin(autonomy_destroyer)
    autonomy_destroyer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()