'''
command_subscriber.py

File for testing subscribe only, never used
'''

import rclpy
from rclpy.node import Node

from .i2c_comms import send_HEX
from .i2c_comms import get_data_dictionary

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
        self.dig_and_dump_srv = self.create_service(Empty, 'dig_and_dump_autonomy', self.dig_and_dump_auto)


    def manual_callback(self, data: Byte) -> None:
        """
        Handle incoming manual control commands

        :param data: Byte message to be parsed & sent to Teensy
        :return: None
        """
        self.get_logger().info("Incoming manual control command: %s" % str(data.data))
        
        # Code from the i2c_comms
        send_HEX(int.from_bytes(data.data, 'little'));
        if data.data == b'\x80':
            dictionary = get_data_dictionary()
            print(dictionary)


    def autonomy_callback(self, data: Int16) -> None:
        """
        Handle incoming autonomy commands

        :param data: Int16 message to be parsed & sent to the Teensy
        :return: None
        """
        self.get_logger().info("Incoming autonomy command: %s" % str(data.data))
        pass
    

    def locomotion_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin excacation autonomy
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'locomotion'])
        return response
    

    def dig_and_dump_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin construction autonomy
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'dig_and_dump'])
        return response
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = CommandSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
