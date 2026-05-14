'''
command_subscriber.py

this file may be deleted or reconfigured in the future
'''

import rclpy
from rclpy.node import Node

from .i2c_comms import send_HEX, i2c_init

from std_msgs.msg import Byte, Int16
from std_srvs.srv import Empty

import subprocess

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        i2c_init()
        # Subscibers for incoming commands
        self.subscription_manual = self.create_subscription(
            Byte, '/robotCmds/manual', self.manual_callback, 10)
        self.subscription_auto = self.create_subscription(
            Int16, '/robotCmds/autonomy', self.autonomy_callback, 10
        )

        # Services for beginning autonomies
        self.excavation_srv = self.create_service(Empty, 'excavation_autonomy', self.excavation_auto)
        self.deposition_srv = self.create_service(Empty, 'deposition_autonomy', self.deposition_auto)


    def manual_callback(self, data: Byte) -> None:
        """
        Handle incoming manual control commands

        :param data: Byte message to be parsed & sent to Teensy
        :return: None
        """
        self.get_logger().info("Incoming manual control command: %s" % str(data.data))
        
        # Code from the i2c_comms
        send_HEX(int.from_bytes(data.data, 'little'));
        # send_HEX(data.data)
        # if data.data == b'\x80':
        #    dictionary = get_data_dictionary()
        #    print(dictionary)
        #  

    def autonomy_callback(self, data: Int16) -> None:
        """
        Handle incoming autonomy commands

        :param data: Int16 message to be parsed & sent to the Teensy
        :return: None
        """
        self.get_logger().info("Incoming autonomy command: %s" % str(data.data))
        pass
    

    def excavation_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin excacation autonomy
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'excavation'])
        return response
    

    def deposition_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin deposition autonomy
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'deposition'])
        return response
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = CommandSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
