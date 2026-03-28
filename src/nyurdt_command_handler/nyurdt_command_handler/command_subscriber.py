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
        self.excavation_srv = self.create_service(Empty, 'excavation_autonomy', self.excavation_auto)
        self.construction_srv = self.create_service(Empty, 'construction_autonomy', self.construction_auto)


    def manual_callback(self, msg: Byte) -> None:
        """
        Handle incoming manual control commands
        """
        # msg.data is already an int for std_msgs/Byte
        val = msg.data
        self.get_logger().info(f"Incoming command: {hex(val)}")
        
        # Send the command to Teensy
        result = send_HEX(val)
        
        if result == -1:
            self.get_logger().error("I2C Write Failed!")
            return

        # If the command was 0x80, fetch the sensor feedback
        if val == 0x80:
            dictionary = get_data_dictionary()
            if dictionary:
                self.get_logger().info(f"Pivot Angle: {dictionary['Pivot angle']}")
            else:
                self.get_logger().warn("Failed to read pivot angle from Teensy")


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
    

    def construction_auto(self, request: Empty, response: Empty) -> Empty:
        """
        Begin construction autonomy
        """
        subprocess.Popen(['ros2', 'run', 'nyurdt_autonomy', 'construction'])
        return response
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = CommandSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
