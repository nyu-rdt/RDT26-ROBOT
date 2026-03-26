import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from my_robot_interfaces.msg import RobotStatus

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # Publisher for sending commands to the Teensy
        self.command_publisher = self.create_publisher(RobotStatus, '/robot/telemtry', 10)

        # Subscriber for receiving sensor data from the Teensy
        self.sensor_subscription = self.create_subscription(
            ByteMultiArray,
            '/raw_usb_data',
            self.unpack_callback,
            10
        )

    def unpack_callback(self, msg):
        """
        Callback function for processing incoming sensor data from the Teensy.
        This is where you can implement logic to determine what commands to send
        based on the sensor data.

        :param msg: The incoming sensor data message
        """
        self.get_logger().info(f"Received sensor data: {msg.data}")

        data = msg.data  # Assuming data is a list of bytes
        status = RobotStatus()
        # EE mapping: 0-255 to 0-20 Amps
        status.total_current = (sum[data[0:8]]/255)*20

        #EE mapping: Sum 4 load cells formass
        status.bin_mass = (sum[data[10:14]]/255)*self.load_cell_factor

        #EE mapping: String pot for conveyor position
        status.conveyor_position = (data[14]/255)*self.conveyor_length

        self.command_publisher.publish(status)
        self.get_logger().info(f"Published RobotStatus: {status}")
