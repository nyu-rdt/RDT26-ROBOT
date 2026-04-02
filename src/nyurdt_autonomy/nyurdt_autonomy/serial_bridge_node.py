import rclpy
from rclpy.node import Node
from std_msgs.msg import Byte
import serial
from nyurdt_interfaces.msg import RobotStatus

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)

        self.status_pub = self.create_publisher(RobotStatus, '/robot/telemtry', 10)
        self.cmd_sub = self.create_subscription(Byte, 'robot/mcu_cmd', self.write_to_teensy, 10)

        self.timer = self.create_timer(0.02, self.spin_serial)


    def spin_serial(self, data):
        if self.ser.in_waiting >=18:
            raw_bytes = self.ser.read(18)
            self.unpack_and_publish(raw_bytes)

    def unpack_and_publish(self, data):
        msg = RobotStatus()

        #1. Currents (Bytes 0-7)
        msg.motor_currents = [(byte/255)*20 for byte in data[0:8]]

        #2. Encoders (Bytes 8-9)
        msg.left_encoder = int(data[8])
        msg.right_encoder = int(data[9])

        #3. Load Cells (Bytes 10-13)
        msg.load_cells = [float(byte)/255 for byte in data[10:14]]
        msg.total_mass = sum(msg.load_cells) 

        #4. Potentiometer (Bytes 14 & 17)
        msg.conveyor_position = (data[14]/255)*100
        msg.gate_position = (data[17]/255)*100

        #5. Flags (Byte 15) - Bitmasking
        flags = data[15]
        msg.heartbeat = bool(flags & 0x01)
        msg.battery_low_3s = bool(flags & 0x02)
        msg.battery_low_6s = bool(flags & 0x04)
        msg.motor_overcurrent = bool(flags & 0x08)

        #6. Fixes (Byte 16
        msg.fixes_attempted = int(data[16])

        self.status_pub.publish(msg)
        self.get_logger().info(f"Published RobotStatus: {msg}")

    def write_to_teensy(self, msg):
        self.ser.write(bytes([msg.data]))
