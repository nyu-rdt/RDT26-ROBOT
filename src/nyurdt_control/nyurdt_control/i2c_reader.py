'''
i2c_reader.py
'''

import rclpy
from rclpy.node import Node

from .i2c_comms import i2c_init, _i2c, teensy_address


# import subprocess

import rclpy
from rclpy.node import Node
import struct
# import os

from nyurdt_interfaces.msg import RobotStatus


PACKET_SIZE = 18
PACKET_FORMAT = '8B2B4BBBBB'  

CURRENT_MAX_AMPS = 20.0
ENCODER_MAX_DEG  = 360.0


class I2CReader(Node):
    def __init__(self):
        super().__init__('i2c_reader')
        i2c_init()

        self.pub = self.create_publisher(RobotStatus, 'robot/telemetry_raw', 10)

        try:
            # self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            self.i2c = _i2c  
            self.get_logger().info("I2C initialized")
        except Exception as e:
            self.get_logger().error(f"I2C init failed: {e}")
            self.i2c = None

        # Read at 20Hz — adjust to however fast MCU can respond
        self.create_timer(0.05, self._read_and_publish)

    def _read_and_publish(self):
        if self.i2c is None:
            return

        try:
            buf = bytearray(PACKET_SIZE)
            self.i2c.readfrom_into(teensy_address, buf)
        except Exception as e:
            self.get_logger().warn(f"I2C read failed: {e}")
            return

        try:
            fields = struct.unpack(PACKET_FORMAT, buf)
        except struct.error as e:
            self.get_logger().error(f"Packet unpack failed: {e}")
            return

        currents_raw  = fields[0:8]
        left_enc_raw  = fields[8]
        right_enc_raw = fields[9]
        load_raw      = fields[10:14]
        conv_raw      = fields[14]
        gate_raw      = fields[15]
        flags         = fields[16]
        fixes         = fields[17]

        msg = RobotStatus()

        # Convert currents: 0-255 → 0-20A
        msg.motor_currents = [
            (v / 255.0) * CURRENT_MAX_AMPS for v in currents_raw
        ]

        # Encoders — keep as raw ticks for now, odometry node handles conversion
        msg.left_encoder  = left_enc_raw
        msg.right_encoder = right_enc_raw

        # Load cells — normalize 0-255 to kg (calibrate scale factor for your cells)
        LOAD_CELL_MAX_KG = 5.0  # replace with your real max
        msg.load_cells = [
            (v / 255.0) * LOAD_CELL_MAX_KG for v in load_raw
        ]

        # Positions: normalize 0-255 to 0.0-1.0
        msg.conveyor_pos = conv_raw / 255.0

        
        msg.gate_pos     = gate_raw 

        # Parse flags byte
        msg.heartbeat        = bool(flags & (1 << 0))
        msg.battery_low_3s   = bool(flags & (1 << 1))
        msg.battery_low_6s   = bool(flags & (1 << 2))
        msg.motor_overcurrent = bool(flags & (1 << 3))

        msg.fixes_attempted = fixes

        # Drop packet if no heartbeat — MCU is not sending valid data
        if not msg.heartbeat:
            self.get_logger().warn("No heartbeat — dropping packet")
            return

        self.pub.publish(msg)
        
 
        
def main(args=None):
    rclpy.init(args=args)
    
    i2c_reader = I2CReader()    
    rclpy.spin(i2c_reader)
    i2c_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()