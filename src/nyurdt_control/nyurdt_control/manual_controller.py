#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import sys
from rclpy.executors import ExternalShutdownException
from nyurdt_control.button_map import ButtonMap  
from nyurdt_control.axis_map import AxisMap  
from nyurdt_control.hex_map import OtherHex, LocomotionHex, ExcavationHex, DepositionHex
from std_msgs.msg import Byte  

class JoyTester(Node):
    DEADZONE = 0.05  
    RATE_LIMIT = 1 / 3  # 3 times per second (0.333 seconds)

    def __init__(self):
        super().__init__('manual_control_publisher')
        self.get_logger().info('Manual Control Publisher Node started')

        self.publisher_ = self.create_publisher(Byte, "/robotCmds/manual", 10)
        self.get_logger().info('Publisher created for topic: robotCmds/manual')
        self.subscription = self.create_subscription(Joy, '/joy', self.read_input, 10)


        # self.subscription = self.create_subscription(Joy, '/joy', self.call_back, 10)
        self.get_logger().info('Subscribed to topic: joy')
        self.last_processed_time = time.time()

        self.get_logger().info('JoyTester node initialized and ready to receive joystick inputs.')

    # def call_back(self, joy_msg: Joy):
    #     self.get_logger().info(f"Received Joy message: {joy_msg}")

    def read_input(self, joy_msg: Joy):
        """Processes joystick and button inputs."""
        current_time = time.time()
        self.get_logger().debug(f"Received Joy message: {joy_msg}")

        # Emergency Stop (Shuts down node immediately)
        if joy_msg.buttons[ButtonMap.XBOX.value] == 1:
            self.publish_msg(OtherHex.EMERGENCY_STOP.value, OtherHex.EMERGENCY_STOP.name)
            self.shutdown_node()
            return

        # Dictionary mapping buttons to corresponding messages
        button_mappings = {
            ButtonMap.EXIT.value: (OtherHex.REQUEST_DATA.value, OtherHex.REQUEST_DATA.name),
            ButtonMap.WINDOW.value: (OtherHex.SWITCH_AUTONOMOUS.value, OtherHex.SWITCH_AUTONOMOUS.name),
            ButtonMap.LB.value: (ExcavationHex.BELT_OUTWARD.value, ExcavationHex.BELT_OUTWARD.name),
            ButtonMap.RB.value: (ExcavationHex.BELT_INWARD.value, ExcavationHex.BELT_INWARD.name),
            ButtonMap.MENU.value: (ExcavationHex.BELT_STOP.value, ExcavationHex.BELT_STOP.name),
            ButtonMap.A.value: (ExcavationHex.ZERO_EXCAVATION.value, ExcavationHex.ZERO_EXCAVATION.name),
            ButtonMap.B.value: (ExcavationHex.LOCOMOTION_POSITION.value, ExcavationHex.LOCOMOTION_POSITION.name),
            ButtonMap.Y.value: (ExcavationHex.EXCAVATION_POSITION.value, ExcavationHex.EXCAVATION_POSITION.name),
        }

        # Process button presses
        for button, (hex_value, hex_name) in button_mappings.items():
            if joy_msg.buttons[button] == 1:
                self.publish_msg(hex_value, hex_name)

        # Dictionary mapping D-pad movements to actions
        dpad_mappings = {
            (AxisMap.DPAD_X.value, -1): (DepositionHex.BIN_OPEN.value, DepositionHex.BIN_OPEN.name),
            (AxisMap.DPAD_X.value, 1): (DepositionHex.BIN_CLOSE.value, DepositionHex.BIN_CLOSE.name),
            (AxisMap.DPAD_Y.value, -1): (DepositionHex.VIBRATE_BIN.value, DepositionHex.VIBRATE_BIN.name),
            (AxisMap.DPAD_Y.value, 1): (DepositionHex.VIBRATE_STOP.value, DepositionHex.VIBRATE_STOP.name),
        }
        

        # Process D-pad inputs
        for (axis, direction), (hex_value, hex_name) in dpad_mappings.items():
            if joy_msg.axes[axis] == direction:
                self.publish_msg(hex_value, hex_name)

        # Handle unused buttons explicitly
        if joy_msg.buttons[ButtonMap.X.value] == 1:
            pass  # No action assigned yet

        if joy_msg.axes[AxisMap.DPAD_Y.value] == 1:
            pass  # No action assigned yet

        # Only apply rate limiting to axis inputs
        if current_time - self.last_processed_time < self.RATE_LIMIT:
            return  # Skip processing if within rate limit

        self.last_processed_time = current_time

        left_stick_y = joy_msg.axes[AxisMap.LEFT_STICK_Y.value]
        right_stick_x = joy_msg.axes[AxisMap.RIGHT_STICK_X.value]

        if self.is_above_deadzone(left_stick_y):
            self.publish_locomotion(left_stick_y, "LEFT_STICK_Y")
        
        if self.is_above_deadzone(right_stick_x):
            self.publish_locomotion(right_stick_x, "RIGHT_STICK_X")

        if not self.is_above_deadzone(left_stick_y) and not self.is_above_deadzone(right_stick_x):
            self.publish_msg(LocomotionHex.LOCOMOTION_STOP.value, LocomotionHex.LOCOMOTION_STOP.name)

    def publish_locomotion(self, joystick_pos: float, joystick_axis: str):
        """Maps joystick positions to corresponding locomotion commands."""
        locomotion_map = {
            "LEFT_STICK_Y": {
                (0.75, 1.01): LocomotionHex.LOCOMOTION_FORWARD_100,
                (0.5, 0.75): LocomotionHex.LOCOMOTION_FORWARD_75,
                (0.25, 0.5): LocomotionHex.LOCOMOTION_FORWARD_50,
                (0.0, 0.25): LocomotionHex.LOCOMOTION_FORWARD_25,
                (-1.01, -0.75): LocomotionHex.LOCOMOTION_BACKWARD_100,
                (-0.75, -0.5): LocomotionHex.LOCOMOTION_BACKWARD_75,
                (-0.5, -0.25): LocomotionHex.LOCOMOTION_BACKWARD_50,
                (-0.25, 0.0): LocomotionHex.LOCOMOTION_BACKWARD_25,
            },
            "RIGHT_STICK_X": {
                (0.75, 1.01): LocomotionHex.LOCOMOTION_RIGHT_100,
                (0.5, 0.75): LocomotionHex.LOCOMOTION_RIGHT_75,
                (0.25, 0.5): LocomotionHex.LOCOMOTION_RIGHT_50,
                (0.0, 0.25): LocomotionHex.LOCOMOTION_RIGHT_25,
                (-1.01, -0.75): LocomotionHex.LOCOMOTION_LEFT_100,
                (-0.75, -0.5): LocomotionHex.LOCOMOTION_LEFT_75,
                (-0.5, -0.25): LocomotionHex.LOCOMOTION_LEFT_50,
                (-0.25, 0.0): LocomotionHex.LOCOMOTION_LEFT_25,
            },
        }

        for (low, high), command in locomotion_map.get(joystick_axis, {}).items():
            if low <= joystick_pos < high:
                self.publish_msg(command.value, command.name)
                break

    def publish_msg(self, data: bytes, hex_name: str):
        """Publishes data to the topic"""
        msg = Byte()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {hex_name} 0x{int.from_bytes(data, 'little'):02x}")

    @staticmethod
    def is_above_deadzone(value: float) -> bool:
        """Checks if the joystick input is outside the deadzone."""
        return abs(value) > JoyTester.DEADZONE


    def shutdown_node(self):
        """Safely shuts down the node."""
        self.get_logger().info("Shutting down JoyTester node...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()
    rclpy.spin(joy_tester)
    rclpy.shutdown()


        
if __name__ == '__main__':
    main()
