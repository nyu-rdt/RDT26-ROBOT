#!/usr/bin/env python3
"""
cmd_vel_bridge.py

Bridges Nav2 /cmd_vel Twist messages to hex locomotion commands
published on /robot/mcu_cmd.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Byte
from enum import Enum
from .i2c_comms import i2c_init, send_HEX

class LocomotionHex(Enum):
    LOCOMOTION_STOP        = 0x10
    LOCOMOTION_FORWARD_25  = 0x20
    LOCOMOTION_FORWARD_50  = 0x21
    LOCOMOTION_FORWARD_75  = 0x22
    LOCOMOTION_FORWARD_100 = 0x23
    LOCOMOTION_BACKWARD_25 = 0x30
    LOCOMOTION_BACKWARD_50 = 0x31
    LOCOMOTION_BACKWARD_75 = 0x32
    LOCOMOTION_BACKWARD_100= 0x33
    LOCOMOTION_LEFT_25     = 0x40
    LOCOMOTION_LEFT_50     = 0x41
    LOCOMOTION_LEFT_75     = 0x42
    LOCOMOTION_LEFT_100    = 0x43
    LOCOMOTION_RIGHT_25    = 0x50
    LOCOMOTION_RIGHT_50    = 0x51
    LOCOMOTION_RIGHT_75    = 0x52
    LOCOMOTION_RIGHT_100   = 0x53


# Tune these to your robot's actual max linear/angular speeds (m/s, rad/s)
MAX_LINEAR  = 0.5   # m/s  — Nav2 will never exceed this
MAX_ANGULAR = 1.0   # rad/s


def velocity_to_hex(linear_x: float, angular_z: float) -> int:
    """
    Map a Twist's linear.x and angular.z to the closest hex command.

    Priority: rotation > translation (matches differential-drive behavior
    where Nav2 often issues pure-rotation commands to face a goal).
    """

    lin_norm = linear_x  / MAX_LINEAR   # [-1.0, 1.0]
    ang_norm = angular_z / MAX_ANGULAR  # [-1.0, 1.0] (+ = left/CCW)

    # Dead-band — treat tiny values as stop
    DEAD = 0.05
    lin_dead = abs(lin_norm) < DEAD
    ang_dead = abs(ang_norm) < DEAD

    if lin_dead and ang_dead:
        return LocomotionHex.LOCOMOTION_STOP.value

    def speed_bucket(norm: float):
        """Map normalised magnitude [0,1] → (25/50/75/100) offset 0-3."""
        mag = abs(norm)
        if   mag < 0.30: return 0   # 25 %
        elif mag < 0.55: return 1   # 50 %
        elif mag < 0.80: return 2   # 75 %
        else:            return 3   # 100 %

    # Rotation dominates when |angular| > |linear|
    if not ang_dead and (ang_dead is False and abs(ang_norm) >= abs(lin_norm)):
        bucket = speed_bucket(ang_norm)
        if angular_z > 0:                          # CCW → turn left
            return LocomotionHex.LOCOMOTION_LEFT_25.value  + bucket
        else:                                      # CW  → turn right
            return LocomotionHex.LOCOMOTION_RIGHT_25.value + bucket

    # Linear motion
    bucket = speed_bucket(lin_norm)
    if linear_x > 0:
        return LocomotionHex.LOCOMOTION_FORWARD_25.value  + bucket
    else:
        return LocomotionHex.LOCOMOTION_BACKWARD_25.value + bucket


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        i2c_init()

        # How many identical commands to skip before re-publishing (reduces chatter)
        self._last_loco_cmd = None
        self._last_system_cmd = None

        self.create_subscription(
            Twist,
            '/robot/cmd_vel',          # Twistmux Output Topic
            self._loco_cb,
            10,
        )
        self.get_logger().info("CmdVelBridge ready — listening on /robot/cmd_vel")

        self.create_subscription(
            Byte,
            '/robotCmds/manual',      # main_manual_control.py (GCS) Topic
            self._excav_depo_cb,
            10,
        )

        self.get_logger().info("CmdVelBridge ready — listening on /robotCmds/manual")

    def _loco_cb(self, msg: Twist):
        hex_cmd = velocity_to_hex(msg.linear.x, msg.angular.z)

        # Only publish on change (avoids hammering the MCU)
        if hex_cmd == self._last_loco_cmd:
            return
        self._last_loco_cmd = hex_cmd

        send_HEX(hex_cmd)

        self.get_logger().debug(
            f"cmd_vel lin={msg.linear.x:.3f} ang={msg.angular.z:.3f} "
            f"→ 0x{hex_cmd:02X}"
        )

    def _excav_depo_cb(self, msg: Byte):
        hex_cmd = msg.data
        if hex_cmd == self._last_system_cmd:
            return
        self._last_system_cmd = hex_cmd

        send_HEX(hex_cmd)
        hex_val = int.from_bytes(hex_cmd, 'little')
        self.get_logger().debug(f"system_cmd → 0x{hex_val:02X}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
