"""
deposition.py

File for running through the full deposition cycle of autonomy.
"""

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Byte, Bool
from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_interfaces.msg import RobotStatus


class DepositionNode(Node):

    def __init__(self):
        super().__init__('deposition_node')

        self.publisher = self.create_publisher(Byte, '/robotCmds', 10)

        self.latest_telemetry = None
        self.create_subscription(RobotStatus, 'robot/telemetry', self._telemetry_cb, 10)

        
        # self.dist_traveled = 0.0   # update this from encoder odometry
        self.depo_start_time = None
        self.depo_time = 30 # how long should vibration motor vibrate
        self.current_state = None
        self._finished = False


    def _telemetry_cb(self, msg: RobotStatus):
        self.latest_telemetry = msg

    def is_done(self) -> bool:
        return self._finished

    def publish_msg(self, data: bytes):
        msg = Byte()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing 0x{int.from_bytes(data, 'little'):02x}")

    def create_machine(self):
        self.machine = StateMachine('deposition_machine')
        self.machine.add_state('stage_1', self.stage_1)
        self.machine.add_state('stage_2', self.stage_2)
        self.machine.add_state('stage_3', self.stage_3)
        self.machine.add_state('stage_4', self.stage_4)
        self.machine.add_state('stage_5', self.stage_5)

    def execute(self):
        self.create_machine()
        self.machine.execute()

    # ── State Machines ───────────────────────────────

    def stage_1(self):
        # robot already turned 180° outside zone; reverse (back end first) into zone
        if self.current_state != self.stage_1:
            self.get_logger().info("Deposition stage 1: Enter zone")
            self.current_state = self.stage_1

        if not self.depo_start_time:
            self.depo_start_time = time.time()

        self.publish_msg(b"\x30")  # backward at 25%

        if time.time() - self.depo_start_time >= 4: #move backwards for however many seconds (adjust if needed)
            self.publish_msg(b"\x10")  # stop
            self.depo_start_time = None
            return 'stage_2'

        return 'stage_1'

    def stage_2(self):
        # opening the hatch
        if self.current_state != self.stage_2:
            self.get_logger().info("Deposition stage 2: Open hatch")
            self.current_state = self.stage_2

        # may need to insert check here that depo bin is actually open

        if not self.depo_start_time:
            self.depo_start_time = time.time()

        if time.time() - self.depo_start_time >= 5:
           self.depo_start_time = None
           return 'stage_3'

        self.publish_msg(b"\x71")

        return 'stage_2'

    def stage_3(self):
        if self.current_state != self.stage_3:
            self.get_logger().info("Deposition stage 3: Vibrating")
            self.current_state = self.stage_3

        if not self.depo_start_time:
            self.depo_start_time = time.time()

        # alternate vibrate while robot moves slightly backward each cycle to help material fall (requested by meche)
        phase = (time.time() - self.depo_start_time) % 3.5
        if phase < 3.0:
            self.publish_msg(b"\x72")  # vibrate
        else:
            self.publish_msg(b"\x20")  # slight forward 

        if time.time() - self.depo_start_time > self.depo_time:
            self.depo_start_time = None
            return 'stage_4'

        return 'stage_3'

    def stage_4(self):
        if self.current_state != self.stage_4:
            self.get_logger().info("Deposition stage 4: Stop vibrating")
            self.current_state = self.stage_4

        if not self.depo_start_time:
           self.depo_start_time = time.time()

        if time.time() - self.depo_start_time >= 2:  # stop vibrate after 5 seconds of stage 3 ending
           self.depo_start_time = None
           return 'stage_5'

        self.publish_msg(b"\x73")  # stop vibrate
        return 'stage_4'

    def stage_5(self):
        # closing the hatch
        if self.current_state != self.stage_5:
            self.get_logger().info("Deposition stage 5: Close hatch")
            self.current_state = self.stage_5

        if not self.depo_start_time:
           self.depo_start_time = time.time()

        if time.time() - self.depo_start_time >= 5:
           return None

        self.publish_msg(b"\x70")
        self._finished = True
        return 'stage_5'


def main(args=None):
    rclpy.init(args=args)
    node = DepositionNode()
    node.execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
