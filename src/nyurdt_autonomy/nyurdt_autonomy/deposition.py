"""
deposition.py

File for running through the full deposition cycle of autonomy.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Byte, Bool
from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_interfaces.msg import RobotStatus


class DepositionNode(Node):

    def __init__(self):                          
        super().__init__('deposition_node')

        self.publisher = self.create_publisher(Byte, '/robotCmds/manual', 10)

        self.latest_telemetry = None
        self.create_subscription(RobotStatus, 'robot/telemetry', self._telemetry_cb, 10)

        self.depo_start_time = None
        self.depo_segment = 5
        self.depo_time = 30
        self.goal_mass = 500      # kg remaining in bin = "empty enough"
        self.current_state = None
        self._finished = False

        self.start_yaw = None      # set when stage_1 begins
        self.depo_turn_time = 3


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
        self.machine.add_state('stage_6', self.stage_6)

    def execute(self):
        self.create_machine()
        self.machine.execute()

    # ── State Machines ───────────────────────────────

    def stage_1(self):
        if self.current_state != self.stage_1:
            self.get_logger().info("Deposition stage 1: Move backwards")
            self.current_state = self.stage_1

        if not self.depo_start_time:
            self.depo_start_time = self.get_clock().now()

        self.publish_msg(b"\x31") # move backwards at 50%

        if self.get_clock().now() - self.depo_start_time >= Duration(seconds=2.5):
            self.depo_start_time = None
            return 'stage_2'
        
        return 'stage_1'
    
    def stage_2(self):
        if self.current_state != self.stage_2:
            self.get_logger().info("Deposition stage 2: Stop moving backwards")
            self.current_state = self.stage_2
        
        #if not self.depo_start_time:
        #    self.depo_start_time = self.get_clock().now()

        self.publish_msg(b"\x10") # stop moving backwards

       # if self.get_clock().now() - self.depo_start_time >= Duration(seconds=2):
       #     self.depo_start_time = None
        return 'stage_3'
        
       # return 'stage_2'

    def stage_3(self):
        # opening the hatch
        if self.current_state != self.stage_3:
            self.get_logger().info("Deposition stage 3: Open hatch")
            self.current_state = self.stage_3

        # may need to insert check here that depo bin is actually open

        if not self.depo_start_time:
            self.depo_start_time=self.get_clock().now()

        if self.get_clock().now() - self.depo_start_time >= Duration(seconds=5): 
            self.depo_start_time = None
            return 'stage_4'
        
        self.publish_msg(b"\x70")

        return 'stage_3'

    def stage_4(self):
        if self.current_state != self.stage_4:
            self.get_logger().info("Deposition stage 4: Vibrating")
            self.current_state = self.stage_4
        
        if not self.depo_start_time:
            self.depo_start_time = self.get_clock().now()
        
        self.publish_msg(b"\x73") # vibrate
        elapsed_seconds = (self.get_clock().now() - self.depo_start_time).nanoseconds / 1e9
        phase = elapsed_seconds % 5.5
        if phase <= 5:
            self.publish_msg(b"\x10")
        else:
            self.publish_msg(b"\x21")  # slight forward 
        
        if self.get_clock().now()- self.depo_start_time >= Duration(seconds=self.depo_time):
            self.depo_start_time = None
            return 'stage_5'

        return 'stage_4'

    def stage_5(self):
        if self.current_state != self.stage_5:
            self.get_logger().info("Deposition stage 5: Stop vibrating")
            self.current_state = self.stage_5

        if not self.depo_start_time:
            self.depo_start_time = self.get_clock().now()
        
        
        if self.get_clock().now()- self.depo_start_time >= Duration(seconds=2):  # stop vibrate after 5 seconds of stage 3 ending
            self.depo_start_time = None
            return 'stage_6'
        
        self.publish_msg(b"\x72") # stop vibrate
        return 'stage_5'

    def stage_6(self):
        # closing the hatch
        if self.current_state != self.stage_6:
            self.get_logger().info("Deposition stage 6: Close hatch")
            self.current_state = self.stage_6
        
        if not self.depo_start_time:
            self.depo_start_time = self.get_clock().now()

        if self.get_clock().now() - self.depo_start_time >= Duration(seconds=5): 
            self._finished = True
            return None

        self.publish_msg(b"\x71")
        return 'stage_6'
    
def main(args=None):
    rclpy.init(args=args)
    node = DepositionNode()
    node.execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
