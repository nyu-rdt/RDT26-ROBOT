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

        self.depo_start_time = None
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
            self.get_logger().info("Deposition stage 1: Turn 180")
            self.current_state = self.stage_1

        if not self.start_yaw:
            self.start_yaw = time.time()  # placeholder for actual yaw reading

        self.publish_msg(b"\x50") # turn right at 25%
      
        # for now this is a timer but this should replaced with 
        # real IMU yaw check from  realsense camera
        if time.time() - self.start_yaw >= 3:
            return 'stage_6'
        
        return 'stage_1'

    def stage_2(self):
        # opening the hatch
        if self.current_state != self.stage_2:
            self.get_logger().info("Deposition stage 2: Open hatch")
            self.current_state = self.stage_2

        # may need to insert check here that depo bin is actually open

        if not self.depo_start_time:
            self.depo_start_time=time.time()

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
        
        self.publish_msg(b"\x72")  # vibrate

        tel = self.latest_telemetry
        self.get_logger().info(f"TELEMETRY DATA:{tel}")
        # if tel is not None and tel.total_mass <= self.goal_mass:
        #     self.get_logger().info(f"Total mass: {tel.total_mass}")
        #     self.depo_start_time = None
        #     return 'stage_4'
        if time.time() - self.depo_start_time > 5:
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
        
        self.publish_msg(b"\x73") # stop vibrate
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
    
    def stage_6(self):
        if self.current_state != self.stage_6:
            self.get_logger().info("Deposition stage 6: Finished")
            self.current_state = self.stage_6
        
        if not self.depo_start_time:
            self.depo_start_time = time.time()

        if time.time() - self.depo_start_time >= 3:
            self.depo_start_time = None
            return 'stage_2'
        
        self.publish_msg(b"\x10")
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
