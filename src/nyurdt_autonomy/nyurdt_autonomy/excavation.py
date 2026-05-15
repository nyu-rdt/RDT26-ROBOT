"""
excavation.py

File for running through the full excavation cycle autonomously.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Byte, Float32
from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_interfaces.msg import RobotStatus


class ExcavationNode(Node):

    def __init__(self):
        super().__init__('excavation_node')

        self.publisher = self.create_publisher(Byte, '/robotCmds/manual', 10)
        self.conveyor_target_pub = self.create_publisher(Float32, '/robot/target_conveyor_pos', 10)

        # Telemetry — stored as data, not the subscription object
        self.latest_telemetry = None
        self.create_subscription(RobotStatus, 'robot/telemetry', self._telemetry_cb, 10)

        self.dig_start_time = 0
        self.lower_timer = 12
        self.dig_time = 20
        self.dig_length = 0.5
        self.goal_mass = 0.4        # kg — set real value
        self.dist_traveled = 0.0   # update this from encoder odometry
        self.current_state = None
        self._finished = False
        self.dig_start = None
        self.lower_timer_tracker = None
        self.send_up = False
    def _set_conveyor_target(self, pos: float):
        """Call this whenever you command a new conveyor position."""
        msg = Float32()
        msg.data = pos
        self.conveyor_target_pub.publish(msg)

    def _telemetry_cb(self, msg: RobotStatus):
        self.latest_telemetry = msg  

    def is_done(self) -> bool:
        return self._finished

    def publish_msg(self, data: bytes):
        msg = Byte()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing 0x{int.from_bytes(data, 'little'):02x}")

    def check_ladder_health(self):
        if self.latest_telemetry is None:
            return
        current_draw = self.latest_telemetry.motor_currents[5]
        nominal = 6.0
        ratio = current_draw / nominal
        if ratio > 2.0:
            self.get_logger().error("LADDER JAM RISK - Triggering Anti-Jam")
            # self.publish_msg(0x00)  # replace with real anti-jam byte
        elif ratio < 0.4:
            self.get_logger().info("Low current - nothing being excavated")

    def create_machine(self):
        self.machine = StateMachine('excavation_machine')
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
        # start conveyor — target is fully lowered (0.0)
        if self.current_state != self.stage_1:
            self.get_logger().info("Excavation stage 1: Start conveyor belt")
            self.current_state = self.stage_1
            self._set_conveyor_target(0.0)
        
        if not self.dig_start:
            self.dig_start = self.get_clock().now()
        if self.get_clock().now() - self.dig_start >= Duration(seconds=1):
            return 'stage_2'

        self.get_logger().info("Publishing excavation start command")
        self.publish_msg(b"\x65")

        return 'stage_1'

    def stage_2(self):
        # moving to excavation position — target is fully lowered (1.0 = down)
        if self.current_state != self.stage_2:
            self.get_logger().info("Excavation stage 2: Move to excavation position")
            self.current_state = self.stage_2
            self._set_conveyor_target(.25)   # adjust real target

        self.get_logger().info("Publishing excavation position command")
        self.publish_msg(b"\x62")

        #tel = self.latest_telemetry

        #if tel is None:
        #    return 'stage_3'
        if not self.lower_timer_tracker:
            self.lower_timer_tracker = self.get_clock().now()
        if self.get_clock().now() - self.lower_timer_tracker >= Duration(seconds=self.lower_timer):
            self.get_logger().info("Excavation position reached successfully")
            self.publish_msg(b"\x60")
            return 'stage_3'
        #if tel.exc_pos_error == True:
        #    self.get_logger().warn("Excavation position error detected, attempting to correct")
        #    self.publish_msg(b"\x61")
        #    self.get_clock().sleep_for(Duration(seconds=3))
        #    self.publish_msg(b"\x62")
        
        #if tel.exc_pos_error == False:
        #    self.get_logger().info("Excavation position reached successfully")
        #    return 'stage_3'
        
        return 'stage_2'

    def stage_3(self):
        if self.current_state != self.stage_3:
            self.get_logger().info("Excavation stage 3: Digging")
            self.current_state = self.stage_3
            self.dig_start_time = self.get_clock().now()
        self.publish_msg(b"\x73") # vibration start
        self.publish_msg(b"\x65")
        self.check_ladder_health()

        tel = self.latest_telemetry
        #if tel is not None and tel.total_mass >= self.goal_mass:
        #    self.get_logger().info("Excavation goal mass reached")
        #    return 'stage_4'
        if self.get_clock().now() - self.dig_start_time >= Duration(seconds=self.dig_time):
            self.get_logger().info("Excavation time limit exceeded")
            self.dig_start_time = None
            return 'stage_4'
       # if self.dist_traveled >= self.dig_length:
        #    return 'stage_4'

        self.publish_msg(b"\x20")  # drive forward
        return 'stage_3'

    def stage_4(self):
        if self.current_state != self.stage_4:
            self.get_logger().info("Excavation stage 4: Stop conveyor")
            self.current_state = self.stage_4
        #self.publish_msg(b"\x63")  # stop belt
        return 'stage_5'
    
    def stage_5(self):
        # retracting — target is fully raised (0.0 = up)
        if self.current_state != self.stage_5:
            self.get_logger().info("Excavation stage 5: Return to locomotion position")
            self.current_state = self.stage_5
            self._set_conveyor_target(0.0)
            self.dig_start_time = self.get_clock().now()
   #     if not send_up:
        self.publish_msg(b"\x61")
        self.publish_msg(b"\x30")   
            #send_up = True

        # for testing
        if self.get_clock().now()- self.dig_start_time >= Duration(seconds=10):
            self.publish_msg(b"\x10")
            self.publish_msg(b"\x63")
            self.publish_msg(b"\x63")
            self.publish_msg(b"\x63")
            self.publish_msg(b"\x63")
            self.publish_msg(b"\x72") # vibration stop
            self._finished = True
            return None
        
#        if not send_up:
 #           self.publish_msg(b"\x61")
  #          send_up = True
        #if self.latest_telemetry is not None and self.latest_telemetry.conveyor_pos < 0.05:
        #    self._finished = True
        #    return None
        return 'stage_5'

    
def main(args=None):
    rclpy.init(args=args)
    node = ExcavationNode()
    node.execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
