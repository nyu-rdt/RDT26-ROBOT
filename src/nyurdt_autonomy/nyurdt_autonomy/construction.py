import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Byte, Bool
from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_interfaces.msg import RobotStatus


class ConstructionNode(Node):

    def __init__(self):                          
        super().__init__('construction_node')

        self.publisher = self.create_publisher(Byte, '/robot/mcu_cmd', 10)
        self.gate_target_pub = self.create_publisher(Bool, '/robot/target_gate_closed', 10)


        self.latest_telemetry = None
        self.create_subscription(RobotStatus, 'robot/telemetry', self._telemetry_cb, 10)

        self.depo_start_time = 0
        self.depo_time = 30
        self.goal_mass = 0.05      # kg remaining in bin = "empty enough"
        self.current_state = None
        self._finished = False

        self.start_yaw = None      # set when stage_1 begins

    def _set_gate_target(self, closed: bool):
        """Call this whenever you command the gate open or closed."""
        msg = Bool()
        msg.data = closed
        self.gate_target_pub.publish(msg)

    def _telemetry_cb(self, msg: RobotStatus):
        self.latest_telemetry = msg

    def is_done(self) -> bool:
        return self._finished

    def publish_msg(self, data: int):
        msg = Byte()
        msg.data = data.to_bytes(1, 'little')
        self.publisher.publish(msg)

    def create_machine(self):
        self.machine = StateMachine('construction_machine')
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
        if self.current_state != self.stage_1:
            self.get_logger().info("Construction stage 1: Turn 180")
            self.current_state = self.stage_1
        self.publish_msg(0x00)  # turn 180 command
        # replace with real IMU yaw check from your pose topic  # uh???
        return 'stage_2'

    def stage_2(self):
        # opening the hatch
        if self.current_state != self.stage_2:
            self.get_logger().info("Construction stage 2: Open hatch")
            self.current_state = self.stage_2
            # What's the point in _set_gate_target if you're just publishing
            #   the hex command anyway?
            self._set_gate_target(False)   # gate is now commanded open
        self.publish_msg(0x70)
        
        # Is there no way to check that the door is open
        #   before trying to dump?
        return 'stage_3'

    def stage_3(self):
        if self.current_state != self.stage_3:
            self.get_logger().info("Construction stage 3: Vibrating")
            self.current_state = self.stage_3
            self.depo_start_time = time.time()

        self.publish_msg(0x72)  # vibrate

        tel = self.latest_telemetry
        if tel is not None and tel.total_mass <= self.goal_mass:
            return 'stage_4'
        if time.time() - self.depo_start_time > self.depo_time:
            return 'stage_4'
        return 'stage_3'

    def stage_4(self):
        if self.current_state != self.stage_4:
            self.get_logger().info("Construction stage 4: Stop vibrating")
            self.current_state = self.stage_4
        self.publish_msg(0x73)  # stop vibrate
        return 'stage_5'

    def stage_5(self):
        # closing the hatch
        if self.current_state != self.stage_5:
            self.get_logger().info("Construction stage 5: Close hatch")
            self.current_state = self.stage_5
            # same question as above
            self._set_gate_target(True)    # gate is now commanded closed

        # isn't this hex command supposed to be 0x71?
        self.publish_msg(0x70)

        # Same question as above
        self._finished = True
        return None