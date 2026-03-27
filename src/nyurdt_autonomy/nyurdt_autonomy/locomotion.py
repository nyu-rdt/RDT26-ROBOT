# TEST FILE FOR GAZEBO STATE MACHINE
import rclpy
from rclpy.node import Node
import sys
import time
from time import sleep
import math

from std_msgs.msg import Byte, String
from geometry_msgs.msg import Twist

from nyurdt_autonomy.nyurdt_state_machine import StateMachine



class TractionLogic:
    def __init__(self):
        self.w1 = 0.4
        self.w2 = 0.3
        self.w3 = 0.3

        self.nominal_current = 5
        self.wheel_radius = 0.1

        self.last_left_tick = 0
        self.last_right_tick = 0
        self.last_time = time.time()

    def calculate_slip_score(self, msg, cmd_vel_x):
        now = time.time()
        dt = now -self
        if dt <= 0: return 0.0, 0x00

        # 1. Calculate measured speed from encoders
        d_left = (msg.left_encoder - self.last_left_tick) * self.wheel_radius /dt
        d_right = (msg.right_encoder - self.last_right_tick) * self.wheel_radius /dt

        # Distance 
        v_left = (d_left / 255) * (2 * math.pi * self.wheel_radius) / dt
        v_right = (d_right / 255) * (2 * math.pi * self.wheel_radius) / dt
        v_measured = (v_left + v_right) / 2

        # 2. Compoments for slip score
        speed_error = abs(cmd_vel_x - v_measured)
        speed_asymmetry = abs(v_left - v_right)

        # Current Ratio 
        avg_drive_current = (msg.motor_currents[0] + msg.motor_currents[1]) / 2
        current_ratio = avg_drive_current / self.nominal_current

        # 3. Calculate slip score
        slip_score = (self.w1 * speed_error) + (self.w2 * speed_asymmetry) + (self.w3 * current_ratio)

        # 4. Send command to EE
        mcu_fix_cmd = 0x00 # no fix needed

        if v_measured > 0.1 and avg_drive_current > 1.0:
            mcu_fix_cmd = 0x00 # slip severe fix

        elif abs(v_measured) < 0.01 and avg_drive_current > 12:
            mcu_fix_cmd = 0x00 # rocking macro fix

        self.last_left_tick = msg.left_encoder
        self.last_right_tick = msg.right_encoder
        self.last_time = now

        return slip_score, mcu_fix_cmd


class LocomotionNode(Node):
    """
    Node combining Digging and Dumping stages
    """

    def __init__(self):
        super().__init__('LocomotionNode')

        # Publisher for autonomy commands
        self.send_mcu_command = self.create_publisher(Byte, 'robot/mcu_cmd', 10)
        
        self.create_subscription(String, '/robot/current_mode', self.mode_callback, 10)

        self.create_subscription(Twist, '/robot/cmd_vel', self.cmd_vel_callback, 10)

        self.create_subscription(Byte, 'robot/cmd_manual', self.manual_callback, 10)

        self.traction = TractionLogic()

        self.target_v_x = 0.0

        # Variable used to avoid repeated logging of state
        self.current_state = None


    def create_machine(self):
        """
        Create a state machine with all desired states
        """
        self.get_logger().info("Initializing Locomotion state machine")
        self.machine = StateMachine('full_dig_dump_machine')

        # Adding states
        self.machine.add_state('stage_1', self.stage_1)
        self.machine.add_state('stage_2', self.stage_2)
        self.machine.add_state('stage_3', self.stage_3)


    def publish_msg(self, data: bytes):
        """
        Method used to publish a message to a topic
        :param data: data to be published
        """
        msg = Byte()
        msg.data = data.to_bytes(1, 'little')
        self.send_mcu_command.publish(msg)
        self.get_logger().info("Publishing: %s" % str(msg.data))

    def manual_callback(self, msg):
        """
        Manual callback to stop the node if a manual control command is sent
        """
      
        sys.exit()

    def mode_callback(self, msg):
        """
        Callback to update which zone we are in
        """
        if msg.data != "NAVIGATION":
            return
        
        self.execute_machine()

    def cmd_vel_callback(self, msg):
        self.target_v_x = msg.linear.x

    def telemetry_callback(self, msg):
        score, fix_byte = self.traction.calculate_slip_score(msg, self.target_v_x)

        if score > 1:
            self.get_logger().warn(f"High slip score detected: {score:.2f}")
        
        if fix_byte != 0x00:
            cmd = Byte()
            cmd.data = fix_byte
            self.send_mcu_command.publish(cmd)
            self.get_logger().info(f"Published traction fix command: {fix_byte} to MCU")

    def execute_machine(self):
        """
        Begin execution of the machine
        """
        self.create_machine()
        self.machine.execute()

    # ------------------ DIGGING STAGES ------------------
    def stage_1(self):
        """
        Stage 1: Spin in a circle to find AprilTag
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Spin in a circle")
            self.current_state = self.stage_1

        # Send command
        self.send_mcu_command(0x00) # make a 180

        if self.spin_start_time == 0:
            self.spin_start_time = time.time()

        if time.time() - self.spin_start_time < self.spin_time:
            return 'stage_1'
        else:
            return 'stage_2'

    
    def stage_2(self):
        """
        Stage 2: Drive Forward
        """
        if self.current_state != self.stage_2:
            self.get_logger().info("Stage 2: Drive Forward")
            self.current_state = self.stage_2

        # Send command
        self.send_mcu_command(0x00) # drive forward 100

        if self.drive_start_time == 0:
            self.drive_start_time = time.time()

        if time.time() - self.drive_start_time < self.drive_time:
            sleep(0.1)
            return 'stage_2'
        else:
            sleep(1)
            return 'stage_3'

    
    def stage_3(self):
        """
        Stage 3: Kill autonomy
        """
        self.get_logger().info("Stage 3: Autonomy Killed")

        self.publish_msg(0x00)


# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    node.execute_machine()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
