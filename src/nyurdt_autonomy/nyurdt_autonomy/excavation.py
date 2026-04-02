import rclpy
from rclpy.node import Node
import sys
import time
from time import sleep
from std_msgs.msg import Byte, String


from nyurdt_autonomy.nyurdt_state_machine import StateMachine

from nyurdt_interfaces.msg import RobotStatus


class ExcavationNode(Node):

    def __init__(self):
        super().__init__('ExcavationNode')

        # Publisher for autonomy commands
        self.publisher = self.create_publisher(Byte, "/robotCmds/manual", 10)


        # Publisher for autonomy commands
        self.publisher = self.create_publisher(Byte, '/robot/mcu_cmd', 10)

        self.is_at_excavation_position = False

        self.telemetry = self.create_subscription(RobotStatus, 'robot/telemtry', self.telemetry_callback, 10)
        
        self.mode_sub = self.create_subscription(String, '/robot/current_mode', self.mode_callback, 10)

        ## Excavation variables ##
        self.dig_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.dig_time = 120  # Digging duration in seconds

        # Using encoders we will get the distance travelled
        self.dig_length = 0.5 # meters

        # Variable used to avoid repeated logging of state
        self.current_state = None

        self.mass_flow = 0

    def create_machine(self):
        """
        Create a state machine with all desired states
        """
        self.get_logger().info("Initializing state machine")
        self.machine = StateMachine('full_dig_dump_machine')

        # Adding states
        self.machine.add_state('stage_1', self.stage_1)
        self.machine.add_state('stage_2', self.stage_2)
        self.machine.add_state('stage_3', self.stage_3)
        self.machine.add_state('stage_4', self.stage_4)
        self.machine.add_state('stage_5', self.stage_5)
        self.machine.add_state('stage_6', self.stage_6)


    def publish_msg(self, data: bytes):
        """
        Method used to publish a message to a topic
        :param data: data to be published
        """
        msg = Byte()
        msg.data = data.to_bytes(1, 'little')
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

    def manual_callback(self, msg):
        """
        Manual callback to stop the node if a manual control command is sent
        """
        sys.exit()

    def position_callback(self, msg):
        """
        Callback to update whether we are at the excavation position or not
        """
        self.is_at_excavation_position = msg.data

    def mode_callback(self, msg):
        """
        Callback to update the current mode of the robot
        """
        self.current_mode = msg.data

    def excavation_loop(self):
        if self.current_mode != "DIGGING":
            return
        
        self.mass_flow = (self.telemetry.total_mass - self.prev_mass) / self.dt
        
        self.execute_machine()

    def execute_machine(self):
        """
        Begin execution of the machine
        """
        self.create_machine()
        self.machine.execute()

    def check_ladder_health(self, telemetry):
        current_draw = telemetry.motor_currents[5]
        nominal_current_draw = 6

        proxy_ratio = current_draw /nominal_current_draw

        if proxy_ratio > 2.0 and self.mass_flow < 0.01:
            self.get_logger().error("LADDER JAM RISK - Triggering Anti-Jam")
            self.send_mcu_cmd(0x00)  # Anti-jam command

        elif proxy_ratio < 0.4 and self.current_mode == "DIGGING":
            self.get_logger.info("Low current - Nothing being excavated")

    # ------------------ DIGGING STAGES ------------------

    def stage_1(self):
        """
        Stage 1: Start rotating conveyor belt
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Start rotating conveyor belt")
            self.current_state = self.stage_1

        self.publish_msg(0x00) # rotate conveyor belt

        return 'stage_2'

    def stage_2(self):
        """
        Stage 2: Move conveyor system to excavation position
        """
        if self.current_state != self.stage_2:
            self.get_logger().info("Stage 2: Move to excavation position")
            self.current_state = self.stage_2

        # Send command
        self.publish_msg(0x00)  # Move conveyor to excavation position

        # Check if pivot is at excavation position, repeat state if not
        if self.is_at_excavation_position:
            self.get_logger().info("Reached excavation position")
            return 'stage_3'
        else:
            return 'stage_2'
        
    def stage_3(self):
        """
        Stage 3: Dig
        """
        if self.current_state != self.stage_3:
            self.get_logger().info("Stage 3: Dig")
            self.current_state = self.stage_3

        if self.dig_start_time == 0:
            self.dig_start_time = time.time()

        # have we digged for x seconds?
        if time.time() - self.dig_start_time > self.dig_time:
            return 'stage_4'
        # have we moved forward more than x meters?
        elif self.dist_traveled > self.dig_length:
            return 'stage_4'
        # have we collected more than x kg of regolith?
        elif self.telemetry.bin_mass > self.goal_mass:
            return 'stage_4'
        # have we collected less than x kg of regolith?
        elif self.telemetry.bin_mass < self.goal_mass:
            self.publish_msg(0x00) #move forward 1 meters
            return 'stage_3'
        elif self.telemtry.motor_stall_flag:
            return 'stage_error_recovery'
        
        return 'stage_3'

    def stage_4(self):
        """
        Stage 4: Stop the motion of the conveyor belt
        """
        if self.current_state != self.stage_4:
            self.get_logger().info("Stage 4: Stop spinning the conveyor belt")
            self.current_state = self.stage_4

        # Stop the belt
        self.publish_msg(0x00)

        return 'stage_5'

    def stage_5(self):
        """
        Stage 5: Move conveyor belt to locomotion position
        """
        if self.current_state != self.stage_5:
            self.get_logger().info("Stage 5: Move to locomotion position")
            self.current_state = self.stage_5

        # Move conveyor belt to locomotion position
        self.publish_msg(0x00)
        sleep(7)

        # Check if pivot at starting position
        sensor_data = self.get_sensor_data()
        if not sensor_data.at_locomotion_position:
            return 'stage_5'

        return 'stage_6'

# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = ExcavationNode()
    node.execute_machine()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
