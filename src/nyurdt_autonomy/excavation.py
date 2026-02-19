import rclpy
from rclpy.node import Node
import sys
import time
from time import sleep

from std_msgs.msg import Byte

from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_localization.pid_controller import PID
from nyurdt_localization.apriltag_detector import AprilTagDetector

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

class ExcavationNode(Node):
    """
    Node combining Digging and Dumping stages
    """

    def __init__(self):
        super().__init__('ExcavationNode')

        # Client to receive sensor data
        self.cli = self.create_client(SensorData, "get_sensor_data")

        # Making sure service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.req = SensorData.Request()

        # Publisher for autonomy commands
        self.publisher = self.create_publisher(Byte, "/robotCmds/manual", 10)

        # Manual control interrupt subscribers
        # self.manual_subscription = self.create_subscription(Byte,
        #                                                     "/robotCmds/manual",
        #                                                     self.manual_callback,
        #                                                     10)

        ## Excavation variables ##
        self.dig_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.dig_time = 120  # Digging duration in seconds

        # Using encoders we will get the distance travelled
        self.dig_length = 0.5 # meters
        self.dist_traveled = self.get_sensor_data()

        # Using 4 load cells we can know how much regolith has been collected
        self.bin_mass = self.get_sensor_data()
        self.goal_mass = 0.14 #kg

        ## Locomotion variables ##
        self.localization_timer = time.time()
        self.LOCALIZATION_TIMEOUT = 30  # Timeout for Apriltag localization
        
        # PID controllers
        self.turn_controller = PID(P=1, I=0, D=0.1)
        self.move_controller = PID(P=0.4, I=0, D=0)

        # Locomotion speed limits
        self.MAX_POWER = 50
        self.MIN_POWER = 20

        # Apriltag variables
        self.STOP_TIME = 1  # Robot stop time after losing Apriltag detection
        self.apriltag_detector = AprilTagDetector()

        # Variable used to avoid repeated logging of state
        self.current_state = None

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

    def execute_machine(self):
        """
        Begin execution of the machine
        """
        self.create_machine()
        self.machine.execute()

    # ------------------ DIGGING STAGES ------------------

    def stage_1(self):
        """
        Stage 1: Start rotating conveyor belt
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Start rotating conveyor belt")
            self.current_state = self.stage_1

        self.publish_msg(0x00) # rotate conveyor belt

        sleep(5)

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
        
        sleep(5)


        # Check if pivot is at excavation position, repeat state if not
        sensor_data = self.get_sensor_data()
        if not sensor_data.at_excavation_position:    
            return "stage_2"

        return 'stage_3'

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
        elif self.bin_mass > self.goal_mass:
            return 'stage_4'
        # have we collected less than x kg of regolith?
        elif self.bin_mass < self.goal_mass:
            self.publish_msg(0x00) #move forward 1 meters
            return 'stage_3'
        
        sleep(1)
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
