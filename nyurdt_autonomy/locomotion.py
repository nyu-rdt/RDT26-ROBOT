# TEST FILE FOR GAZEBO STATE MACHINE
import rclpy
from rclpy.node import Node
import sys
import time
from time import sleep

from std_msgs.msg import Byte

from nyurdt_autonomy.nyurdt_state_machine import StateMachine

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

class LocomotionNode(Node):
    """
    Node combining Digging and Dumping stages
    """

    def __init__(self):
        super().__init__('LocomotionNode')

        # Client to receive sensor data
        self.cli = self.create_client(SensorData, "get_sensor_data")

        # Making sure service is available
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Service not available, waiting again...")
        # self.req = SensorData.Request()

        # Publisher for autonomy commands
        self.publisher = self.create_publisher(Byte, "/robotCmds/manual", 10)

        # Manual control interrupt subscribers
        # self.manual_subscription = self.create_subscription(Byte,
        #                                                     "/robotCmds/manual",
        #                                                     self.manual_callback,
        #                                                     10)

        ## Excavation variables ##
        self.spin_start_time = 0
        self.drive_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.spin_time = 5  # Digging duration in seconds
        self.drive_time = 5

        ## Locomotion variables ##
        self.localization_timer = time.time()
        self.LOCALIZATION_TIMEOUT = 30  # Timeout for Apriltag localization
        
        # PID controllers
        # self.turn_controller = PID(P=1, I=0, D=0.1)
        # self.move_controller = PID(P=0.4, I=0, D=0)

        # Locomotion speed limits
        self.MAX_POWER = 50
        self.MIN_POWER = 20

        # Apriltag variables
        self.STOP_TIME = 1  # Robot stop time after losing Apriltag detection
        # self.apriltag_detector = AprilTagDetector()
        
        ## Deposition variables ##
        # timestamp to keep track of deposition time
        self.depo_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.depo_time = 30

        # Variable used to avoid repeated logging of state
        self.current_state = None

        # Identify which zone we are currently in
        self.current_zone = None

        # Set the goal destination
        self.goal_dest = None

        if self.current_zone == "construction":
            self.goal_dest = [-3, 5, 0] # CHANGE THESE VALUES TO MATCH ARENA
        else: # excavation
            self.goal_dest = [0, 1, 0] # CHANGE THESE VALUES TO MATCH ARENA

            

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
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: %s" % str(msg.data))

    def clamp(self, value, low, high):
        """
        Clamp a value between low and high
        """
        return min(max(value, low), high)

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
        Stage 1: Spin in a circle for 5 sec to find apriltag to localize robot
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Localize self")
            self.current_state = self.stage_1

        # Send command
        self.publish_msg(0x53)  # REPLACE with hex command that allows robot to turn in place

        if self.spin_start_time == 0:
            self.spin_start_time = time.time()

        #apriltag detection 
        #if found stage 2 
        # if not found 

        if time.time() - self.spin_start_time < self.spin_time:
            sleep(0.1)
            return 'stage_1'
        else:
            sleep(1)
            return 'stage_2'

    
    def stage_2(self):
        """
        Stage 2: Drive Forward
        """
        if self.current_state != self.stage_2:
            self.get_logger().info("Stage 2: Drive Forward")
            self.current_state = self.stage_2

        # Send command
        self.publish_msg(0x23)  # drive forward 100

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

        self.publish_msg(0x01)


# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    node.execute_machine()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
