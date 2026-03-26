import rclpy
from rclpy.node import Node
import sys
import time
from time import sleep

from std_msgs.msg import Byte, String

from nyurdt_autonomy.nyurdt_state_machine import StateMachine
from nyurdt_localization.pid_controller import PID
from nyurdt_localization.apriltag_detector import AprilTagDetector

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

class ConstructionNode(Node):
    """
    Node combining Digging and Dumping stages
    """

    def __init__(self):
        super().__init__('ConstructionNode')

        # Publisher for autonomy commands
        self.publisher = self.create_publisher(Byte, "/robotCmds/manual", 10)

        self.mode_sub = self.create_subscription(String, '/robot/current_mode', self.mode_callback, 10)
        
        ## Locomotion variables ##
        self.localization_timer = time.time()
        self.LOCALIZATION_TIMEOUT = 30  # Timeout for Apriltag localization

        # Use the realsense IMU to determine orientation for a 180 turn
        self.current_pose = self.get_sensor_data()
        self.start_pose = self.get_sensor_data()
  
  
        # Apriltag variables
        self.STOP_TIME = 1  # Robot stop time after losing Apriltag detection
        self.apriltag_detector = AprilTagDetector()
        
        ## Deposition variables ##
        # timestamp to keep track of deposition time
        self.depo_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.depo_time = 30

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

    def mode_callback(self, msg):
        """
        Mode callback to stop the node if the mode changes from excavation to something else
        """
        if msg.data != "DUMPING":
            return
        
        self.execute_machine()

    def execute_machine(self):
        """
        Begin execution of the machine
        """
        self.create_machine()
        self.machine.execute()

    # ------------------ CONSTRUCTION STAGES ------------------

    def stage_1(self):
        """
        Stage 1: Turn in place 180 degrees
        """
        self.get_logger().info("Stage 1: Make a 180")

        self.publish_msg(0x00) # turn 180
        
        # check IMU data from realsense camera to see if we have turned fully 
        if self.current_pose - self.start_pose > 180:
            return 'stage_2'
        
        return 'stage_1'

    def stage_2(self):
        """
        Stage 2: Open depo bin hatch
        """
        self.get_logger().info('Stage 2: Open depo bin hatch')

        self.publish_msg(0x00)  # open depo bin hatch
        
        return 'stage_3'
    
    def stage_3(self):
        """
        Stage 3: Vibrate depo motor to start dumping
        """
        self.get_logger().info('Stage 3: Vibrate depo motor')

        self.publish_msg(0x00) # vibrate depo motor

        if self.depo_start_time == 0:
            self.depo_start_time = time.time()

        # is depo bin weight less than x kg?
        if self.bin_mass < self.goal_mass:
            return 'stage_4'
        elif time.time() - self.depo_start_time > self.depo_time:
            return 'stage_4'

        return 'stage_3'
    
    def stage_4(self):
        """
        Stage 4: Stop vibrating depo motor
        """
        self.get_logger().info('Stage 4: Stop vibrating depo motor')

        self.publish_msg(0x00) # stop vibrating depo motor

        return 'stage_5'
    
    def stage_5(self):
        """
        Stage 5: Close depo bin hatch
        """
        self.get_logger().info('Stage 5: Close depo bin hatch')

        self.publish_msg(0x00) # close depo bin hatch

        return None

# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = ConstructionNode()
    node.execute_machine()
    rclpy.shutdown()

if __name__ == "__main__":
    main()