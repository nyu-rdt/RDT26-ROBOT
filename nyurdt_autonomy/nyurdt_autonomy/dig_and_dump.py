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

class DigDumpNode(Node):
    """
    Node combining Digging and Dumping stages
    """

    def __init__(self):
        super().__init__('DigDumpNode')

        # Client to receive sensor data
        # self.cli = self.create_client(SensorData, "get_sensor_data")

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
        self.dig_start_time = 0
        # NEED TO CHANGE VALUE -----------------------------------------------
        self.dig_time = 120  # Digging duration in seconds

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
        self.machine.add_state('stage_6', self.stage_6)
        self.machine.add_state('stage_7', self.stage_7)
        self.machine.add_state('stage_8', self.stage_8)

    def publish_msg(self, data: bytes):
        """
        Method used to publish a message to a topic
        :param data: data to be published
        """
        msg = Byte()
        msg.data = data.to_bytes(1, 'little')
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

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
        Stage 1: Move acme system to excavation position
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Move to excavation position")
            self.current_state = self.stage_1

        # Send command
        self.publish_msg(0x62)  # Move conveyor to excavation position
        
        sleep(5)

        # THIS SEEMS IMPORTANT WE MAY NEED TO IMPLEMENT THIS
        # # Check if pivot is at excavation position, repeat state if not
        # sensor_data = self.get_sensor_data()
        # if not sensor_data.at_excavation_position:    
        #     return "stage_1"

        return 'stage_2'

    def stage_2(self):
        """
        Stage 2: Start spinning the conveyor belt to dig
        """
        if self.current_state != self.stage_2:
            self.get_logger().info("Stage 2: Start spinning the conveyor belt")
            self.current_state = self.stage_2
        
        # changed to manual control equiv
        self.publish_msg(0x65)  # Belt inward (digging)

        if self.dig_start_time == 0:
            self.dig_start_time = time.time()

        if time.time() - self.dig_start_time > self.dig_time:
            return 'stage_3'
        else:
            sleep(1)
            return 'stage_2'

    def stage_3(self):
        """
        Stage 3: Stop the motion of the conveyor belt
        """
        if self.current_state != self.stage_3:
            self.get_logger().info("Stage 3: Stop spinning the conveyor belt")
            self.current_state = self.stage_2

        # Stop the belt
        self.publish_msg(0x63)

        return 'stage_4'

    def stage_4(self):
        """
        Stage 4: Move pivot system to locomotion position
        """
        if self.current_state != self.stage_4:
            self.get_logger().info("Stage 4: Move to locomotion position")
            self.current_state = self.stage_4

        # Move conveyor belt to locomotion position
        self.publish_msg(0x61)
        sleep(7)

        # Check if pivot at starting position
        # sensor_data = self.get_sensor_data()
        # if not sensor_data.at_locomotion_position:
        #     return 'stage_4'

        return 'stage_5'

    # ------------------ CONSTRUCTION STAGES ------------------

    def stage_5(self):
        """
        Stage 5: Turn towards Apriltag
        """
        self.get_logger().info("Stage 5: Turn towards Apriltag")

        # Kill autonomy after timeout triggers
        if time.time() - self.localization_timer >= self.LOCALIZATION_TIMEOUT:
            sys.exit()
        elif time.time() - self.localization_timer > self.STOP_TIME:
            self.publish_msg(0x10)  # Locomotion stop

        # Get an image capture
        ret, frame = self.apriltag_detector.get_frame()
        if not ret:
            return 'stage_5'

        # Check for apriltag detections
        detections = self.apriltag_detector.get_detections(frame)
        self.apriltag_detector.draw_results(detections, frame)
        ret, _, rotation = self.apriltag_detector.get_pose(detections)
        if not ret:
            return 'stage_5'

        # Reset timer
        self.localization_timer = time.time()

        # Get current yaw
        angle = rotation[1]

        # Target angle
        TARGET_ANGLE = 0
        ANGLE_THRESHOLD = 5

        # Check if we are facing the target
        error = TARGET_ANGLE - angle
        if abs(error) <= ANGLE_THRESHOLD:
            self.publish_msg(0x10)
            return 'stage_6'

        # Get needed motor powers
        output = self.turn_controller.update(error)
        motor_power = int(abs(output))
        motor_power = self.clamp(motor_power, self.MIN_POWER, self.MAX_POWER)

        # Check if we need to turn left or right
        if output > 0:
            command = 0x40  # Turn left
        else:
            command = 0x50  # Turn right
        
        self.publish_msg(command)
        sleep(0.1)
        return 'stage_5'

    def stage_6(self):
        """
        Stage 6: Move robot towards Apriltag
        """
        self.get_logger().info("Stage 6: Move robot towards apriltag")

        if time.time() - self.localization_timer >= self.LOCALIZATION_TIMEOUT:
            sys.exit()
        elif time.time() - self.localization_timer > 5:
            self.publish_msg(0x10)  # Locomotion stop
        
        # Get an image capture
        ret, frame = self.apriltag_detector.get_frame()
        if not ret:
            return 'stage_6'

        # Check for apriltag detections
        detections = self.apriltag_detector.get_detections(frame)
        self.apriltag_detector.draw_results(detections, frame)
        ret, translation, _ = self.apriltag_detector.get_pose(detections)
        if not ret:
            return 'stage_6'

        # Reset timer
        self.localization_timer = time.time()

        # Get current distance from tags
        distance = translation[-1]
        TARGET_DISTANCE = 70

        # Check if we are close enough to the tags
        if distance <= TARGET_DISTANCE:
            self.publish_msg(0x10)
            return 'stage_7'

        # Calculate needed motor power
        error = TARGET_DISTANCE - distance
        output = self.move_controller.update(error)
        motor_power = int(abs(output))
        motor_power = self.clamp(motor_power, self.MIN_POWER, self.MAX_POWER)

        # Move the robot forward
        self.publish_msg(0x20)
        return 'stage_6'

    def stage_7(self):
        """
        Stage 7: Reverse Conveyor Belt
        """
        self.get_logger().info('Stage 7: Reverse Conveyor Belt')

        # Reverse conveyor to dump material
        self.publish_msg(0x62)  # Belt outward
        sleep(10)
        
        return 'stage_8'

    def stage_8(self):
        """
        Stage 8: Stop Conveyor Belt
        """
        self.get_logger().info('Stage 8: Stop Conveyor Belt')

        # Stop the belt
        self.publish_msg(0x63)
        return None

# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = DigDumpNode()
    node.execute_machine()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
