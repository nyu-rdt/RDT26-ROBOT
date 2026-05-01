#!/usr/bin/env python3

"""
autonomy_manager.py

File for calling respective autonomies at the correct time. 
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Byte

from .excavation import ExcavationNode
from .deposition import DepositionNode

import math

from .nyurdt_state_machine import StateMachine




# KSC
# ZONES = {
#     "excavation":   {"x": (-4.5720, 9.14 - 4.5720), "y": (0.00, 4.00)},
#     "deposition": {"x": (4.5720 - 2.00, 4.5720), "y": (5.50, 8.10)},
#     "obstacle":     {"x": (-4.5720, 4.5720 - 2.00), "y": (4.00, 8.10)},
#     "starting":     {"x": (-2.00, 2.00), "y": (0.00, 2.00)},
# }

# ORIGIN IS AT THE CENTER OF THE FIDUCIAL RAIL
# UCF
ZONES = {
    "excavation":   {"x": (0.00, 4.00), "y": (-3.570, 1.570)},
    "deposition": {"x": (6.10, 8.10), "y": (-3.570, -1.570)},
    "obstacle":     {"x": (4.10, 8.10), "y": (-3.750, 1.570)},
    "starting":     {"x": (0.00, 2.00), "y": (-1.00, 1.00)},
}

# pose where robot can see the Apriltag
RELOCALIZE_POSE = {"x": 0.0, "y": 0.0, "yaw": 0.0}

# covariance thresholds
COV_X_THRESH = 0.05
COV_Y_THRESH = 0.05
COV_SETTLE_SEC = 2.0

ZONE_ENTRY_MARGIN = 0.12

# queue for 3 cycles of autonomy
TASK_QUEUE = [
    {"zone": "excavation", "action": "excavate"},
    {"zone": "deposition", "action": "deposit"},
    {"zone": "excavation", "action": "excavate"},
    {"zone": "deposition", "action": "deposit"},
    {"zone": "excavation", "action": "excavate"},
    {"zone": "deposition", "action": "deposit"},
]

class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')

        # State
        self.current_state = self.stage_1
        self.task_index = 0
        self.current_zone = None
        self.pose = None
        self.covariance = [999.0] * 36
        self.nav_goal_handle = None
        self.nav_in_flight = False
        self.relocalize_after = None
        self.cov_ok_since = None
        self.bin_mass = None
        self.goal_bin_mass = 0.4 # REPLACE WITH GOAL MASS 
        self.last_apriltag_correction = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/apriltag/robot_pose',
            self._apriltag_cb,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "rtabmap/localization_pose",
            self._pose_cb,
            10,
        )

        # self.create_subscription(Byte, 'robot/cmd_manual', self.manual_callback, 10)

        self.task_pub = self.create_publisher(String, "/autonomy/current_task", 10)

        self.status_pub = self.create_publisher(String, "/autonomy/status", 10)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.publisher = self.create_publisher(Byte, '/robotCmds', 10)

        self.get_logger().info("AutonomyManager started - state: INIT_LOCALIZING")


    def manual_callback(self, msg):
        """
        Manual callback to stop the node if a manual control command is sent
        """

        sys.exit()

    def create_machine(self):
        """
        Create a state machine with all desired states
        """
        self.get_logger().info("Initializing state machine")
        self.machine = StateMachine('autonomy_manager_machine')

        # Adding states
        self.machine.add_state('stage_1', self.stage_1)
        self.machine.add_state('stage_2', self.stage_2)
        self.machine.add_state('stage_3', self.stage_3)
        self.machine.add_state('stage_4', self.stage_4)
        self.machine.add_state('stage_5', self.stage_5)
    
    def execute_machine(self):
        """
        Begin execution of the machine
        """
        self.create_machine()
        self.machine.execute()

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose.pose
        self.covariance = msg.pose.covariance

    def _apriltag_cb(self, msg):
        self.last_apriltag_correction = self.get_clock().now()

    # Nav2 helpers
 
    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available.")
            return
 
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
 
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.nav_in_flight = True
        self._nav_result_ready = False
 
    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            self.nav_in_flight = False
            return
        self.nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)
 
    def _nav_result_cb(self, future):
        self._nav_result = future.result()
        self._nav_result_ready = True
 
    def _nav_goal_succeeded(self) -> bool:
        return (
            getattr(self, "_nav_result_ready", False)
            and self._nav_result.status == GoalStatus.STATUS_SUCCEEDED
        )
 
    def _cancel_nav_goal(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        self.nav_in_flight = False
 
    # Localization helpers
 
    def _covariance_ok(self) -> bool:
        """True when x and y positional uncertainty are below thresholds."""
        return (
            self.covariance[0] < COV_X_THRESH
            and self.covariance[7] < COV_Y_THRESH
        )
 
    def _inside_zone(self, zone_name: str, margin: float = 0.0) -> bool:
        """True when current pose is inside the named zone (with inward margin)."""
        if self.pose is None:
            return False
        z = ZONES.get(zone_name)
        if z is None:
            return False
        x = self.pose.position.x
        y = self.pose.position.y
        return (
            z["x"][0] + margin <= x <= z["x"][1] - margin
            and z["y"][0] + margin <= y <= z["y"][1] - margin
        )
 
    def _current_zone_name(self) -> str | None:
        """Return the name of whichever zone the robot is currently inside."""
        for name in ZONES:
            if self._inside_zone(name):
                return name
        return None
 
 
    def _publish_status(self):
        task_name = (
            TASK_QUEUE[self.task_index]["zone"]
            if self.task_index < len(TASK_QUEUE)
            else "none"
        )
        msg = String(data=f"state={self.current_state.__name__} task={task_name} "
                         f"cov_x={self.covariance[0]:.4f} cov_y={self.covariance[7]:.4f}")
        self.status_pub.publish(msg)

    def publish_msg(self, data: int):
        msg = Byte()
        msg.data = data.to_bytes(1, 'little')
        self.publisher.publish(msg)

    
    # ── State Machines ───────────────────────────────


    def stage_1(self):
        """
        Stage 1: Initial Localization
        """
        if self.current_state != self.stage_1:
            self.get_logger().info("Stage 1: Initial Localization")
            self.current_state = self.stage_1

        # turn until an apriltag is spotted
        self.publish_msg(0x50) # turn right at 25%
        
        if self._covariance_ok():
            self.get_logger().info("Initial Localization aquired")
            return 'stage_2'
        
        return 'stage_1'

    def stage_2(self):
        """
        Stage 2: Send a Nav2 goal for the current task's zone, monitor for drift.
        """
        if self.current_state != self.stage_2:
            self.get_logger().info("Stage 2: Navigating")
            self.current_state = self.stage_2

        if self.task_index >= len(TASK_QUEUE):
            return 'stage_6' # finished with autonomy
 
        # Check for drift mid-navigation
        if not self._covariance_ok():
            self.get_logger().warn("Drift detected while navigating — relocalizing.")
            self._cancel_nav_goal()
            self.relocalize_after = 'stage_2'
            return 'stage_3'
 
        # Send goal if not already in flight
        if not self.nav_in_flight:
            task = TASK_QUEUE[self.task_index]
            zone = ZONES[task["zone"]]
            target_x = (zone["x"][0] + zone["x"][1]) / 2.0
            target_y = (zone["y"][0] + zone["y"][1]) / 2.0
            self._send_nav_goal(target_x, target_y)
            return 'stage_2'
 
        # Check if we've arrived (Nav2 goal succeeded)
        if self._nav_goal_succeeded():
            self.get_logger().info(f"Arrived near {TASK_QUEUE[self.task_index]['zone']}.")
            self.nav_in_flight = False
            return 'stage_4'
        
    def stage_3(self):  
        if self.current_state != self.stage_3:
            self.get_logger().info("Stage 3: Relocalizing")
            self.current_state = self.stage_3

        if not self.nav_in_flight:
            p = RELOCALIZE_POSE
            self._send_nav_goal(p["x"], p["y"], yaw=p["yaw"])
            return 'stage_3'

        if self._nav_goal_succeeded():
            self.nav_in_flight = False

        # Wait for BOTH covariance to drop AND a fresh tag correction
        tag_seen_recently = (
            self.last_apriltag_correction is not None
            and (self.get_clock().now() - self.last_apriltag_correction).nanoseconds / 1e9 < 2.0
        )

        if self._covariance_ok() and tag_seen_recently:
            self.get_logger().info("Relocalization confirmed by AprilTag.")
            self.last_apriltag_correction = None
            return self.relocalize_after or 'stage_2'

        return 'stage_3'
    
    def stage_4(self):
        """
        Stage 4: Confirm we're inside the correct zone with enough confidence before acting.
        """
        if self.current_state != self.stage_4:
            self.get_logger().info('Stage 4: In Zone')
            self.current_state = self.stage_4

        task = TASK_QUEUE[self.task_index]
        zone_name = task["zone"]
 
        # Check covariance first — if bad, relocalize then come back
        if not self._covariance_ok():
            self.get_logger().warn("Covariance too high in-zone — relocalizing before acting.")
            self.relocalize_after = 'stage_4'
            return 'stage_3'
 
        # Track how long covariance has been OK (require a short hold)
        now = self.get_clock().now()
        if self._cov_ok_since is None:
            self._cov_ok_since = now
            return 'stage_4'
        elapsed = (now - self._cov_ok_since).nanoseconds / 1e9
        if elapsed < COV_SETTLE_SEC:
            return  'stage_4' # still settling
        self._cov_ok_since = None
 
        # Confirm physical zone membership (with margin)
        if not self._inside_zone(zone_name, margin=ZONE_ENTRY_MARGIN):
            self.get_logger().warn(
                f"Covariance OK but not inside {zone_name} boundary — re-navigating."
            )
            self.nav_in_flight = False
            return 'stage_2'
 
        self.get_logger().info(f"Confirmed in zone '{zone_name}' — starting task.")
        return 'stage_5'
    
    def stage_5(self):
        if self.current_state != self.stage_5:
            self.get_logger().info("Stage 5: Executing task")
            self.current_state = self.stage_5
            task = TASK_QUEUE[self.task_index]["action"]
            if task == "excavate":
                self._task_node = ExcavationNode()
            elif task == "deposit":
                self._task_node = DepositionNode()
            self._task_node.execute()

        if self._task_node.is_done():
            self._task_node = None
            self.task_index += 1
            self.nav_in_flight = False
            return 'stage_2'

        return 'stage_5'
    
    def stage_6(self):
        """
        Stage 6: Idle
        """
        if self.current_state != self.stage_6:
            self.get_logger().info('Stage 6: Idle')
            self.current_state = self.stage_6

        return 'stage_6'
    
def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManager()
    try:
        node.execute_machine()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 