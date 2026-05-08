#!/usr/bin/env python3

"""
autonomy_manager.py

File for calling respective autonomies at the correct time. 
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String, Byte

from .excavation import ExcavationNode
from .deposition import DepositionNode

import math
import time

from .nyurdt_state_machine import StateMachine
import threading


# KSC
# ZONES = {
#     "excavation":   {"x": (-4.5720, 9.14 - 4.5720), "y": (0.00, 4.00)},
#     "deposition": {"x": (4.5720 - 2.00, 4.5720), "y": (5.50, 8.10)},
#     "obstacle":     {"x": (-4.5720, 4.5720 - 2.00), "y": (4.00, 8.10)},
#     "starting":     {"x": (-2.00, 2.00), "y": (0.00, 2.00)},
# }

# Coordinate system — X=0 (fiducial wall / construction side) → X=8.10 (ingress/starting)
#   Y: −2.285 (bottom) → +2.285 m (top)
#
# Arena layout (UCF 2026 — 8.10 m × 4.57 m, bird's-eye view):
#
#   ┌──────────────────────────────┬─────────────────────────────────────┐  y=+2.275
#   │    OBSTACLE ZONE  (4.1 m)    │       EXCAVATION ZONE  (4.0 m)      │
#   │  ┌────────────────────────┐  │  ┌─────────────────────────────┐    │
#   │  │ CONSTRUCTION  2.6×2 m  │  │  │  STARTING ZONE   2×2 m     │    │  y≈+0.285
#   │  │  ┌──────────────────┐  │  │  │  (robot spawns here)       │    │
#   │  │  │  BERM ZONE       │  │  │  └─────────────────────────────┘    │
#   │  │  │  1.5 × 0.9 m     │  │  │                                     │
#   │  │  │  ← deposit here  │  │  │       (dig regolith here)           │
#   │  │  └──────────────────┘  │  │                                     │
#   │  └────────────────────────┘  │                                     │
#   │                              │                                     │
#   └──────────────────────────────┴─────────────────────────────────────┘  y=−2.275
#   x=0 (fiducials)            x≈4.10                               x=8.10 (ingress)
#
# Zone boundaries:
#   Vertical divider at x=4.10 m: OBSTACLE (left 4.1 m) | EXCAVATION (right 4.0 m)
#   Construction zone right edge: x=2.60 m (within obstacle zone)
#   Construction zone top:        y=−0.285 m (2 m up from bottom wall)
#   Starting zone left edge:      x=6.10 m (within excavation zone)
#   Starting zone bottom:         y=+0.285 m (2 m down from top wall)

# SDF world coords (origin at left wall):
# ZONES = {
#     "excavation":   {"x": (4.10, 8.10), "y": (-2.275,  2.275)},
#     "deposition":   {"x": (0.00, 2.60), "y": (-2.275, -0.285)},
#     "berm":         {"x": (0.85, 2.35), "y": (-1.335, -0.435)},
#     "obstacle":     {"x": (0.00, 4.10), "y": (-2.275,  2.275)},
#     "starting":     {"x": (6.10, 8.10), "y": ( 0.285,  2.275)},
# }
# RELOCALIZE_POSE = {"x": 6.5, "y": 1.75, "yaw": 0.0}

# Map frame coords = SDF world − (7.10, 1.28).
# Map origin = centre of 2×2m starting zone → robot spawns near map(0.35, 0.62).
ZONES = {
    "excavation":   {"x": (-3.00,  1.00), "y": (-3.555,  0.995)},  # SDF x=4.1–8.1
    "deposition":   {"x": (-7.10, -4.50), "y": (-3.555, -1.565)},  # construction zone 2.6×2 m
    "berm":         {"x": (-6.25, -4.75), "y": (-2.615, -1.715)},  # BBZ 1.5×0.9 m
    "obstacle":     {"x": (-7.10, -3.00), "y": (-3.555,  0.995)},  # left 4.1 m
    "starting":     {"x": (-1.00,  1.00), "y": (-0.995,  0.995)},  # 2×2 m at origin
}
# Relocalize: go near right wall (map x≈−0.60) facing +X to see fiducial rail
RELOCALIZE_POSE = {"x": -0.60, "y": 0.47, "yaw": 0.0}

# covariance thresholds
# Real robot: 0.05 (tight — RTAB-Map + AprilTags should hold this)
# Sim: raised to 0.5 — sim clock jumps cause temporary TF buffer clears that
# spike RTAB-Map's localization covariance, triggering spurious relocalization.
COV_X_THRESH = 0.5
COV_Y_THRESH = 0.5
COV_SETTLE_SEC = 2.0

ZONE_ENTRY_MARGIN = 0.12

# Sim testing — navigate to the Berm Zone (BBZ) center and deposit.
# Goal = center of "berm" zone = (1.60, −0.885).
TASK_QUEUE = [
    {"zone": "berm", "action": "deposit"},
]
# Full 3-cycle queue (competition):
# TASK_QUEUE = [
#     {"zone": "excavation", "action": "excavate"},
#     {"zone": "berm",        "action": "deposit"},
#     {"zone": "excavation",  "action": "excavate"},
#     {"zone": "berm",        "action": "deposit"},
#     {"zone": "excavation",  "action": "excavate"},
#     {"zone": "berm",        "action": "deposit"},
# ]

threads = list()

class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')

        # State
        self.current_state = self.stage_1
        self.task_index = 0
        self.current_zone = None
        self.pose = None
        self.covariance = [.01] * 36 #CHANGE TO HIGH VALUE INITIALLY
        self.nav_goal_handle = None
        self.nav_in_flight = False
        self._nav_result_ready = False
        self._nav_result = None
        self.relocalize_after = None
        self.cov_ok_since = None
        self.bin_mass = None
        self.goal_bin_mass = 0.4 # REPLACE WITH GOAL MASS 
        self.last_apriltag_correction = None
        self._task_thread = None
        self._localization_start_time = None

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


        self.task_pub = self.create_publisher(String, "/autonomy/current_task", 10)

        self.status_pub = self.create_publisher(String, "/autonomy/status", 10)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._bt_state_client = self.create_client(GetState, '/bt_navigator/get_state')

        self.publisher = self.create_publisher(Byte, '/robotCmds', 10)

        self.get_logger().info("AutonomyManager started - state: INIT_LOCALIZING")


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
        self.machine.add_state('stage_6', self.stage_6)
    
    def execute(self):
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
    def _wait_for_nav2_active(self, timeout_sec=30.0):
        """Block until bt_navigator is active or timeout."""
        import time
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                future = self._bt_state_client.call_async(GetState.Request())
                # Busy-wait on the future without spinning
                start = time.time()
                while not future.done() and time.time() - start < 2.0:
                    time.sleep(0.05)
                if future.done() and future.result():
                    state = future.result().current_state.label
                    if state == 'active':
                        self.get_logger().info('Nav2 is active.')
                        return True
                    self.get_logger().info(f'Nav2 state: {state}, waiting...')
            except Exception as e:
                self.get_logger().warn(f'State check failed: {e}')
            time.sleep(1.0)
        return False

    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available.")
            self.nav_in_flight = False  # allow stage_2 to retry on next iteration
            return

        if not self._wait_for_nav2_active(timeout_sec=30.0):
            self.get_logger().error("Nav2 never became active, aborting.")
            self.nav_in_flight = False  # allow stage_2 to retry on next iteration
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

    def _nav_goal_failed(self) -> bool:
        if not getattr(self, "_nav_result_ready", False):
            return False
        return self._nav_result.status not in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_ACCEPTED,
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
            
        if not self._localization_start_time:
            self._localization_start_time = self.get_clock().now()

        # (old) turn until an apriltag is spotted
        # self.publish_msg(0x50) # turn right at 25%
        # (old) timer for testing
        # if self.get_clock().now() - self._localization_start_time > Duration(seconds=3):
        #     return 'stage_2'
        # if self._covariance_ok():
        #     self.get_logger().info("Initial Localization aquired")
        #     return 'stage_2'

        elapsed_sec = (self.get_clock().now() - self._localization_start_time).nanoseconds / 1e9

        # Phase A: stay stationary ≥ 8 s so RTAB-Map populates cloud_obstacles
        # into the global costmap before Nav2 plans the first path.
        # Back cam (rpy="0 0 -π") faces +X when robot faces −X, sees fiducial rail.
        if elapsed_sec < 8.0:
            self.get_logger().info(
                f"Stage 1: building map ({elapsed_sec:.1f}/8.0 s)",
                throttle_duration_sec=2.0,
            )
            return 'stage_1'

        # Phase B: wait for back-camera AprilTag fix AND low covariance
        tag_age_sec = float('inf')
        if self.last_apriltag_correction is not None:
            tag_age_sec = (self.get_clock().now() - self.last_apriltag_correction).nanoseconds / 1e9

        if tag_age_sec < 3.0 and self._covariance_ok():
            self.get_logger().info(
                f"Stage 1: AprilTag fix confirmed (age {tag_age_sec:.1f} s, "
                f"cov_x={self.covariance[0]:.4f}). Starting navigation."
            )
            return 'stage_2'

        # Fallback: 30 s timeout (sim without colcon build, or no cameras yet)
        if elapsed_sec > 30.0:
            self.get_logger().warn(
                "Stage 1: 30 s timeout — proceeding without AprilTag fix."
            )
            return 'stage_2'

        self.get_logger().info(
            f"Stage 1: waiting for AprilTag fix "
            f"(tag age {tag_age_sec:.1f} s, cov_x={self.covariance[0]:.4f})",
            throttle_duration_sec=2.0,
        )
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
 
        # Only check for drift when a goal is actively in flight — skip before
        # the first goal is sent so RTAB-Map's initial high covariance doesn't
        # immediately send the robot to relocalize before it has moved at all.
        if self.nav_in_flight and not self._covariance_ok():
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
            self.nav_in_flight = True
            self._send_nav_goal(target_x, target_y)
            self.get_logger().info("Sent goal!!")   
            
            return 'stage_2'

 
        # Check if we've arrived (Nav2 goal succeeded)
        if self._nav_goal_succeeded():
            self.get_logger().info(f"Arrived near {TASK_QUEUE[self.task_index]['zone']}.")
            self.nav_in_flight = False
            return 'stage_5'

        # If nav2 failed or aborted, reset so we retry on the next iteration
        if self._nav_goal_failed():
            self.get_logger().warn("Nav2 goal failed — will retry.")
            self.nav_in_flight = False
            self._nav_result_ready = False

        time.sleep(0.05)
        return 'stage_2'
        
    def stage_3(self):  
        if self.current_state != self.stage_3:
            self.get_logger().info("Stage 3: Relocalizing")
            self.current_state = self.stage_3

        if not self.nav_in_flight:
            p = RELOCALIZE_POSE
            self._send_nav_goal(p["x"], p["y"], yaw=p["yaw"])
            self.nav_in_flight = True
            return 'stage_3'

        if self._nav_goal_succeeded():
            self.nav_in_flight = False
            return self.relocalize_after # for testing w/o apriltag

        # Wait for BOTH covariance to drop AND a fresh tag correction
        tag_seen_recently = (
            self.last_apriltag_correction is not None
            and (self.get_clock().now() - self.last_apriltag_correction).nanoseconds / 1e9 < 2.0
        )

        if self._covariance_ok() and tag_seen_recently:
            self.get_logger().info("Relocalization confirmed by AprilTag.")
            self.last_apriltag_correction = None
            return self.relocalize_after or 'stage_2'

        time.sleep(0.05)
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
        zone = TASK_QUEUE[self.task_index]["zone"]

        if task == "excavate":
            self.get_logger().info(
                f"╔══════════════════════════════╗\n"
                f"║  EXCAVATION START  (#{self.task_index + 1})      ║\n"
                f"╚══════════════════════════════╝"
            )
            task_node = ExcavationNode()

        elif task == "deposit":
            self.get_logger().info(
                f"╔══════════════════════════════╗\n"
                f"║  DEPOSITION START  (#{self.task_index + 1})       ║\n"
                f"║  target zone: {zone:<15}║\n"
                f"╚══════════════════════════════╝"
            )
            task_node = DepositionNode()

        self._task_thread = threading.Thread(
            target=task_node.execute, daemon=True
        )

        self._task_thread.start()
        threads.append(self._task_thread)
        self._task_thread.join()  # blocks this state machine thread, not the spin thread

        self.get_logger().info(
            f"╔══════════════════════════════╗\n"
            f"║  {task.upper()} END    (#{self.task_index + 1})          ║\n"
            f"╚══════════════════════════════╝"
        )

        self._task_thread = None
        self.task_index += 1
        self.nav_in_flight = False
        return 'stage_2'
    
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
    
    autonomy = threading.Thread(target=node.execute, daemon=True)
    threads.append(autonomy)
    autonomy.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 
