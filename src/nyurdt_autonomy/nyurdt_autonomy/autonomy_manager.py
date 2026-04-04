import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from enum import Enum, auto
import math

ZONES = {
    "excavation":   {"x": (-4.5720, 9.14 - 4.5720), "y": (0.00, 4.00)},
    "construction": {"x": (4.5720 - 2.00, 4.5720), "y": (5.50, 8.10)},
    "obstacle":     {"x": (-4.5720, 4.5720 - 2.00), "y": (4.00, 8.10)},
    "starting":     {"x": (-2.00, 2.00), "y": (0.00, 2.00)},
}

# pose where robot can see the Apriltag 
RELOCALIZE_POSE = {"x": 0.0, "y": 0.0, "yaw": 0.0}

# covariance thresholds
COV_X_THRESH = 0.05
COV_Y_THRESH = 0.05
COV_SETTLE_SEC = 2.0

ZONE_ENTRY_MARGIN = 0.12

class State(Enum):
    INIT_LOCALIZING = auto() # boot: find the Apriltag
    NAVIGATING      = auto()
    RELOCALIZING    = auto()
    IN_ZONE         = auto()
    EXECUTING_TASK  = auto()
    IDLE            = auto()

TASK_QUEUE = [
    {"zone": "excavation", "action": "excavate"},
    {"zone": "construction", "action": "deposit"},
]

class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')

        # State
        self.state = State.INIT_LOCALIZING
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

        self.create_subscription(
            PoseWithCovarianceStamped,
            "rtabmap/localization_pose",
            self._pose_cb,
            10,
        )

        self.task_pub = self.create_publisher(String, "/autonomy/current_task", 10)

        self.status_pub = self.create_publisher(String, "/autonomy/status", 10)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.1, self._loop)

        self.get_logger().info("AutonomyManager started - state: INIT_LOCALIZING")


    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose.pose
        self.covariance = msg.pose.covariance

    def _loop(self):
        self._publish_status()

        if self.state == State.INIT_LOCALIZING:
            self._handle_init_localizing()

        elif self.state == State.RELOCALIZING:
            self._handle_navigating()

        elif self.state == State.IN_ZONE:
            self._handle_in_zone()

        elif self.state == State.EXECUTING_TASK:
            self._handle_executing_task()

        elif self.state == State.IDLE:
            pass

    def _handle_init_localizing(self):
        """Spin in place until we have a solid fix on the Apriltag."""
        if self._covariance_ok():
            self.get_logger().info("Initial localization acquired")
            self._transition(State.NAVIGATING)
        else:
            pass

    def _handle_navigating(self):
        """Send a Nav2 goal for the current task's zone, monitor for drift."""
        if self.task_index >= len(TASK_QUEUE):
            self._transition(State.IDLE)
            return
 
        # Check for drift mid-navigation
        if not self._covariance_ok():
            self.get_logger().warn("Drift detected while navigating — relocalizing.")
            self._cancel_nav_goal()
            self.relocalize_after = State.NAVIGATING
            self._transition(State.RELOCALIZING)
            return
 
        # Send goal if not already in flight
        if not self.nav_in_flight:
            task = TASK_QUEUE[self.task_index]
            zone = ZONES[task["zone"]]
            target_x = (zone["x"][0] + zone["x"][1]) / 2.0
            target_y = (zone["y"][0] + zone["y"][1]) / 2.0
            self._send_nav_goal(target_x, target_y)
            return
 
        # Check if we've arrived (Nav2 goal succeeded)
        if self._nav_goal_succeeded():
            self.get_logger().info(f"Arrived near {TASK_QUEUE[self.task_index]['zone']}.")
            self.nav_in_flight = False
            self._transition(State.IN_ZONE)
 
    def _handle_relocalizing(self):
        """Navigate to the AprilTag view pose; wait for covariance to settle."""
        if not self.nav_in_flight:
            p = RELOCALIZE_POSE
            self._send_nav_goal(p["x"], p["y"], yaw=p["yaw"])
            self.get_logger().info("Heading to relocalization pose.")
            return
 
        if self._nav_goal_succeeded():
            self.nav_in_flight = False
            self.get_logger().info("At relocalization pose — waiting for covariance to settle.")
 
        # Stay here until confidence is restored
        if self._covariance_ok():
            self.get_logger().info("Covariance settled — resuming mission.")
            next_state = self.relocalize_after or State.NAVIGATING
            self.relocalize_after = None
            self._transition(next_state)
 
    def _handle_in_zone(self):
        """Confirm we're inside the correct zone with enough confidence before acting."""
        task = TASK_QUEUE[self.task_index]
        zone_name = task["zone"]
 
        # Check covariance first — if bad, relocalize then come back
        if not self._covariance_ok():
            self.get_logger().warn("Covariance too high in-zone — relocalizing before acting.")
            self.relocalize_after = State.IN_ZONE
            self._transition(State.RELOCALIZING)
            return
 
        # Track how long covariance has been OK (require a short hold)
        now = self.get_clock().now()
        if self._cov_ok_since is None:
            self._cov_ok_since = now
            return
        elapsed = (now - self._cov_ok_since).nanoseconds / 1e9
        if elapsed < COV_SETTLE_SEC:
            return  # still settling
        self._cov_ok_since = None
 
        # Confirm physical zone membership (with margin)
        if not self._inside_zone(zone_name, margin=ZONE_ENTRY_MARGIN):
            self.get_logger().warn(
                f"Covariance OK but not inside {zone_name} boundary — re-navigating."
            )
            self.nav_in_flight = False
            self._transition(State.NAVIGATING)
            return
 
        self.get_logger().info(f"Confirmed in zone '{zone_name}' — starting task.")
        self._transition(State.EXECUTING_TASK)
 
    def _handle_executing_task(self):
        """
        Dispatch to zone-specific logic.
        """
        task = TASK_QUEUE[self.task_index]
        action = task["action"]
        done = False
 
        if action == "excavate":
            done = self._do_excavate()
        elif action == "deposit":
            done = self._do_deposit()
        else:
            self.get_logger().warn(f"Unknown action '{action}' — skipping.")
            done = True
 
        if done:
            self.task_pub.publish(String(data="none"))
            self.task_index += 1
            if self.task_index >= len(TASK_QUEUE):
                self._transition(State.IDLE)
            else:
                self.nav_in_flight = False
                self._transition(State.NAVIGATING)
 
    def _do_excavate(self) -> bool:
        """Return True when the excavation task is complete."""
        self.task_pub.publish(String(data="excavating"))
        self.get_logger().info("[stub] Excavating...")

        if self.bin_mass >= self.goal_bin_mass:
            return True  
 
    def _do_deposit(self) -> bool:
        """Return True when the deposit task is complete."""
        self.task_pub.publish(String(data="depositing"))
        self.get_logger().info("[stub] Depositing...")

        if self.bin_mass == 0:
            return True
 
    # =======================================================================
    # Nav2 helpers
    # =======================================================================
 
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
 
    # =======================================================================
    # Localization helpers
    # =======================================================================
 
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
 
    # =======================================================================
    # Utilities
    # =======================================================================
 
    def _transition(self, new_state: State):
        self.get_logger().info(f"State: {self.state.name} → {new_state.name}")
        self.state = new_state
 
    def _publish_status(self):
        task_name = (
            TASK_QUEUE[self.task_index]["zone"]
            if self.task_index < len(TASK_QUEUE)
            else "none"
        )
        msg = String(data=f"state={self.state.name} task={task_name} "
                         f"cov_x={self.covariance[0]:.4f} cov_y={self.covariance[7]:.4f}")
        self.status_pub.publish(msg)
 

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 