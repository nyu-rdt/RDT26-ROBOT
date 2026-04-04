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
        """ Spin in place until we have a solid fix on the Apriltag """
        if self._covariance_ok():
            self.get_logger().info("Initial localization acquired")
            self._transition(State.NAVIGATING)
        else:
            pass

    def _handle_navigating(self):
        pass

    def update_state(self):
        if self.localization_confidence_low():
            self.state = "RELOCALIZING" # look for Apriltag
        elif self.in_zone("excavation"):
            self.state = "EXCAVATING"
        elif self.in_zone("construction"):
            self.state = "CONSTRUCTING"
        
    def localization_confidence_low(self):
        return self.covariance[0] > X_THRESH or self.covariance[7] > Y_THRESH