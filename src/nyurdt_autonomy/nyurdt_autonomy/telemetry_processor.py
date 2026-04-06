import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

from nyurdt_interfaces.msg import RobotStatus

# Slip score weights
W1, W2, W3 = 0.4, 0.3, 0.3

# Thresholds
NOMINAL_DRIVE_CURRENT  = 5.0   # amps
NOMINAL_LADDER_CURRENT = 6.0   # amps
NOMINAL_GATE_CURRENT   = 2.0   # amps
WHEEL_RADIUS           = 0.1   # metres
EMPTY_THRESHOLD        = 0.05  # kg — bin counts as empty below this
POS_TOLERANCE          = 0.05  # 0.0-1.0 scale


class TelemetryProcessor(Node):
    def __init__(self):
        super().__init__('telemetry_processor')

        self.create_subscription(RobotStatus, 'robot/telemetry_raw', self._cb, 10)
        self.create_subscription(Twist, '/robot/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Float32, '/robot/target_conveyor_pos', self._conveyor_target_cb, 10)
        self.create_subscription(Bool, '/robot/target_gate_closed', self._gate_target_cb, 10)

        self.pub = self.create_publisher(RobotStatus, 'robot/telemetry', 10)

        self.prev_mass          = 0.0
        self.prev_left_enc      = 0
        self.prev_right_enc     = 0
        self.prev_time          = self.get_clock().now()
        self.target_vel         = 0.0
        self.target_conveyor_pos = 0.0   # 0.0-1.0
        self.target_gate_closed  = True  # assume closed at startup

    def _cmd_vel_cb(self, msg: Twist):
        self.target_vel = msg.linear.x

    def _conveyor_target_cb(self, msg: Float32):
        self.target_conveyor_pos = msg.data

    def _gate_target_cb(self, msg: Bool):
        self.target_gate_closed = msg.data

    # ── Load cell computations ──────────────────────────────────────────────

    def _compute_mass(self, msg: RobotStatus, dt: float):
        msg.total_mass = sum(msg.load_cells)
        msg.mass_flow  = (msg.total_mass - self.prev_mass) / dt
        # mass_flow > 0  → filling (excavation)
        # mass_flow < 0  → emptying (deposition)
        return msg

    # ── Encoder + motor current computations ───────────────────────────────

    def _compute_locomotion(self, msg: RobotStatus, dt: float):
        # Encoder ticks → wheel surface speed (m/s)
        d_left  = (msg.left_encoder  - self.prev_left_enc)
        d_right = (msg.right_encoder - self.prev_right_enc)

        # ticks are 0-255 per full rotation → convert to radians
        rad_left  = (d_left  / 255.0) * (2 * 3.14159)
        rad_right = (d_right / 255.0) * (2 * 3.14159)

        v_left  = (rad_left  * WHEEL_RADIUS) / dt
        v_right = (rad_right * WHEEL_RADIUS) / dt
        v_measured = (v_left + v_right) / 2.0

        avg_drive_current = (msg.motor_currents[0] + msg.motor_currents[1]) / 2.0
        current_ratio     = avg_drive_current / NOMINAL_DRIVE_CURRENT

        speed_error     = abs(self.target_vel - v_measured)
        speed_asymmetry = abs(v_left - v_right)

        msg.slip_score = (W1 * speed_error) + (W2 * speed_asymmetry) + (W3 * current_ratio)

        # Locomotion flags
        msg.slip_warn     = msg.slip_score > 0.5
        msg.slip_severe   = msg.slip_score > 1.0
        msg.stall_risk    = v_measured < 0.01 and avg_drive_current > 12.0
        msg.digging_in    = v_measured < 0.05 and avg_drive_current > 8.0

        return msg

    # ── String pot (conveyor) computations ─────────────────────────────────

    def _compute_excavation(self, msg: RobotStatus, dt: float):
        ladder_current = msg.motor_currents[5]   # adjust index to match your wiring
        ladder_ratio   = ladder_current / NOMINAL_LADDER_CURRENT

        # High current + barely any mass flowing in = jam risk
        msg.ladder_jam_risk = ladder_ratio > 2.0 and 0 < msg.mass_flow < 0.01

        # Low current + barely any mass flowing in = cutting air
        msg.empty_cut = ladder_ratio < 0.4 and msg.mass_flow < 0.01

        # Conveyor not reaching target (you set target_conveyor_pos from your task node)
        msg.exc_pos_error = (
            abs(msg.conveyor_pos - self.target_conveyor_pos) > POS_TOLERANCE
        )

        return msg

    # ── Gate position + current computations ───────────────────────────────

    def _compute_deposition(self, msg: RobotStatus, dt: float):
        gate_current = msg.motor_currents[7]     # adjust index to match your wiring
        gate_ratio   = gate_current / NOMINAL_GATE_CURRENT

        # Gate commanded closed but still reading open
        msg.gate_not_closed = self.target_gate_closed and msg.gate_pos > 0.05

        # Gate stalled — high current and position not changing
        msg.gate_stall = gate_ratio > 2.5 and msg.gate_not_closed

        # Bin empty
        msg.depo_empty = msg.total_mass < EMPTY_THRESHOLD

        # Gate is open but mass flow out is near zero = blockage
        msg.depo_flow_blocked = (
            msg.gate_pos > 0.5
            and msg.mass_flow > -0.001   # barely any material leaving
        )

        return msg

    # ── Main callback ───────────────────────────────────────────────────────

    def _cb(self, msg: RobotStatus):
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9

        if dt <= 0:
            return

        msg = self._compute_mass(msg, dt)
        msg = self._compute_locomotion(msg, dt)
        msg = self._compute_excavation(msg, dt)
        msg = self._compute_deposition(msg, dt)

        # Update previous values
        self.prev_mass      = msg.total_mass
        self.prev_left_enc  = msg.left_encoder
        self.prev_right_enc = msg.right_encoder
        self.prev_time      = now

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()