"""
Unused class, deadcode

Either use somewhere or just delete it?

Also seems to repeat a lot the functionality from telemetry_processor.py?
    (or telemetry_processor.py repeats this file? Not sure which came first)
In general has a lot of the same issues as telemetry_processor.py
"""
import time
import math

class TractionMonitor:
    def __init__(self, nominal_current: float = 5.0, wheel_radius: float = 0.1):
        self.nominal_current = nominal_current
        self.wheel_radius = wheel_radius
        # How were these weights determined?
        self.w1, self.w2, self.w3 = 0.4, 0.3, 0.3

        self.last_left_tick = 0
        self.last_right_tick = 0
        self.last_time = time.time()

    def calculate_slip_score(self, msg, cmd_vel_x: float):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return 0.0, 0x00

        # This math doesn't seem correct
        #   Seems you you end up double counting self.wheel_radius / dt
        d_left  = (msg.left_encoder  - self.last_left_tick)  * self.wheel_radius / dt
        d_right = (msg.right_encoder - self.last_right_tick) * self.wheel_radius / dt

        # This unit ends up as something like m^2 / s^2
        v_left  = (d_left  / 255) * (2 * math.pi * self.wheel_radius) / dt
        v_right = (d_right / 255) * (2 * math.pi * self.wheel_radius) / dt
        v_measured = (v_left + v_right) / 2

        speed_error     = abs(cmd_vel_x - v_measured)
        speed_asymmetry = abs(v_left - v_right)
        avg_current     = (msg.motor_currents[0] + msg.motor_currents[1]) / 2
        current_ratio   = avg_current / self.nominal_current

        slip_score = (self.w1 * speed_error) + (self.w2 * speed_asymmetry) + (self.w3 * current_ratio)

        mcu_fix_cmd = 0x00
        if v_measured > 0.1 and avg_current > 1.0:
            mcu_fix_cmd = 0x01  # slip fix — replace 0x00 with your real command bytes
        elif abs(v_measured) < 0.01 and avg_current > 12:
            mcu_fix_cmd = 0x02  # rocking fix

        self.last_left_tick  = msg.left_encoder
        self.last_right_tick = msg.right_encoder
        self.last_time = now

        return slip_score, mcu_fix_cmd