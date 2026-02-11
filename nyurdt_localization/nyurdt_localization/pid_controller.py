"""
pid_controller.py

Utilizes given info to create a Proportional, Intregral, Derivative controller.
P = kP (constant P) * curr_err (desired state - current state)
I = kI (constant I) * summ_err (summation of previous errors)
D = kD (constant D) * change_err (current error - previous error)

Together the PID calculate the Offset and the robot's drive vector will be updated accordingly
in order to reach the desired state

This file is complete
"""

import time

class PID:
    """ 
    PID Controller Initialization
    Has P, I, D constants, windup constant, and current_time
    """
    def __init__(self, P=0.1, I=0.0, D=0.0, windup=360.0, current_time=None):
        
        # Setting controller constants
        self.Kp = P
        self.Ki = I
        self.Kd = D

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = windup
        
        # Variables regarding time are used to keep terms in relation to seconds
        # A sample time of 0.00 is good for our purpose, can be updated if required
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    """
    Clears PID computations and coefficients
    """
    def clear(self): 
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.output = 0.0

    """
    Uses error to calculate PID value using P,I,D constants
    k_P * error + k_I * summation_error + k_D * delta_error
    """
    def update(self, error, current_time=None):
        # Determining change in time
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time

        # Determining change in error
        delta_error = error - self.last_error

        # Checking if enough time has passed to update terms
        if (delta_time >= self.sample_time):
            # Determining PTerm and ITerm
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            # Using windup to prevent over-accumulation of ITerm
            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            # Determining DTerm
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        return self.output

    """
    Determines how aggressively the PID reacts to the current error with setting Proportional Gain
    """
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    """
    Determines how aggressively the PID reacts to the current error with setting Integral Gain
    """
    def setKi(self, integral_gain):
        self.Ki = integral_gain

    """
    Determines how aggressively the PID reacts to the current error with setting Derivative Gain
    """
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    """
    Windup is used to make sure that the integral term does not accumulate excess
    """
    def setWindup(self, windup):
        self.windup_guard = windup
    
    """
    Sample_time is used so PID value updates at regular interval
    """
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
