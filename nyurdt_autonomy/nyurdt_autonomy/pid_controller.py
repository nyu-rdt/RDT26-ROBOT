class PID:
    """
    A simple PID controller
    """
    def __init__(self, P=1.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.last_error = 0
        self.integral = 0

    def update(self, error, dt):
        """
        Update the PID controller with new error value
        
        Args:
            error: Current error value
            dt: Time step since last update
            
        Returns:
            float: Control output
        """
        # Calculate proportional term
        P = self.Kp * error
        
        # Calculate integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Calculate derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        D = self.Kd * derivative
        
        # Store error for next iteration
        self.last_error = error
        
        # Return sum of terms
        return P + I + D

    def reset(self):
        """Reset the PID controller"""
        self.last_error = 0
        self.integral = 0