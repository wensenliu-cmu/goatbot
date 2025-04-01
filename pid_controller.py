# pid_controller.py

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None)):
        """
        Initialize the PID controller with given gains.
        
        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            setpoint (float): Desired setpoint.
            output_limits (tuple): Tuple (min, max) for limiting output.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits  # (min, max)
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        """Reset the PID controller state."""
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, measurement, dt):
        """
        Update the PID controller.
        
        Args:
            measurement (float): The current value from the sensor.
            dt (float): Time step (in seconds).
        
        Returns:
            output (float): The control output (e.g., torque or PWM signal).
        """
        error = self.setpoint - measurement
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self._integral += error * dt
        I = self.Ki * self._integral
        
        # Derivative term (avoid division by zero)
        D = self.Kd * ((error - self._prev_error) / dt) if dt > 0 else 0.0
        
        # Save error for next update
        self._prev_error = error
        
        output = P + I + D
        
        # Apply output limits if defined
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)
            
        return output