import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-float('inf'), float('inf')), integral_limits=(-float('inf'), float('inf'))):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral_limits = integral_limits

        self._integral = 0.0
        self._previous_error = 0.0
        self._previous_time = time.time() # Initialize with current time
        self._first_update = True
    def update(self, current_value, dt=None):
        current_time = time.time()
        if dt is None:
            if self._first_update: # For the very first call, dt cannot be calculated based on previous_time
                dt = 1e-6 # A very small non-zero dt, or handle as special case
            else:
                dt = current_time - self._previous_time
        
        if self._first_update:
            self._previous_time = current_time # Set previous time for next calculation
            self._first_update = False


        error = self.setpoint - current_value

        # Proportional term
        P_out = self.Kp * error

        # Integral term (with anti-windup)
        self._integral += error * dt
        self._integral = max(self.integral_limits[0], min(self.integral_limits[1], self._integral))
        I_out = self.Ki * self._integral

        # Derivative term
        # Avoid division by zero or large derivative spike on first call if previous_error is 0
        if dt > 1e-6:
             # For the very first valid derivative calculation, previous_error might be 0 if not set.
             # If it's the first *actual* update cycle post initialization, previous_error is still 0.
             # This is typically fine, derivative will be (error - 0)/dt.
            derivative = (error - self._previous_error) / dt
            D_out = self.Kd * derivative
        else:
            D_out = 0.0

        # Total output
        output = P_out + I_out + D_out

        # Clamp output to defined limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Store error and time for next iteration
        self._previous_error = error
        self._previous_time = current_time # Update for next dt calculation if dt is None

        return output

    def reset(self, setpoint=None):
        self._integral = 0.0
        self._previous_error = 0.0
        self._previous_time = time.time()
        self._first_update = True
        if setpoint is not None:
            self.setpoint = setpoint
            
    def set_gains(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None: self.Kp = Kp
        if Ki is not None: self.Ki = Ki
        if Kd is not None: self.Kd = Kd

    def set_output_limits(self, min_output, max_output):
        self.output_limits = (min_output, max_output)
    def set_integral_limits(self, min_integral, max_integral):
        self.integral_limits = (min_integral, max_integral)
