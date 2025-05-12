import time
import math
import numpy as np # For np.pi and np.sin, though math module can also be used.

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




# Now we will make something with a speed changing over time.
# This will be something that moves in a sinusoidal speed
# around 1 m/s as the baseline.

class MovingObject:
    def __init__(self, initial_position=0.0, initial_speed=0.0):
        self.position = initial_position
        self.speed = initial_speed

    def update_position(self, dt):
        self.position += self.speed * dt

def run_simulation():
    # --- Simulation Parameters ---
    dt = 0.2  # seconds, update interval
    simulation_duration = 360  # seconds (6 minutes)
    current_time_sim = 0.0

    # --- Target Parameters ---
    target_base_speed = 1.0  # m/s
    target_speed_variation_amplitude = 0.1  # m/s
    target_speed_period = 5 * 60  # 5 minutes in seconds
    target_angular_frequency = 2 * np.pi / target_speed_period
    
    target = MovingObject(initial_position=0.0, initial_speed=target_base_speed) # Initial speed will be updated

    # --- Follower (Device) Parameters ---
    initial_follower_speed = 0.0  # m/s
    desired_distance = 1.0  # meter
    initial_distance = 1.0 # meter
    
    # To achieve initial_distance = 1m, if target starts at 0, follower starts at -1
    # (distance = target_pos - follower_pos)
    initial_follower_position = target.position - initial_distance
    follower = MovingObject(initial_position=initial_follower_position, initial_speed=initial_follower_speed)

    # Follower constraints
    max_follower_speed = 2.0  # m/s
    min_follower_speed = -0.5 # m/s (allow some reverse or ability to stop fully)
    max_follower_acceleration = 0.8 # m/s^2 (PID output_limits will be this)
    min_follower_acceleration = -0.8 # m/s^2

    # --- PID Controller Setup ---
    # These gains will likely need tuning for optimal performance.
    Kp = 2.5  # Proportional gain
    Ki = 0.1  # Integral gain
    Kd = 0.75 # Derivative gain
    
    pid = PIDController(
        Kp=Kp, Ki=Ki, Kd=Kd,
        setpoint=desired_distance,
        output_limits=(min_follower_acceleration, max_follower_acceleration), # PID output is acceleration
        integral_limits=(-1.0, 1.0) # Limit integral term to prevent excessive windup
    )
    pid._previous_time = time.time() - dt # Pre-initialize previous_time for the first PID dt calculation

    print(f"{'Sim Time (s)':<12} | {'Target Pos (m)':<15} | {'Follower Pos (m)':<17} | {'Target Spd (m/s)':<18} | {'Follower Spd (m/s)':<20} | {'Distance (m)':<14} | {'PID Error':<10} | {'Follower Acc (m/s^2)':<20}")
    print("-" * 150)

    # --- Simulation Loop ---
    while current_time_sim <= simulation_duration:
        # 1. Update Target's Speed and Position
        target.speed = target_base_speed + target_speed_variation_amplitude * np.sin(target_angular_frequency * current_time_sim)
        target.update_position(dt)

        # 2. Calculate Current Distance from Follower to Target
        current_distance = target.position - follower.position
        
        # 3. Get PID Output (interpreted as follower's acceleration)
        # The error for the PID is (desired_distance - current_distance).
        # If current_distance > desired_distance (too far), error is negative.
        #   PID output (with Kp>0) is negative.
        #   We need positive acceleration: acceleration = -PID_output.
        # If current_distance < desired_distance (too close), error is positive.
        #   PID output (with Kp>0) is positive.
        #   We need negative acceleration: acceleration = -PID_output.
        
        pid_output_acceleration_signed_raw = pid.update(current_value=current_distance, dt=dt)
        follower_acceleration = -pid_output_acceleration_signed_raw # Apply the negation as discussed

        # 4. Update Follower's Speed and Position
        follower.speed += follower_acceleration * dt
        # Clamp follower speed
        follower.speed = max(min_follower_speed, min(max_follower_speed, follower.speed))
        follower.update_position(dt)

        # --- Logging ---
        # PID internal error: desired_distance - current_distance
        pid_internal_error = desired_distance - current_distance 
        
        print(f"{current_time_sim:<12.1f} | {target.position:<15.3f} | {follower.position:<17.3f} | {target.speed:<18.3f} | {follower.speed:<20.3f} | {current_distance:<14.3f} | {pid_internal_error:<10.3f} | {follower_acceleration:<20.3f}")

        # 5. Advance Simulation Time
        current_time_sim += dt
        # time.sleep(dt) # Uncomment to slow down simulation for real-time viewing (approx)

if __name__ == '__main__':
    run_simulation()
