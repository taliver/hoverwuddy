import time
import math
import numpy as np # For np.pi and np.sin, though math module can also be used.
import pid as pidcontroller

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
    
    pid = pidcontroller.PIDController(
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
