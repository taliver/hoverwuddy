import sys
import struct
import serial
import time # Optional: add a small delay after opening port
import cv2
import numpy as np
import pid as pidcontroller
import yellow

# --- Constants ---
START_FRAME = 0xAAAA
SERIAL_PORT = "/dev/ttyUSB0"  # Default serial port used in C code
if sys.argv[1]:
	SERIAL_PORT = sys.argv[1]

# Important: Check if baud rate etc. are needed for your device
BAUD_RATE = 19200 # Common default, adjust if needed

# --- Structure Definition (using struct format string) ---
# < : Little-endian (check if receiver needs Big-endian '>')
# H : unsigned short (uint16_t, 2 bytes) - start_of_frame
# h : short (int16_t, 2 bytes) - steer
# h : short (int16_t, 2 bytes) - speed
# H : unsigned short (uint16_t, 2 bytes) - checksum
# Total size = 2 + 2 + 2 + 2 = 8 bytes
PACK_FORMAT = '<HhhH' # Ensure endianness matches receiver!

print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
# --- Follower (Device) Parameters ---
initial_follower_speed = 0.0  # m/s
desired_distance = 1.0  # meter
initial_distance = 1.0 # meter

max_follower_speed = 2.0  # m/s
min_follower_speed = -0.5 # m/s (allow some reverse or ability to stop fully)
max_follower_acceleration = 0.8 # m/s^2 (PID output_limits will be this)
min_follower_acceleration = -0.8 # m/s^2
# --- PID Controller Setup ---
# These gains will likely need tuning for optimal performance.
Kp = 2.5  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.75 # Derivative gain
 
speed_pid = pidcontroller.PIDController(
        Kp=Kp, Ki=Ki, Kd=Kd,
        setpoint=desired_distance,
        output_limits=(min_follower_acceleration, max_follower_acceleration), # PID output is acceleration
        integral_limits=(-1.0, 1.0) # Limit integral term to prevent excessive windup
    )

speed_pid._previous_time = time.time() - 0.2 # Pre-initialize previous_time for the first PID dt calculation
  
direction_pid = pidcontroller.PIDController(
        Kp=Kp, Ki=Ki, Kd=Kd,
        setpoint=0,
        output_limits=(-10, 10), # PID output i a radial acceleration
        integral_limits=(-1.0, 1.0) # Limit integral term to prevent excessive windup
    )


direction = 0
direction_previous_update = time.time()

def GetDirection(x):
    global direction_pid
    global direction
    global direction_previous_update
    target = x - 160
    if x < 0:
	# In ths case, we are slowly going to go straight again.
        target = 0
    dt = time.time() - direction_previous_update
    raw_dir_output = direction_pid.update(current_value=target, dt=dt)
    direction = direction - (raw_dir_output * dt)
    direction_previous_update = time.time()
    if direction < -300:
        direction = -300
        return -300
    if direction > 300:
        direction = 300
        return 300
    return direction


# Initial speed of robot
speed = 0
speed_previous_update = time.time()

def GetSpeed(diameter):
        global speed_previous_update
        global speed
        global speed_pid
        global desired_distance
        if diameter < 0:
            current_distance = desired_distance
        else:
            current_distance = diameter / 30.0  # Note, make this value be meters.
        dt = time.time() - speed_previous_update
        pid_output_acceleration_signed_raw = speed_pid.update(current_value=current_distance, dt=dt)
        acceleration = -pid_output_acceleration_signed_raw # Apply the negation as discussed

        speed += acceleration * dt
        speed_previous_update = time.time()
        return speed



        


# --- Checksum Function ---
def calculate_checksum(steer_val, speed_val):
    """Calculates the checksum based on the C logic."""
    # Note: Python's integers handle arbitrary size.
    # The XOR operation works directly.
    # We mask with 0xFFFF to ensure the result fits uint16_t.
    # Ensure steer/speed are treated as numbers for XOR
    checksum = (START_FRAME ^ steer_val ^ speed_val) & 0xFFFF
    return checksum

def Control(r_steer_in, r_speed_in):
    speed_in = int(r_speed_in)
    steer_in = int(r_steer_in)
    # 3. Calculate Checksum
    checksum_val = calculate_checksum(steer_in, speed_in)

    print(f"Start Frame: {START_FRAME:#04x}")
    print(f"Steer      : {steer_in}")
    print(f"Speed      : {speed_in}")
    print(f"Checksum   : {checksum_val:#04x} ({checksum_val})")

    # 4. Pack the data into a binary structure
    try:
        packed_data = struct.pack(PACK_FORMAT, START_FRAME, steer_in, speed_in, checksum_val)
        print(f"Packed data ({len(packed_data)} bytes): {packed_data.hex()}")
    except struct.error as e:
        print(f"Error packing data: {e}")
        print("This might happen if steer/speed values are outside the int16 range.")
        sys.exit(1)

            # Optional: Wait a moment for the serial port to initialize
    time.sleep(0.1)
    print(f"Writing {len(packed_data)} bytes...")
    bytes_written = ser.write(packed_data)
    print(f"Successfully wrote {bytes_written} bytes.")
    # Optional: Ensure data is sent before closing
    ser.flush()


# --- Configuration ---
# Camera resolution (Lower resolution is faster on Pi Zero)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# HSV Color Range for Yellow
# --- IMPORTANT: You WILL likely need to TUNE these values ---
# Hue: 0-179, Saturation: 0-255, Value: 0-255 in OpenCV
# Common Yellow Range: Hue ~20-35
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([35, 255, 255])

# Minimum contour area to filter out noise
MIN_CONTOUR_AREA = 20

# --- Camera Initialization ---
# Use 0 for the default camera (often /dev/video0)
# If using Pi Camera module and it's not /dev/video0, you might need
# specific setup or try index -1 or 1.
# Ensure 'legacy camera support' is enabled via raspi-config if using older methods.
#cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

yellow_finder = yellow.Finder()
yellow_finder.Init()
print("Yellow finder started")

# --- Main Loop ---
while True:
    # Optional: Calculate FPS
    # current_time = time.time()
    # fps = 1 / (current_time - prev_time)
    # prev_time = current_time
    # print(f"FPS: {fps:.1f}")

    # 1. Read frame from camera
    found, center_x, center_y, diameter = yellow_finder.FindCircle()
    if found:
        center = (center_x, center_y)
        direction = GetDirection(center_x)
        speed = GetSpeed(diameter)
        print(f"Yellow Circle Found! Center: ({center[0]}, {center[1]}), Diameter: {diameter} pixels")
        Control(direction, speed)
    else:
        direction = GetDirection(-1000)
        speed = GetSpeed(-1000)
        print(f"No Circle found...")
        Control(direction, speed)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

# --- Cleanup ---
print("Releasing camera and closing windows.")
cap.release()
cv2.destroyAllWindows()

