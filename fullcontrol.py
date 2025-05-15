import sys
import struct
import serial
import time # Optional: add a small delay after opening port
import cv2
import numpy as np
import pid as pidcontroller

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
 
pid = pidcontroller.PIDController(
        Kp=Kp, Ki=Ki, Kd=Kd,
        setpoint=desired_distance,
        output_limits=(min_follower_acceleration, max_follower_acceleration), # PID output is acceleration
        integral_limits=(-1.0, 1.0) # Limit integral term to prevent excessive windup
    )

pid._previous_time = time.time() - 0.2 # Pre-initialize previous_time for the first PID dt calculation
  

def GetDirection(x):
    offset = x - 160
    offset = offset * 10
    if offset < -500:
        return -500
    if offset > 500:
        return 500
    return offset


# Initial speed of robot
speed = 0
previous_update = time.time()
print("Previous update: ", previous_update)

def GetSpeed(diameter):
        global previous_update
        global speed
        global pid
        current_distance = diameter * 0.01  # Note, make this value be meters.
        dt = time.time() - previous_update
        pid_output_acceleration_signed_raw = pid.update(current_value=current_distance, dt=dt)
        acceleration = -pid_output_acceleration_signed_raw # Apply the negation as discussed

        speed += acceleration * dt
        previous_update = time.time()
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
MIN_CONTOUR_AREA = 200

# --- Camera Initialization ---
# Use 0 for the default camera (often /dev/video0)
# If using Pi Camera module and it's not /dev/video0, you might need
# specific setup or try index -1 or 1.
# Ensure 'legacy camera support' is enabled via raspi-config if using older methods.
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

print(f"Camera initialized at {FRAME_WIDTH}x{FRAME_HEIGHT}")
print("Press 'q' to quit.")

    # 5. Open serial port and write data


# Optional: FPS calculation variables
# prev_time = 0

# --- Main Loop ---
while True:
    # Optional: Calculate FPS
    # current_time = time.time()
    # fps = 1 / (current_time - prev_time)
    # prev_time = current_time
    # print(f"FPS: {fps:.1f}")

    # 1. Read frame from camera
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break

    # 2. Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 3. Create a mask for yellow color
    mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

    # 4. (Optional but Recommended) Morphological Operations
    # Erode shrinks the white regions (removes small noise)
    mask = cv2.erode(mask, None, iterations=1)
    # Dilate expands the white regions (closes gaps)
    mask = cv2.dilate(mask, None, iterations=1)

    # --- DEBUG: Show the mask (Requires a display connected to Pi) ---
    # cv2.imshow('Mask', mask)
    # ---

    # 5. Find contours in the mask
    # Use RETR_EXTERNAL to get only outer contours
    # Use CHAIN_APPROX_SIMPLE to save memory
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    found_circle = False
    if contours:
        # Find the largest contour by area (assuming the largest yellow object is the circle)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area > MIN_CONTOUR_AREA:
            # 6. Find the minimum enclosing circle for the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            # Check if radius is reasonably large (filters tiny noise circles)
            if radius > 5: # Adjust this minimum radius threshold if needed
                center = (int(x), int(y))
                diameter = int(radius * 2)
                direction = GetDirection(int(x))
                speed = GetSpeed(diameter)
                found_circle = True
                # 7. Output the results
                print(f"Yellow Circle Found! Center: ({center[0]}, {center[1]}), Diameter: {diameter} pixels")
                Control(direction, speed)
                # --- Optional: Draw on the original frame (Requires display) ---
                # Draw the circle outline
                #cv2.circle(frame, center, int(radius), (0, 255, 0), 2) # Green circle
                # Draw the center point
                #cv2.circle(frame, center, 3, (0, 0, 255), -1) # Red dot
                # Put text
                #cv2.putText(frame, f"D:{diameter}", (center[0]-20, center[1]-20),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                # ---

    if not found_circle:
        # print("No significant yellow circle detected.") # Uncomment for continuous feedback
        pass # Or print nothing if no circle found

    # --- Optional: Display the resulting frame (Requires display) ---
    # cv2.imshow('Frame', frame)
    # ---

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
print("Releasing camera and closing windows.")
cap.release()
cv2.destroyAllWindows()

