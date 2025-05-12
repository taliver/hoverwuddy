import sys
import struct
import serial
import time # Optional: add a small delay after opening port
import cv2
import numpy as np

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


def GetDirection(x):
    offset = x - 160
    offset = offset * 10
    if offset < -500:
        return -500
    if offset > 500:
        return 500
    return offset

def GetSpeed(diameter):
    target_diameter = 55  # Ideal size of the circle (distance where you want to stay)
    error = target_diameter - diameter

    # Proportional gain (tune this value to get better response)
    kP = 5

    speed = error * kP

    # Clamp the speed to a reasonable range
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100

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

def Control(steer_in, speed_in):

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
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

