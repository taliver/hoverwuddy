import sys
import struct
import time # Optional: add a small delay after opening port
import cv2
import numpy as np
import yellow

# --- Constants ---
START_FRAME = 0xAAAA


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
MIN_CONTOUR_AREA = 5

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
        print(f"Yellow Circle Found! Center: ({center[0]}, {center[1]}), Diameter: {diameter} pixels")

# --- Cleanup ---
print("Releasing camera and closing windows.")
cap.release()
cv2.destroyAllWindows()

