import cv2
import numpy as np
from picamera2 import Picamera2
import time
import logging # For better feedback from the library

# Configure basic logging for the library
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(name)s: %(message)s')
logger = logging.getLogger(__name__) # Logger for this module

class Finder:
    def __init__(self, img_width=320, img_height=240,
                 lower_yellow_hsv=np.array([20, 100, 100]),
                 upper_yellow_hsv=np.array([30, 255, 255]),
                 min_radius_pixels=5,
                 camera_warmup_time=1.5):
        """
        Initializes the Yellow Circle Finder configuration.
        Call Init() to initialize and start the camera.

        Args:
            img_width (int): Width of the image for processing. Lower is faster.
            img_height (int): Height of the image for processing. Lower is faster.
            lower_yellow_hsv (np.array): Lower bound for yellow in HSV format.
            upper_yellow_hsv (np.array): Upper bound for yellow in HSV format.
            min_radius_pixels (int): Minimum radius in pixels for a detected circle.
            camera_warmup_time (float): Seconds to wait for the camera sensor to adjust after starting.
        """
        self.img_width = img_width
        self.img_height = img_height
        self.lower_yellow = lower_yellow_hsv
        self.upper_yellow = upper_yellow_hsv
        self.min_radius = min_radius_pixels
        self.camera_warmup_time = camera_warmup_time

        self.picam2 = None
        self.is_initialized = False
        logger.info("Finder instance created. Call Init() to start the camera.")

    def Init(self):
        """
        Initializes and starts the camera.

        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        if self.is_initialized:
            logger.warning("Camera is already initialized.")
            return True

        try:
            logger.info("Initializing camera...")
            self.picam2 = Picamera2()

            video_config = self.picam2.create_video_configuration(
                main={"size": (self.img_width, self.img_height), "format": "RGB888"},
                controls={
                    "FrameDurationLimits": (33333, 66666), # Aim for 15-30 FPS capture
                    "AeEnable": True,
                }
            )
            self.picam2.configure(video_config)
            self.picam2.start()
            logger.info(f"Camera started. Allowing {self.camera_warmup_time}s for sensor to adjust...")
            time.sleep(self.camera_warmup_time)
            self.is_initialized = True
            logger.info("Camera initialization successful.")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}", exc_info=True)
            self.picam2 = None # Ensure picam2 is None if init failed
            self.is_initialized = False
            return False

    def _process_frame(self, bgr_image):
        """
        Internal method to find the largest yellow circle in a BGR image.

        Args:
            bgr_image: The input image in BGR format (NumPy array).

        Returns:
            A tuple (center_x, center_y, diameter) if a circle is found,
            otherwise None.
        """
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)

        # Optional: Morphological operations (can be slow, use with caution on Pi Zero)
        # kernel = np.ones((3, 3), np.uint8)
        # yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        # yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)

        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        largest_circle_params = None
        max_radius = 0

        if contours:
            print("Found countours:", len(contours))
            for contour in contours:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                radius_int = int(radius)
                center = (int(x), int(y))

                if radius_int >= self.min_radius and radius_int > max_radius:
                    max_radius = radius_int
                    largest_circle_params = (center[0], center[1], radius_int * 2) # diameter

        return largest_circle_params

    def FindCircle(self):
        """
        Captures a frame from the camera, processes it to find the largest
        yellow circle.

        Returns:
            tuple: (found, center_x, center_y, diameter)
                   `found` (bool): True if a circle meeting criteria is found.
                   `center_x` (int | None): X-coordinate of the circle center.
                   `center_y` (int | None): Y-coordinate of the circle center.
                   `diameter` (int | None): Diameter of the circle.
                   Returns (False, None, None, None) if not initialized or no circle found.
        """
        if not self.is_initialized or self.picam2 is None:
            logger.warning("FindCircle called but camera not initialized or init failed.")
            return False, None, None, None

        try:
            # Capture an image as a NumPy array (RGB format from config)
            rgb_image = self.picam2.capture_array("main")

            # Convert RGB to BGR for OpenCV
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            circle_info = self._process_frame(bgr_image)

            if circle_info:
                center_x, center_y, diameter = circle_info
                return True, center_x, center_y, diameter
            else:
                return False, None, None, None

        except Exception as e:
            logger.error(f"Error during FindCircle: {e}", exc_info=True)
            return False, None, None, None

    def Close(self):
        """
        Stops the camera and releases resources.
        """
        if self.picam2 and self.is_initialized:
            try:
                logger.info("Stopping camera...")
                self.picam2.stop()
                self.picam2.close() # picam2 recommends close after stop
                logger.info("Camera stopped and closed.")
            except Exception as e:
                logger.error(f"Error stopping/closing camera: {e}", exc_info=True)
        elif self.picam2 and not self.is_initialized: # If init failed but picam2 object exists
             try:
                self.picam2.close()
                logger.info("Camera (partially initialized) closed.")
             except Exception as e:
                logger.error(f"Error closing partially initialized camera: {e}", exc_info=True)

        self.picam2 = None
        self.is_initialized = False


# --- Example Usage (for testing the library directly) ---
if __name__ == "__main__":
    print("--- Testing Yellow Circle Finder Library ---")

    # Example: Customize parameters
    # finder = yellow.Finder(img_width=320, img_height=240, min_radius_pixels=5)
    finder = Finder() # Use default parameters

    if finder.Init():
        logger.info("Initialization successful. Starting tracking loop for 100 frames (approx 10s)...")
        try:
            for i in range(100): # Run for a limited number of frames for testing
                start_time = time.time()
                found, center_x, center_y, diameter = finder.FindCircle()
                proc_time = time.time() - start_time

                if found:
                    print(f"Frame {i+1}: Found circle at ({center_x}, {center_y}) with D={diameter}px. Time: {proc_time:.4f}s")
                else:
                    print(f"Frame {i+1}: No yellow circle found. Time: {proc_time:.4f}s")

                time.sleep(0.05) # Simulate some work in the robot's loop, adjust as needed

        except KeyboardInterrupt:
            print("\nTest loop interrupted by user.")
        finally:
            print("Closing finder...")
            finder.Close()
            print("Finder closed.")
    else:
        print("Failed to initialize the Yellow Circle Finder.")

    print("--- Test Finished ---")

