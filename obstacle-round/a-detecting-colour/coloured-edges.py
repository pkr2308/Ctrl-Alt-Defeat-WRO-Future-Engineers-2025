import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

#Define colour ranges
lower_red = np.array([0, 120, 88])
upper_red = np.array([20, 255, 255])
lower_green = np.array([52, 120, 78])
upper_green = np.array([70, 255, 255])
lower1_black = np.array([37, 65, 20])
upper1_black = np.array([65, 130, 60])
lower2_black = np.array([40, 130, 50])
upper2_black = np.array([49, 175, 90])
# The 'magenta' parking pieces also show up as red!

def canny_edge_detection(frame):
    # Convert the frame to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and smoothen edges
    blurred = cv2.GaussianBlur(src=gray, ksize=(3, 5), sigmaX=0.5)

    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 70, 135)

    return blurred, edges

while True:
    
    frame = picam2.capture_array()
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_frame, lower1_black, upper1_black) + cv2.inRange(hsv_frame, lower2_black, upper2_black)

    mask = mask_red + mask_green + mask_black

    # Apply the mask on the original image
    masked_img = cv2.bitwise_and(frame, frame, mask=mask)
    # Apply edge detection
    blurred, edges = canny_edge_detection(masked_img)

    # Display the original frame and the result
    cv2.imshow("Original", corrected_frame)
    cv2.imshow("Colours", masked_img)
    cv2.imshow("Edge-Colour Detection", edges)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
picam2.stop_preview()
picam2.stop()
cv2.destroyAllWindows()