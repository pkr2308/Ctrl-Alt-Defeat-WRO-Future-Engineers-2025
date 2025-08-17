import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219_noir.json")
picam2 = Picamera2(tuning = tuning)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

while True:
    
    frame = picam2.capture_array()
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(corrected_frame, cv2.COLOR_RGB2HSV)

    # Define the range for white color in HSV space
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([179, 30, 255])
    # Hue for white specifically is from 0-179, since it is achromatic

    # Create a mask to detect white color
    mask1 = cv2.inRange(hsv, lower_white, upper_white)

    # Define another range for white to account for different shades of white
    lower_white2 = np.array([200, 200, 200])
    upper_white2 = np.array([255, 255, 255])

    # Create another mask and combine it with the first one
    #mask2 = cv2.inRange(hsv, lower_white2, upper_white2)

    # Combine both masks to get a better detection
    mask = mask1 #+ mask2

    # Apply the mask on the original image
    result = cv2.bitwise_and(corrected_frame, corrected_frame, mask=mask)

    # Display the original frame and the result
    cv2.imshow("Original", corrected_frame)
    cv2.imshow("white Object Detection", result)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
picam2.stop()
cv2.destroyAllWindows()
