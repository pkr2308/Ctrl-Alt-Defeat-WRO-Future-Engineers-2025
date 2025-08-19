import cv2
import numpy as np
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()
# Global variables to store mouse coordinates and HSV value
mouse_x, mouse_y = -1, -1
hsv_value = None

def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y, hsv_value
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x, mouse_y = x, y
        if param is not None:  # Ensure a frame is passed
            # Get BGR value at mouse position
            bgr = param[y, x]
            # Convert BGR to HSV
            hsv_value = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]

cv2.namedWindow('Original')

while True:
    
    frame = picam2.capture_array()
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    cv2.setMouseCallback('Original', mouse_callback, param=corrected_frame)

    if hsv_value is not None:
        print(f"HSV at ({mouse_x}, {mouse_y}): {hsv_value}")
        hsv_value = None # Reset to avoid repeated printing for the same position

    # Define the range for white color in HSV space
    #lower_white = np.array([0, 0, 150])
    #upper_white = np.array([179, 30, 255])
    # Hue for white specifically is from 0-179, since it is achromatic

    # Create a mask to detect white color
    #mask_white = cv2.inRange(hsv_frame, lower_white, upper_white)

    lower_red = np.array([0, 120, 88])
    upper_red = np.array([25, 255, 255])
    lower_green = np.array([52, 120, 78])
    upper_green = np.array([70, 255, 255])
    lower1_black = np.array([37, 65, 20])
    upper1_black = np.array([65, 130, 60])
    lower2_black = np.array([40, 130, 50])
    upper2_black = np.array([49, 175, 90])
    # The 'magenta' parking pieces also show up as red!

    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_frame, lower1_black, upper1_black) + cv2.inRange(hsv_frame, lower2_black, upper2_black)

    mask = mask_red + mask_green

    # Apply the mask on the original image
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame and the result
    cv2.imshow("Original", corrected_frame)
    cv2.imshow("Black Mask", mask_black)
    cv2.imshow("Colours", result)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
picam2.stop_preview()
picam2.stop()
cv2.destroyAllWindows()
