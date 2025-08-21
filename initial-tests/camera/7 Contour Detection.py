import cv2
from picamera2 import Picamera2
import time

# For standard camera use "imx219.json"
tuning = Picamera2.load_tuning_file("imx219_noir.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()

while True:
    # Read a frame from the camera
    frame = picam2.capture_array()

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # Apply thresholding
    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    #edges = cv2.Canny(blurred, 90, 200)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    cv2.drawContours(frame, contours, -1, (0, 255, 0), -1)

    # Display the resulting frame
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow('Real-time Contour Detection', corrected_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the capture and destroy all windows
picam2.stop_preview()
picam2.stop()
cv2.destroyAllWindows()