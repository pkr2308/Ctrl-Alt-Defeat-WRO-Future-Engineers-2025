import cv2
import numpy as np

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    # Read each frame from the webcam
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Define the range for white color in HSV space
    lower_white = np.array([130, 100, 110])
    upper_white = np.array([200, 170, 180])

    # Create a mask to detect white color
    mask1 = cv2.inRange(hsv, lower_white, upper_white)

    # Define another range for white to account for different shades of white
    lower_white2 = np.array([200, 200, 200])
    upper_white2 = np.array([255, 255, 255])

    # Create another mask and combine it with the first one
    mask2 = cv2.inRange(hsv, lower_white2, upper_white2)

    # Combine both masks to get a better detection
    mask = mask1 + mask2

    # Apply the mask on the original image
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame and the result
    cv2.imshow("Original", frame)
    cv2.imshow("white Object Detection", result)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
