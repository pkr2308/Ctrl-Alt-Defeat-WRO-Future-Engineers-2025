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

    # Define the range for green color in HSV space
    lower_green = np.array([10, 50, 20])
    upper_green = np.array([70, 100, 80])

    # Create a mask to detect green color
    mask1 = cv2.inRange(hsv, lower_green, upper_green)

    # Define another range for green to account for different shades of green
    lower_green2 = np.array([100, 130, 70])
    upper_green2 = np.array([150, 255, 180])

    # Create another mask and combine it with the first one
    mask2 = cv2.inRange(hsv, lower_green2, upper_green2)

    # Combine both masks to get a better detection
    mask = mask1 + mask2

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour and compute its center
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw the center of the contour on the original frame
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Center: ({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Apply the mask on the original image
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame and the result
    cv2.imshow("Original", frame)
    cv2.imshow("green Object Detection", result)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()