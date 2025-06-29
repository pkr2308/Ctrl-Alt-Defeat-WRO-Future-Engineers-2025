import cv2  # Import the OpenCV library, which is used for computer vision tasks like image and video processing

# Open the camera (0 refers to the default camera connected to the system, like the built-in webcam)
cap = cv2.VideoCapture(0)

# Continuously capture frames from the camera
while True:
    # 'ret' is a boolean indicating whether the frame was successfully captured (True) or not (False)
    # 'frame' is the actual image captured from the camera (each frame)
    ret, frame = cap.read()

    if not ret:  # If 'ret' is False, it means the frame could not be captured
        print("Failed to grab frame")  # Print an error message
        break  # Exit the loop if the frame capture fails

    # Display the captured frame in a window named 'Camera'
    cv2.imshow('Camera', frame)

    # Wait for 1 millisecond to check if the 'q' key is pressed
    # cv2.waitKey(1) captures keyboard input, and 0xFF is used to mask unwanted bits from the keypress
    # ord('q') checks if the 'q' key is pressed, which will stop the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break  # Exit the loop if 'q' is pressed

# Release the camera when done capturing
cap.release()

# Close all OpenCV windows that were opened during the program
cv2.destroyAllWindows()