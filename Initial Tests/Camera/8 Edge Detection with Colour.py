import cv2
import numpy as np

# Start video capture
cap = cv2.VideoCapture(0)

def canny_edge_detection(frame):
    # Convert the frame to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and smoothen edges
    blurred = cv2.GaussianBlur(src=gray, ksize=(3, 5), sigmaX=0.5)

    # Perform Canny edge detection
    edges = cv2.Canny(blurred, 70, 135)

    return blurred, edges

while True:
    # Read each frame from the webcam
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to hsv color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    '''
    Hue (H): 0 - 179 (Represents the color type, dominant wavelength)
    Saturation (S): 0 - 255 (Represents the intensity or purity of the color) 
    Value (V): 0 - 255 (Represents the brightness or darkness of the color)
    '''

    # Define the range for colour color in hsv space
    lower_colour1 = np.array([10, 110, 180])
    upper_colour1 = np.array([18, 175, 255])
    lower_colour2 = np.array([35, 90, 150])
    upper_colour2 = np.array([50, 130, 210])
    # Create a mask to detect colour
    mask = cv2.inRange(hsv, lower_colour2, upper_colour2) + cv2.inRange(hsv, lower_colour1, upper_colour1)

    # Apply the mask on the original image
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)

    blurred, edges = canny_edge_detection(masked_image)
    # Display the original frame and the result
    cv2.imshow("Original", masked_image)
    cv2.imshow("Colour Object Detection", edges)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
