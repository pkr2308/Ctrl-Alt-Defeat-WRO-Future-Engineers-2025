import cv2
import numpy as np

# Initialize variables to store HSV range
clicked_hsv = None

# Define smaller tolerances for Hue, Saturation, and Value
tolerance_hue = 30    # Smaller range for Hue (color)
tolerance_sat = 30    # Moderate range for Saturation (color intensity)
tolerance_val = 30    # Moderate range for Value (brightness)

# Callback function for mouse click
def click_to_mask(event, x, y, flags, param):
   global clicked_hsv
   if event == cv2.EVENT_LBUTTONDOWN:
       clicked_hsv = hsv[y, x]
       print(f"New Clicked HSV: {clicked_hsv}")

# Start video capture
cap = cv2.VideoCapture(0)

cv2.namedWindow("Original")
cv2.setMouseCallback("Original", click_to_mask)

while True:
   ret, frame = cap.read()
   if not ret:
       break

   # Convert frame to HSV
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

   # Initialize mask
   mask = np.zeros(frame.shape[:2], dtype="uint8")

   # Apply mask if an HSV value is selected
   if clicked_hsv is not None:
       lower_hsv = np.array([
           max(clicked_hsv[0] - tolerance_hue, 0),
           max(clicked_hsv[1] - tolerance_sat, 0),
           max(clicked_hsv[2] - tolerance_val, 0)
       ])
       upper_hsv = np.array([
           min(clicked_hsv[0] + tolerance_hue, 179),
           min(clicked_hsv[1] + tolerance_sat, 255),
           min(clicked_hsv[2] + tolerance_val, 255)
       ])
       mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

   # Apply the mask to the frame
   masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

   # Display the original frame and the masked result
   cv2.imshow("Original", frame)
   cv2.imshow("Masked Color", masked_frame)

   # Press 'q' to exit
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

cap.release()
cv2.destroyAllWindows()