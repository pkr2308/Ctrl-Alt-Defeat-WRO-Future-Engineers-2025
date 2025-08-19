from picamera2 import Picamera2
import cv2
import time

tuning = Picamera2.load_tuning_file("imx219_noir.json")
picam2 = Picamera2(tuning = tuning)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up

picam2.start()
while True:
    frame = picam2.capture_array()
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Different standards are used
    #cv2.imshow("Picamera2 Frame", frame)
    cv2.imshow("Corrected Frame", corrected_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
