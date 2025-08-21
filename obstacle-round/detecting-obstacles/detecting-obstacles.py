from picamera2 import Picamera2
import numpy as np
import cv2
import time

tuning = Picamera2.load_tuning_file("imx219_noir.json")
picam2 = Picamera2(tuning = tuning)

config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)

picam2.start_preview()
time.sleep(2)  # Let the camera warm up
picam2.start()

def init():
    global lower_red, upper_red, lower_green, upper_green, lower1_black, upper1_black, lower2_black, upper2_black
    lower_red = np.array([0, 120, 88])
    upper_red = np.array([25, 255, 255])
    lower_green = np.array([52, 120, 78])
    upper_green = np.array([70, 255, 255])
    lower1_black = np.array([37, 65, 20])
    upper1_black = np.array([65, 130, 60])
    lower2_black = np.array([40, 130, 50])
    upper2_black = np.array([49, 175, 90])
    # The 'magenta' parking pieces also show up as red!  

def process_frame():
    global masked_img, hsv_frame
    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_frame, lower1_black, upper1_black) + cv2.inRange(hsv_frame, lower2_black, upper2_black)

    mask = mask_red + mask_green

    # Apply the mask on the original image
    masked_img = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Find contours for red,green and black. We don't want the hierarchy
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(corrected_frame, red_contours, -1, (255,100,100), -1)
    cv2.drawContours(corrected_frame, red_contours, -1, (100,255,100), -1)
    cv2.drawContours(corrected_frame, red_contours, -1, (120,120,120), -1)
    

    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(red_contours)
    green_obs = get_obstacle_positions(green_contours)

    # Draw contours for visualization
    for x, y, w, h in red_obs:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
    for x, y, w, h in green_obs:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

    return frame, red_obs, green_obs

def get_obstacle_positions(contours):
    obs = []
    min_area = 600  # minimum contour area to be obstacle in pixels
    print(frame.shape)
    fh, fw = frame.shape[0], frame.shape[8]

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area:
            x,y,w,h = cv2.boundingRect(cnt)
            # Determine grid cell (3 rows x 2 columns)
            row = 0 if y+h/2 < fh/3 else (1 if y+h/2 < 2*fh/3 else 2)
            col = 0 if x+w/2 < fw/2 else 1
            obs.append({"box": (x,y,w,h), "grid": (row, col)})
    return obs


def decide_path(red_obs, green_obs):
    # Needs to be updated
    # Grid cell positions occupied by obstacles
    red_cells = [pos['grid'] for pos in red_obs]
    green_cells = [pos['grid'] for pos in green_obs]

    # Logic:
    # If red obstacle detected as nearest, drive left of it
    # If green obstacle detected as nearest, drive right of it
    # If two obstacles, they are at ends, so robot drives the middle lane

    # Simplified path selection
    if len(red_cells) == 1 and not green_cells:
        # Obstacle red detected: drive left of obstacle means steer left
        action = "STEER_LEFT"
    elif len(green_cells) == 1 and not red_cells:
        # Green obstacle detected: drive right of obstacle means steer right
        action = "STEER_RIGHT"
    elif len(red_cells) == 2 or len(green_cells) == 2 or (len(red_cells) == 1 and len(green_cells) == 1):
        # Obstacles on ends → drive middle lane straight
        action = "STRAIGHT"
    else:
        # No obstacles detected
        action = "STRAIGHT"

    return action

def run():
    global frame, hsv_frame, corrected_frame
    while True:
        # Read a frame from the camera
        frame = picam2.capture_array()
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Process camera frame for obstacles
        frame_processed, red_obs, green_obs = process_frame()

        # Decide navigation based on obstacle detection
        path_action = decide_path(red_obs, green_obs)

        # Show output
        cv2.putText(frame_processed, f"Path: {path_action}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        cv2.imshow("Obstacle Detection", frame_processed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop_preview()
    picam2.stop()
    cv2.destroyAllWindows()


init()
run()