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

def process_frame(frame):

    # Convert frame to HSV for detection
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    

    # Create a mask to detect colour
    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_frame, lower1_black, upper1_black) + cv2.inRange(hsv_frame, lower2_black, upper2_black)

    # Find contours for both colors
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(contours_red, frame.shape)
    green_obs = get_obstacle_positions(contours_green, frame.shape)

    # Draw contours for visualization
    for x, y, w, h in red_obs:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
    for x, y, w, h in green_obs:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

    return frame, red_obs, green_obs

def get_obstacle_positions(contours, frame_shape):
    obs = []
    min_area = 600  # minimum contour area to be obstacle
    print(frame_shape)
    fh, fw = frame_shape[0], frame_shape[8]

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area:
            x,y,w,h = cv2.boundingRect(cnt)
            # Determine grid cell (3 rows x 2 columns)
            row = 0 if y+h/2 < fh/3 else (1 if y+h/2 < 2*fh/3 else 2)
            col = 0 if x+w/2 < fw/2 else 1
            obs.append({"box": (x,y,w,h), "grid": (row, col)})
    return obs

def decide_path(red_obs, green_obs):
    # Grid cell positions occupied by obstacles
    red_cells = [pos['grid'] for pos in red_obs]
    green_cells = [pos['grid'] for pos in green_obs]

    # Drive lane choices: 3 rows, 2 columns → 3 lanes robot can drive through
    # Logic:
    # If red obstacle detected, drive left of it
    # If green obstacle detected, drive right of it
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
    while True:
        frame = picam2.capture_array()
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process camera frame for obstacles
        frame_processed, red_obs, green_obs = process_frame(frame)

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

run()