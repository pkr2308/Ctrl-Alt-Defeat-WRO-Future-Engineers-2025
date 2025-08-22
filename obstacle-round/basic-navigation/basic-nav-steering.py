'''
This is a program to test a basic straight drive of the robot over 3 sections
There is no turning algorithm here
'''

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import math

#Camera config
tuning = Picamera2.load_tuning_file("imx219.json")
picam2 = Picamera2(tuning = tuning)
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start_preview()
time.sleep(1.5)  # Let the camera warm up
picam2.start()

# Serial config
# usb-Raspberry_Pi_Pico_E6625887D3859130-if00 - Pranav
# usb-Raspberry_Pi_Pico_E6625887D3482132-if00 - Adbhut
ser = serial.Serial('/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6625887D3859130-if00', 115200, timeout=1)

def init():
    global lower_red, upper_red, lower_green, upper_green, lower1_black, upper1_black, lower2_black, upper2_black
    global red_obs, green_obs, turning
    global yaw, distance, left_dist, right_dist, front_dist
    global prev_dist, min_dist, prev_str, prev_obs

    # Some starting values
    yaw, distance = 0, 0
    left_dist, front_dist, right_dist = 35, 100, 35
    turning = False
    prev_dist, min_dist, prev_str = 0, 0, 90
    prev_obs = [(0,0,0,0),'']

    #Define colour ranges
    lower_red = np.array([0, 120, 88])
    upper_red = np.array([19, 255, 255])
    lower_green = np.array([52, 120, 78])
    upper_green = np.array([70, 255, 255])
    lower1_black = np.array([37, 65, 20])
    upper1_black = np.array([65, 130, 60])
    lower2_black = np.array([40, 130, 50])
    upper2_black = np.array([49, 175, 90])
    # The 'magenta' parking pieces also show up as red!

    # These are currently seen obstacles
    red_obs = []
    green_obs = []


def process_frame():
    global hsv_frame, red_obs, green_obs, hsv_roi, corrected_frame

    # Create a mask to detect colours
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    #mask_black = cv2.inRange(hsv_roi, lower1_black, upper1_black) + cv2.inRange(hsv_roi, lower2_black, upper2_black)
    
    # Find contours for red and green. We don't want the hierarchy
    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter and locate obstacle bounding boxes
    red_obs = get_obstacle_positions(red_contours, red_obs)
    green_obs = get_obstacle_positions(green_contours, green_obs)
  
    # Draw contours for visualization
    #print(f"Red Obs: {red_obs}")
    #print(f"Green Obs: {green_obs}")

    # Draw rectangles around object for checking
    for item in red_obs:
        x = item[0][0]
        y = item[0][1] + 350    #ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(corrected_frame, (x,y), (x+w,y+h), (0,255,0), 2)
    for item in green_obs:
        x = item[0][0]
        y = item[0][1] + 350    # ROI was cropped
        w = item[0][2]
        h = item[0][3]
        cv2.rectangle(corrected_frame, (x,y), (x+w,y+h), (0,255,0), 2)

    return corrected_frame, red_obs, green_obs

def get_obstacle_positions(contours, obs):
    obs = []
    min_area = 500  # minimum contour area to be obstacle in pixels
    max_area = 30000

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < max_area:
            x,y,w,h = cv2.boundingRect(cnt)
            if h > w:
                # TODO when driving integration done; Second tuple gives grid pos
                obs.append([(x,y,w,h), (0,0,0)])           
    return obs

def nearest_obstacle():
    global red_obs, green_obs

    nearest_obs = [(0,0,0,0),'']
    for obs in red_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'red'
    for obs in green_obs: 
        if nearest_obs[0][1] < obs[0][1]: 
            nearest_obs = obs
            nearest_obs[1] = 'green'
    return nearest_obs

def decide_path():
    global min_dist, prev_dist, prev_str, prev_obs, turning
    # red obstacle --> left; green obstacle --> right
    speed = 95                                  # Low speed for testing
    steering = prev_str
    path = 'Straight'
    current_obs = nearest_obstacle()
    x, y, w, h = current_obs[0][0], current_obs[0][1], current_obs[0][2], current_obs[0][3]
    obs_colour = nearest_obstacle[1]
    # All these values are currently estimates, need to be updated later with data
    image_centre = 600      # Obstacles have some size, so 640 is reached easily
    min_offset = 100        # Offset x-coordinate of the obstacle from centre to pass it
    max_offset = 400
    if not turning:
        '''Old buggy code
        if prev_obs[1]!= current_obs[1]:
            min_dist, prev_dist, prev_str = 3, distance, steering
            print("\n\n\nPassed Obstacle\n\n\n")
        if distance - prev_dist > min_dist:
            min_dist, prev_dist = 0, distance
            print(f'\nThe current obstacle to tackle is {current_obs}')
            x, y = (current_obs[0][0] + current_obs[0][2])/2, current_obs[0][1] 
            if y > 35:
                if current_obs[1] == 'red': steering, path = 90 - max(x*(100-y)/620*2.1,0), 'Left'
                elif current_obs[1] == 'green': steering, path = 90 + max(x*(85-y)/620*1.7,0), 'Right'
        '''
        norm_y = (current_obs - 350) / (720 - 350)
        proximity = 1 - math.exp(-1 * norm_y)           # Exponential antilog-style non-linear scaling of y wrt distance
        offset_x = min_offset + (max_offset - min_offset) * proximity # Calculate the ideal position of the obstacle for this case
        if obs_colour == 'red': target_x = 600 + offset_x         # 
        elif obs_colour == 'green': target_x = 600 - offset_x
        target_x = max(0, min(1200, target_x)) # Set limits on the target
        delta = target_x - x
        if delta >= 0: k = 60/600
        elif delta < 0: k = 90/600
        steering = 90 + k * delta

    elif turning:
        pass        # We'll look into this later

    # Weird steering - 0°->150° (About 65° each way)!!!
    steering = max(min(steering,150),1)         # Set limits for steering value
    prev_obs = current_obs

    return path, speed, steering


def drive_data(motor_speed,servo_steering):
    # It sends driving commands to RP2040 and gets back sensor data
    global yaw, distance, left_dist, front_dist, right_dist
    # Send command
    command = f"{motor_speed},{servo_steering}\n"
    ser.write(command.encode())

    # Wait for response from RP2040
    response = ser.readline().decode().strip()
    values = response.split(",")[0:]
    yaw = float(values[0])
    distance = -float(values[14]) / 43
    left_dist = int(values[12])
    front_dist = int(values[9])
    right_dist = int(values[10])
    print(f"Received Data - Yaw: {yaw}, Distance: {distance} Left: {left_dist}, Front: {front_dist}, Right: {right_dist}\n")


try:
    init()
    global frame, hsv_frame, corrected_frame, hsv_roi
    global red_obs, green_obs
    global yaw, distance, left_dist, right_dist, front_dist
    while True:
        # Read a frame from the camera
        frame = picam2.capture_array()
        corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        hsv_roi = hsv_frame[350:715, 0:1280]        # Our region of interest is only the bottom half of the camera feed

        # Process camera frame for obstacles
        frame_processed, red_obs, green_obs = process_frame()

        # Decide navigation based on obstacle detection
        path_action, speed, steering = decide_path()

        # Send the command to the peripherals board and get back data from sensors
        print(f'Steering: {steering}, Speed: {speed}')
        drive_data(speed,steering)

        # Show data on camera feed
        cv2.putText(frame_processed, f"Path:{path_action} Steering:{steering}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("Obstacle Detection", frame_processed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    command = f"0,90\n"
    ser.write(command.encode())
    picam2.stop_preview()
    picam2.stop()
    cv2.destroyAllWindows()