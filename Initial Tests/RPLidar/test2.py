import time
from math import floor
from adafruit_rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

scan_data = [0]*360
last_print_time = 0
PRINT_INTERVAL = 1.0  # seconds

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
       
        current_time = time.time()
        if current_time - last_print_time >= PRINT_INTERVAL:
            for i in range(360):
                if i % 10 == 0:
                    print(f'Angle: {i}, Distance: {scan_data[i]}') 
            last_print_time = current_time

except KeyboardInterrupt:
    print('Stopping.')
finally:
    lidar.stop()
    lidar.disconnect()