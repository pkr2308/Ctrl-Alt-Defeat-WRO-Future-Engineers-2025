from math import floor
from adafruit_rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME, timeout=3)

def process_data(data):
    for i in range(360):
        print(f'Angle: {i}, Distance: {data[i]}')
        if data[i] < 100 and data[i] != 0:
            print("Nearby object detected!") 
    #print(f'Data - {data} \n')

scan_data = [0]*360
try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)

except KeyboardInterrupt:
    print('Stopping.')
finally:
    lidar.stop()
    lidar.disconnect()