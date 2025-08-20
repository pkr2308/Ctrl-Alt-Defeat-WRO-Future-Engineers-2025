import time
from adafruit_rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'  # RPLidar USB port (check with ls /dev/ttyUSB*)

lidar = RPLidar(None, PORT_NAME)

try:
    print('Starting scan...')
    for scan in lidar.iter_scans():
        print('Scan received:')
        for (_,angle,distance) in scan:
            print('Angle: %0.1f, Distance: %0.1f' % (angle, distance))
        time.sleep(1)
except KeyboardInterrupt:
    print('Stopping.')
finally:
    lidar.stop()
    lidar.disconnect()
