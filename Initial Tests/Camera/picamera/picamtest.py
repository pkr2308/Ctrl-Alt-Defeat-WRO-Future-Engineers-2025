#This just shows input from the picam

from picamzero import Camera
from time import sleep

cam = Camera()
cam.start_preview()

sleep(5)

cam.stop_preview()
