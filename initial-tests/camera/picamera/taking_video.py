from picamzero import Camera
import os
from datetime import datetime

home_dir = os.environ['HOME']
cam = Camera()

cam.start_preview()
cam.annotate(str(datetime.now()))
cam.record_video(f"{home_dir}/Documents/Camera/picamzero/timestamps.mp4", duration=5)
#5 sec video
cam.stop_preview()
