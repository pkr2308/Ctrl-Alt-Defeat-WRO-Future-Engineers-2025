from picamzero import Camera
import os

home_dir = os.environ['HOME']
cam = Camera()
cam.vid_size = (1600, 900)
#cam.vid_size = (3200, 1800)

cam.start_preview()
cam.record_video(f"{home_dir}/Documents/Camera/picamzero/resolutions2.mp4", duration=5)
cam.stop_preview()
