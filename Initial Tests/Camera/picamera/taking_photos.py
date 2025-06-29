from picamzero import Camera
import os

home_dir = os.environ['HOME']
cam = Camera()

cam.start_preview()
#cam.take_photo(f"{home_dir}/Documents/Camera/picamzero/new_img.jpg")
cam.capture_sequence(f"{home_dir}/Documents/Camera/picamzero/new_img.jpg", num_images=4, interval =2)
#4 images in 2 secs
cam.stop_preview()
