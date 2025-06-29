import RPi.GPIO as GPIO
from time import sleep

forward = 17
backward = 18
encoder_a = 23
encoder_b = 24

encoderValue = 0
target = 615
GPIO.setmode(GPIO.BCM)
GPIO.setup(forward, GPIO.OUT)
GPIO.setup(backward, GPIO.OUT)
GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
rpm = 0
direction = 0

def updateEncoder(channel = 0):
    global encoderValue
    if direction == -1: encoderValue-=1
    if direction == 1: encoderValue+=1
    
GPIO.add_event_detect(encoder_a, GPIO.BOTH, callback = updateEncoder)

def start_motor():
    if direction == 1:
        GPIO.output(forward, GPIO.HIGH)
        GPIO.output(backward, GPIO.LOW)
    elif direction == -1:
        GPIO.output(forward, GPIO.LOW)
        GPIO.output(backward, GPIO.HIGH)

def stop_motor():
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.LOW)
    
def turn(degrees=0, rotations=0, user_target = 0):
    global direction, encoderValue
    encoderValue = 0
    if user_target == 0: target = 1.69718*(degrees + 360*rotations) - 46.9
    #else: target = user_target
    print(f"Target: {target}")
    if target > 0: direction = 1
    elif target < 0: direction = -1
    start_motor()
    while abs(encoderValue) < target: pass
    stop_motor()
    print("Target achieved")


try:
    turn(90,0)
    
    
except KeyboardInterrupt:
    print("Exit")
    
finally:
    GPIO.cleanup()