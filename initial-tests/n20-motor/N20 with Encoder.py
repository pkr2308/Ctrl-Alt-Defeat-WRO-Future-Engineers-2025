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

try:
    direction = -1
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.HIGH)
    sleep(60)
    #while abs(encoderValue) < (target - 60): pass 
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.LOW)
    rpm = abs(encoderValue / 420)
    print(f"Encoder : {encoderValue}; RPM : {rpm}")
    
except KeyboardInterrupt:
    print("Exit")
    
finally:
    GPIO.cleanup()