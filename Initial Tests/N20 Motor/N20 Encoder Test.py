import RPi.GPIO as GPIO
from time import sleep

forward = 17
backward = 18
encoder_a = 23
encoder_b = 24
pwm = 16
standby = 25

encoderValue = 0
lastEncoded = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(forward, GPIO.OUT)
GPIO.setup(backward, GPIO.OUT)
#GPIO.setup(pwm, GPIO.OUT)
#GPIO.setup(standby, GPIO.OUT)
GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

direction = 0

def updateEncoder(channel = 0):
    global lastEncoded, encoderValue
    #MSB = GPIO.input(encoder_a)
    '''
    LSB = GPIO.input(encoder_b)
    #print(f"MSB : {MSB}, LSB = {LSB}")
    encoded = (MSB << 1) | LSB
    sum = (lastEncoded << 2) | encoded
    
    #if sum in (0b0100, 0b1101,0b0010, 0b1011) and direction == -1:
    #    encoderValue-=1
    #elif sum in (0b1110,0b0111,0b0001,0b1000) and direction == 1:
    #    encoderValue+=1
        
    if sum in (0b0100, 0b1101,0b0010, 0b1011,0b1110,0b0111,0b0001,0b1000):
        if direction == -1: encoderValue-=1
        if direction == 1: encoderValue+=1
        
    lastEncoded = encoded
    
    if direction == -1: encoderValue-=1
    if direction == 1: encoderValue+=1
    '''
    
    encoderValue += 1
    
GPIO.add_event_detect(encoder_a, GPIO.BOTH, callback = updateEncoder)
#GPIO.add_event_detect(encoder_b, GPIO.BOTH, callback = updateEncoder, bouncetime = 1)

try:
    direction = -1
    #GPIO.output(standby, GPIO.HIGH)
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.HIGH)
    #GPIO.output(standby, GPIO.LOW)
    for i in range(0,15):
        print(encoderValue)
        encoderValue = 0
        sleep(0.5)
    #GPIO.output(standby, GPIO.HIGH)
    encoderValue = 0
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.LOW)
    sleep(0.5)
    print(encoderValue)
    #GPIO.output(standby, GPIO.LOW)
    
except KeyboardInterrupt:
    print("Exit")
    
finally:
    GPIO.cleanup()