import RPi.GPIO as GPIO
import time
import threading

forward = 17
backward = 18
encoder_a = 23
encoder_b = 24

cur_time = time.time()

encoderValue = 0
prevEncVal = 0
speed = 0
adjspeed = 0

input_received = False
breaker = False
stop = False

rpm = 0.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(forward, GPIO.OUT)
GPIO.setup(backward, GPIO.OUT)
GPIO.setup(encoder_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encoder_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
direction = 0

def update_speed():
    global speed, input_received, breaker, adjspeed
    user_input = input("")
    if user_input == 'exit':
        print("Exiting")
        breaker = True
    else:
        speed = int(user_input)
        adjspeed = speed
        input_received = True
    
def updateEncoder(channel = 0):
    global encoderValue
    
    if direction == -1: encoderValue-=1
    if direction == 1: encoderValue+=1
    
GPIO.add_event_detect(encoder_a, GPIO.BOTH, callback = updateEncoder)

try:
    while True:
        input_thread = threading.Thread(target = update_speed)
        input_thread.daemon = True
        input_thread.start()
        prev_time = cur_time
        prevEncVal = encoderValue
        
        if input_received:
            print(f"Speed {speed}")
            rpm = speed * 0.8
            input_received = False
        
        if speed > 0:
            GPIO.output(forward, GPIO.HIGH)
            GPIO.output(backward, GPIO.LOW)
            direction = 1
        elif speed < 0:
            GPIO.output(forward, GPIO.LOW)
            GPIO.output(backward, GPIO.HIGH)
            direction = -1
        stop = False
        target = (abs(speed) * 0.8 - rpm)
        if adjspeed >= 0:
            if target > 0.75:adjspeed += 1
            elif target < -0.75 and adjspeed > 0: adjspeed -= 1
        print(adjspeed)
        for i in range(0,50):
            if i > (abs(adjspeed) + (100 - abs(adjspeed))*0.20)/2 and stop == False:
                GPIO.output(forward, GPIO.LOW)
                GPIO.output(backward, GPIO.LOW)
                direction = 0
                stop = True
            time.sleep(0.001)
        cur_time = time.time()
        time_diff = cur_time - prev_time
        rpm = (abs(encoderValue) - abs(prevEncVal)) / (420 * time_diff) * 60
        print(f"RPM : {rpm}")
        
        
        if breaker == True:
            break
            
finally:
    GPIO.output(forward, GPIO.LOW)
    GPIO.output(backward, GPIO.LOW)
    GPIO.cleanup()