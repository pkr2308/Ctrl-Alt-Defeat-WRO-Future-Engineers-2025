# This is used with the launcher file to blink an LED twice after startup of Pi is complete 
# In a similar manner, the final program files too will be run

import RPi.GPIO as GPIO
from time import sleep

led = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)

duration=1

try:
    for i in range(0,2):
        GPIO.output(led, GPIO.HIGH)
        sleep(duration)
        GPIO.output(led, GPIO.LOW)
        sleep(duration)

finally:
    GPIO.cleanup()
