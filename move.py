#! /usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(33,GPIO.OUT)

motor_1_pwm = GPIO.PWM(36,1000)
try:
    motor_1_pwm.start(10)
    sleep(1)
    motor_1_pwm.stop()
except KeyboardInterrupt:
    GPIO.cleanup()
    
GPIO.cleanup()
print("done!")