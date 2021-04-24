#! /usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
#GPIO.setup(33,GPIO.OUT)

motors = GPIO.PWM(32,4000)
try:
    motors.start(10)
    sleep(0.1)
    motors.start(20)
    sleep(0.1)
    motors.start(30)
    sleep(0.1)
    motors.start(40)
    sleep(0.1)
    motors.start(50)
    sleep(0.1)
    motors.start(60)
    sleep(0.1)
    motors.start(70)
    sleep(0.1)
    motors.start(80)
    sleep(0.1)
    motors.start(90)
    sleep(0.1)
    motors.start(100)
    sleep(2)

    motors.start(80)
    sleep(0.1)
    motors.start(60)
    sleep(0.1)
    motors.start(40)
    sleep(0.1)
    motors.start(20)
    sleep(0.1)
    motors.start(10)
    sleep(0.1)
    motors.stop()
except KeyboardInterrupt:
    GPIO.cleanup()
    
GPIO.cleanup()
print("done!")