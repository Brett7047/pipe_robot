#! /usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep

#PIN 32 = PWM OUTPUT
#PIN 36 = MOTOR DIR OUTPUT

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(36.GPIO.OUT)

motors = GPIO.PWM(32,4000)

motors.start(10)
sleep(2)
GPIO.OUTPUT(36,True)
