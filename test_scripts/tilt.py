#! /usr/bin/env python3
import time
import board
from numpy import zeros
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from ahrs.filters import Tilt

# initializing i2c and device
i2c = board.I2C()
sensor = LSM6DS33(i2c)
acc_arr = zeros((1,3))

while True:
    acc_arr[0] = sensor.acceleration
    tilt = Tilt(acc=acc_arr, as_angles=True)
    print(not(round(tilt.Q[0,1],2))
    time.sleep(0.01)



