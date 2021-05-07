#! /usr/bin/env python3
import time
import board
import math
from numpy import zeros
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from adafruit_lis3mdl import LIS3MDL
from ahrs.filters import AQUA
from ahrs import Quaternion

# initializing i2c and device
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = LSM6DS33(i2c)
mag = LIS3MDL(i2c)
# arrays for sensor readings

gyro_arr = zeros((1,3))
acc_arr = zeros((1,3))
mag_arr = zeros((1,3))
samp_start = 0
samp_end = 0
while True:
    samp_start = time.time()
    mag_arr[0] = mag.magnetic
    gyro_arr[0] = sensor.gyro
    acc_arr[0] = sensor.acceleration
    samp_end = time.time()
    total_samp_time = samp_end-samp_start
    aqua  = AQUA(gyr = gyro_arr, acc = acc_arr, mag = mag_arr , frequency = 1/total_samp_time)
    #print(aqua.Q)
    qq = Quaternion(aqua.Q[0])
    #print(qq)
    rad_q = qq.to_angles()
    deg_q = ((rad_q*180)/math.pi)
    print(deg_q)

    time.sleep(0.01)