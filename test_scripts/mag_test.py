#! /usr/bin/env python3

import time
import board
from adafruit_lis3mdl import LIS3MDL, Range
from math import atan2, degrees
i2c = board.I2C() # uses board.SCL and board.SDA
sensor = LIS3MDL(i2c)
Range.RANGE_4_GAUSS
'''
while True:

    for mag_range in [
    Range.RANGE_4_GAUSS,
    Range.RANGE_8_GAUSS,
    Range.RANGE_12_GAUSS,
    Range.RANGE_16_GAUSS,
    ]:
        sensor.range = mag_range
        print("Range: %d Gauss" % Range.string[sensor.range])
        mag_x, mag_y, mag_z = sensor.magnetic
        print("X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT".format(mag_x, mag_y, mag_z))
        print("")
        time.sleep(0.3)
'''
def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    return vector_2_degrees(magnet_x, magnet_y)


while True:
    print("heading: {:.2f} degrees".format(get_heading(sensor)))
    time.sleep(0.5)
