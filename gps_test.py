#! /usr/bin/env python3

import serial
import pynmea2

gps = serial.Serial(port="/dev/ttyACM0",baudrate=9600,timout=1)
gps.open()
acquired_initial_gps = False
while acquired_initial_gps == False:
    gps_data = pynmea2.parse(gps.readline().decode())
    if gps_data.sentence_type != "GGA":
        pass
    else:
        print(gps_data.latitude)
        print(gps_data.longitude)
        break
print("out of loop, and hopefully have stuff")