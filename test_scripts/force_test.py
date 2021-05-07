#! /usr/bin/env python3
from time import sleep
import argparse
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice
from gpiozero import SmoothedInputDevice


sensor_1 = DigitalInputDevice(26) # USED BROADCOM ADDRESSING
sensor_2 = DigitalInputDevice(19) # USED BROADCOM ADDRESSING
sensor_3 = DigitalInputDevice(13) # USED BROADCOM ADDRESSING

#sensor_1 = SmoothedInputDevice(26,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING
#sensor_2 = SmoothedInputDevice(19,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING
#sensor_3 = SmoothedInputDevice(13,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING

while True:
    print("Sensor 1: ",sensor_1.is_active)
    print("Sensor 2: ",sensor_2.is_active)
    print("Sensor 3: ",sensor_3.is_active)
    sleep(0.05)