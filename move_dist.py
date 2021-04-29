#! /usr/bin/env python3

from time import sleep
import argparse
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice

TICK_DISTANCE_MM = 0.1636246 #millimeters per tick of encoder(using 1 wire, 32 CPR)
TICK_DISTANCE_INCHES = TICK_DISTANCE_MM * 0.0393701 # inches per tick

if __name__ == "__main__":

    # initilization of GPIO pins
    # All of these use BROADCOM adressing for their pins.
    rot_enc1 = DigitalInputDevice(7) 
    rot_enc2 = DigitalInputDevice(8) 
    motor    = PhaseEnableMotor(16,12) # dir,spd
    sensor_1 = DigitalInputDevice(26) 
    sensor_2 = DigitalInputDevice(19)
    sensor_3 = DigitalInputDevice(13)

    detected_leak = False
    first_tick = True
    ticks_traveled = 0
    enc1_total_ticks = 0
    enc2_total_ticks = 0
    enc_1_value = 0
    enc_2_value = 0
    prev_tick_1 = 0
    prev_tick_2 = 0

    parser = argparse.ArgumentParser(description="Drive a specified distance")
    parser.add_argument("distance", help="Enter a distance(in feet)",type=float)
    parser.add_argument("speed",help="Enter speed(0-1)",type=float)
    parser.add_argument("direction",help="Enter direction 1 or 0", type=int)

    args=parser.parse_args()
    distance = args.distance
    direction = args.direction
    speed = args.speed

    ticks_to_travel = (distance*12) / TICK_DISTANCE_INCHES
    print("Total ticks needed to travel: ",distance," feet is: ",ticks_to_travel)
    print("Going to move ",ticks_to_travel,"ticks at ",speed," speed. Starting in 5 seconds")
    sleep(5)

    if direction == 1:
        motor.forward(speed)
    elif direction == 0:
        motor.backward(speed)

    while ticks_to_travel > ticks_traveled:
        if first_tick == True: # first iteration
                    first_tick = False 
                    prev_tick_1 = rot_enc1.value # reads GPIO pin 7
                    prev_tick_2 = rot_enc2.value # reads GPIO pin 8
                    if motor.value > 0: # makes sure motor is moving forward
                        enc1_total_ticks =  enc1_total_ticks + 1 # adds 1 to start always because there is no prev tick yet
                        enc2_total_ticks =  enc2_total_ticks + 1 # adds 1 to start always because there is no prev tick yet
        else: # every iteration after first
            # first encoder
            enc_1_value = rot_enc1.value # reads GPIO pin 7 for rotary encoder level
            if enc_1_value != prev_tick_1: # if the prev level and current are different(meaning it changed)
                enc1_total_ticks = enc1_total_ticks + 1 # increment ticks(forward)
                prev_tick_1 = enc_1_value # last value = current value
            # second encoder    
            enc_2_value = rot_enc2.value 
            if enc_2_value != prev_tick_2: 
                enc2_total_ticks = enc2_total_ticks + 1 
                prev_tick_2 = enc_2_value

            ticks_traveled = ((enc1_total_ticks + enc2_total_ticks) / 2) # avg of ticks
            #print(ticks_traveled)

        if sensor_1.value == 1 or sensor_2.value == 1 or sensor_3.value == 1:
            detected_leak = True
            print("DETECTED LEAK")
            motor.stop()
            break

    if detected_leak is True:
        print("out of loop, because we detected leak!")
    else:
        print("done!")    
