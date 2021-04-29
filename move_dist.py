#! /usr/bin/env python3
from time import sleep
import argparse
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice
#from gpiozero import SmoothedInputDevice

#1 tick = 0.1636246mm
TICK_DISTANCE_MM = 0.1636246 #millimeters per tick of encoder(using 1 wire)
TICK_DISTANCE_INCHES = TICK_DISTANCE_MM * 0.0393701 # inches per tick

if __name__ == "__main__":

    rot_enc1 = DigitalInputDevice(7) # USED BROADCOM ADDRESSING
    rot_enc2 = DigitalInputDevice(8) # USED BROADCOM ADDRESSING
    motor = PhaseEnableMotor(16,12) # USED BROADCOM ADDRESSING

    sensor_1 = DigitalInputDevice(26) # USED BROADCOM ADDRESSING
    sensor_2 = DigitalInputDevice(19) # USED BROADCOM ADDRESSING
    sensor_3 = DigitalInputDevice(13) # USED BROADCOM ADDRESSING
    #sensor_1 = SmoothedInputDevice(26,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING
    #sensor_2 = SmoothedInputDevice(19,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING
    #sensor_3 = SmoothedInputDevice(13,threshold=0.5,queue_len=1) # USED BROADCOM ADDRESSING

    detected_leak = False
    first_tick = True
    ticks_traveled = 0
    enc_1_ticks = 0
    enc_2_ticks = 0
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
    print("Going to move ",ticks_to_travel,"ticks at ",speed," speed. Starting in 2 seconds")
    sleep(10)
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
                        enc_1_ticks =  enc_1_ticks + 1 # adds 1 to start always because there is no prev tick yet
                        enc_2_ticks =  enc_2_ticks + 1 # adds 1 to start always because there is no prev tick yet
        else: # every iteration after first
            # first encoder
            enc_1_value = rot_enc1.value # reads GPIO pin 7 for rotary encoder level
            if enc_1_value != prev_tick_1: # if the prev level and current are different(meaning it changed)
                enc_1_ticks = enc_1_ticks + 1 # increment ticks(forward)
                prev_tick_1 = enc_1_value # last value = current value
            # second encoder    
            enc_2_value = rot_enc2.value # reads GPIO pin 8 for rotary encoder level
            if enc_2_value != prev_tick_2: # if the prev level and current are different(meaning it changed)
                enc_2_ticks = enc_2_ticks + 1 # increment ticks(forward)
                prev_tick_2 = enc_2_value # last value = current value

            ticks_traveled = ((enc_1_ticks + enc_2_ticks) / 2) # avg of ticks
            #print(ticks_traveled)
        '''
        if sensor_1.is_active == True or sensor_2.is_active == True or sensor_3.is_active == True:
            detected_leak = True
            print("DETECTED LEAK")
            motor.stop()
            break
        else:
            print("Got here every time")   
        '''
        if sensor_1.value == 1 or sensor_2.value == 1 or sensor_3.value == 1:
            detected_leak = True
            print("DETECTED LEAK")
            motor.stop()
            break

    if detected_leak is True:
        print("out of loop, because we detected leak!")
    else:
        print("done!")    



    #print("Ticks traveled is: ",ticks_traveled)
    #print("encoder 1 ticks: ",enc_1_ticks)
    #print("encoder 2 ticks: ",enc_2_ticks)