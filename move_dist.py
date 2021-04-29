#! /usr/bin/env python3

from time import sleep
import argparse
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice

TICK_DISTANCE_MM = 0.1636246 #millimeters per tick of encoder(using 1 wire, 32 CPR)
TICK_DISTANCE_INCHES = TICK_DISTANCE_MM * 0.0393701 # inches per tick

class encoders(self,enc,motor):
    def __init__:
        self.enc = enc #saving reference to encoder obj
        self.motor = motor # saving reference to motor obj
        self.first_tick = True
        self.prev_tick = 0
        self.enc_value_read = 0
        self.total_ticks = 0
        
    def encoder_run(self):
        if self.first_tick == True: # first iteration
            self.first_tick = False 
            self.prev_tick = self.enc.value # reads GPIO pin
            if self.motor.value > 0: # makes sure motor is moving forward
                self.total_ticks =  self.total_ticks + 1 # adds 1 to start always because there is no prev tick yet
            else:
                print("Motor not moving")
                return() # motors arent moving, dont do anything    
        else: # every iteration after first
            self.enc_value_read = self.enc.value # reads GPIO pin 7 for rotary encoder level
            if self.enc_value_read != self.prev_tick: # if the prev level and current are different(meaning it changed)
                if self.motor.value > 0:
                    self.total_ticks = self.total_ticks + 1 # increment ticks(forward)
                elif self.motor.value < 0:
                    self.total_ticks = self.total_ticks - 1 # increment ticks(forward)
                else:
                    print("Motor not moving")
                    return() # motors arent moving, dont do anything
            self.prev_tick = self.enc_value_read # last value = current value  

    def encoder_total_ticks(self):
        return(self.total_ticks)

if __name__ == "__main__":

    # Initilization of GPIO pins
    # All of these use BROADCOM adressing for their pins.
    rotary_encoder1 = DigitalInputDevice(7) 
    rotary_encoder2 = DigitalInputDevice(8) 
    motor = PhaseEnableMotor(16,12) # dir,spd
    sensor_1 = DigitalInputDevice(26) 
    sensor_2 = DigitalInputDevice(19)
    sensor_3 = DigitalInputDevice(13)

    enc_1 = encoders(rotary_encoder1,motor)
    enc_2 = encoders(rotary_encoder2,motor)

    detected_leak = False
    ticks_traveled = 0

    # argument parser for command line inputs
    parser = argparse.ArgumentParser(description="Drive a specified distance")
    parser.add_argument("distance", help="Enter a distance(in feet)",type=float)
    parser.add_argument("speed",help="Enter speed(0-1)",type=float)
    #parser.add_argument("direction",help="Enter direction 1 or 0", type=int)

    args=parser.parse_args()
    distance = args.distance
    direction = args.direction
    speed = args.speed

    ticks_to_travel = (distance*12) / TICK_DISTANCE_INCHES
    print("Total ticks needed to travel: ",distance," feet is: ",ticks_to_travel)
    print("Going to move ",ticks_to_travel,"ticks at ",speed," speed. Starting in 5 seconds")
    sleep(5)

    motor.forward(speed)

    # MAIN LOOP 
    while ticks_to_travel > ticks_traveled:
        enc_1.encoder_run()
        enc_2.encoder_run()
        print("Encoder 1 ticks: ",enc_1.encoder_total_ticks())
        print("Encoder 2 ticks: ",enc_2.encoder_total_ticks())
        ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks

        if sensor_1.value == 1 or sensor_2.value == 1 or sensor_3.value == 1:
            detected_leak = True
            print("DETECTED LEAK")
            motor.stop()
            break

    if detected_leak is True:
        print("out of loop, because we detected leak!")
    else:
        print("done!")    
