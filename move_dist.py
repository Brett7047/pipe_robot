#! /usr/bin/env python3
from time import sleep
import argparse
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice

TICK_DISTANCE_MM = 0.1636246 #millimeters per tick of encoder(using 1 wire, 32 CPR)
TICK_DISTANCE_INCHES = TICK_DISTANCE_MM * 0.0393701 # inches per tick

class encoders():
    def __init__(self,enc,motor):
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

    enc_1 = encoders(rotary_encoder1,motor) # initializes class
    enc_2 = encoders(rotary_encoder2,motor) # initializes class

    detected_leak = False
    ticks_traveled = 0

    # argument parser for command line inputs
    parser = argparse.ArgumentParser(description="Drive a specified distance")
    parser.add_argument("distance", help="Enter a distance(in feet)",type=float)
    parser.add_argument("speed",help="Enter speed(0-1)",type=float)
    parser.add_argument("-b","--goback",help="Tells the robot to return to start after driving to dist",
                        action="store_true")
    args=parser.parse_args()
    distance = args.distance
    speed = args.speed


    ticks_to_travel = (distance*12) / TICK_DISTANCE_INCHES
    print("Total ticks needed to travel: ",distance," feet is: ",ticks_to_travel)
    print("Going to move ",ticks_to_travel,"ticks at ",speed," speed. Starting in 5 seconds")
    sleep(5)

    motor.forward(speed) # begins the motor moving forward

    # MAIN LOOP 
    while ticks_to_travel > ticks_traveled:
        enc_1.encoder_run()
        enc_2.encoder_run()
        ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks

        if sensor_1.value == 1 or sensor_2.value == 1 or sensor_3.value == 1:
            detected_leak = True
            print("DETECTED LEAK")
            motor.stop()
            """
            ******************************************************
            ******************************************************
            DO CAMERA STUFF, RECORD LOCATIONs, TRANSMIT STUFF HERE
            ******************************************************
            ******************************************************
            """
            # TODO - After leak detection, code in logic to make sure it doesnt instantly 
            # throw another ddetection when it begins driving to the end of the pipe. AKA
            # make sure the next X ticks dont throw a sensor. Probably hard code a specified
            # distance in ticks to drive to release the membrane from the vaccum, then begin
            # sensing again OR just stop sensing after a single detection(cheatsies)
            break

    # Go back to start if -b specified on cmd line
    # Does not do any leak detecting when returning to start.
    if args.goback == True:
        print("Returning to start!")
        motor.backward(speed)
        while ticks_traveled > 0:
            enc_1.encoder_run()
            enc_2.encoder_run()
            ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks



    if detected_leak is True:
        print("out of loop, because we detected leak!")
    else:
        print("done!")    
