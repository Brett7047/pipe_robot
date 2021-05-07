#! /usr/bin/env python3
# Date Created : Spring 2021
# Author : Brett Ryan, James Lechak ryanbm@sunypoly.edu, lechakj@sunypoly.edu
# Language: Python3.7.3

from time import sleep
import argparse
import os
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice
from gpiozero import AngularServo
from picamera import PiCamera
import gps # class file for gps

MM_PER_TICK = 0.1636246 #millimeters per tick of encoder(using 1 wire, 32 CPR)
INCH_PER_TICK = MM_PER_TICK * 0.0393701 # inches per tick
ROBOT_LENGTH_IN_TICKS = (16/INCH_PER_TICK) # length of robot from camera to center of gyro membrane

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
                    self.total_ticks = self.total_ticks - 1 # decrement ticks(backward)
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
    servo = AngularServo(17,min_angle=-45,max_angle=45)
    motor = PhaseEnableMotor(16,12) # dir,spd
    sensor_1 = DigitalInputDevice(26) 
    sensor_2 = DigitalInputDevice(19)
    sensor_3 = DigitalInputDevice(13)
    camera = PiCamera()
    camera.resolution = (640,480) #set resolution

    enc_1 = encoders(rotary_encoder1,motor) # initializes class
    enc_2 = encoders(rotary_encoder2,motor) # initializes class
    gps = gps.GPS() # initializes gps class from gps.py

    # argument parser for command line inputs
    parser = argparse.ArgumentParser(description="Drive a specified distance")
    parser.add_argument("distance", help="Enter a distance(in feet)",type=float)
    parser.add_argument("speed",help="Enter speed(0-1)",type=float)
    parser.add_argument("bearing",help="Enter the heading for the robot(0-360)",type=int)
    parser.add_argument("-b","--goback",help="Tells the robot to not return to start. Returns by default",
                        action="store_false",default=True)
    parser.add_argument("-nd","--nodetect",help="Tells the robot to ignore sensor readings and just drive",
                        action="store_true")
    args=parser.parse_args()

    distance = args.distance
    speed = args.speed
    if (args.bearing > 360) or (args.bearing < 0):
        print("invalid bearing, please input 0-360")
        exit()
    else:
        bearing = args.bearing

    # variables used in while loop
    slow_down_1 = False # used to slow robot nearing end of pipe
    slow_down_2 = False # used to slow robot nearing end of pipe
    detected_leak = False
    ticks_traveled = 0 # total ticks traveled. Incremented forward, decrememnted reverse
    leak_detected_ticks = 0 # ticks at detected leak
    gps_capture_increment = 1000 # tick intervals to record gps point(s)

    # these 2 OS calls manipulate the image captured. Downsizing it, then converting to .wav
    # they are called in the leak detected section.
    osCall1 = "convert leak.jpg -resize 320x240\! leak_resize.jpg" 
    osCall2 = "pysstv --mode Robot36 /home/pi/seniordesign/leak_resize.jpg /home/pi/seniordesign/leak.wav" 

    ticks_to_travel = (distance*12) / INCH_PER_TICK
    gps.get_initial_point() # hangs until initial point is acquired
    print("Distance to travel: ",distance," feet. Ticks to travel: ",ticks_to_travel," ticks.")
    print("Going to move ",ticks_to_travel,"ticks at ",speed," speed. Starting in 5 seconds")
    sleep(5)

    motor.forward(speed) # begins the motor moving forward
    # MAIN LOOP 
    while ticks_to_travel > ticks_traveled:

        # encoder work
        enc_1.encoder_run() # encoder 1 function. Call as fast as possible for highest accuracy
        enc_2.encoder_run() # encoder 2 function. Call as fast as possible for highest accuracy
        ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks
        # incremental GPS captures
        if ticks_traveled > gps_capture_increment: # grabs GPS coordinate every 1000 ticks
            print("computing gps point at: ",(ticks_traveled*MM_PER_TICK),"mm")
            gps.compute_latlon(bearing,(ticks_traveled*MM_PER_TICK))
            gps_capture_increment = gps_capture_increment + 1000

        # detected leak
        if ((sensor_1.value == 1 or sensor_2.value == 1 or sensor_3.value == 1) and
        detected_leak == False and args.nodetect == False):
            print("DETECTED LEAK")
            osCall1 = "convert leak.jpg -resize 320x240\! leak_resize.jpg"
            detected_leak = True # prevents robot from detecting another leak after this. 
            motor.stop()
            sleep(2)
            # the robot now needs to go back exactly 16 inches because the gryo assembly sits in the rear of the robot, and
            # the camera is in the front. This makes the robot reverse 16 inches so the camera can capture an image of the leak
            ticks_to_reverse_to = ticks_traveled- ROBOT_LENGTH_IN_TICKS
            print("Backing up to leak for camera to be centered on leak")
            motor.backward(0.1)
            while ticks_traveled > ticks_to_reverse_to:
                enc_1.encoder_run() # encoder 1 function. Call as fast as possible for highest accuracy
                enc_2.encoder_run() # encoder 2 function. Call as fast as possible for highest accuracy
                ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks
            motor.stop()

            # Gps work
            leak_detected_ticks = ticks_traveled # records point of detected leak
            gps.compute_latlon(bearing,(leak_detected_ticks*MM_PER_TICK),leak=True) # records gps point at leak.
            #Camera Work
            camera.capture("leak.jpg")
            print("Capturing image, resizing and converting to .wav!")
            os.system(osCall1) # resizes image to usable format with pysstv
            os.system(osCall2) # converts resized image to .wav file
            # here is where we would have sent data over radio had we gotten that working.
            sleep(2)
            motor.forward(speed)


        # this section slows down the robot when it nears the end of its specified distance.
        # its mainly here to prevent really hard stops and possible slippage when at high speeds
        if ticks_to_travel - ticks_traveled < 2000 and slow_down_1 is False and speed > 0.5:
            speed = 0.5
            motor.forward(speed)
            slow_down_1 = True
            print("slowing down 1!")
        elif ticks_to_travel - ticks_traveled < 1000 and slow_down_2 is False and speed > 0.2:
            motor.forward(0.2)
            slow_down_2 = True
            print("slowing down 2!")
        else:
            pass


    # Returns to start. Decrements counted ticks until it hits 0 so that it
    # returns to starting location exactly. Does not do any force sensing when
    # returning to start.
    if args.goback == True:
        speed = args.speed
        slow_down_1 = False
        slow_down_2 = False
        motor.stop()
        sleep(2)
        print("Returning to start!")
        motor.backward(speed)
        while ticks_traveled > 0: 
            enc_1.encoder_run()
            enc_2.encoder_run()
            ticks_traveled = ((enc_1.encoder_total_ticks() + enc_2.encoder_total_ticks()) / 2) # avg of ticks
            if ticks_traveled < 2000 and slow_down_1 is False and speed > 0.5:
                speed = 0.5
                motor.backward(speed)
                slow_down_1 = True
                print("slowing down 1!")
            elif ticks_traveled < 1000 and slow_down_2 is False and speed > 0.2:
                motor.backward(0.2)
                slow_down_2 = True
                print("slowing down 2!")
            else:
                pass


    if detected_leak is True:
        print("Detected leak at: ", (leak_detected_ticks * INCH_PER_TICK)," inches.")
    

    # input gps map data to https://www.gpsvisualizer.com/map_input

