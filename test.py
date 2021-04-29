#! /usr/bin/env python3
import threading
from time import sleep
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice

motor = PhaseEnableMotor(16,12) # USED BROADCOM ADDRESSING

def encoders(motor):
    rot_enc1 = DigitalInputDevice(7)
    rot_enc2 = DigitalInputDevice(8)
    rot_enc3 = DigitalInputDevice(25)
    rot_enc4 = DigitalInputDevice(24)
    enc_1_ticks = 0
    enc_2_ticks = 0
    enc_3_ticks = 0
    enc_4_ticks = 0
    enc_1_value = 0
    enc_2_value = 0
    enc_3_value = 0
    enc_4_value = 0
    prev_tick_1 = 0
    prev_tick_2 = 0
    prev_tick_3 = 0
    prev_tick_4 = 0
    first_tick = True

    while encoder_start_flag.is_set() == True:
        while motor_moving_flag.is_set() == True:
            # This part of the code reads the changes in the encoder. 
            # Im only using 1 pin instead of 2 pins per encoder to save on CPU usage
            # It should still give us 32*30 = 960 ticks per rotation of accuracy which equivalates
            # to(2*pi*25mm) / 960 ticks = 0.1636246mm per tick of precision. This means each tick is
            # a little over 0.16mm
            if first_tick == True: # first iteration
                    first_tick = False 
                    prev_tick_1 = rot_enc1.value # reads GPIO pin 7
                    prev_tick_2 = rot_enc2.value # reads GPIO pin 8
                    prev_tick_3 = rot_enc3.value # reads GPIO pin 25
                    prev_tick_4 = rot_enc4.value # reads GPIO pin 24
                    if motor.value > 0: # makes sure motor is moving forward
                        enc_1_ticks =  enc_1_ticks + 1 # adds 1 to start always because there is no prev tick yet
                        enc_2_ticks =  enc_2_ticks + 1 # adds 1 to start always because there is no prev tick yet
                        enc_3_ticks =  enc_3_ticks + 1 # adds 1 to start always because there is no prev tick yet
                        enc_4_ticks =  enc_4_ticks + 1 # adds 1 to start always because there is no prev tick yet

            else: # every iteration after first
                # first encoder
                enc_1_value = rot_enc1.value # reads GPIO pin 7 for rotary encoder level
                if enc_1_value != prev_tick_1: # if the prev level and current are different(meaning it changed)
                    if motor.value > 0: # moving forward
                        enc_1_ticks = enc_1_ticks + 1 # increment ticks(forward)
                    elif motor.value < 0: # moving backward
                        enc_1_ticks = enc_1_ticks - 1 #decrement ticks(backward)
                    else:
                        print("Stopped!")
                    prev_tick_1 = enc_1_value # last value = current value
                # second encoder    
                enc_2_value = rot_enc2.value # reads GPIO pin 8 for rotary encoder level
                if enc_2_value != prev_tick_2: # if the prev level and current are different(meaning it changed)
                    if motor.value > 0: # moving forward
                        enc_2_ticks = enc_2_ticks + 1 # increment ticks(forward)
                    elif motor.value < 0: # moving backward
                        enc_2_ticks = enc_2_ticks - 1 #decrement ticks(backward)
                    else:
                        print("Stopped!")
                    prev_tick_2 = enc_2_value # last value = current value
                '''
                # third encoder
                enc_3_value = rot_enc3.value # reads GPIO pin 25 for rotary encoder level
                if enc_3_value != prev_tick_3: # if the prev level and current are different(meaning it changed)
                    if motor.value > 0: # moving forward
                        enc_3_ticks = enc_3_ticks + 1 # increment ticks(forward)
                    elif motor.value < 0: # moving backward
                        enc_3_ticks = enc_3_ticks - 1 #decrement ticks(backward)
                    else:
                        print("Stopped!")
                    prev_tick_3 = enc_3_value # last value = current value

                # fourth encoder
                enc_4_value = rot_enc4.value # reads GPIO pin 24 for rotary encoder level
                if enc_4_value != prev_tick_4: # if the prev level and current are different(meaning it changed)
                    if motor.value > 0: # moving forward
                        enc_4_ticks = enc_4_ticks + 1 # increment ticks(forward)
                    elif motor.value < 0: # moving backward
                        enc_4_ticks = enc_4_ticks - 1 #decrement ticks(backward)
                    else:
                        print("Stopped!")
                    prev_tick_4 = enc_4_value # last value = current value
                '''
    print("Total counted encoder 1 ticks: ",enc_1_ticks)
    print("Total counted encoder 2 ticks: ",enc_2_ticks)
    #print("Total counted encoder 3 ticks: ",enc_3_ticks)
    #print("Total counted encoder 4 ticks: ",enc_4_ticks)
    print("Exiting Encoder Thread!")

def main(motor):
    encoder_start_flag.set()
    motor.forward(0.1)
    motor_moving_flag.set()
    print("Motor started!")
    sleep(4)
    motor.stop()
    motor_moving_flag.clear()
    encoder_start_flag.clear()
        

if __name__ == "__main__":

    encoder_start_flag = threading.Event() # flag for encoder thread
    motor_moving_flag = threading.Event() #flag for encoder thread

    encoder_thread = threading.Thread(target=encoders,args=(motor,))
    main_thread = threading.Thread(target=main,args=(motor,))
    encoder_thread.start()
    main_thread.start()

    main_thread.join()
    encoder_thread.join()

    motor.stop()
