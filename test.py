#! /usr/bin/env python3
from time import time
from time import sleep
from gpiozero import PhaseEnableMotor
from gpiozero import DigitalInputDevice

rot_enc1 = DigitalInputDevice(7) 
motor = PhaseEnableMotor(16,12) # USED BROADCOM ADDRESSING
enc_ticks = 0
timeout = 0.1
'''
TODO currently it reads the value of the encoder input continuously,
which leads it to reading the same value over and over even when moving slow
i need to code it so that duplicate readings, or reading the same value twice doesnt
add a tick to the count. It should be giving me 480 ticks per rotation if I only count
the rising edge of the pin. then at 200rpm, thats 3.3rps, so ~ 1500 ticks per second at
the max speed it should go. Need work on this!
'''
try:
    start = time.time()
    motor.forward(0.1)
    while time.time() < start+timeout:

        enc_ticks =  enc_ticks + rot_enc1.value
    print("The motor went",enc_ticks" ticks!")

except KeyboardInterrupt:
    motor.stop()

#motor.(0.5)
