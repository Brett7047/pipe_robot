# class to use motors with raspberry pi4b. Contains functions to move them in 
# certain directions, as well as stop and start and alter speed. Uses basic RPi.GPIO 
# libraries.

import RPi.GPIO as GPIO



class Motors(GPIO_PWM,GPIO_DIR,PWM_FREQ):

    def __init__(self):
        self.pwm_pin = GPIO_PWM
        self.dir_pin = GPIO_DIR
        self.freq = PWM_FREQ
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin,GPIO.OUT)
        self.m = GPIO.PWM(self.pwm_pin,self.freq)
        self.last_dir = False
        self.last_speed = 0
        self.first_move = True

    def move(self,dir,speed):
        ''' 
        Inputs:
        dir     -> Forward = True, Backwards = False
        speed   -> number b/w 0-100 in increments of 5 
        The script checks to see if this is the first run, if it
        is then it initializes the direction pin, and begins the
        robot moving in the desired direction. If its the second
        time its called or further, it checks to see what dir
        and slows the robot and then moves in the opposite direction.
        This is to prevent hard stopping on the motors.
        '''
        if self.first_move is True
            self.first_move = False       
            if dir == True:
                GPIO.output(self.dir_pin,True)
                self.last_dir = True
            elif dir == False:
                GPIO.output(self.dir_pin,False)
                self.last_dir = False
            if speed (speed > 100) or ((speed % 5) != 0):
                return("Incorrect speed. Please input a number between 0 and 100 divisible by 5")
                else: # move robot for first time
                    
        else
            if dir != self.last_dir:
                self.last_dir = not self.last_dir


                GPIO.output(self.dir_pin,dir)




        else:
            if speed > 20
                while speed > 20:
                        
