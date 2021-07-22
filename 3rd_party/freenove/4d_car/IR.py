# Class of Line_Tracking is made by Freenove.
# The run definition is heavily modified and the motors are not in this code.
# This responds to contrast of light and dark.

import time
import RPi.GPIO as GPIO
class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01,GPIO.IN)
        GPIO.setup(self.IR02,GPIO.IN)
        GPIO.setup(self.IR03,GPIO.IN)

    def infrared(self,position):
        if position == 1:
            if GPIO.input(self.IR01) == True:
                print("Left has been detected on bright")
        elif position == 2:
            if GPIO.input(self.IR02) == True:
                print("Middle has been detected on bright")
        elif position == 3:
            if GPIO.input(self.IR03) == True:
                print("Right has been detected on bright")
        else:
            print("Nothing has been detected.")

IR_read=Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':

    while True:
        IR_read.infrared(2)
