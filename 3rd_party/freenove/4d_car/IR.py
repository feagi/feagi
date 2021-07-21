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
    def run(self):
        while True:
            if GPIO.input(self.IR01)==True:
                print("Left")
            if GPIO.input(self.IR02)==True:
                print("Middle")
            if GPIO.input(self.IR03)==True:
                print("Right")

infrared=Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    infrared.run()



