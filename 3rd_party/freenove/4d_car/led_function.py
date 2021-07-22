# This code is heavily modified and added function to make it easy to use.
# The original code: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/test.py
# The original code is designed by Freenove.


import time
from Led import *
led=Led()

#IMPORTANT INFORMATION ABOUT THE LED_ON function

###########LED ID on the board###############################


#Board ID:     led_ID list:
#D12           0x01
#D13           0x02
#D14           0x04
#D15           0x08
#D16           0x10
#D17           0x20
#D18           0x40
#D19           0x80


#############################################################

# Intensity of light goes from 1 to 255. 1 is the dimmest. 255 is the brightest.

# LED_on using the RGB: https://www.w3schools.com/colors/colors_rgb.asp





def LED_on(led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
    try:
        led.ledIndex(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
    except KeyboardInterrupt:
        led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/

def test_Led():
    try:
        led.ledIndex(0x01,255,0,0)      #Red
        led.ledIndex(0x02,255,125,0)    #orange
        led.ledIndex(0x04,255,255,0)    #yellow
        led.ledIndex(0x08,0,255,0)      #green
        led.ledIndex(0x10,0,255,255)    #cyan-blue
        led.ledIndex(0x20,0,0,255)      #blue
        led.ledIndex(0x40,128,0,128)    #purple
        led.ledIndex(0x80,255,255,255)  #white'''
        print ("The LED has been lit, the color is red orange yellow green cyan-blue blue white")
        time.sleep(3)               #wait 3s
        led.colorWipe(led.strip, Color(0,0,0))  #turn off the light
        print ("\nEnd of program")
    except KeyboardInterrupt:
        led.colorWipe(led.strip, Color(0,0,0))  #turn off the light
        print ("\nEnd of program")

def leds_off():
    led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/



# Main program logic follows:
if __name__ == '__main__':

    LED_on(0x01,50,0,0) #Example to use LED_on function.
    time.sleep(2) #This is the duration
    leds_off() #This is to turn all leds off.
