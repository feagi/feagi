# Designed by Freenove. This code is to test and verify the photoresistor's function

import time

from ADC import *

adc=Adc()

def test_Adc():
    try:
        while True:
            Left_IDR=adc.recvADC(0)
            print ("The photoresistor voltage on the left is "+str(Left_IDR)+"V")
            Right_IDR=adc.recvADC(1)
            print ("The photoresistor voltage on the right is "+str(Right_IDR)+"V")
            Power=adc.recvADC(2)
            print ("The battery voltage is "+str(Power*3)+"V")
            time.sleep(1)
            print ('\n')
    except KeyboardInterrupt:
        print ("\nEnd of program")

