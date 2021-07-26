"""
This module contains all the needed functions to control the Freenove 4WD Car

FEAGI IPU/OPU directly interact with this module to operate the 4WD CAR
"""

import RPi.GPIO as GPIO
from Led import *


class LED:
    def __init__(self):
        self.led = Led()

    def LED_on(self, led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
        try:
            self.led.ledIndex(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
        except KeyboardInterrupt:
            self.led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/

    def test_Led(self):
        try:
            self.led.ledIndex(0x01, 255, 0, 0)  # Red
            self.led.ledIndex(0x02, 255, 125, 0)  # orange
            self.led.ledIndex(0x04, 255, 255, 0)  # yellow
            self.led.ledIndex(0x08, 0, 255, 0)  # green
            self.led.ledIndex(0x10, 0, 255, 255)  # cyan-blue
            self.led.ledIndex(0x20, 0, 0, 255)  # blue
            self.led.ledIndex(0x40, 128, 0, 128)  # purple
            self.led.ledIndex(0x80, 255, 255, 255)  # white'''
            print("The LED has been lit, the color is red orange yellow green cyan-blue blue white")
            time.sleep(3)  # wait 3s
            self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
            print("\nEnd of program")
        except KeyboardInterrupt:
            self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
            print("\nEnd of program")

    def leds_off(self):
        self.led.colorWipe("", Color(0, 0, 0))  ##This is to turn all leds off/


class IR:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

    def read(self, position):
        if position == 1:
            if GPIO.input(self.IR01):
                print("Left has been detected on bright")
        elif position == 2:
            if GPIO.input(self.IR02):
                print("Middle has been detected on bright")
        elif position == 3:
            if GPIO.input(self.IR03):
                print("Right has been detected on bright")
        else:
            print("Nothing has been detected.")

class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)
    def send_trigger_pulse(self):
        GPIO.output(self.trigger_pin,True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin,False)

    def wait_for_echo(self,value,timeout):
        count = timeout
        while GPIO.input(self.echo_pin) != value and count>0:
            count = count-1
    def getDistance(self):
        distance_cm=[0,0,0]
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True,10000)
            start = time.time()
            self.wait_for_echo(False,10000)
            finish = time.time()
            pulse_len = finish-start
            distance_cm[i] = pulse_len/0.000058
        distance_cm=sorted(distance_cm)
        return int(distance_cm[1])



class Buzzer(object):
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.buzzer_pin = 17  # set to GPIO pin 17. That's where the buzzer included.
        GPIO.setup(self.buzzer_pin, GPIO.IN)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        print("buzzer ready")

    def __del__(self):
        class_name = self.__class__.__name__
        print(class_name, "finished")

    def buzz(self, pitch, duration):  # create the function “buzz” and feed it the pitch and duration)

        if pitch == 0:
            time.sleep(duration)
            return
        period = 1.0 / pitch  # in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
        delay = period / 2  # calcuate the time for half of the wave
        cycles = int(duration * pitch)  # the number of waves to produce is the duration times the frequency

        for i in range(cycles):  # start a loop from 0 to the variable “cycles” calculated above
            GPIO.output(self.buzzer_pin, True)  # set pin 18 to high
            time.sleep(delay)  # wait with pin 18 high
            GPIO.output(self.buzzer_pin, False)  # set pin 18 to low
            time.sleep(delay)  # wait with pin 18 low

    def play(self, pitch_level, seconds): # The higher number, the higher pitch. The lower number, the lower pitch.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        x = 0
        pitches = [pitch_level]
        duration = seconds
        for p in pitches:
            self.buzz(p, duration)  # feed the pitch and duration to the function, “buzz”
            time.sleep(duration * 0.5)

        GPIO.setup(self.buzzer_pin, GPIO.IN)


if __name__ == '__main__':

#    bwuk=Buzzer()
#    bwuk.buzz(1,1)
