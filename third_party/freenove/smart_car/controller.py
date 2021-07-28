
"""
This module contains all the needed functions to control the Freenove 4WD Car

FEAGI IPU/OPU directly interact with this module to operate the 4WD CAR
"""

import RPi.GPIO as GPIO
from PCA9685 import PCA9685
from Led import *
from ADC import *


class LED:
    def __init__(self):
        self.led = Led()

    def LED_on(self, led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
        """
        Parameters
        ----------
        led_ID: This is the ID of leds. It can be from 1 to 8
        Red_Intensity: 1 to 255, from dimmest to brightest
        Blue_Intensity: 1 to 255, from dimmest to brightest
        Green_intensity: 1 to 255, from dimmest to brightest
        -------
        """
        try:
            self.led.ledIndex(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
        except KeyboardInterrupt:
            self.led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/

    def test_Led(self):
        """
        This is to test all leds and do several different leds.
        """
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
        """
        Parameters
        ----------
        position: 1 to 3, There's number on the board.
        -------
        """
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
        """
        Parameters
        ----------
        pitch: pitch level
        duration: Seconds
        -------
        """

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
        """
        Parameters
        ----------
        pitch_level: pitch level
        seconds: duration
        -------
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        x = 0
        pitches = [pitch_level]
        duration = seconds
        for p in pitches:
            self.buzz(p, duration)  # feed the pitch and duration to the function, “buzz”
            time.sleep(duration * 0.5)

        GPIO.setup(self.buzzer_pin, GPIO.IN)


class Servo:
    """
    Functions: head_UP_DOWN and head_RIGHT_LEFT only. Other functions are just a support and defined system for Servo
    class to work with functions.
    """
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.PwmServo.setServoPulse(8,1500)
        self.PwmServo.setServoPulse(9,1500)

    def setServoPwm(self,channel,angle,error=10):
        angle = int(angle)
        if channel == '0':
            self.PwmServo.setServoPulse(8,2500-int((angle+error)/0.09))
        elif channel == '1':
            self.PwmServo.setServoPulse(9,500+int((angle+error)/0.09))
        elif channel == '2':
            self.PwmServo.setServoPulse(10,500+int((angle+error)/0.09))
        elif channel == '3':
            self.PwmServo.setServoPulse(11,500+int((angle+error)/0.09))
        elif channel == '4':
            self.PwmServo.setServoPulse(12,500+int((angle+error)/0.09))
        elif channel == '5':
            self.PwmServo.setServoPulse(13,500+int((angle+error)/0.09))
        elif channel == '6':
            self.PwmServo.setServoPulse(14,500+int((angle+error)/0.09))
        elif channel == '7':
            self.PwmServo.setServoPulse(15,500+int((angle+error)/0.09))

    def head_UP_DOWN(self, num):
        """
        Parameters
        ----------
        num: degree from 0 to 180.
        -------
        """
        self.setServoPwm('1', num) #90 to 0 degree is turn the head down. 90 to 180 is to turn the head up

    def head_RIGHT_LEFT(self, num):
        """
        Parameters
        ----------
        num: degree from 0 to 180.
        -------
        """
        self.setServoPwm('0', num) #90 to 0 degree is turn the head left. 90 to 180 is to turn the head right


class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    @staticmethod
    def duty_range(duty1, duty2, duty3, duty4):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095

        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095

        if duty3 > 4095:
            duty3 = 4095
        elif duty3 < -4095:
            duty3 = -4095

        if duty4 > 4095:
            duty4 = 4095
        elif duty4 < -4095:
            duty4 = -4095
        return duty1, duty2, duty3, duty4

    def left_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def stop(self):
        self.setMotorModel(0, 0, 0, 0)

    # Test Functions
    @staticmethod
    def Backward(self):
        """
        This will go backward only
        """
        self.setMotorModel(2000, 2000, 2000, 2000)

    def stop(self):
        """
        This will halt all motors.
        """
        self.setMotorModel(0, 0, 0, 0)

    def Forward(self):
        """
        this will go forward.
        """
        self.setMotorModel(-2000, -2000, -2000, -2000)

    def Right_Backward(self):
        """
        Right side goes backward
        """
        self.setMotorModel(-500, -500, 2000, 2000)

    def Left_Backward(self):
        """
        The left side go backward.
        """
        self.setMotorModel(2000, 2000, -500, -500)  # Right

    def Left_Forward(self):
        """
        The left side will go forward
        """
        self.setMotorModel(-2000, -2000, -500, -500)

    def Right_Forward(self):
        """
        The right side will go forward
        """
        self.setMotorModel(-500, -500, -2000, -2000)

    def M3F(self):
        """
        The rear left forward
        """
        self.setMotorModel(0, 0, 0, -2000)  # M3 forward

    def M3B(self):
        """
        The rear left backward
        """
        self.setMotorModel(0, 0, 0, 2000)  # M3 backward

    def M1F(self):
        """
        The front right motor goes forward
        """
        self.setMotorModel(-2000, 0, 0, 0)  # M1 forward

    def M1B(self):
        """
        The front left motor goes forward
        """
        self.setMotorModel(2000, 0, 0, 0)  # M1 backward

    def M2F(self):
        """
        The rear right motor goes forward
        """
        self.setMotorModel(0, -2000, 0, 0)  # M2 forward

    def M2B(self):
        """
        The rear right motor goes backward
        """
        self.setMotorModel(0, 2000, 0, 0)  # M2 backward

    def M4F(self):
        """
        The front left motor goes forward
        """
        self.setMotorModel(0, 0, -2000, 0)  # M4 forward

    def M4B(self):
        """
        The front left motor goes backward
        """
        self.setMotorModel(0, 0, 2000, 0)  # M4 backward

    def motor_test_all(self):
        """
        This will test all motor individually per 3 seconds.
        """
        self.M1F()
        time.sleep(3)
        self.M1B()
        time.sleep(3)
        self.M2F()
        time.sleep(3)
        self.M2B()
        time.sleep(3)
        self.M3F()
        time.sleep(3)
        self.M3B()
        time.sleep(3)
        self.M4F()
        time.sleep(3)
        self.M4B()
        time.sleep(3)
        self.stop()

class Photoresistor:
    """
    Photoresistor has two photoresistors. 0 is the left photoresistor and 1 is the right photoresistor on the board.
    """
    def photoresistor(self, number):  # 0 is the left photoressitor and 1 is the right photoresistor
        adc = Adc()
        if number > 2 or number < 0:
            print("Please put 1 or 0 as an input only")
        elif number < 2 or number <= 0:
            output = adc.recvADC(number)
            # print(output)
            return output


class Battery:
    def battery_total(self):  ##It gives a full volt remain of battery
        """
        It reads the battery total. It will return the battery value.
        -------

        """
        adc = Adc()
        Power = adc.recvADC(2) * 3
        # print(Power)
        return Power