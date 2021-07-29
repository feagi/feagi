"""
This module contains all the needed functions to control the Freenove 4WD Car

FEAGI IPU/OPU directly interact with this module to operate the 4WD CAR
"""

#coding:utf-8
import RPi.GPIO as GPIO
import smbus
import time
import math
import Adafruit_PCA9685
from Led import *
from Kalman import *
from mpu6050 import mpu6050


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
            self.led.colorWipe(led.strip, Color(0, 0, 0))  # This is to turn all leds off/

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
        self.led.colorWipe("", Color(0, 0, 0))  # This is to turn all LEDs off/


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
        """

        Returns distance_cm in INT value
        -------

        """
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
        delay = period / 2  # calculate the time for half of the wave
        cycles = int(duration * pitch)  # the number of waves to produce is the duration times the frequency

        for i in range(cycles):  # start a loop from 0 to the variable “cycles” calculated above
            GPIO.output(self.buzzer_pin, True)  # set pin 18 to high
            time.sleep(delay)  # wait with pin 18 high
            GPIO.output(self.buzzer_pin, False)  # set pin 18 to low
            time.sleep(delay)  # wait with pin 18 low

    def play(self, pitch_level, seconds):   # The higher number, the higher pitch. The lower number, the lower pitch.
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


class ADS7830:  # This is to read the current battery level.
    def __init__(self):
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        # I2C address of the device
        self.ADS7830_DEFAULT_ADDRESS = 0x48
        # ADS7830 Command Set
        self.ADS7830_CMD = 0x84 # Single-Ended Inputs

    def readAdc(self, channel):
        """Select the Command data from the given provided value above"""
        command_set = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
        self.bus.write_byte(self.ADS7830_DEFAULT_ADDRESS, command_set)
        data = self.bus.read_byte(self.ADS7830_DEFAULT_ADDRESS)
        return data

    def battery(self, channel):
        data=['','','','','','','','','']
        for i in range(9):
            data[i] = self.readAdc(channel)
        data.sort()
        battery_voltage = data[4]/255.0*5.0*3
        return battery_voltage


class IMU:
    def __init__(self):
        self.Kp = 100
        self.Ki = 0.002
        self.halfT = 0.001

        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.sensor = mpu6050(address=0x68)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        self.kalman_filter_AX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AZ = Kalman_filter(0.001, 0.1)

        self.kalman_filter_GX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GZ = Kalman_filter(0.001, 0.1)

        self.Error_value_accel_data, self.Error_value_gyro_data = self.average_filter()

    def average_filter(self):
        sum_accel_x = 0
        sum_accel_y = 0
        sum_accel_z = 0

        sum_gyro_x = 0
        sum_gyro_y = 0
        sum_gyro_z = 0
        for i in range(100):
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

            sum_accel_x += accel_data['x']
            sum_accel_y += accel_data['y']
            sum_accel_z += accel_data['z']

            sum_gyro_x += gyro_data['x']
            sum_gyro_y += gyro_data['y']
            sum_gyro_z += gyro_data['z']

        sum_accel_x /= 100
        sum_accel_y /= 100
        sum_accel_z /= 100

        sum_gyro_x /= 100
        sum_gyro_y /= 100
        sum_gyro_z /= 100

        accel_data['x'] = sum_accel_x
        accel_data['y'] = sum_accel_y
        accel_data['z'] = sum_accel_z - 9.8

        gyro_data['x'] = sum_gyro_x
        gyro_data['y'] = sum_gyro_y
        gyro_data['z'] = sum_gyro_z

        return accel_data, gyro_data

    def imuUpdate(self):
        """

        Returns z,x, and y
        -------

        """
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()
        ax = self.kalman_filter_AX.kalman(accel_data['x'] - self.Error_value_accel_data['x'])
        ay = self.kalman_filter_AY.kalman(accel_data['y'] - self.Error_value_accel_data['y'])
        az = self.kalman_filter_AZ.kalman(accel_data['z'] - self.Error_value_accel_data['z'])
        gx = self.kalman_filter_GX.kalman(gyro_data['x'] - self.Error_value_gyro_data['x'])
        gy = self.kalman_filter_GY.kalman(gyro_data['y'] - self.Error_value_gyro_data['y'])
        gz = self.kalman_filter_GZ.kalman(gyro_data['z'] - self.Error_value_gyro_data['z'])

        norm = math.sqrt(ax * ax + ay * ay + az * az)

        ax = ax / norm
        ay = ay / norm
        az = az / norm

        vx = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3

        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        self.exInt += ex * self.Ki
        self.eyInt += ey * self.Ki
        self.ezInt += ez * self.Ki

        gx += self.Kp * ex + self.exInt
        gy += self.Kp * ey + self.eyInt
        gz += self.Kp * ez + self.ezInt

        self.q0 += (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * self.halfT
        self.q1 += (self.q0 * gx + self.q2 * gz - self.q3 * gy) * self.halfT
        self.q2 += (self.q0 * gy - self.q1 * gz + self.q3 * gx) * self.halfT
        self.q3 += (self.q0 * gz + self.q1 * gy - self.q2 * gx) * self.halfT

        norm = math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        pitch = math.asin(-2 * self.q1 * self.q3 + 2 * self.q0 * self.q2) * 57.3
        roll = math.atan2(2 * self.q2 * self.q3 + 2 * self.q0 * self.q1,
                          -2 * self.q1 * self.q1 - 2 * self.q2 * self.q2 + 1) * 57.3
        yaw = math.atan2(2 * (self.q1 * self.q2 + self.q0 * self.q3),
                         self.q0 * self.q0 + self.q1 * self.q1 - self.q2 * self.q2 - self.q3 * self.q3) * 57.3
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        return self.pitch, self.roll, self.yaw


class Servo:
    def __init__(self):
        self.angleMin = 18
        self.angleMax = 162
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)               # Set the cycle frequency of PWM
    # Convert the input angle to the value of pca9685

    def map(self, value, fromLow, fromHigh, toLow, toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

    def setServoAngle(self, channel, angle):
        if angle < self.angleMin:
            angle = self.angleMin
        elif angle > self.angleMax:
            angle = self.angleMax
        date = self.map(angle,0,180,102,512)
        # print(date,date/4096*0.02)
        self.pwm.set_pwm(channel, 0, int(date))

    def move_servo(self, servo_id, degree):
        """
        Servo_id mapping table:
        1:
        2:
        3:
        4:  front left top servo
        5:  front left middle servo
        6:  rear left middle servo
        7:
        8:  rear right top servo?
        9:  rear right top servo?
        10: rear right top servo?
        11: front right top servo
        12: front right middle servo
        13:
        14:
        15: Head

        """
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            try:
                servo.setServoAngle(servo_id, degree)   # front left of the top leg
            except:
                print("ERROR with setServoAngle")
        else:
            print("Error: Servo can operate within 0 to 180 degree only")   # A protection to not damage the servos.
