# MIT License
#
# Copyright (c) 2020 Lionkk
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from microbit import *
from Cutebot import *
import _i2c as i2c

class Motor():
    def __init__(self):
        i2c.init()
        self.__pinL = pin13
        self.__pinR = pin14
        self.__pinL.set_pull(self.__pinL.PULL_UP)
        self.__pinR.set_pull(self.__pinR.PULL_UP)

    def set_motors_speed(self, left_wheel_speed: int, right_wheel_speed: int):
        """
        set motor speed
        :param left_wheel_speed: -100 to 100
        :param right_wheel_speed: -100 to 100
        :return: none
        This code is from https://elecfreaks.com/learn-en/microbitKit/smart_cutebot/cutebot-python.html
        """
        if left_wheel_speed > 100 or left_wheel_speed < -100:
            raise ValueError('speed error,-100 to 100 only')
        if right_wheel_speed > 100 or right_wheel_speed < -100:
            raise ValueError('select motor error,1,2,3,4')
        left_direction = 0x02 if left_wheel_speed > 0 else 0x01
        right_direction = 0x02 if right_wheel_speed > 0 else 0x01
        left_wheel_speed = left_wheel_speed if left_wheel_speed > 0 else left_wheel_speed * -1
        right_wheel_speed = right_wheel_speed if right_wheel_speed > 0 else right_wheel_speed * -1
        i2c.write(CUTEBOT_ADDR, bytearray(
            [0x01, left_direction, left_wheel_speed, 0]))
        i2c.write(CUTEBOT_ADDR, bytearray(
            [0x02, right_direction, right_wheel_speed, 0]))
class Ultrasonic:
    def __init__(self):
        i2c.init()
        self.__pin_e = pin12
        self.__pin_t = pin8

    def ultrasonic(self):
        """
        Get ultrasonic data
        :return: m
        """
        self.__pin_e.read_digital()
        self.__pin_t.write_digital(1)
        sleep_us(10)
        self.__pin_t.write_digital(0)
        ts = time_pulse_us(self.__pin_e, 1, 17150)

        distance = (ts * 9 / 6 / 58) * 0.1
        return distance

class IR:
    def __init__(self):
        i2c.init()
        self.__pinL = pin13
        self.__pinR = pin14
        self.__pinL.set_pull(self.__pinL.PULL_UP)
        self.__pinR.set_pull(self.__pinR.PULL_UP)

    def IR_left(self):
        data = self.__pinL.read_digital()
        return data
    def IR_right(self):
        data = self.__pinR.read_digital()
        return data
class LED:
    def headlight_right(self, R: int, G: int, B: int):
        """
        set headlight on right only
        :param light
        :param R: 0-255
        :param G: 0-255
        :param B: 0-255
        :return:none
        This code is from https://elecfreaks.com/learn-en/microbitKit/smart_cutebot/cutebot-python.html
        """
        if R > 255 or G > 255 or B > 255:
            raise ValueError('RGB is error')
        i2c.write(CUTEBOT_ADDR, bytearray([right, R, G, B]))

    def headlight_left(self, R: int, G: int, B: int):
        """
        set headlight on left only
        :param light
        :param R: 0-255
        :param G: 0-255
        :param B: 0-255
        :return:none
        This code is from https://elecfreaks.com/learn-en/microbitKit/smart_cutebot/cutebot-python.html
        """
        if R > 255 or G > 255 or B > 255:
            raise ValueError('RGB is error')
        i2c.write(CUTEBOT_ADDR, bytearray([left, R, G, B]))

class Accelerometer:
    def accelerometer(self):
        array_stored=[]
        x = accelerometer.get_x()
        y = accelerometer.get_y()
        array_stored.append(x)
        array_stored.append(y)
        return array_stored

class LCD:
    def emotions(self):
        """
        a lot of images. TODO: Add and see what is necessary for this script
        """

    def print_lcd(self, message):
        if message.isdigit() != True:
            display.show(message)
        else:
            print("Please put some string values.")

class Buttons:
    """
    returns 0 or 1
    """
    def button_A(self):
        if button_a.was_pressed():
            return True
    def button_B(self):
        if button_b.was_pressed():
            return True
