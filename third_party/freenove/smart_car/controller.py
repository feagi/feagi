#!/usr/bin/env python3

"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

import sys
import time
import router
import RPi.GPIO as GPIO
import math
import traceback
import configuration

from collections import deque
from datetime import datetime
from ADC import *
from configuration import *
from configuration import message_to_feagi
from Led import *
from PCA9685 import PCA9685
from router import *

runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'servo_status': {}
}

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def compose_message_to_feagi(original_message):
    """
    accumulates multiple messages in a data structure that can be sent to feagi
    """
    if "data" not in message_to_feagi:
        message_to_feagi["data"] = dict()
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = dict()
    for sensor in original_message:
        if sensor not in message_to_feagi["data"]["sensory_data"]:
            message_to_feagi["data"]["sensory_data"][sensor] = dict()
        for sensor_data in original_message[sensor]:
            if sensor_data not in message_to_feagi["data"]["sensory_data"][sensor]:
                message_to_feagi["data"]["sensory_data"][sensor][sensor_data] = original_message[sensor][sensor_data]
    #message_to_feagi["data"]["sensory_data"]["battery"] = {1: runtime_params["battery_charge_level"] / 100}

def feagi_registration(feagi_host, api_port):
    app_host_info = router.app_host_info()
    runtime_data["host_network"]["host_name"] = app_host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = app_host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("Awaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = router.register_with_feagi(app_name=configuration.app_name,
                                                                     feagi_host=feagi_host,
                                                                     api_port=api_port,
                                                                     app_capabilities=configuration.capabilities,
                                                                     app_host_info=runtime_data["host_network"]
                                                                     )
        except:
            pass
        sleep(1)

def window_average(sequence):
    return abs(sum(sequence) // len(sequence))


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

class Servo:
    """
    Functions: head_UP_DOWN and head_RIGHT_LEFT only. Other functions are just a support and defined system for Servo
    class to work with functions.
    """
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.device_position = float()
        self.servo_ranges = {0: [0, 180],
                             1: [0, 180]}

    def setServoPwm(self,channel,angle,error=10):
        angle = float(angle)
        if channel == '0':
            self.PwmServo.setServoPulse(8,2500-float((angle+error)/0.09))
        elif channel == '1':
            self.PwmServo   .setServoPulse(9,500+float((angle+error)/0.09))
        elif channel == '2':
            self.PwmServo.setServoPulse(10,500+float((angle+error)/0.09))
        elif channel == '3':
            self.PwmServo.setServoPulse(11,500+float((angle+error)/0.09))
        elif channel == '4':
            self.PwmServo.setServoPulse(12,500+float((angle+error)/0.09))
        elif channel == '5':
            self.PwmServo.setServoPulse(13,500+float((angle+error)/0.09))
        elif channel == '6':
            self.PwmServo.setServoPulse(14,500+float((angle+error)/0.09))
        elif channel == '7':
            self.PwmServo.setServoPulse(15,500+float((angle+error)/0.09))

    def set_default_position(self):
        try:
            # Setting the initial position for the servo
            servo_0_initial_position = 90
            self.device_position = servo_0_initial_position
            self.move(0,self.device_position)
            runtime_data['servo_status'][0] = self.device_position
            print("Servo 0 was moved to its initial position")

            servo_1_initial_position = 90
            self.device_position = servo_1_initial_position
            self.move(1,self.device_position)
            runtime_data['servo_status'][1] = self.device_position
            print("Servo 1 was moved to its initial position")
        except Exception as e:
            print("Error while setting initial position for the servo:", e)

    def move(self, feagi_device_id, power):
        try:
            if feagi_device_id > 2 * capabilities['servo']['count']:
                print("Warning! Number of servo channels from FEAGI exceed available Motor count!")
            # Translate feagi_motor_id to motor backward and forward motion to individual motors
            device_index = feagi_device_id // 2
            if feagi_device_id % 2 == 1:
                power *= 1
            else:
                power *= -1
            if device_index not in runtime_data['servo_status']:
                runtime_data['servo_status'][device_index] = device_index

            device_current_position = runtime_data['servo_status'][device_index]
            self.device_position = float((power * network_settings['feagi_burst_speed'] / 0.5) +
                                              device_current_position)

            self.device_position = self.keep_boundaries(device_id=device_index,
                                                             current_position=self.device_position)

            runtime_params['servo_status'][device_index] = self.device_position
            #print("device index, position, power = ", device_index, self.device_position, power)
            #self.servo_node[device_index].publish(self.device_position)
            self.setServoPwm(str(device_index), self.device_position)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)

    def keep_boundaries(self, device_id, current_position):
        """
        Prevent Servo position to go beyond range
        """
        if current_position > self.servo_ranges[device_id][1]:
            adjusted_position = float(self.servo_ranges[device_id][1])
        elif current_position < self.servo_ranges[device_id][0]:
            adjusted_position = float(self.servo_ranges[device_id][0])
        else:
            adjusted_position = float(current_position)
        return adjusted_position

    def servo_id_converter(self, servo_id):
        """
        This will convert from godot to motor's id. Let's say, you have 4x10 (width x depth from static_genome).
        So, you click 2 (actually 4 but 2 for one servo on backward/forward) to go forward. It will be like this:
        o__ser': {'1-0-9': 1, '3-0-9': 1}
        which is 1,3. So this code will convert from 1,3 to 0,1 on motor id.

        Since 0-1 is servo 0, 2-3 is servo 1 and so on. In this case, 0 and 2 is for forward and 1 and 3 is for backward.
        """
        if servo_id <= 1:
            return 0
        elif servo_id <= 3:
            return 1
        else:
            print("Input has been refused. Please put motor ID.")

    def power_convert(self, motor_id, power):
        if motor_id == 1:
            return -1 * power
        elif motor_id == 3:
            return -1 * power
        else:
            return abs(power)




class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.motor_channels = [[0, 1], [3, 2], [4, 5], [6, 7]]


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

    def move(self, motor_index, speed):
        if speed > 0:
            print("from move(): ", motor_index)
            self.pwm.setMotorPwm(self.motor_channels[motor_index][0], 0)
            self.pwm.setMotorPwm(self.motor_channels[motor_index][1], speed)
        elif speed < 0:
            self.pwm.setMotorPwm(self.motor_channels[motor_index][1], 0)
            self.pwm.setMotorPwm(self.motor_channels[motor_index][0], abs(speed))
        else:
            self.pwm.setMotorPwm(self.motor_channels[motor_index][0], 4095)
            self.pwm.setMotorPwm(self.motor_channels[motor_index][1], 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def stop(self):
        self.setMotorModel(0, 0, 0, 0)

    def motor_converter(self, motor_id):
        """
        This will convert from godot to motor's id. Let's say, you have 8x10 (width x depth from static_genome).
        So, you click 4 to go forward. It will be like this:
        o__mot': {'1-0-9': 1, '5-0-9': 1, '3-0-9': 1, '7-0-9': 1}
        which is 1,3,5,7. So this code will convert from 1,3,5,7 to 0,1,2,3 on motor id.

        Since 0-1 is motor 1, 2-3 is motor 2 and so on. In this case, 0 is for forward and 1 is for backward.
        """
        # motor_total = capabilities['motor']['count'] #be sure to update your motor total in configuration.py
        # increment = 0
        # for motor in range(motor_total):
        #     if motor_id <= motor + 1:
        #         print("motor_id: ", motor_id)
        #         increment += 1
        #         return increment
        if motor_id <= 1:
            return 0
        elif motor_id <= 3:
            return 1
        elif motor_id <= 5:
            return 2
        elif motor_id <= 7:
            return 3
        else:
            print("Input has been refused. Please put motor ID.")

    def power_convert(self, motor_id, power):
        if motor_id == 0:
            return -1 * power
        elif motor_id == 2:
            return -1 * power
        elif motor_id == 4:
            return -1 * power
        elif motor_id == 6:
            return -1 * power
        else:
            return abs(power)


class IR:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

    def read(self):
        gpio_state = []
        ir_sensors = [self.IR01, self.IR02, self.IR03]
        for idx, sensor in enumerate(ir_sensors):
            if GPIO.input(sensor):
                gpio_state.append(idx)
        return gpio_state


class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def send_trigger_pulse(self):
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin, False)

    def wait_for_echo(self, value, timeout):
        count = timeout
        while GPIO.input(self.echo_pin) != value and count > 0:
            count = count - 1

    def get_distance(self):
        distance_cm = [0, 0, 0]
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True, 10000)
            start = time.time()
            self.wait_for_echo(False, 10000)
            finish = time.time()
            pulse_len = finish - start
            distance_cm[i] = pulse_len / 0.000058
        distance_cm = sorted(distance_cm)
        return int(distance_cm[1])


# class Battery:
#     def battery_total(self):
#         adc = Adc()
#         Power = adc.recvADC(2) * 3
#         # print(Power)
#         return Power


def main(args=None):
    flag = False
    counter = 0
    print("Connecting to FEAGI resources...")

    motor = Motor()
    ir = IR()
    ultrasonic = Ultrasonic()
    # battery = Battery()
    servo = Servo()

    rolling_window_len = capabilities['motor']['rolling_window_len']
    motor_count = capabilities['motor']['count']
    msg_counter = 0
    # LED.test_Led()

    rolling_window = {}
    for motor_id in range(motor_count):
        rolling_window[motor_id] = deque([0] * rolling_window_len)

    try:
        while True:
            # transmit data to FEAGI IPU
            ir_data = ir.read()
            if ir_data:
                formatted_ir_data = {'ir': {sensor: True for sensor in ir_data}}
            else:
                formatted_ir_data = {}

            if ir_data:
                for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                    if ir_sensor not in formatted_ir_data['ir']:
                        formatted_ir_data['ir'][ir_sensor] = False
            else:
                formatted_ir_data['ir'] = {}
                for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                    formatted_ir_data['ir'][ir_sensor] = False

            for ir_sensor in range(int(configuration.capabilities['infrared']['count'])):
                if ir_sensor not in formatted_ir_data['ir']:
                    formatted_ir_data['ir'][ir_sensor] = False

            ultrasonic_data = ultrasonic.get_distance()
            if ultrasonic_data:
                formatted_ultrasonic_data = {
                    'ultrasonic': {
                        sensor: data for sensor, data in enumerate([ultrasonic_data])
                    }
                }
            else:
                formatted_ultrasonic_data = {}

            # battery_data = battery.battery_total()
            # if battery_data:
            #     formatted_battery_data = {
            #         'battery': {
            #             sensor: data for sensor, data in enumerate([battery_data])
            #         }
            #     }
            # else:
            #     formatted_battery_data = {}

            compose_message_to_feagi(
                original_message={**formatted_ir_data, **formatted_ultrasonic_data})##Removed battery due to error
            # Process OPU data received from FEAGI and pass it along
            message_from_feagi = feagi_opu_channel.receive()
           # print("Received:", opu_data)
            if message_from_feagi is not None:
                opu_data = message_from_feagi["opu_data"]
                if 'o__mot' in opu_data:
                    for data_point in opu_data['o__mot']:
                        data_point = block_to_array(data_point)
                        device_id = motor.motor_converter(data_point[0])
                        device_power = data_point[2]
                        device_power = motor.power_convert(data_point[0], device_power)
                        motor.move(device_id, (device_power * 400))
                        flag = True
                if 'o__ser' in opu_data:
                    for data_point in opu_data['o__ser']:
                        data_point = block_to_array(data_point)
                        device_id = data_point[0]
                        device_power = data_point[2]
                        # device_id = servo.servo_id_converter(device_id)
                        # device_power = servo.power_convert(data_point[0], device_power)
                        servo.move(feagi_device_id=device_id, power=device_power)
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
            msg_counter += 1

            #time.sleep(network_settings['feagi_burst_speed'])
            if flag:
                if counter < 3:
                    counter += msg_counter
                else:
                    motor.stop()
                    counter = 0

            # LED.leds_off()
    except KeyboardInterrupt as ke: ##Keyboard error
        motor.stop()
        print(ke)


if __name__ == '__main__':
    GPIO.cleanup()
    print("Connecting to FEAGI resources...")

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_outbound_port']

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]

    feagi_registration(feagi_host=feagi_host, api_port=api_port)

    print("** **", runtime_data["feagi_state"])
    network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + runtime_data["feagi_state"]['feagi_inbound_port_gazebo']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + network_settings['feagi_host'] + ':' + \
                          runtime_data["feagi_state"]['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    main()
