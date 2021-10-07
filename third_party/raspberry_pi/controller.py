#!/usr/bin/env python3

import sys
import time
import traceback
from datetime import datetime
from router import *
from random import randrange
from threading import Thread

import RPi.GPIO as GPIO

from PCA9685 import PCA9685
from configuration import *


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


# def connect_to_feagi():
#     address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
#     feagi_state = find_feagi(address=address)

#     print("** **", feagi_state)
#     sockets = feagi_state['sockets']
#     # router_settings['feagi_burst_speed'] = feagi_state['burst_frequency']
#     router_settings['feagi_burst_speed'] = 1

#     print("--->> >> >> ", sockets)

#     # todo: to obtain this info directly from FEAGI as part of registration
#     ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
#     print("IPU_channel_address=", ipu_channel_address)
#     opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']

#     feagi_ipu_channel = Pub(address=ipu_channel_address)
#     feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)


def send_to_feagi(message, counter, channel):
    print("Sending message to FEAGI...")
    print("Original message:", message)

    # pause before sending to FEAGI IPU SUB (avoid losing connection)
    time.sleep(router_settings['feagi_burst_speed'])
    message['timestamp'] = datetime.now()
    message['counter'] = counter
    channel.send(message)


# class Motor:
#     def __init__(self, count, identifier, model):
#         self.motor_node = publisher_initializer(model_name=model, topic_count=count, topic_identifier=identifier)

#         # todo: figure a way to extract wheel parameters from the model
#         self.wheel_diameter = 1

#     def move(self, motor_index, speed):
#         try:
#             motor_position = std_msgs.msg.Float64()
#             try:
#                 motor_current_position = model_properties['motor']['motor_statuses'][motor_index]
#                 motor_position.data = float(speed * router_settings['feagi_burst_speed'] / 10 + motor_current_position)
#             except KeyError:
#                 model_properties['motor']['motor_statuses'][motor_index] = 0
#                 motor_position.data = float(speed)

#             model_properties['motor']['motor_statuses'][motor_index] += motor_position.data
#             print("Motor index + position = ", motor_index, motor_position)
#             self.motor_node[motor_index].publish(motor_position)
#         except Exception:
#             exc_info = sys.exc_info()
#             traceback.print_exception(*exc_info)


class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.motor_channels = [[0, 1], [3, 2], [4, 5], [6, 7]]

    def move(self, motor_index, speed):
        if speed > 0:
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


class IR:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
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


def main(args=None):
    print("Connecting to FEAGI resources...")

    motor = Motor()

    try:
        while True:
            # Process OPU data received from FEAGI and pass it along
            opu_data = feagi_opu_channel.receive()
            print("Received:", opu_data)
            if opu_data is not None:
                if 'motor' in opu_data:
                    for motor_id in opu_data['motor']:
                        motor.move(motor_id, opu_data['motor'][motor_id] * 10)

            time.sleep(router_settings['feagi_burst_speed'])
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    # connect_to_feagi()
    address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
    feagi_state = find_feagi(address=address)

    print("** **", feagi_state)
    sockets = feagi_state['sockets']
    # router_settings['feagi_burst_speed'] = feagi_state['burst_frequency']
    router_settings['feagi_burst_speed'] = 1

    print("--->> >> >> ", sockets)

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + router_settings['ipu_port']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']

    feagi_ipu_channel = Pub(address=ipu_channel_address)
    feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)

    main()
