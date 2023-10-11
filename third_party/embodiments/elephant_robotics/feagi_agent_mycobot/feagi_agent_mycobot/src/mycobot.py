#! /usr/bin/env python3

from pymycobot.mycobot import MyCobot
from feagi_agent import feagi_interface as FEAGI
from collections import deque
from threading import Thread
from configuration import *
from rclpy.node import Node
from version import __version__
from feagi_agent import pns_gateway as pns
import cv2
import os
from datetime import datetime
import geometry_msgs.msg
import rclpy
import sys
import traceback
import std_msgs.msg
from rclpy.qos import qos_profile_sensor_data
import time

previous_data_frame = dict()


def publisher_initializer(topic_count, topic_identifier):
    node = rclpy.create_node('Controller_py')

    target_node = {}
    for target in range(1, topic_count, 1):
        topic_string = topic_identifier + str(target)
        target_node[target] = node.create_publisher(std_msgs.msg.Float64, topic_string, 10)
    return target_node


def check_direction_extra_sensitive(encoder_id):
    """
    encoder_id: 1-6 servos.
    This function will return true or false. True is the positive and False is the negative.
    """
    a = (runtime_data['actual_encoder_position'][encoder_id][0] +
         runtime_data['actual_encoder_position'][encoder_id][1]) / 2
    b = (runtime_data['actual_encoder_position'][encoder_id][3] +
         runtime_data['actual_encoder_position'][encoder_id][4]) / 2
    if a > b:
        return "forward"
    elif a == b:
        return "no_movement"  # Is this even needed?
    else:
        return "backward"


class Servo(Node):
    def __init__(self, count, identifier):
        super().__init__('servo_publisher')
        topic_name = str(identifier) + str(count)
        self.servo_node = publisher_initializer(topic_count=capabilities['servo']['count'],
                                                topic_identifier=identifier)
        timer_period = 0.1  # seconds
        self.msg = 0
        self.timer = self.create_timer(timer_period, self.auto_update_position)
        self.i = 0

    def auto_update_position(self):
        msg = std_msgs.msg.Float64()
        for servo_number in self.servo_node:
            self.msg = float(runtime_data['actual_encoder_position'][servo_number][4])
            msg.data = float(runtime_data['actual_encoder_position'][servo_number][4])
            self.servo_node[servo_number].publish(msg)
            self.main_program_for_servos(servo_number)
            self.i += 1

    def main_program_for_servos(self, encoder_id):
        # if self.arm_status(encoder_id):  # Collision detection
        a = runtime_data['actual_encoder_position'][encoder_id][4]
        b = runtime_data['target_position'][encoder_id]
        # print("encoder id: ", encoder_id, " target: ", b, " actual: ", a)
        # servo_string = str(encoder_id) + ' '
        # print(servo_string * 40)
        # if self.check_direction_extra_sensitive(encoder_id) == "forward" and a > b:
        #     if self.arm_status(encoder_id):
        #         if capabilities['servo']['servo_range'][str(encoder_id)][1] >= (
        #                 runtime_data['target_position'][encoder_id]) >= \
        #                 capabilities['servo']['servo_range'][str(encoder_id)][0]:
        #             runtime_data['target_position'][encoder_id] = runtime_data['target_position'][encoder_id] - 100
        #             print("NEGATIVE 100")
        # if self.check_direction_extra_sensitive(encoder_id) == "backward" and a < b:
        #     if self.arm_status(encoder_id):
        #         if capabilities['servo']['servo_range'][str(encoder_id)][1] >= (
        #                 runtime_data['target_position'][encoder_id]) >= \
        #                 capabilities['servo']['servo_range'][str(encoder_id)][0]:
        #             runtime_data['target_position'][encoder_id] = runtime_data['target_position'][encoder_id] + 100
        #             print("POSITIVE")
        # elif self.check_direction_extra_sensitive(encoder_id) == "no_movement" and (a < b or a > b):
        #     print("touched while not moving")
        try:
            if runtime_data['actual_encoder_position'][encoder_id][4] != \
                    runtime_data['target_position'][encoder_id]:
                if capabilities['servo']['servo_range'][str(encoder_id)][1] >= (
                        runtime_data['target_position'][encoder_id]) >= \
                        capabilities['servo']['servo_range'][str(encoder_id)][
                            0]:  # cap using servo_range
                    if runtime_data['actual_encoder_position'][encoder_id][4] > \
                            runtime_data['actual_encoder_position'][encoder_id][4] - \
                            capabilities['servo']['power'] > runtime_data['target_position'][
                        encoder_id]:
                        if speed[encoder_id]:
                            global_arm['0'].set_speed(abs(speed[encoder_id]))
                        global_arm['0'].set_encoder(encoder_id,
                                                    runtime_data['target_position'][
                                                        encoder_id])  # move the arm
                    elif runtime_data['actual_encoder_position'][encoder_id][4] < \
                            runtime_data['actual_encoder_position'][encoder_id][4] + \
                            capabilities['servo'][
                                'power'] < \
                            runtime_data['target_position'][encoder_id]:
                        if speed[encoder_id]:
                            global_arm['0'].set_speed(abs(speed[encoder_id]))
                        global_arm['0'].set_encoder(encoder_id,
                                                    runtime_data['target_position'][
                                                        encoder_id])  # move the arm
        except Exception as e:
            print("ERROR: ", e)
            traceback.print_exc()

        direction = check_direction_extra_sensitive(encoder_id)  # Check if reverse is true or
        # false

    def collision_detection_sensitive(self, encoder_id):
        """
        encoder_id: 1-6 servos.
        This function will return true or false. True is the positive and False is the negative.
        """
        a = (runtime_data['actual_encoder_position'][encoder_id][0] +
             runtime_data['actual_encoder_position'][encoder_id][1]) / 2
        b = (runtime_data['actual_encoder_position'][encoder_id][3] +
             runtime_data['actual_encoder_position'][encoder_id][4]) / 2
        try:
            if a > b + int(capabilities['servo']['sensitivity']['micro']):
                return "1"
            elif b > a + int(capabilities['servo']['sensitivity']['micro']):
                return "2"
            else:
                return "0"  # Is this even needed?
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)

    def collision_detection_less_sensitive(self, encoder_id):
        """
        encoder_id: 1-6 servos.
        This function will return 1,2, or 3 in string.
        """
        a = (runtime_data['actual_encoder_position'][encoder_id][0] +
             runtime_data['actual_encoder_position'][encoder_id][1]) / 2
        b = (runtime_data['actual_encoder_position'][encoder_id][3] +
             runtime_data['actual_encoder_position'][encoder_id][4]) / 2
        if a > b + capabilities['servo']['sensitivity']['macro']:
            return "1"
        elif b > a + capabilities['servo']['sensitivity']['macro']:
            return "2"
        else:
            return "0"

    def arm_status(self, encoder_id):
        """
        encoder_id is the servo id.

        This will detect if the arm is being collision. It's currently WIP.
        """
        arm_status_ = self.collision_detection_sensitive(encoder_id)
        arm_status__ = self.collision_detection_less_sensitive(encoder_id)
        if arm_status_ != arm_status__:
            print("Collision on servo: ", encoder_id)
            return True
        return False


class ServoPosition(Node):
    def __init__(self):
        """
        This will update encoder position in real time and it will ignore the '-1' error.
        """
        super().__init__('Servo_position')
        self.publisher_ = self.create_publisher(std_msgs.msg.String, 'servo_data', 0)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = std_msgs.msg.String()
        for i in range(1, capabilities['servo']['count'], 1):
            runtime_data['time'][i] = time.time()
            if i != 2:
                new_data = arm.get_encoder(i)
                if new_data != -1:
                    if runtime_data['actual_encoder_position'][i]:
                        runtime_data['actual_encoder_position'][i].append(new_data)
                        runtime_data['actual_encoder_position'][i].popleft()
        msg.data = str(runtime_data['actual_encoder_position'])
        self.publisher_.publish(msg)
        self.i += 1


class Arm:
    @staticmethod
    def connection_initialize(port='/dev/ttyUSB0'):
        """
        :param port: The default would be '/dev/ttyUSB0'. If the port is different, put a different port.
        :return:
        """
        return MyCobot(port)

    @staticmethod
    def initialize(count):
        default = [2048, 2048, 2048, 2048, 2048, 2048]
        global_arm['0'].set_encoders(default, 100)
        for number_id in range(1, count + 1, 1):
            # if number_id != 0 and number_id != 0:
            if number_id not in runtime_data['target_position']:
                runtime_data['target_position'][number_id] = 2048
        time.sleep(1)

    @staticmethod
    def power_convert(encoder_id, power):
        if encoder_id % 2 == 0:
            return -1 * power
        else:
            return abs(power)

    @staticmethod
    def encoder_converter(encoder_id):
        """
        This will convert from godot to motor's id. Let's say, you have 8x10 (width x depth from static_genome).
        So, you click 4 to go forward. It will be like this:
        o__mot': {'1-0-9': 1, '5-0-9': 1, '3-0-9': 1, '7-0-9': 1}
        which is 1,3,5,7. So this code will convert from 1,3,5,7 to 0,1,2,3 on motor id.
        Since 0-1 is motor 1, 2-3 is motor 2 and so on. In this case, 0 is for forward and 1 is for backward.
        """
        if encoder_id <= 1:
            return 1
        elif encoder_id <= 3:
            return 2
        elif encoder_id <= 5:
            return 3
        elif encoder_id <= 7:
            return 4
        elif encoder_id <= 9:
            return 5
        elif encoder_id <= 11:
            return 6
        else:
            print("Input has been refused. Please put encoder ID.")


def action(obtained_data, device_list, runtime_data):
    for device in device_list:
        if 'servo_position' in obtained_data:
            try:
                if obtained_data['servo_position'] is not {}:
                    for data_point in obtained_data['servo_position']:
                        device_id = data_point + 1
                        encoder_position = ((capabilities['servo']['servo_range'][str(device_id)][1] -
                                             capabilities['servo']['servo_range'][str(device_id)][
                                                 0]) / 20) * \
                                           obtained_data['servo_position'][data_point]
                        runtime_data['target_position'][device_id] = encoder_position
                        # print(encoder_position, " is encoder id: ", device_id)
                        # print(runtime_data['target_position'][device_id])
                        print("CLICKED")
                        speed[device_id] = (obtained_data['servo_position'][data_point] - 10) * 10
            except Exception as e:
                print("ERROR: ", e)
                traceback.print_exc()

        if 'servo' in obtained_data:
            try:
                if obtained_data['servo'] is not {}:
                    for data_point in obtained_data['servo']:
                        encoder_position = obtained_data['servo'][data_point]
                        if data_point % 2 != 0:
                            encoder_position *= -1
                        device_id = (data_point // 2) + 1
                        test = runtime_data['target_position'][device_id] + encoder_position
                        print("ENCODER: ", encoder_position, " datapoint: ", data_point )
                        print("FIRST: ", capabilities['servo']['servo_range'][str(device_id)][1],
                              " SECOND: ", test, " THIRD: ", capabilities['servo']['servo_range'][str(device_id)][0])
                        if capabilities['servo']['servo_range'][str(device_id)][1] >= test >= \
                                capabilities['servo']['servo_range'][str(device_id)][0]:
                            runtime_data['target_position'][device_id] += encoder_position
                            speed[device_id] = (obtained_data['servo'][data_point] - 10)
            except Exception as e:
                print("ERROR: ", e)
                traceback.print_exc()


runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'target_position': {},
    'actual_encoder_position': {},
    'time': {},
    'position_difference': {}
}

# # FEAGI REACHABLE CHECKER # #
feagi_flag = False
print("retrying...")
print("Waiting on FEAGI...")
while not feagi_flag:
    feagi_flag = FEAGI.is_FEAGI_reachable(
        os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
        int(os.environ.get('FEAGI_OPU_PORT', "3000")))
    time.sleep(2)

# # FEAGI REACHABLE CHECKER COMPLETED # #

# mycobot initialize section # # #
# todo: To figure how to make this part scalable
global_arm = dict()
mycobot = Arm()
print("initaializing the arm...")
arm = mycobot.connection_initialize()
global_arm['0'] = arm  # Is this even allowed?
mycobot.initialize(capabilities['servo']['count'])
for i in range(1, capabilities['servo']['count'], 1):
    runtime_data['actual_encoder_position'][i] = deque([0, 0, 0, 0, 0])
global_arm['0'].set_speed(100)
device_list = pns.generate_OPU_list(capabilities)  # get the OPU sensors

# # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - #
feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
    FEAGI.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities, __version__)
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Camera section
# camera = PiCamera()
# camera = cv2.VideoCapture('/dev/video2')
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))

# Necessary variables section:
flag = True
keyboard_flag = True
msg_counter = 0
encoder_speed = dict()
encoder_speed['encoder_speed'] = dict()
for i in range(capabilities['servo']['count']):
    encoder_speed['encoder_speed'][i] = {}

speed = dict()
for i in range(1, 7, 1):
    speed[i] = {}

# ROS 2 initialize section
rclpy.init(args=None)
executor = rclpy.executors.MultiThreadedExecutor()  # uncomment this when we are starting with
# sensors
servo = Servo(count=capabilities['servo']['count'],
              identifier=capabilities['servo']['topic_identifier'])
servo_position = ServoPosition()
executor.add_node(servo_position)
executor.add_node(servo)
executor_thread = Thread(target=executor.spin, daemon=True)
executor_thread.start()

while keyboard_flag:
    try:
        # OPU section
        message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
        if message_from_feagi is not None:
            obtained_signals = pns.obtain_opu_data(device_list, message_from_feagi)
            action(obtained_signals, device_list, runtime_data)
            # print(opu_data)

            # Encoder speed IPU
            for i in range(1, capabilities['servo']['count'], 1):
                runtime_data['position_difference'][i - 1] = speed[i]
            encoder_speed['encoder_speed'] = runtime_data['position_difference']
            message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=encoder_speed,
                                                                   data=message_to_feagi)
        # Encoder position
        encoder_for_feagi = dict()
        encoder_for_feagi['encoder_data'] = dict()
        try:
            for encoder_data in runtime_data['actual_encoder_position']:
                encoder_for_feagi['encoder_data'][encoder_data] = \
                runtime_data['actual_encoder_position'][encoder_data][
                    4]
            message_to_feagi, bat = FEAGI.compose_message_to_feagi(
                original_message=encoder_for_feagi,
                data=message_to_feagi)
        except Exception as e:
            print("error: ", e)

        # SENDING MESSAGE TO FEAGI SECTION # #
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        pns.afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings)
        message_to_feagi.clear()

        # Doing the misc background work (check on sync setting #
        msg_counter += 1
        flag += 1
        for i in encoder_speed['encoder_speed']:
            encoder_speed['encoder_speed'][i] = {}
        for i in speed:
            speed[i] = {}

    except KeyboardInterrupt as ke:  # Keyboard error
        arm.release_all_servos()
        keyboard_flag = False
        servo.destroy_node()
    except Exception as e:
        arm.release_all_servos()
        keyboard_flag = False
        servo.destroy_node()
        print("ERROR: ", e)
arm.release_all_servos()
rclpy.shutdown()
