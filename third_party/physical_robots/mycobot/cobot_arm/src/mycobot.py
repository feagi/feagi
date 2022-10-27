#! /usr/bin/env python3

from pymycobot.mycobot import MyCobot
from feagi_agent import feagi_interface as FEAGI
from feagi_agent import retina as retina
from threading import Thread
from configuration import *
from rclpy.node import Node
# from picamera.array import PiRGBArray
# from picamera import PiCamera
from datetime import datetime
import requests
import geometry_msgs.msg
import rclpy
import sys
import traceback
import std_msgs.msg
from rclpy.qos import qos_profile_sensor_data
import time

previous_data_frame = dict()
old_data = dict()


def publisher_initializer(topic_count, topic_identifier):
    node = rclpy.create_node('Controller_py')

    target_node = {}
    for target in range(1, topic_count, 1):
        topic_string = topic_identifier + str(target)
        target_node[target] = node.create_publisher(std_msgs.msg.Float64, topic_string, 10)
    return target_node


# class ScalableSubscriber(Node):
#     def __init__(self, subscription_name, msg_type, topic):
#         super().__init__(subscription_name)
#         self.subscription = self.create_subscription(
#             msg_type,
#             topic,
#             self.listener_callback,
#             qos_profile=qos_profile_sensor_data)
#         self.topic = topic
#         self.counter = 0
#
#     def listener_callback(self, msg):
#         try:
#             # This generated none
#             formatted_msg = FEAGI.msg_processor(self, msg=msg, msg_type=self.topic)  # Needs to check on this
#             message_to_feagi, runtime_data["battery_charge_level"] = FEAGI.compose_message_to_feagi(
#                 original_message=formatted_msg, data=message_to_feagi)
#             self.counter += 1
#         except Exception as e:
#             print("Error in listener callback...", e)
#             exc_info = sys.exc_info()
#             traceback.print_exception(*exc_info)
#
#
# class ServoSubscriber(ScalableSubscriber):
#     def __init__(self, subscription_name, msg_type, topic):
#         super().__init__(subscription_name, msg_type, topic)
#
# #
class Servo(Node):
    def __init__(self, count, identifier):
        super().__init__('servo_publisher')
        topic_name = str(identifier) + str(count)
        self.servo_node = publisher_initializer(topic_count=capabilities['servo']['count'], topic_identifier=identifier)
        timer_period = 0.1  # seconds
        # self.servo_ranges = capabilities['servo']['servo_range'][count]
        self.msg = 0
        self.timer = self.create_timer(timer_period, self.auto_update_position)
        self.i = 0

    def auto_update_position(self):
        msg = std_msgs.msg.Float64()
        for servo_number in self.servo_node:
            if servo_number != 2:
                self.msg = float(runtime_data['servo_status'][servo_number])
                self.servo_node[servo_number].publish(msg)
                self.verify(servo_number)
                self.i += 1

    @staticmethod
    def verify(encoder_id):
        if old_data[encoder_id] != runtime_data['servo_status'][encoder_id]:
            if capabilities['servo']['servo_range'][str(encoder_id)][1] >= (
                    runtime_data['servo_status'][encoder_id]) >= \
                    capabilities['servo']['servo_range'][str(encoder_id)][0]:
                global_arm['0'].set_encoder(encoder_id, runtime_data['servo_status'][encoder_id])
                old_data[encoder_id] = runtime_data['servo_status'][encoder_id]


class ServoPosition(Node):
    def __init__(self):
        super().__init__('Servo_position')
        self.publisher_ = self.create_publisher(std_msgs.msg.String, 'servo_data', 0)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = std_msgs.msg.String()
        for i in range(1, capabilities['servo']['count'], 1):
            if i != 2:
                new_data = arm.get_encoder(i)
                if new_data != -1:
                    runtime_data['servo_position'][i] = new_data
        msg.data = str(runtime_data['servo_position'])
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
    def get_coordination(robot):
        """
        :return: 6 servos' coordination
        """
        data = robot.get_coords()
        return data

    @staticmethod
    def initialize(robot, count):
        for number_id in range(count):
            if number_id != 2 and number_id != 0:
                mycobot.move(robot, number_id, 2048)

    def move(self, robot, encoder_id, power):
        """
        :param encoder_id: servo ID
        :param power: A power to move to the point
        """
        if encoder_id not in runtime_data['servo_status']:
            runtime_data['servo_status'][encoder_id] = power
        if capabilities['servo']['servo_range'][str(encoder_id)][1] >= (
                runtime_data['servo_status'][encoder_id] + power) >= \
                capabilities['servo']['servo_range'][str(encoder_id)][0]:
            runtime_data['servo_status'][encoder_id] = runtime_data['servo_status'][encoder_id] + power

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


runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'servo_status': {},
    'servo_position': {}
}

# # # FEAGI registration # # #
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
feagi_host, api_port = FEAGI.feagi_setting_for_registration()
runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_host=feagi_host, api_port=api_port)
ipu_channel_address = FEAGI.feagi_inbound(runtime_data["feagi_state"]['feagi_inbound_port_gazebo'])
opu_channel_address = FEAGI.feagi_outbound(network_settings['feagi_host'],
                                           runtime_data["feagi_state"]['feagi_outbound_port'])
feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address)
feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
api_address = FEAGI.feagi_gui_address(feagi_host, api_port)
stimulation_period_endpoint = FEAGI.feagi_api_burst_engine()
burst_counter_endpoint = FEAGI.feagi_api_burst_counter()
network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

# # Camera section
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))

# Necessry variables section:
flag = True
keyboard_flag = True
msg_counter = 0

# mycobot initialize section
# todo: To figure how to make this part scalable
global_arm = dict()
mycobot = Arm()
arm = mycobot.connection_initialize()
mycobot.initialize(arm, capabilities['servo']['count'])
global_arm['0'] = arm  # Is this even allowed?
for i in range(1, capabilities['servo']['count'], 1):
    old_data[i] = {} # Just to be able to see the difference and move the current position to new position

# ROS 2 initialize section
rclpy.init(args=None)
executor = rclpy.executors.MultiThreadedExecutor()  # uncomment this when we are starting with sensors
servo = Servo(count=capabilities['servo']['count'], identifier=capabilities['servo']['topic_identifier'])
servo_position = ServoPosition()
executor.add_node(servo_position)
executor.add_node(servo)
executor_thread = Thread(target=executor.spin, daemon=True)
executor_thread.start()

while keyboard_flag:
    try:
        #         # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #         #     if keyboard_flag:
        #         #         image = frame.array
        #         #         rawCapture.truncate(0)
        #         #         if capabilities['camera']['disabled'] is not True:
        #         #             retina_data = retina.frame_split(image,
        #         #                                              capabilities['camera']['retina_width_percent'],
        #         #                                              capabilities['camera']['retina_height_percent'])
        #         #             for i in retina_data:
        #         #                 if 'C' in i:
        #         #                     retina_data[i] = retina.center_data_compression(retina_data[i],
        #         #                                                                     capabilities['camera'][
        #         #                                                                         "central_vision_compression"]
        #         #                                                                     )
        #         #                 else:
        #         #                     retina_data[i] = retina.center_data_compression(retina_data[i],
        #         #                                                                     capabilities['camera']
        #         #                                                                     ['peripheral_vision_compression'])
        #         #             rgb = dict()
        #         #             rgb['camera'] = dict()
        #         #             if previous_data_frame == {}:
        #         #                 for i in retina_data:
        #         #                     previous_name = str(i) + "_prev"
        #         #                     previous_data_frame[previous_name] = {}
        #         #             for i in retina_data:
        #         #                 name = i
        #         #                 if 'prev' not in i:
        #         #                     data = retina.ndarray_to_list(retina_data[i])
        #         #                     if 'C' in i:
        #         #                         previous_name = str(i) + "_prev"
        #         #                         rgb_data, previous_data_frame[previous_name] = \
        #         #                             retina.get_rgb(data,
        #         #                                            capabilities[
        #         #                                                'camera'][
        #         #                                                'central_vision_compression'],
        #         #                                            previous_data_frame[
        #         #                                                previous_name],
        #         #                                            name,
        #         #                                            capabilities['camera']['deviation_threshold'])
        #         #                     else:
        #         #                         previous_name = str(i) + "_prev"
        #         #                         rgb_data, previous_data_frame[previous_name] = \
        #         #                             retina.get_rgb(data, capabilities['camera']['peripheral_vision_compression'],
        #         #                                            previous_data_frame[previous_name], name,
        #         #                                            capabilities['camera']['deviation_threshold'])
        #         #                     for a in rgb_data['camera']:
        #         #                         rgb['camera'][a] = rgb_data['camera'][a]
        #         #         else:
        #         #             rgb = {}
        #         #
        #         #     message_to_feagi, bat = FEAGI.compose_message_to_feagi(original_message=rgb, data=message_to_feagi)
        #
        #         message_to_feagi['timestamp'] = datetime.now()
        #         message_to_feagi['counter'] = msg_counter
        #         feagi_ipu_channel.send(message_to_feagi)
        #         message_to_feagi.clear()
        #         msg_counter += 1
        #         flag += 1
        #         if flag == 10:
        #             feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
        #             feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
        #             flag = 0
        #             if msg_counter < feagi_burst_counter:
        #                 feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
        #                 if feagi_burst_speed != network_settings['feagi_burst_speed']:
        #                     network_settings['feagi_burst_speed'] = feagi_burst_speed
        message_from_feagi = feagi_opu_channel.receive()
        if message_from_feagi is not None:
            opu_data = FEAGI.opu_processor(message_from_feagi)
            if 'motor' in opu_data:
                if opu_data['motor'] is not {}:
                    for data_point in opu_data['motor']:
                        device_power = opu_data['motor'][data_point]
                        device_power = mycobot.power_convert(data_point, device_power)
                        device_id = mycobot.encoder_converter(data_point)
                        test = runtime_data['servo_status'][device_id] + (device_power * 10)
                        if capabilities['servo']['servo_range'][str(device_id)][1] >= test >= \
                                capabilities['servo']['servo_range'][str(device_id)][0]:
                            runtime_data['servo_status'][device_id] += device_power * 10
    except KeyboardInterrupt as ke:  # Keyboard error
        arm.release_all_servos()
        keyboard_flag = False
        servo.destroy_node()
arm.release_all_servos()
rclpy.shutdown()
