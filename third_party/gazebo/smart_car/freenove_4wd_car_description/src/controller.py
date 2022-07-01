#!/usr/bin/env python3

# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import sys
import traceback
import router
import math
import geometry_msgs.msg
import rclpy
import std_msgs.msg
import configuration
import os
import os.path
import subprocess
import requests
import xml.etree.ElementTree as ET


from std_msgs.msg import String
from subprocess import PIPE, Popen
from configuration import message_to_feagi
from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, BatteryState
from rclpy.qos import qos_profile_sensor_data
from configuration import *
from threading import Thread
from datetime import datetime

runtime_data = {
    "cortical_data": {},
    "current_burst_id": None,
    "stimulation_period": None,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {},
    "battery_charge_level": 1,
    'motor_status': {},
    'servo_status': {},
    'GPS': {}
}

runtime_data["GPS"]["x"] = dict()
runtime_data["GPS"]["y"] = dict()
runtime_data["GPS"]["z"] = dict()
runtime_data["GPS"]["counter"] = dict()
runtime_data["GPS"]["x_counter"] = dict()
runtime_data["GPS"]["y_counter"] = dict()
runtime_data["GPS"]["n_x_counter"] = dict()
runtime_data["GPS"]["n_y_counter"] = dict()
runtime_data["GPS"]["counter"] = 0
runtime_data["GPS"]["x_counter"] = 0
runtime_data["GPS"]["y_counter"] = 0
runtime_data["GPS"]["n_x_counter"] = 0
runtime_data["GPS"]["n_y_counter"] = 0

location_stored = dict()
tile_margin = dict()
old_list = location_stored.copy()
location_stored["0"] = [0,0] #by default

layer_index = [[0,0]]#Starts at 0,0 by default
layer_dimesion = [Gazebo_world["size_of_plane"]["x"],Gazebo_world["size_of_plane"]["y"]]
##Don't change above line, change it in configuration.py inside "size_of_plane"

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

launch_checker = os.path.exists('type_res.txt')
if launch_checker:
    print("low res")
    model_name = "models/sdf/low_res_freenove_smart_car"  # This is for the gazebo sdf file path
else:
    model_name = Model_data["path_to_robot"] + Model_data["file_name"]  # This is for the gazebo sdf file path
    print("no low res")

robot_name = Model_data["file_name"]  # This is the model name in gazebo without sdf path involves.
x = str(capabilities["position"]["x"])
y = str(capabilities["position"]["y"])
z = str(capabilities["position"]["z"])
first_part = "ign service -s /world/free_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 300 --req 'sdf_filename:'\'\""
second_part = model_name + "\" pose: {position: { x: " + x + ", y"
third_part = ": " + y + ", z: " + z + "}}\' &"
add_model = first_part + second_part + third_part
os.system(add_model)
### Just to define for future. This section will need to be updated later once its finalized
remove_model = """
ign service -s /world/free_world/remove \
--reqtype ignition.msgs.Entity \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'name: "freenove_smart_car" type: MODEL'
""" ##not used atm


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


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def publisher_initializer(model_name, topic_count, topic_identifier):
    node = rclpy.create_node('Controller_py')

    target_node = {}
    for target in range(topic_count):
        topic_string = topic_identifier + str(target)
        target_node[target] = node.create_publisher(std_msgs.msg.Float64, topic_string, 10)

    # is this used for anything?
    node.create_publisher(geometry_msgs.msg.Twist, model_name, 10)

    return target_node


class ScalableSubscriber(Node):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name)
        self.subscription = self.create_subscription(
            msg_type,
            topic,
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.topic = topic
        self.counter = 0

    def listener_callback(self, msg):
        # self.get_logger().info("Raw Message: {}".format(msg))
        try:
            formatted_msg = self.msg_processor(msg, self.topic)
            compose_message_to_feagi(original_message=formatted_msg)
            self.counter += 1
        except Exception as e:
            print("Error in listener callback...", e)
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)

    @staticmethod
    def msg_processor(msg, msg_type):
        # TODO: give each subclass a specific msg processor method?
        # TODO: add an attribute that explicitly defines message type (instead of parsing topic name)?
        if 'ultrasonic' in msg_type and msg.ranges[1]:
            return {
                msg_type: {
                    idx: val for idx, val in enumerate([msg.ranges[1]])
                }
            }
        elif 'IR' in msg_type:
            rgb_vals = list(msg.data)
            avg_intensity = sum(rgb_vals) // len(rgb_vals)

            sensor_topic = msg_type.split('/')[0]
            sensor_id = int(''.join(filter(str.isdigit, sensor_topic)))

            # print("\n***\nAverage Intensity = ", avg_intensity)
            if avg_intensity > 25:
                return {
                    'ir': {
                        sensor_id: False
                    }
                }
            else:
                return {
                    'ir': {
                        sensor_id: True
                    }
                }
        # elif 'battery' in msg_type and msg.percentage:
        #     return {
        #         'battery': {
        #             idx: val for idx, val in enumerate([msg.percentage])
        #         }
        #     }


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
    message_to_feagi["data"]["sensory_data"]["battery"] = {1: runtime_data["battery_charge_level"] / 100}


class UltrasonicSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class BatterySubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class IRSubscriber(ScalableSubscriber):
    def __init__(self, subscription_name, msg_type, topic):
        super().__init__(subscription_name, msg_type, topic)


class Battery:
    def __init__(self):
        print("Battery has been initialized")
        if "battery" in capabilities:
            runtime_data["battery_charge_level"] = capabilities["battery"]["capacity"]

    @staticmethod
    def charge_battery(intensity=1):
        print("Charging battery    + + + + + +   *************    ^^^^^^^^^^^^^^^^^^^")
        runtime_data["battery_charge_level"] += capabilities["battery"]["charge_increment"] * intensity
        if runtime_data["battery_charge_level"] > capabilities["battery"]["capacity"]:
            runtime_data["battery_charge_level"] = capabilities["battery"]["capacity"]

    @staticmethod
    def discharge_battery(intensity=1):
        print("dis-Charging battery    - - - - - -   *************    ^^^^^^^^^^^^^^^^^^^")
        runtime_data["battery_charge_level"] -= capabilities["battery"]["charge_increment"] * intensity
        if runtime_data["battery_charge_level"] < 0:
            runtime_data["battery_charge_level"] = 0

    @staticmethod
    def consume_battery():
        # print("Consuming battery ")
        runtime_data["battery_charge_level"] -= capabilities["battery"]["depletion_per_burst"]


class Motor:
    def __init__(self, count, identifier, model):
        self.motor_node = publisher_initializer(model_name=model, topic_count=count, topic_identifier=identifier)

        # todo: figure a way to extract wheel parameters from the model
        self.wheel_diameter = capabilities["motor"]["wheel_diameter"]

    def move(self, feagi_device_id, power):
        try:
            if feagi_device_id > 2 * capabilities['motor']['count']:
                print("Warning! Number of motor channels from FEAGI exceed available device count!")
            # Translate feagi_device_id to device backward and forward motion to individual devices
            device_index = feagi_device_id // 2
            if feagi_device_id % 2 == 1:
                power *= -1

            device_position = std_msgs.msg.Float64()

            if device_index not in runtime_data['motor_status']:
                runtime_data['motor_status'][device_index] = 0

            device_current_position = runtime_data['motor_status'][device_index]
            # device_position.data = float(
            #     (power * network_settings['feagi_burst_speed'] * configuration.capabilities["motor"]["motor_power"]) + device_current_position)
            device_position.data = float((power * capabilities["motor"]["power_coefficient"] * 6.28319) + device_current_position)
            runtime_data['motor_status'][device_index] = device_position.data
            # print("device index, position, power = ", device_index, device_position.data, power)
            self.motor_node[device_index].publish(device_position)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)


class Servo:
    def __init__(self, count, identifier, model):
        self.servo_node = publisher_initializer(model_name=model,
                                                topic_count=count,
                                                topic_identifier=identifier)
        self.device_position = std_msgs.msg.Float64()
        # todo: figure a way to extract servo parameters from the model
        self.servo_ranges = {0: [0, 3.14],
                             1: [0, 3.14]}

    def set_default_position(self):
        try:
            # Setting the initial position for the servo
            servo_0_initial_position = float(1.5708)
            self.device_position.data = servo_0_initial_position
            self.servo_node[0].publish(self.device_position)
            runtime_data['servo_status'][0] = self.device_position.data
            print("Servo 0 was moved to its initial position")

            servo_1_initial_position = float(1.3)
            self.device_position.data = servo_1_initial_position
            self.servo_node[1].publish(self.device_position)
            runtime_data['servo_status'][1] = self.device_position.data
            print("Servo 1 was moved to its initial position")
        except Exception as e:
            print("Error while setting initial position for the servo:", e)

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

    def move(self, feagi_device_id, power):
        try:
            if feagi_device_id > 2 * capabilities['servo']['count']:
                print("Warning! Number of servo channels from FEAGI exceed available Motor count!")
            # Translate feagi_motor_id to motor backward and forward motion to individual motors
            device_index = feagi_device_id // 2
            if feagi_device_id % 2 == 1:
                power *= -1

            if device_index not in runtime_data['servo_status']:
                runtime_data['servo_status'][device_index] = device_index

            device_current_position = runtime_data['servo_status'][device_index]
            # print("servo ", device_index, device_current_position)
            self.device_position.data = float((power * network_settings['feagi_burst_speed'] / 20) +
                                              device_current_position)

            self.device_position.data = self.keep_boundaries(device_id=device_index,
                                                             current_position=self.device_position.data)

            runtime_data['servo_status'][device_index] = self.device_position.data
            # print("device index, position, power = ", device_index, self.device_position.data, power)
            self.servo_node[device_index].publish(self.device_position)
        except Exception:
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)


class PosInit:
    def __init__(self):
        init_pos_x = capabilities['position']['x']
        init_pos_y = capabilities['position']['y']

    def reset_position(self):
        print("## ## ## ## Resetting robot position ## ## ## ##")
        # Remove the robot
        x = str(capabilities["position"]["x"])
        y = str(capabilities["position"]["y"])
        z = str(capabilities["position"]["z"])
        name = Model_data["file_name"].replace(".sdf", "")
        first_part_r = "ign service -s /world/free_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 300 --req \'name: "
        second_part_r = ' "' + name + '" ' + ", position: { x: " + x + ", y: "
        third_part_r = y + ", z: " + z + "}' &"
        respawn = first_part_r + second_part_r + third_part_r
        print("++++++++++++++++++++++++++")
        print(respawn)
        os.system(respawn)
        Pose().remove_floor()
        # runtime_data["motor_status"] = {}
        # runtime_data['servo_status'] = {}
        # runtime_data['battery_charge_level'] = 1
        # Create the robot
        # os.system(add_model)
        servo.set_default_position()


class TileManager(Node):
    def __init__(self):
        super().__init__('pose')
        self.publisher_ = self.create_publisher(String, '/GPS', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.pose_updated)
        self.i = 0
        self.tile_tracker = dict()
        """
        tile_tracker = {
            "ground_plane0": {
                "index": [0, 0],
                "visibility": True
                },
            "ground_plane1": {
                "index": [0, 1],
                "visibility" False
                },
            "ground_plane2": [-1, 1],
            ...
            ...       
        }
        """
        self.tile_dimensions = [configuration.Gazebo_world["size_of_plane"]["x"],
                                configuration.Gazebo_world["size_of_plane"]["y"]]
        self.tile_margin = configuration.Gazebo_world["margin"]
        self.last_tile_name_id = 0
        self.tile_name_prefix = "ground_plane"
        self.tile_tracker["ground_plane"] = {
            "index": [0, 0],
            "visibility": True
        }
        self.visible_tiles = list()
        self.visible_tiles.append([0, 0])

    def pose_updated(self):
        """
        This updates robot's GPS automatically
        """
        msg = String()
        msg.data = str(runtime_data["GPS"])
        # Pose data
        name = Model_data["file_name"]
        name = name.replace(".sdf", "")
        pose = subprocess.Popen(["ign topic -e  -t world/free_world/pose/info -n1 | grep \"" + name +"\" -A6"],
                                shell=True, stdout=PIPE)
        pose_xyz = pose.communicate()[0].decode("utf-8")
        for item in pose_xyz.split("\n"):
            if "x: " in item:
                number_only = item.replace("x: ", "")
                runtime_data["GPS"]["x"] = float(number_only)
            if "y: " in item:
                number_only = item.replace("y: ", "")
                runtime_data["GPS"]["y"] = float(number_only)
            if "z: " in item:
                number_only = item.replace("z: ", "")
                runtime_data["GPS"]["z"] = float(number_only)
        self.publisher_.publish(msg)

    def tile_index_to_xyz(self, tile_index):
        """
        Return the coordinate of the center of the tile
        """
        center_of_tile_location = [self.tile_dimensions[0] * tile_index[0] - self.tile_dimensions[0] / 2,
                                   self.tile_dimensions[1] * tile_index[1] - self.tile_dimensions[1] / 2, 0]
        return center_of_tile_location

    def gps_to_tile_index(self, gps):
        """
        Identifies what tile_index is associated with robot GPS location.
        """

        tile_index = [
            int((gps[0] + (self.tile_dimensions[0] / 2)) // self.tile_dimensions[0]),
            int((gps[1] + (self.tile_dimensions[1] / 2)) // self.tile_dimensions[1])
        ]
        return tile_index

    def tile_index_to_name(self, tile_index):
        for tile_name in self.tile_tracker:
            if self.tile_tracker[tile_name]["index"] == tile_index:
                return tile_name
            else:
                print("Warning! Tile tracker does not container a tile matching index ", tile_index, self.tile_tracker)
                return None

    def add_tile(self, tile_index):
        # Tile location will be in the form of [x, y, z=0]
        tile_location = self.tile_index_to_xyz(tile_index)

        tile_location_x = str(tile_location[0] + (self.tile_dimensions[0] / 2))[:4]
        tile_location_y = str(tile_location[1] + (self.tile_dimensions[1] / 2))[:4]
        tile_location_z = str(0)

        tile_name = self.tile_index_to_name(tile_index)
        if tile_name is None:
            self.last_tile_name_id += 1
            tile_name = self.tile_name_prefix + str(self.last_tile_name_id)
            self.tile_tracker[tile_name] = {}
            self.tile_tracker[tile_name]["index"] = tile_index
            self.tile_tracker[tile_name]["visibility"] = True

        first_phrase = "ign service -s /world/free_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 300 --req 'sdf_filename: ''\"models/sdf/new_ground.sdf\""
        second_phrase = " pose: {position: {x:" + tile_location_x + ", y:" + tile_location_y + ", z:" + tile_location_z + "}} '\'name: \"" + tile_name + "\" '\' allow_renaming: false' &"

        print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", first_phrase + second_phrase)
        os.system(first_phrase + second_phrase)
        print("++++++++++++++++++++++++++++++++++++\n", first_phrase + second_phrase)
        print(">>> New tile added:", tile_name, tile_location, tile_index)

    def remove_tile(self, tile_index):
        print("Attemping to remove:", tile_index)
        print("------------------------------------")
        print("------------------------------------")
        print("------------------------------------")
        tile_name = self.tile_index_to_name(tile_index)
        if tile_name in self.tile_tracker:
            command = "ign service -s /world/free_world/remove \
            --reqtype ignition.msgs.Entity \
            --reptype ignition.msgs.Boolean \
            --timeout 300 \
            --req 'name: \"" + tile_name + "\" type: MODEL' &"
            os.system(command)
            self.tile_tracker[tile_name]["visibility"] = False
        else:
            print("Error! Tile removal failed. Tile name not found within tile_tracker", tile_index, tile_name)

    def add_visible_tile(self, tile_index):
        if tile_index not in self.visible_tiles:
            self.visible_tiles.append(tile_index)
            print("MMMMM=+++")

    def tile_visibility_checker(self, gps):
        """
        Returns the list of tiles to be visible in the form of [[0,0], [1,5], ...]
        """
        current_tile_index = self.gps_to_tile_index(gps=gps)
        #print("@@@@@@@ --------------------- current tile index:", current_tile_index)
        # center_of_tile = self.tile_index_to_xyz(tile_index=current_tile_index)
        #
        # robot_relative_loc_to_tile = [gps[0] - center_of_tile[0],
        #                               gps[1] - center_of_tile[1], 0]

        # adding the current tile to the list of visible tiles by default
        # self.add_visible_tile(current_tile_index)

        self.visible_tiles = [current_tile_index]

        #print(">>>> > > > > > >> > > Visible Tiles:", self.visible_tiles)

        # if robot_relative_loc_to_tile[0] > self.tile_dimensions[0] / 2 - self.tile_dimensions[0] * self.tile_margin:
        #     self.add_visible_tile([[current_tile_index[0] + 1, current_tile_index[0]]])
        #     print("-------------------------------   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ -- 1")
        #
        # if robot_relative_loc_to_tile[0] < -1 * self.tile_dimensions[0] / 2 + self.tile_dimensions[0] * self.tile_margin:
        #     self.add_visible_tile([current_tile_index[0] - 1, current_tile_index[1]])
        #     print("-------------------------------   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ -- 2")
        #
        # if robot_relative_loc_to_tile[1] > self.tile_dimensions[1] / 2 - self.tile_dimensions[1] * self.tile_margin:
        #     self.add_visible_tile([current_tile_index[0], current_tile_index[1] + 1])
        #     print("-------------------------------   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ -- 3")
        #
        # if robot_relative_loc_to_tile[1] < -1 * self.tile_dimensions[1] / 2 + self.tile_dimensions[1] * self.tile_margin:
        #     self.add_visible_tile([current_tile_index[0], current_tile_index[1] - 1])
        #     print("-------------------------------   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ -- 4")

        #print("tile visibility checker output:", self.visible_tiles)
        # todo: Need to remove the tiles that are not needed if robot is inside the margin on all sides
        # if robot_relative_loc_to_tile[0] > -1 ...

    def tile_update(self, gps):
        """
        Based on robot location, turns the needed tile on or off
        """
        # Remove tiles that are not supposed to be displayed
        self.tile_visibility_checker(gps=gps)
        # for tile in self.tile_tracker:
        #     if self.tile_tracker[tile]["index"] not in self.visible_tiles:
        #         self.remove_tile(self.tile_tracker[tile]["index"])

        # Add the new tiles that has matched the display criteria
        existing_visible_tiles = []
        for tile in self.tile_tracker:
            if self.tile_tracker[tile]["visibility"]:
                existing_visible_tiles.append(self.tile_tracker[tile]["index"])

        for tile_index in self.visible_tiles:
            if tile_index not in existing_visible_tiles:
                self.add_tile(tile_index=tile_index)
        #print("\n************************\nActive tile list:", self.tile_tracker)


def update_physics():
    physics_data_array = [str(Model_data['mu']), str(Model_data['mu2']), str(Model_data['fdir1']), str(Model_data['slip1']),
                          str(Model_data['slip2'])]
    physics_data_array[2] = physics_data_array[2].replace(",", "")
    physics_data_array[2] = physics_data_array[2].replace("[", "")
    physics_data_array[2] = physics_data_array[2].replace("]", "")
    file = open((Model_data["path_to_robot"] + Model_data["file_name"]), "r")
    file2 = open((Model_data["path_to_robot"] + Model_data["file_name"]), "r")
    original_file = file2.read()
    file_sdf = open("empty.sdf", "w")
    new_sdf = original_file
    file_xml = ET.parse(file)
    myroot = file_xml.getroot()
    flag = 0
    for i in myroot.find("./model/link/collision/surface/friction/ode"):
        current = i.text
        i.text = physics_data_array[flag]
        old_line = "<" + str(i.tag) + ">" + current + "</" + str(i.tag) + ">"
        new_line = "<" + str(i.tag) + ">" + i.text + "</" + str(i.tag) + ">"
        new_sdf = new_sdf.replace(old_line, new_line)
        flag += 1

    file_sdf.write(new_sdf)
    file_sdf.close()
    file2.close()
    file.close()

# class Pose(Node):
#     def __init__(self):
#         super().__init__('pose')
#         self.publisher_ = self.create_publisher(String, '/GPS', 10)
#         timer_period = 0.5
#         self.timer = self.create_timer(timer_period, self.pose_updated)
#         self.i = 0
#
#     def pose_updated(self):
#         """
#         This updates robot's GPS automatically
#         """
#         msg = String()
#         msg.data = str(runtime_data["GPS"])
#         # Pose data
#         pose = subprocess.Popen(["ign topic -e  -t world/free_world/pose/info -n1 | grep \"freenove_smart_car\" -A6"],
#                                 shell=True, stdout=PIPE)
#         pose_xyz = pose.communicate()[0].decode("utf-8")
#         for item in pose_xyz.split("\n"):
#             if "x: " in item:
#                 number_only = item.replace("x: ", "")
#                 runtime_data["GPS"]["x"] = float(number_only)
#             if "y: " in item:
#                 number_only = item.replace("y: ", "")
#                 runtime_data["GPS"]["y"] = float(number_only)
#             if "z: " in item:
#                 number_only = item.replace("z: ", "")
#                 runtime_data["GPS"]["z"] = float(number_only)
#         self.publisher_.publish(msg)
#
#     @staticmethod
#     def location_updated(new_location):
#         """
#         This function adds per new location. Let's say, you add 2,2 to the list so it can remember the location
#         """
#         total_length = len(location_stored) #This doesn't include 0-index so we use this advantage
#         location_stored[str(total_length)] = new_location
#
#     @staticmethod
#     def margin_defined(margin, list_index, dimesions, counter):
#         """
#         :param margin: This is the "limit" to detect if robot past the margin. By default, it would be 5% or 0.05 so if your dimesion is 10x10, it would be 9.95 x 9.95. Once
#         the robot's gps reached to the margin, it will create a new plane on whichever the robot reached.
#         :param list_index: The full list of locations stored
#         :param dimesions: the size of dimesion so it can do math formula properly
#         :return: new list of locations with margin defined
#         """
#         new_list = [dimesions[0]/2 + (dimesions[0] * list_index[str(counter)][0] - margin * dimesions[0]),dimesions[0]/2 + dimesions[1] * list_index[str(counter)][1] - (margin * dimesions[1])]
#         return new_list
#
#     @staticmethod
#     def converter_for_gazebo(list_location_data, dimesion):
#         return [list_location_data[0] * dimesion[0],list_location_data[1] * dimesion[1]]
#
#     @staticmethod
#     def floor_generated(x, y):
#         z = 0 #Always zero since floor doesn't float. This would be a static floor
#         first_phrase = "ign service -s /world/free_world/create " \
#                        "--reqtype ignition.msgs.EntityFactory " \
#                        "--reptype ignition.msgs.Boolean " \
#                        "--timeout 300 " \
#                        "--req 'sdf_filename: ''\"models/sdf/new_ground.sdf\""
#         second_phrase = " pose: {position: {x: %s, y: %s, z: %s}} '\'name: \"ground_plane\" '\' allow_renaming: true' &" % (
#             x, y, z)
#         os.system(first_phrase + second_phrase)
#         print(first_phrase + second_phrase)
#
#     @staticmethod
#     def remove_floor():
#         global old_list
#         total = int(runtime_data["GPS"]["counter"])
#         for i in range(total):
#             name = "ground_plane_" + str(i)
#             command = "ign service -s /world/free_world/remove \
#             --reqtype ignition.msgs.Entity \
#             --reptype ignition.msgs.Boolean \
#             --timeout 300 \
#             --req 'name: \"" + name + "\" type: MODEL' &"
#             os.system(command)
#         runtime_data["GPS"]["counter"] = 0
#         runtime_data["GPS"]["n_y_counter"] = 0
#         runtime_data["GPS"]["n_x_counter"] = 0
#         runtime_data["GPS"]["y_counter"] = 0
#         runtime_data["GPS"]["x_counter"] = 0
#         location_stored.clear()
#         old_list.clear()
#         location_stored["0"] = [0, 0]
#         old_list = location_stored.copy()
#
#
#
#     @staticmethod
#     def gazebo_process(current_list):
#         """
#         main script for world controls
#         """
#         global old_list ##Is there another way to get rid of this though?
#         flag = 0
#
# ##      Don't forget to clean this up.
#         print("Current list: ", location_stored)
#         print("Old list: ", old_list)
#         print("GPS: ", runtime_data["GPS"])
#         print("counter: ", runtime_data["GPS"]["counter"])
#
#
#         #Generate new margin list
#         if old_list != current_list:
#             old_list = current_list.copy()
#             defined_margin_list = Pose().margin_defined(Gazebo_world["margin"], current_list, layer_dimesion, runtime_data["GPS"]["counter"])
#             tile_margin[runtime_data["GPS"]["counter"]] = defined_margin_list
#             print("defined_margin_list: ", tile_margin)
#             runtime_data["GPS"]["counter"] = runtime_data["GPS"]["counter"] + 1
#
#         ## Check if it is greater than margin
#         if runtime_data["GPS"]["x"] > tile_margin[runtime_data["GPS"]["counter"]-1][0]: #minus to keep it away frmo out of index
#             runtime_data["GPS"]["x_counter"] = runtime_data["GPS"]["x_counter"] + 1
#             flag = 1
#         elif runtime_data["GPS"]["y"] > tile_margin[runtime_data["GPS"]["counter"]-1][1]:
#             print(runtime_data["GPS"]["y_counter"] , " > ", tile_margin[runtime_data["GPS"]["y_counter"]][1])
#             runtime_data["GPS"]["y_counter"] = runtime_data["GPS"]["y_counter"] + 1
#             flag = 1
#         elif runtime_data["GPS"]["x"] > tile_margin[runtime_data["GPS"]["counter"]-1][0] and runtime_data["GPS"]["y"] > tile_margin[runtime_data["GPS"]["counter"]-1][1]:
#             runtime_data["GPS"]["y_counter"] = runtime_data["GPS"]["y_counter"] + 1
#             runtime_data["GPS"]["x_counter"] = runtime_data["GPS"]["x_counter"] + 1
#             flag = 1
#         if flag == 1:
#             flag = 0
#             Pose().location_updated([runtime_data["GPS"]["x_counter"], runtime_data["GPS"]["y_counter"]])
#             new_data = Pose().converter_for_gazebo([runtime_data["GPS"]["x_counter"], runtime_data["GPS"]["y_counter"]], layer_dimesion)
#             print("++++++++++++++++++++++++++++++: ", new_data)
#             print("COUNTER: ", runtime_data["GPS"]["counter"])
#             Pose().floor_generated(new_data[0], new_data[1])


def main(args=None):
    print("Connecting to FEAGI resources...")

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_outbound_port']

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]
    api_address = 'http://' + feagi_host + ':' + api_port
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'

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

    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate controller classes with Publisher nature
    motor = Motor(count=capabilities['motor']['count'], identifier=capabilities['motor']['topic_identifier'],
                  model='freenove_motor')
    servo = Servo(count=capabilities['servo']['count'], identifier=capabilities['servo']['topic_identifier'],
                  model='freenove_servo')

    position_init = PosInit()

    # Instantiate controller classes with Subscriber nature

    # Ultrasonic Sensor
    ultrasonic_feed = UltrasonicSubscriber('ultrasonic0', LaserScan, 'ultrasonic0')
    executor.add_node(ultrasonic_feed)

    # GPS
    pose = TileManager()
    pose.pose_updated()
    executor.add_node(pose)

    # Battery
    # Moving away from using the Gazebo based battery model and adopting a controller based model instead
    # battery_feed = BatterySubscriber('battery', BatteryState, 'model/freenove_smart_car/battery/linear_battery/state')
    # # todo: Change the topic name and make it scalable
    # executor.add_node(battery_feed)
    battery = Battery()

    # Infrared Sensor
    ir_feeds = {}
    ir_topic_id = capabilities['infrared']['topic_identifier']
    for ir_node in range(capabilities['infrared']['count']):
        ir_feeds[ir_node] = IRSubscriber(f'infrared_{ir_node}', Image, f'{ir_topic_id}{ir_node}/image')
        executor.add_node(ir_feeds[ir_node])

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    msg_counter = 0
    flag = 0

    # Positioning servos to a default position
    servo.set_default_position()


    try:
        while True:
            robot_pose = [runtime_data["GPS"]["x"], runtime_data["GPS"]["y"], runtime_data["GPS"]["z"]]
            pose.tile_update(robot_pose)
            #print("Current robot pose_:", robot_pose)

            # Process OPU data received from FEAGI and pass it along
            message_from_feagi = feagi_opu_channel.receive()
            battery.consume_battery()
            try:
                opu_data = message_from_feagi["opu_data"]
                if opu_data is not None:
                    if 'o__mot' in opu_data:
                        for data_point in opu_data['o__mot']:
                            data_point = block_to_array(data_point)
                            device_id = data_point[0]
                            device_power = data_point[2]
                            motor.move(feagi_device_id=device_id, power=device_power)
                    if 'o__ser' in opu_data:
                        if opu_data['o__ser']:
                            for data_point in opu_data['o__ser']:
                                data_point = block_to_array(data_point)
                                device_id = data_point[0]
                                device_power = data_point[2]
                                servo.move(feagi_device_id=device_id, power=device_power)
                    if 'o_cbat' in opu_data:
                        if opu_data['o__bat']:
                            for data_point in opu_data['o_cbat']:
                                intensity = data_point[2]
                                battery.charge_battery(intensity=intensity)

                    if 'o_dbat' in opu_data:
                        if opu_data['o__bat']:
                            for data_point in opu_data['o_dbat']:
                                intensity = data_point[2]
                                battery.discharge_battery(intensity=intensity)

                    if 'o_init' in opu_data:
                        if opu_data['o_init']:
                            position_init.reset_position()

                control_data = message_from_feagi['control_data']
                if control_data is not None:
                    if 'motor_power_coefficient' in control_data:
                        capabilities["motor"]["power_coefficient"] = float(control_data['motor_power_coefficient'])
                    if 'robot_starting_position' in control_data:
                        capabilities["position"]["x"] = float(control_data['robot_starting_position'][0])
                        capabilities["position"]["y"] = float(control_data['robot_starting_position'][1])
                        capabilities["position"]["z"] = float(control_data['robot_starting_position'][2])

                model_data = message_from_feagi['model_data']
                if model_data is not None:
                    update_flag = False
                    if 'file_name' in model_data:
                        if Model_data["file_name"] != model_data['file_name']:
                            Model_data["file_name"] = model_data['file_name']
                            update_flag = True
                    if 'mu' in model_data:
                        if Model_data["mu"] != model_data['mu']:
                            Model_data["mu"] = float(model_data['mu'])
                            update_flag = True
                    if 'mu2' in model_data:
                        if Model_data["mu2"] != model_data["mu2"]:
                            Model_data["mu2"] = float(model_data['mu2'])
                            update_flag = True
                    if 'fdir' in model_data:
                        if Model_data["fdir1"] != model_data['fdir']:
                            Model_data["fdir1"] = model_data['fdir']
                            update_flag = True
                    if 'slip1' in model_data:
                        if Model_data["slip1"] != model_data['slip1']:
                            Model_data["slip1"] = float(model_data['slip1'])
                            update_flag = True
                    if 'slip2' in model_data:
                        if Model_data["slip2"] != model_data['slip2']:
                            Model_data["slip2"] = float(model_data['slip2'])
                            update_flag = True
                    if update_flag:
                        update_physics()

            except Exception:
                pass
                # print("")
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            if message_from_feagi is not None:
                message_from_feagi['pose'] = robot_pose
            feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
            msg_counter += 1
            flag += 1
            if flag == 10:
                feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
                feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
                flag = 0
                if feagi_burst_speed != network_settings['feagi_burst_speed']:
                    network_settings['feagi_burst_speed'] = feagi_burst_speed
                    msg_counter = feagi_burst_counter
            sleep(network_settings['feagi_burst_speed'])
    except KeyboardInterrupt:
        pass

    ultrasonic_feed.destroy_node()
    pose.destroy_node()
    # battery_feed.destroy_node()

    for ir_node in ir_feeds:
        ir_feeds[ir_node].destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()




