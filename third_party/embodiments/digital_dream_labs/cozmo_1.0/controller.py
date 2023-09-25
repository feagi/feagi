#!/usr/bin/env python
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

import time
from PIL import Image
import pycozmo
from feagi_agent import feagi_interface as FEAGI
from feagi_agent import retina as retina
from feagi_agent import pns_gateway as pns
from configuration import *
import requests
import sys
import os
import threading
import asyncio
from datetime import datetime
from collections import deque
import numpy as np
from time import sleep
import traceback
import cv2

runtime_data = {
    "current_burst_id": 0,
    "feagi_state": None,
    "cortical_list": (),
    "battery_charge_level": 1,
    "host_network": {},
    'motor_status': {},
    'servo_status': {}
}

rgb_array = dict()
rgb_array['current'] = {}
previous_data_frame = dict()
robot = {'accelerator': [], "ultrasonic": [], "gyro": [], 'servo_head': [], "battery": []}


def on_robot_state(cli, pkt: pycozmo.protocol_encoder.RobotState):
    """
    timestamp: The timestamp associated with the robot state.
    pose_frame_id: The ID of the frame of reference for the robot's pose.
    pose_origin_id: The ID of the origin for the robot's pose.
    pose_x, pose_y, pose_z: The x, y, and z coordinates of the robot's pose.
    pose_angle_rad: The angle of the robot's pose in radians.
    pose_pitch_rad: The pitch angle of the robot's pose in radians.
    lwheel_speed_mmps: Speed of the left wheel in millimeters per second.
    rwheel_speed_mmps: Speed of the right wheel in millimeters per second.
    head_angle_rad: The angle of the robot's head in radians.
    lift_height_mm: The height of the lift in millimeters.
    accel_x, accel_y, accel_z: Acceleration values along the x, y, and z axes.
    gyro_x, gyro_y, gyro_z: Gyroscopic values along the x, y, and z axes.
    battery_voltage: The voltage of the robot's battery.
    status: A status code associated with the robot.
    cliff_data_raw: Raw data related to cliff sensors.
    backpack_touch_sensor_raw: Raw data from the robot's backpack touch sensor.
    curr_path_segment: The ID of the current path segment.
    """
    robot['accelerator'] = [pkt.accel_x - 1000, pkt.accel_y - 1000, pkt.accel_z - 1000]
    robot['ultrasonic'] = pkt.cliff_data_raw
    robot["gyro"] = [pkt.gyro_x, pkt.gyro_y, pkt.gyro_z]
    robot['servo_head'] = pkt.head_angle_rad
    robot['battery'] = pkt.battery_voltage


async def expressions():
    expressions_array = [
        pycozmo.expressions.Anger(),
        pycozmo.expressions.Sadness(),
        pycozmo.expressions.Happiness(),
        pycozmo.expressions.Surprise(),
        pycozmo.expressions.Disgust(),
        pycozmo.expressions.Fear(),
        pycozmo.expressions.Pleading(),
        pycozmo.expressions.Vulnerability(),
        pycozmo.expressions.Despair(),
        pycozmo.expressions.Guilt(),
        pycozmo.expressions.Disappointment(),
        pycozmo.expressions.Embarrassment(),
        pycozmo.expressions.Horror(),
        pycozmo.expressions.Skepticism(),
        pycozmo.expressions.Annoyance(),
        pycozmo.expressions.Fury(),
        pycozmo.expressions.Suspicion(),
        pycozmo.expressions.Rejection(),
        pycozmo.expressions.Boredom(),
        pycozmo.expressions.Tiredness(),
        pycozmo.expressions.Asleep(),
        pycozmo.expressions.Confusion(),
        pycozmo.expressions.Amazement(),
        pycozmo.expressions.Excitement(),
    ]
    while True:
        if face_selected:
            face_generator = pycozmo.procedural_face.interpolate(
                pycozmo.expressions.Neutral(), expressions_array[face_selected[0]],
                pycozmo.robot.FRAME_RATE // 3)
            for face in face_generator:
                # Render face image.
                im = face.render()

                # The Cozmo protocol expects a 128x32 image, so take only the even lines.
                np_im = np.array(im)
                np_im2 = np_im[::2]
                im2 = Image.fromarray(np_im2)

                # Display face image.
                cli.display_image(im2)
            face_selected.pop()
        else:
            time.sleep(0.05)


def face_starter():
    asyncio.run(expressions())


def on_body_info(cli, pkt: pycozmo.protocol_encoder.BodyInfo):
    print("pkt: ", pkt)


def on_camera_image(cli, image):
    rgb_value = list(image.getdata())  # full rgb data
    new_rgb = np.array(rgb_value)
    new_rgb = new_rgb.reshape(240, 320, 3)
    new_rgb = new_rgb.astype(np.uint8)
    rgb_array['current'] = new_rgb
    time.sleep(0.01)


def move_head(cli, angle, max, min):
    if min <= angle <= max:
        cli.set_head_angle(angle)  # move head
        return True
    else:
        print("reached to limit")
        return False


def lift_arms(cli, angle, max, min):
    if min <= angle <= max:
        cli.set_lift_height(angle)  # move head
        return True
    else:
        print("reached to limit")
        return False


def action(obtained_data, device_list, feagi_settings, head_angle, arms_angle):
    for device in device_list:  # TODO: work on 'device'
        if "motor" in obtained_data:
            wheel_speeds = {"rf": 0, "rb": 0, "lf": 0, "lb": 0}

            for i, value in obtained_data["motor"].items():
                if i in [0, 1]:
                    wheel_speeds["r" + ["f", "b"][i]] = float(value)
                if i in [2, 3]:
                    wheel_speeds["l" + ["f", "b"][i - 2]] = float(value)
            rwheel_speed = wheel_speeds["rf"] - wheel_speeds["rb"]
            lwheel_speed = wheel_speeds["lf"] - wheel_speeds["lb"]
            cli.drive_wheels(lwheel_speed=lwheel_speed,
                             rwheel_speed=rwheel_speed,
                             duration=feagi_settings['feagi_burst_speed'])
        if "servo" in obtained_data:
            for i in obtained_data['servo']:
                if i == 0:
                    test_head_angle = head_angle
                    test_head_angle += obtained_data['servo'][i] / capabilities["servo"][
                        "power_amount"]
                    if move_head(cli, test_head_angle, max, min):
                        head_angle = test_head_angle
                elif i == 1:
                    test_head_angle = head_angle
                    test_head_angle -= obtained_data['servo'][i] / capabilities["servo"][
                        "power_amount"]
                    if move_head(cli, head_angle, max, min):
                        head_angle = test_head_angle
                if i == 2:
                    test_head_angle = arms_angle
                    test_head_angle += obtained_data['servo'][i] / 100
                    if lift_arms(cli, test_head_angle, max_lift, min_lift):
                        arms_angle = test_head_angle
                elif i == 3:
                    test_head_angle = arms_angle
                    test_head_angle -= obtained_data['servo'][i] / 100
                    if lift_arms(cli, test_head_angle, max_lift, min_lift):
                        arms_angle = test_head_angle
        if "misc" in obtained_data:
            if obtained_data["misc"]:
                for i in obtained_data["misc"]:
                    face_selected.append(i)
    return arms_angle, head_angle


# # FEAGI REACHABLE CHECKER # #
feagi_flag = False
print("retrying...")
print("Waiting on FEAGI...")
while not feagi_flag:
    feagi_flag = FEAGI.is_FEAGI_reachable(
        os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
        int(os.environ.get('FEAGI_OPU_PORT', "3000")))
    sleep(2)

# # FEAGI REACHABLE CHECKER COMPLETED # #

# # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - #
feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
    FEAGI.connect_to_feagi(feagi_settings,runtime_data, agent_settings, capabilities)
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
face_selected = deque()
threading.Thread(target=face_starter, daemon=True).start()
msg_counter = 0
rgb = {'camera': {}}
genome_tracker = 0
capabilities['camera']['current_select'] = []
get_size_for_aptr_cortical = api_address + '/v1/FEAGI/genome/cortical_area?cortical_area=o_aptr'
raw_aptr = requests.get(get_size_for_aptr_cortical).json()
aptr_cortical_size = pns.fetch_aptr_size(10, raw_aptr, None)
# Raise head.
cli = pycozmo.Client()
cli.start()
cli.connect()
cli.wait_for_robot()
print("max in rad: ", pycozmo.robot.MAX_HEAD_ANGLE.radians)  # 0.7766715171374767
print("min in rad: ", pycozmo.robot.MIN_HEAD_ANGLE.radians)  # -0.4363323129985824
max = pycozmo.robot.MAX_HEAD_ANGLE.radians - 0.1
min = pycozmo.robot.MIN_HEAD_ANGLE.radians + 0.1
max_lift = pycozmo.MAX_LIFT_HEIGHT.mm - 5
min_lift = pycozmo.MIN_LIFT_HEIGHT.mm + 5
angle_of_head = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
angle_of_arms = 50  # TODO: How to obtain the arms encoders in real time
cli.set_head_angle(angle_of_head)  # move head
lwheel_speed = 0  # Speed in millimeters per second for the left wheel
rwheel_speed = 0  # Speed in millimeters per second for the right wheel
lwheel_acc = 0  # Acceleration in millimeters per second squared for the left wheel
rwheel_acc = 0  # Acceleration in millimeters per second squared for the right wheel
duration = 0  # Duration in seconds for how long to drive the wheels

# vision capture
cli.enable_camera(enable=True, color=True)

cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)
cli.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state)
time.sleep(2)
# vision ends
device_list = pns.generate_OPU_list(capabilities)  # get the OPU sensors
while True:
    try:
        start = time.time()
        message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
        if message_from_feagi is not None:
            # Obtain the size of aptr
            if aptr_cortical_size is None:
                aptr_cortical_size = pns.check_aptr(raw_aptr)
            # Update the aptr
            capabilities = pns.fetch_aperture_data(message_from_feagi, capabilities,
                                                   aptr_cortical_size)
            # Update the ISO
            capabilities = pns.fetch_iso_data(message_from_feagi, capabilities, aptr_cortical_size)
            # OPU section STARTS
            obtained_signals = pns.obtain_opu_data(device_list, message_from_feagi)
            angle_of_arms, angle_of_head = action(obtained_signals, device_list, feagi_settings,
                                                  angle_of_arms, angle_of_head)
            # OPU section ENDS
        new_rgb = rgb_array['current']
        previous_data_frame, rgb['camera'], capabilities['camera']['current_select'] = \
            pns.generate_rgb(new_rgb,
                             capabilities['camera']['central_vision_allocation_percentage'][0],
                             capabilities['camera']['central_vision_allocation_percentage'][1],
                             capabilities['camera']["central_vision_resolution"],
                             capabilities['camera']['peripheral_vision_resolution'],
                             previous_data_frame,
                             capabilities['camera']['current_select'],
                             capabilities['camera']['iso_default'],
                             capabilities['camera']["aperture_default"])
        battery = robot['battery']
        try:
            ultrasonic_data = robot['ultrasonic'][0]
            if ultrasonic_data:
                formatted_ultrasonic_data = {
                    'ultrasonic': {
                        sensor: data for sensor, data in enumerate([ultrasonic_data])
                    }
                }
            else:
                formatted_ultrasonic_data = {}
            message_to_feagi, battery = FEAGI.compose_message_to_feagi(
                original_message={**formatted_ultrasonic_data}, battery=battery)
        except Exception as ERROR:
            print("ERROR IN ULTRASONIC: ", ERROR)
            ultrasonic_data = 0
        try:
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = {}
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = {}
            message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
        except Exception as e:
            print("error: ", e)
        # Add accelerator section
        try:
            runtime_data['accelerator']['0'] = robot['accelerator'][0]
            runtime_data['accelerator']['1'] = robot['accelerator'][1]
            runtime_data['accelerator']['2'] = robot['accelerator'][2]
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = {}
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = {}
            message_to_feagi["data"]["sensory_data"]['accelerator'] = runtime_data['accelerator']
        except Exception as ERROR:
            message_to_feagi["data"]["sensory_data"]['accelerator'] = {}
        # End accelerator section

        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        if message_from_feagi is not None:
            feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
        sleep(feagi_settings['feagi_burst_speed'])

        pns.afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings)
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
    except Exception as e:
        print("ERROR: ", e)
        traceback.print_exc()
        break
