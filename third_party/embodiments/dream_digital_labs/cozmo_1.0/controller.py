    #!/usr/bin/env python

import time
from PIL import Image
import pycozmo
from feagi_agent import feagi_interface as FEAGI
from feagi_agent import retina as retina
from configuration import *
import requests
import sys
import os
from datetime import datetime
from collections import deque
import numpy as np
from time import sleep
import pickle
import lz4.frame
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


def on_body_info(cli, pkt: pycozmo.protocol_encoder.BodyInfo):
    print("pkt: ", pkt)


def on_camera_image(cli, image):
    rgb_value = list(image.getdata())  ## full rgb data
    new_rgb = np.array(rgb_value)
    new_rgb = new_rgb.reshape(240, 320, 3)
    new_rgb = new_rgb.astype(np.uint8)
    rgb_array['current'] = new_rgb
    time.sleep(0.01)


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
print("Connecting to FEAGI resources...")
feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                       feagi_settings=feagi_settings,
                                                       agent_settings=agent_settings,
                                                       capabilities=capabilities)
api_address = runtime_data['feagi_state']["feagi_url"]
# agent_data_port = agent_settings["agent_data_port"]
agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
print("** **", runtime_data["feagi_state"])
feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

# todo: to obtain this info directly from FEAGI as part of registration
# ipu_channel_address = FEAGI.feagi_inbound(agent_settings["agent_data_port"])
ipu_channel_address = FEAGI.feagi_outbound(feagi_settings['feagi_host'], agent_data_port)

print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = FEAGI.feagi_outbound(feagi_settings['feagi_host'],
                                           runtime_data["feagi_state"]['feagi_opu_port'])

feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address, bind=False)
feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - #

msg_counter = 0
rgb = {}
rgb['camera'] = {}
genome_tracker = 0
get_size_for_aptr_cortical = api_address + '/v1/FEAGI/genome/cortical_area?cortical_area=o_aptr'
raw_aptr = requests.get(get_size_for_aptr_cortical).json()
try:
    aptr_cortical_size = raw_aptr['cortical_dimensions'][2]
except:
    aptr_cortical_size = None
# Raise head.
cli = pycozmo.Client()
cli.start()
cli.connect()
cli.wait_for_robot()
print("max in rad: ", pycozmo.robot.MAX_HEAD_ANGLE.radians)  # 0.7766715171374767
print("min in rad: ", pycozmo.robot.MIN_HEAD_ANGLE.radians)  # -0.4363323129985824
angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
cli.set_head_angle(angle)  # move head
lwheel_speed = 0  # Speed in millimeters per second for the left wheel
rwheel_speed = 0  # Speed in millimeters per second for the right wheel
lwheel_acc = 0  # Acceleration in millimeters per second squared for the left wheel
rwheel_acc = 0  # Acceleration in millimeters per second squared for the right wheel
duration = 0  # Duration in seconds for how long to drive the wheels
# cli.drive_wheels(lwheel_speed, rwheel_speed, lwheel_acc, rwheel_acc, duration)


# vision capture
cli.enable_camera(enable=True, color=True)
cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)
cli.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state)
time.sleep(2)
# time.sleep(5)
# vision ends
while True:
    try:
        start = time.time()
        received_data = feagi_opu_channel.receive()  # Obtain data from FEAGI
        if received_data is not None:
            if isinstance(received_data, bytes):
                decompressed_data = lz4.frame.decompress(received_data)
                message_from_feagi = pickle.loads(decompressed_data)
            else:
                message_from_feagi = received_data
        else:
            message_from_feagi = None
        # Decompression section ends

        if message_from_feagi is not None:
            # OPU section STARTS
            if "o_aptr" in message_from_feagi["opu_data"]:
                if message_from_feagi["opu_data"]["o_aptr"]:
                    for i in message_from_feagi["opu_data"]["o_aptr"]:
                        feagi_aptr = (int(i.split('-')[-1]))
                        if aptr_cortical_size is None:
                            aptr_cortical_size = check_aptr(aptr_cortical_size)
                        elif aptr_cortical_size <= feagi_aptr:
                            aptr_cortical_size = check_aptr(aptr_cortical_size)
                        max_range = capabilities['camera']['aperture_range'][1]
                        min_range = capabilities['camera']['aperture_range'][0]
                        capabilities['camera']["aperture_default"] = \
                            ((feagi_aptr / aptr_cortical_size) *
                             (max_range - min_range)) + min_range
            if "o__dev" in message_from_feagi["opu_data"]:
                if message_from_feagi["opu_data"]["o__dev"]:
                    for i in message_from_feagi["opu_data"]["o__dev"]:
                        dev_data = i
                        digits = dev_data.split('-')
                        third_digit = int(digits[2])
                        capabilities['camera']["deviation_threshold"] = third_digit / 10
            opu_data = FEAGI.opu_processor(message_from_feagi)
            if "motor" in opu_data:
                rwheel_speed, lwheel_speed, rf, rb, lf, lb = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                for i in opu_data["motor"]:
                    if 1 == i:
                        rb = float(opu_data["motor"][i])
                    if i == 0:
                        rf = float(opu_data["motor"][i])
                    if i == 2:
                        lf = float(opu_data["motor"][i])
                    if i == 3:
                        lb = float(opu_data["motor"][i])
                    rwheel_speed = rf - rb
                    lwheel_speed = lf - lb
                cli.drive_wheels(lwheel_speed=lwheel_speed, rwheel_speed=rwheel_speed, duration=feagi_settings['feagi_burst_speed'])

            # OPU section ENDS
        new_rgb = rgb_array['current']
        # cv2.imshow("test", new_rgb)
        # cv2.waitKey(30)
        retina_data = retina.frame_split(new_rgb,
                                         capabilities['camera']['retina_width_percent'],
                                         capabilities['camera'][
                                             'retina_height_percent'])
        for i in retina_data:
            if 'C' in i:
                retina_data[i] = \
                    retina.center_data_compression(retina_data[i],
                                                   capabilities['camera']
                                                   ["central_vision_compression"])
            else:
                retina_data[i] = \
                    retina.center_data_compression(retina_data[i],
                                                   capabilities['camera']
                                                   ['peripheral_vision_compression'])
        if not previous_data_frame:
            for i in retina_data:
                PREVIOUS_NAME = str(i) + "_prev"
                previous_data_frame[PREVIOUS_NAME] = {}
        for i in retina_data:
            name = i
            if 'prev' not in i:
                data = retina.ndarray_to_list(retina_data[i])
                if 'C' in i:
                    PREVIOUS_NAME = str(i) + "_prev"
                    rgb_data, previous_data_frame[PREVIOUS_NAME] = \
                        retina.get_rgb(data,
                                       capabilities['camera'][
                                           'central_vision_compression'],
                                       previous_data_frame[
                                           PREVIOUS_NAME],
                                       name,
                                       capabilities[
                                           'camera'][
                                           'deviation_threshold'],
                                       capabilities['camera']["aperture_default"])
                else:
                    PREVIOUS_NAME = str(i) + "_prev"
                    rgb_data, previous_data_frame[PREVIOUS_NAME] = \
                        retina.get_rgb(data,
                                       capabilities[
                                           'camera'][
                                           'peripheral_vision_compression'],
                                       previous_data_frame[
                                           PREVIOUS_NAME],
                                       name,
                                       capabilities[
                                           'camera'][
                                           'deviation_threshold'],
                                       capabilities['camera']["aperture_default"])
                for a in rgb_data['camera']:
                    rgb['camera'][a] = rgb_data['camera'][a]
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
                message_to_feagi["data"]["sensory_data"]['accelerator'] = runtime_data[
                    'accelerator']
            except Exception as ERROR:
                message_to_feagi["data"]["sensory_data"]['accelerator'] = {}
            # End accelerator section
            # Psychopy game ends
        # message_to_feagi, battery = FEAGI.compose_message_to_feagi({**rgb},
        # battery=aliens.healthpoint*10)
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        if message_from_feagi is not None:
            feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
        sleep(feagi_settings['feagi_burst_speed'])

        if agent_settings['compression']:
            serialized_data = pickle.dumps(message_to_feagi)
            feagi_ipu_channel.send(message=lz4.frame.compress(serialized_data))
        else:
            feagi_ipu_channel.send(message_to_feagi)
        print("battery: ", battery)
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
    except Exception as e:
        print("ERROR: ", e)
        break

# pycozmo.setup_basic_logging(log_level="DEBUG", protocol_log_level="DEBUG")

# Needs to figure how to connect without needed wifi to cozmo
# conn = pycozmo.conn.Connection(("192.168.50.227", 5551))
# conn.start()
# conn.connect()
# conn.wait_for_robot()
# conn.drive_wheels(lwheel_speed=50.0, rwheel_speed=50.0, duration=0.2)
# conn.disconnect()
# conn.stop()
## testing section ends

# cli = pycozmo.Client()
# cli.start()
# cli.connect()
# cli.wait_for_robot()
# cli.enable_camera(enable=True, color=True)
# cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=True)


# cli = pycozmo.connect()

# Raise head.
# cli = pycozmo.Client()
# cli.start()
# cli.connect()
# cli.wait_for_robot()
# print("max in rad: ", pycozmo.robot.MAX_HEAD_ANGLE.radians) # 0.7766715171374767
# print("min in rad: ", pycozmo.robot.MIN_HEAD_ANGLE.radians) # -0.4363323129985824
# angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
# # cli.set_head_angle(angle) # move head
# # lwheel_speed = 100.0  # Speed in millimeters per second for the left wheel
# # rwheel_speed = 100.0  # Speed in millimeters per second for the right wheel
# # lwheel_acc = 30.0  # Acceleration in millimeters per second squared for the left wheel
# # rwheel_acc = 30.0  # Acceleration in millimeters per second squared for the right wheel
# # duration = 2.0  # Duration in seconds for how long to drive the wheels
# # cli.drive_wheels(lwheel_speed, rwheel_speed, lwheel_acc, rwheel_acc, duration)
#
#
# # vision capture
# cli.enable_camera(enable=True, color=True)
# cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)
# time.sleep(5)
# vision ends
