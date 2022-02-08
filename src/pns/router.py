# -*- coding: utf-8 -*-


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

from pns import stimuli_translator
import traceback
from datetime import datetime
from evo.blocks import *
from pns import action_translator, action_processor
from evo.stats import opu_activity_report


"""
This module manages the routing of all IPU/OPU related data

todo: figure how the exposure counter can work in synchrony with the burst engine
todo: convert IPU library to a plug-in based architecture
"""

print("IPU controller initialized")
runtime_data.last_ipu_activity = datetime.now()


# def proximity_controller(self):
#     while not runtime_data.exit_condition:
#         try:
#             source.lidar_translator()
#         except Exception as e:
#             traceback.print_exc()
#         finally:
#             runtime_data.last_ipu_activity = datetime.now()


def stimuli_router(ipu_data):
    """
    Decodes the message received from the ipu router and distribute the sub-messages to corresponding IPU modules

    expected ipu_data structure:

    ipu_data = {
        "capabilities": {},
        "network": {},
        "data": {
            "direct_stimulation": {
                "cortical_area_id": {voxel},
                "cortical_area_id": sensor_data,
                "cortical_area_id": sensor_data
                ...
                },
            "sensory_data": {
                "sensor type": sensor data,
                "sensor type": sensor data,
                "sensor type": sensor data,
                ...
            }
        }
    """

    if type(ipu_data) == dict and "data" in ipu_data:
        if "direct_stimulation" in ipu_data["data"]:
            if ipu_data["data"]["direct_stimulation"] is not None:
                try:
                    stimuli_translator.stimulation_injector(stimulation_data=ipu_data["data"]["direct_stimulation"])
                except:
                    print("ERROR while processing Stimulation IPU", ipu_data["data"]["direct_stimulation"])

        if "sensory_data" in ipu_data["data"]:
            for sensor_type in ipu_data["data"]["sensory_data"]:
                # Ultrasonic / Lidar Handler
                # todo: need a more consistent naming convention when it comes to lidar vs ultrasonic vs proximity
                # todo: find a way to generalize the handling of all IPU data instead of using all the if statements

                if 'ultrasonic' in sensor_type and \
                        ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.lidar_translator(
                            proximity_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing lidar function", traceback.format_exc())

                # Infrared Handler
                if 'ir' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.convert_ir_to_fire_list(ir_data=ipu_data[
                            "data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Infrared IPU", traceback.format_exc())

                if 'battery' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.battery_translator(sensor_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Battery IPU", traceback.format_exc())

        else:
            print("ERROR: IPU handler encountered non-compliant data")


def action_router():
    """
    This function is intended to handle all the OPU processing that needs to be addressed in burst level as opposed
    to individual neuron fire
    """
    # todo: Introduce a generalized approach to cover all OPUs

    # LED handler
    if 'o__led' in runtime_data.fire_candidate_list and runtime_data.hardware == 'raspberry_pi':
        active_led_neurons = active_neurons_in_blocks(cortical_area='led_opu')
        led_data = action_translator.led.convert_neuron_activity_to_rgb_intensities(active_led_neurons)
        action_translator.led.activate_leds(led_data)

    # todo: need a better differentiation between movement and motor modules
    # Movement handler
    if 'o__mot' in runtime_data.fire_candidate_list:
        # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
        # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
        # movement.activate_motor(data)
        activity_report = opu_activity_report(cortical_area='o__mot')
        # print("motor activity report", activity_report)
        motor_data = dict()
        for device in activity_report:
            # if there are "ties" w/r/t block activity, this will select the first index in the list w/ the tie value
            # todo: need a better method
            # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
            try:
                block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
                tmp_list = set(activity_report[device][0])
                tmp_list.remove(max(activity_report[device][0]))
                block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
                chosen_block = max(block_with_max_z, block_with_2nd_max)
            except ValueError:
                chosen_block = 0
            if device not in motor_data:
                motor_data[device] = dict()
            motor_data[device]['speed'] = chosen_block
        action_processor.activate_device(device_type='motor', device_data=motor_data)

    if 'o__ser' in runtime_data.fire_candidate_list:
        # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
        # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
        # movement.activate_motor(data)
        activity_report = opu_activity_report(cortical_area='servo_opu')
        device_data = dict()
        for device in activity_report:
            # if there are "ties" w/r/t block activity, this will select the first index in the list w/ the tie value
            # todo: need a better method
            # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
            try:
                block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
                tmp_list = set(activity_report[device][0])
                tmp_list.remove(max(activity_report[device][0]))
                block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
                chosen_block = max(block_with_max_z, block_with_2nd_max)
            except ValueError:
                chosen_block = 0
            if device not in device_data:
                device_data[device] = dict()
            device_data[device]['angle'] = chosen_block
        action_processor.activate_device(device_type='servo', device_data=device_data)

    if 'o__bat' in runtime_data.fire_candidate_list:
        activity_report = opu_activity_report(cortical_area='battery_opu')
        device_data = dict()
        for device in activity_report:
            action_processor.activate_device(device_type='battery', device_data=device_data)