# -*- coding: utf-8 -*-


#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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


import logging
from src.pns import stimuli_translator
import traceback
from datetime import datetime
from src.evo.voxels import *
from src.evo.stats import opu_activity_report
from src.inf.byte_processor import bytes_to_feagi_data


logger = logging.getLogger(__name__)

"""
This module manages the routing of all IPU/OPU related data

todo: figure how the exposure counter can work in synchrony with the burst engine
todo: convert IPU library to a plug-in based architecture
"""

logger.info("IPU controller initialized")
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
                except Exception as e:
                    print("ERROR while processing Stimulation IPU", ipu_data["data"]["direct_stimulation"], ">>", e,
                          traceback.format_exc())

        if "connected_agents" in ipu_data["data"]:
            if ipu_data["data"]["connected_agents"]:
                for connected_agent in ipu_data["data"]["connected_agents"]:
                    if not runtime_data.connected_agents.get(connected_agent):
                        runtime_data.connected_agents[connected_agent] = True

        if "sensory_data" in ipu_data["data"]:
            for sensor_type in ipu_data["data"]["sensory_data"]:


                # Ultrasonic / Lidar Handler
                # todo: need a more consistent naming convention when it comes to lidar vs ultrasonic vs proximity
                # todo: find a way to generalize the handling of all IPU data instead of using all the if statements
                if 'encoder_data' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.servo_position_translator(
                            servo_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Servo IPU", traceback.format_exc())
                # if 'encoder' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                #     try:
                #         stimuli_translator.encoder_translator(
                #             encoder_data=ipu_data["data"]["sensory_data"][sensor_type])
                #     except Exception:
                #         print("ERROR while processing Encoder IPU", traceback.format_exc())
                if 'encoder_speed' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.encoder_speed_translator(
                            encoder_speed_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Encoder IPU", traceback.format_exc())
                if 'gyro' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.gyro_translator(gyroscope_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Gyro IPU", traceback.format_exc())

                if 'accelerator' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.accelerator_translator(accelerator_data=ipu_data["data"]["sensory_data"]
                        [sensor_type])
                    except Exception:
                        print("ERROR while processing Accelerator IPU", traceback.format_exc())

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

                if 'camera' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        if "00CC" in ipu_data["data"]["sensory_data"][sensor_type]:
                            runtime_data.color_img_feed = ipu_data["data"]["sensory_data"][sensor_type]["00CC"]
                        stimuli_translator.vision_translator(vision_data=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Camera IPU", traceback.format_exc())

                if 'training' in sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        stimuli_translator.training_translator(stimulation=ipu_data["data"]["sensory_data"][sensor_type])
                    except Exception:
                        print("ERROR while processing Object Identification Training IPU", traceback.format_exc())
                if 'generic_ipu' == sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:

                        dict_ipu_data = ipu_data["data"]["sensory_data"]['generic_ipu']
                        if "iv00CC" in dict_ipu_data:
                            runtime_data.color_img_feed = dict_ipu_data["iv00CC"]
                        stimuli_translator.generic_ipu_translator(
                            ipu_data=dict_ipu_data)
                    except Exception:
                        print("ERROR while processing Object Identification Generic IPU", traceback.format_exc())

                if 'generic_ipu_b' == sensor_type and ipu_data["data"]["sensory_data"][sensor_type] is not None:
                    try:
                        byte_ipu_data = ipu_data["data"]["sensory_data"]['generic_ipu_b']
                        dict_ipu_data = bytes_to_feagi_data(byte_ipu_data)

                        if "iv00CC" in dict_ipu_data:
                            runtime_data.color_img_feed = dict_ipu_data["iv00CC"]
                        stimuli_translator.generic_ipu_translator(
                            ipu_data=dict_ipu_data)
                    except Exception:
                        print("ERROR while processing Object Identification Generic IPU", traceback.format_exc())






def opu_router():
    """
    Relays neuronal activities to the controller.
    Sample data format for runtime_data.opu_data:

    {'o__bat': {}, 'o__mot': {(0, 0, 0): 47, (0, 0, 5): 48, (0, 0, 15): 50, (0, 0, 1): 45}}

    """
    for cortical_area in runtime_data.fire_candidate_list.copy():
        if str(cortical_area)[0] == 'o':
            if cortical_area not in runtime_data.opu_data:
                runtime_data.opu_data[cortical_area] = {}
            runtime_data.opu_data[cortical_area] = opu_percentage_report(cortical_area=cortical_area)

#
# def action_router():
#     """
#     This function is intended to handle all the OPU processing that needs to be addressed in burst level as opposed
#     to individual neuron fire
#     """
#     # todo: Introduce a generalized approach to cover all OPUs
#
#     # LED handler
#     if 'o__led' in runtime_data.fire_candidate_list and runtime_data.hardware == 'raspberry_pi':
#         active_led_neurons = active_neurons_in_blocks(cortical_area='led_opu')
#         led_data = action_translator.led.convert_neuron_activity_to_rgb_intensities(active_led_neurons)
#         action_translator.led.activate_leds(led_data)
#
#     # todo: need a better differentiation between movement and motor modules
#     # Movement handler
#     if 'o__mot' in runtime_data.fire_candidate_list:
#         if len(runtime_data.fire_candidate_list["o__mot"]) > 0:
#             # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
#             # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
#             # movement.activate_motor(data)
#             activity_report = opu_activity_report(cortical_area='o__mot')
#             # print("motor activity report", activity_report)
#             motor_data = dict()
#             for device in activity_report:
#                 # if there are "ties" w/r/t block activity, this will select the first index in the list w/the tie value
#                 # todo: need a better method
#                 # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
#                 try:
#                     block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
#                     tmp_list = set(activity_report[device][0])
#                     tmp_list.remove(max(activity_report[device][0]))
#                     block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
#                     chosen_block = max(block_with_max_z, block_with_2nd_max)
#                 except ValueError:
#                     chosen_block = 0
#                 if device not in motor_data:
#                     motor_data[device] = dict()
#                 motor_data[device]['speed'] = chosen_block
#             action_processor.activate_device(device_type='motor', device_data=motor_data)
#
#     if 'o__ser' in runtime_data.fire_candidate_list:
#         if len(runtime_data.fire_candidate_list["o__ser"]) > 0:
#             # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
#             # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
#             # movement.activate_motor(data)
#             activity_report = opu_activity_report(cortical_area='o__ser')
#             device_data = dict()
#             for device in activity_report:
#                 # if there are "ties" w/r/t block activity, this will select the first index in the list w/the tie value
#                 # todo: need a better method
#                 # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
#                 try:
#                     block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
#                     tmp_list = set(activity_report[device][0])
#                     tmp_list.remove(max(activity_report[device][0]))
#                     block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
#                     chosen_block = max(block_with_max_z, block_with_2nd_max)
#                 except ValueError:
#                     chosen_block = 0
#                 if device not in device_data:
#                     device_data[device] = dict()
#                 device_data[device]['angle'] = chosen_block
#             action_processor.activate_device(device_type='servo', device_data=device_data)
#
#     if 'o__bat' in runtime_data.fire_candidate_list:
#         if len(runtime_data.fire_candidate_list["o__bat"]) > 0:
#             activity_report = opu_activity_report(cortical_area='o__bat')
#             device_data = dict()
#             for device in activity_report:
#                 action_processor.activate_device(device_type='battery', device_data=device_data)
