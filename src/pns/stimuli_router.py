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
from inf import runtime_data


"""

This module manages all IPU related modules

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


def ipu_handler(ipu_data):
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
                        stimuli_translator.lidar_translator(proximity_data=ipu_data["data"]["sensory_data"][sensor_type])
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
