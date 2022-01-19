
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

"""
This module reads LIDAR data from a message queue and makes them available to the proximity processor.
"""
import time

from ipu.processor import range
from inf import runtime_data


def translate(proximity_data, type=None):
    """
    Translate the lidar messages based on its type.

    todo: add details here about the message format and expectations


    Type is not needed at this point given the lidar vs sonar data is automatically differentiated within the func.
    """

    if proximity_data is not None:
        # print("SLOT_TYPES", message.SLOT_TYPES)
        # print("angle_increment:", message.angle_increment)
        # print("angle_max:", message.angle_max)
        # print("angle_min:", message.angle_min)
        # print("get_fields_and_field_types:", message.get_fields_and_field_types)
        # print("header:", message.header)
        # print("intensities:", message.intensities)
        # print("range_max:", message.range_max)
        # print("range_min:", message.range_min)
        # print("ranges:", message.ranges)
        # print("scan_time:", message.scan_time)
        # print("time_increment:", message.time_increment)
        # print("-----")

        for sensor in proximity_data:
            # differentiate between LIDAR/SONAR data
            if hasattr(proximity_data[sensor], '__iter__'):
                detections = range.lidar_to_coords(proximity_data[sensor])
            else:
                detections = range.sonar_to_coords(proximity_data[sensor])

            neurons = range.coords_to_neuron_ids(
                detections, cortical_area='proximity_ipu'
            )

            # TODO: Add proximity feeder function in fcl_injector
            runtime_data.fcl_queue.put({'proximity_ipu': set(neurons)})
