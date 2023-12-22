#!/usr/bin/env python3
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

from feagi_agent import pns_gateway as pns


def fetch_infrared_sensor(ir_list, message_to_feagi, capabilities):
    formatted_ir_data = {sensor: True for sensor in ir_list}
    for ir_sensor in range(int(capabilities['infrared']['count'])):
        if ir_sensor not in formatted_ir_data:
            formatted_ir_data[ir_sensor] = False
    return pns.append_sensory_data_for_feagi('ir', formatted_ir_data, message_to_feagi)


def fetch_ultrasonic_sensor(ultrasonic_list, message_to_feagi):
    formatted_ultrasonic_data = {
        'ultrasonic': {sensor: data for sensor, data in enumerate([ultrasonic_list])}}
    return pns.append_sensory_data_for_feagi('ultrasonic', formatted_ultrasonic_data, message_to_feagi)
