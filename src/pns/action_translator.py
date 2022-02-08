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

import sys
import traceback
from evo.blocks import *
from inf import runtime_data
from importlib.machinery import SourceFileLoader


def motor_operator(motor_data):
    """
    motor_data = {
        motor_id: {
            "direction": 'F',
            "...": ...
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        if motor_data:
            runtime_data.opu_data["motor"] = motor_data
            if runtime_data.parameters["Logs"]["print_opu_info"]:
                for motor_id in motor_data:
                    print(f">>>>>>>>>>>>>>>>>>>>>>>> motor {motor_id} "
                          f"activation command sent to router with direction {motor_data[motor_id]}")

    except Exception as e:
        print("ERROR at Motor module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)


def servo_operator(servo_data):
    """
    servo_data = {
        servo_id: {
            "angle": 30
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        runtime_data.opu_data["servo"] = servo_data
        if runtime_data.parameters["Logs"]["print_opu_info"]:
            for servo_id in servo_data:
                print(f">>>>>>>>>>>>>>>>>>>>>>>> {servo_id} "
                      f"activation command sent to router with angle {servo_data[servo_id]['angle']}")

    except Exception as e:
        print("ERROR at Servo module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)


def convert_neuron_activity_to_rgb_intensities(self, blocks_with_active_neurons, cortical_area='led_opu'):
    led_x_dim = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][0]

    led_z_dim = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][-1]

    # create template for accommodating LED data insertion
    led_data = {led_id: [0] * led_z_dim for led_id in range(led_x_dim)}

    for block_ref in blocks_with_active_neurons:
        block_id = block_ref_2_id(block_ref)
        block_z_idx = block_id[-1]
        led_id = block_id[0]

        percent_active = percent_active_neurons_in_block(block_ref, cortical_area='led_opu')
        mapped_intensity = round(self.range.map_value(percent_active, 0, 100, 1, 255))
        if led_id in led_data:
            led_data[led_id][block_z_idx] = mapped_intensity

    return led_data


def activate_leds(led_data):
    controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()

    for led_id in led_data:
        R = led_data[led_id][0]
        G = led_data[led_id][1]
        B = led_data[led_id][2]
        controller.led.LED_on(led_id, R, G, B)


def convert_neuron_activity_to_utf8_char(cortical_area, neuron_id):
    char = int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])
    activity_history = list(runtime_data.brain[cortical_area][neuron_id]['activity_history'])
    # todo: move collection span to parameters
    collection_span_counter = len(activity_history) - 1
    membrane_potential_total = 0
    while collection_span_counter > 0:
        membrane_potential_total += activity_history[collection_span_counter][1]
        collection_span_counter -= 1

    activity_rank = membrane_potential_total / len(activity_history)
    return chr(char), int(activity_rank)


def battery_charger():
    """
    battery_data = {
        battery_id: {
            "charge": True
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        runtime_data.opu_data["battery"] = True
        if runtime_data.parameters["Logs"]["print_opu_info"]:
            print(">>>  Battery is charging +++    charge command sent to router ")

    except Exception as e:
        print("ERROR at Battery module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
