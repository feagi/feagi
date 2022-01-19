
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

from inf import runtime_data
from opu.processor import led, movement
from evo.blocks import active_neurons_in_blocks, percent_active_neurons_in_block, block_ref_2_id
from evo.stats import opu_activity_report


def initialize():
    return


def opu_handler():
    """
    This function is intended to handle all the OPU processing that needs to be addressed in burst level as opposed
    to individual neuron fire
    """
    # todo: Introduce a generalized approach to cover all OPUs

    # LED handler
    if 'led_opu' in runtime_data.fire_candidate_list and runtime_data.hardware == 'raspberry_pi':
        active_led_neurons = active_neurons_in_blocks(cortical_area='led_opu')
        led_data = led.convert_neuron_activity_to_rgb_intensities(active_led_neurons)
        led.activate_leds(led_data)

    # todo: need a better differentiation between movement and motor modules
    # Movement handler
    if 'motor_opu' in runtime_data.fire_candidate_list:
        # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
        # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
        # movement.activate_motor(data)
        activity_report = opu_activity_report(cortical_area='motor_opu')
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
        movement.activate_device(device_type='motor', device_data=motor_data)

    if 'servo_opu' in runtime_data.fire_candidate_list:
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
        movement.activate_device(device_type='servo', device_data=device_data)
