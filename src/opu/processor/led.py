
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

from evo.blocks import block_ref_2_id, percent_active_neurons_in_block
from inf import runtime_data
from ipu.processor.range import map_value
from importlib.machinery import SourceFileLoader


def convert_neuron_activity_to_rgb_intensities(blocks_with_active_neurons, cortical_area='led_opu'):
    led_x_dim = runtime_data.genome['blueprint'] \
                                   [cortical_area] \
                                   ['neuron_params'] \
                                   ['block_boundaries'][0]

    led_z_dim = runtime_data.genome['blueprint'] \
                                   [cortical_area] \
                                   ['neuron_params'] \
                                   ['block_boundaries'][-1]

    # create template for accommodating LED data insertion
    led_data = {led_id: [0] * led_z_dim for led_id in range(led_x_dim)}
    
    for block_ref in blocks_with_active_neurons:
        block_id = block_ref_2_id(block_ref)
        block_z_idx = block_id[-1]
        led_id = block_id[0]
        
        percent_active = percent_active_neurons_in_block(block_ref, cortical_area='led_opu')
        mapped_intensity = round(map_value(percent_active, 0, 100, 1, 255))
        if led_id in led_data:
            led_data[led_id][block_z_idx] = mapped_intensity
    
    return led_data


def activate_leds(led_data):
    controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()
    led = controller.LED()
    for led_id in led_data:
        R = led_data[led_id][0]
        G = led_data[led_id][1]
        B = led_data[led_id][2]
        led.LED_on(led_id, R, G, B)
