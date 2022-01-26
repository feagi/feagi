
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
This module facilitates the translation of cortical stimulation information to actual stimulation within connectome.

Stimulation data received will have the following data structure:
{ 'stimulation': {
        'motor_opu': [[x1, y1, z1], [x2, y2, z2], .....],
        'IR_opu': [[x1, y1, z1], [x2, y2, z2], .....],
        ...
        ...
    }
}

"""

from inf import runtime_data
from evo.blocks import neurons_in_the_block, block_reference_builder


def stimulation_injector(stimulation_data):
    for cortical_area in stimulation_data:
        print("stimulating...", cortical_area)
        neuron_list = set()
        for voxel in stimulation_data[cortical_area]:
            relative_coords = \
                runtime_data.genome['blueprint'][cortical_area]['neuron_params'].get('relative_coordinate')
            cortical_block_ref = [voxel[0] - relative_coords[0],
                                  voxel[1] - relative_coords[1],
                                  voxel[2] - relative_coords[2]]
            print("FEAGI received stimulation from Godot and processing...", cortical_block_ref)
            in_the_block = neurons_in_the_block(cortical_area=cortical_area,
                                                block_ref=block_reference_builder(cortical_block_ref))
            for neuron in in_the_block:
                neuron_list.add(neuron)
        runtime_data.fcl_queue.put({cortical_area: neuron_list})
        print(">>> >> >> > > >> >>>>>>> Stimulation data from Godot has been injected in FCL!")
