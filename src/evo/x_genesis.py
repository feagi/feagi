
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
This module is responsible for updating the commectome structure during brain life and post neuroembryogenesis.
"""

import logging
import json
import datetime
import concurrent.futures
from evo import neuron, synapse, stats, genetics, voxels
from functools import partial
from multiprocessing import Pool, Process
from inf import disk_ops
from inf import settings
from inf import runtime_data


def x_neurogenesis():
    pass


def x_synaptogenesis():
    pass


def x_corticogenesis():
    pass


def x_cortical_resize():
    pass


def x_cortical_reposition(cortical_area, new_coordinates):
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][0] = new_coordinates['x']
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][1] = new_coordinates['y']
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][2] = new_coordinates['z']

    print("MN" * 40)


def x_cortical_rewire():
    pass


def x_update_neuronal_properties():
    pass


def x_cortical_manipulator(data):
    cortical_id = data["cortical_id"]

    pass


def x_synaptic_pruning():
    pass


def x_apoptosis():
    pass


def x_neurodegeneration():
    pass


def change_request_processor(change_request):
    """
    Supported changes:
    - Cortical coordinates
    - Neuron firing threshold
    - Neuron leak coefficient
    - Neuron post synaptic potential uniform distribution switch
    - Neuron post synaptic potentials
    - Cortical name

    """

    if 'cortical_id' not in change_request:
        print("ERROR: Cortical change request did not include --cortical id--")
        return

    cortical_area = change_request['cortical_id']

    if 'cortical_name' in change_request:
        runtime_data.genome['blueprint'][cortical_area]["cortical_name"] = \
            change_request['cortical_name']

    if 'cortical_coordinates' in change_request:
        x_cortical_reposition(cortical_area=cortical_area,
                              new_coordinates=change_request['cortical_coordinates'])

    if 'neuron_fire_threshold' in change_request:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["firing_threshold"] = \
            change_request['neuron_fire_threshold']

    if 'neuron_leak_coefficient' in change_request:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["leak_coefficient"] = \
            change_request['neuron_leak_coefficient']

    if 'neuron_psp_uniform_distribution' in change_request:
        runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] = \
            change_request['neuron_psp_uniform_distribution']

    if 'neuron_post_synaptic_potential' in change_request:
        for neuron_id in runtime_data.brain[cortical_area]:
            for dst_neuron in runtime_data.brain[cortical_area][neuron_id]["neighbors"]:
                runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    change_request['neuron_post_synaptic_potential']

    if 'neuron_refractory_period' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["refractory_period"] = \
            change_request['neuron_refractory_period']

    if 'neuron_snooze_period' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["snooze_length"] = \
            change_request['neuron_snooze_period']

    if 'neuron_degeneracy_coefficient' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
            change_request['neuron_degeneracy_coefficient']

    if 'neuron_plasticity_constant' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["plasticity_constant"] = \
            change_request['neuron_plasticity_constant']

    if 'neuron_post_synaptic_potential_max' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
            change_request['neuron_post_synaptic_potential_max']

    if 'neuron_consecutive_fire_count' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["consecutive_fire_cnt_max"] = \
            change_request['neuron_consecutive_fire_count']

    if 'cortical_visibility' in change_request:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["visualization"] = \
            change_request['cortical_visibility']

    # Todo
    if 'cortical_group' in change_request:
        pass

    cortical_regeneration = ['cortical_dimension',
                             'cortical_neuron_per_vox_count',
                             'cortical_synaptic_attractivity', 'cortical_destinations']

    for request in change_request:
        if request in cortical_regeneration:
            pass

    if 'cortical_dimension' in change_request:
        pass

    if 'cortical_neuron_per_vox_count' in change_request:
        pass

    if 'cortical_synaptic_attractivity' in change_request:
        pass

    if 'cortical_destinations' in change_request:
        pass

    print("@ @ " * 20)
    print("Cortical change request for %s has been processed" % cortical_area)
    print("@ @ " * 20)
