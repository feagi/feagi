
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

    if change_request['cortical_name'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["cortical_name"] = \
            change_request['cortical_name']

    if change_request['cortical_coordinates'] is not None:
        x_cortical_reposition(cortical_area=cortical_area,
                              new_coordinates=change_request['cortical_coordinates'])


    if change_request['neuron_fire_threshold'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["firing_threshold"] = \
            change_request['neuron_fire_threshold']


    if change_request['neuron_leak_coefficient'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["leak_coefficient"] = \
            change_request['neuron_leak_coefficient']

    if change_request['neuron_psp_uniform_distribution'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] = \
            change_request['neuron_psp_uniform_distribution']

    if change_request['neuron_post_synaptic_potential'] is not None:
        for neuron_id in runtime_data.brain[cortical_area]:
            for dst_neuron in runtime_data.brain[cortical_area][neuron_id]["neighbors"]:
                runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    change_request['neuron_post_synaptic_potential']

    if change_request['neuron_refractory_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["refractory_period"] = \
            change_request['neuron_refractory_period']

    if change_request['neuron_snooze_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["snooze_length"] = \
            change_request['neuron_snooze_period']

    if change_request['neuron_degeneracy_coefficient'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
            change_request['neuron_degeneracy_coefficient']

    if change_request['neuron_plasticity_constant'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["plasticity_constant"] = \
            change_request['neuron_plasticity_constant']

    if change_request['neuron_post_synaptic_potential_max'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
            change_request['neuron_post_synaptic_potential_max']

    if change_request['neuron_consecutive_fire_count'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["consecutive_fire_cnt_max"] = \
            change_request['neuron_consecutive_fire_count']

    if change_request['cortical_visibility'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["visualization"] = \
            change_request['cortical_visibility']

    # Todo
    if change_request['cortical_group'] is not None:
        pass

    if change_request['cortical_dimension'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][0] = \
            change_request['cortical_dimension']["x"]
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][1] = \
            change_request['cortical_dimension']["y"]
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][2] = \
            change_request['cortical_dimension']["z"]

    if change_request['cortical_neuron_per_vox_count'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
            change_request['cortical_neuron_per_vox_count']

    if change_request['cortical_synaptic_attractivity'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
            change_request['cortical_synaptic_attractivity']

    if change_request['cortical_destinations'] is not None:
        pass

    cortical_regeneration_triggers = ['cortical_dimension',
                                      'cortical_neuron_per_vox_count',
                                      'cortical_synaptic_attractivity',
                                      'cortical_destinations']

    regeneration_flag = False
    for request in change_request:
        if request in cortical_regeneration_triggers:
            regeneration_flag = True
            print("Regeneration condition has been detected!")

    if regeneration_flag:
        cortical_mappings = synapse.cortical_mapping()
        upstream_cortical_areas = set()
        downstream_cortical_areas = set(cortical_mappings[cortical_area])
        for area in cortical_mappings:
            if cortical_area in cortical_mappings[area]:
                upstream_cortical_areas.add(area)

        # Todo: Implement a nondestructive method
        # Remove effected areas


        # Recreate neurons


        # Recreate synapses



    print("@ @ " * 20)
    print("Cortical change request for %s has been processed" % cortical_area)
    print("@ @ " * 20)
