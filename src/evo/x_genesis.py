
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
import traceback

from evo import neuron, synapse, stats, genetics, voxels, neuroembryogenesis
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


def update_cortical_properties(cortical_properties, new_area=False):
    """
    Supported changes:
    - Cortical coordinates
    - Neuron firing threshold
    - Neuron leak coefficient
    - Neuron post synaptic potential uniform distribution switch
    - Neuron post synaptic potentials
    - Cortical name

    """
    regeneration_flag = False

    if 'cortical_id' not in cortical_properties:
        print("ERROR: Cortical change request did not include --cortical id--")
        return

    cortical_area = cortical_properties['cortical_id']

    if new_area:
        neuroembryogenesis.reset_connectome_file(cortical_area=cortical_area)
        runtime_data.voxel_dict[cortical_area] = dict()
        runtime_data.genome['blueprint'][cortical_area] = dict()
        runtime_data.genome['blueprint'][cortical_area]['neuron_params'] = dict()
        runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'] = [1, 1, 1]
        runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'] = [0, 0, 0]
        runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'] = dict()
        runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
            cortical_properties["cortical_neuron_per_vox_count"]
        runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
            cortical_properties["cortical_synaptic_attractivity"]

    if cortical_properties['cortical_name'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["cortical_name"] = \
            cortical_properties['cortical_name']

    if cortical_properties['cortical_coordinates'] is not None:
        x_cortical_reposition(cortical_area=cortical_area,
                              new_coordinates=cortical_properties['cortical_coordinates'])

    if cortical_properties['neuron_fire_threshold'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["firing_threshold"] = \
            cortical_properties['neuron_fire_threshold']
        if not new_area:
            for neuron_ in runtime_data.brain[cortical_area]:
                runtime_data.brain[cortical_area][neuron_]['firing_threshold'] = cortical_properties['neuron_fire_threshold']

    if cortical_properties['neuron_leak_coefficient'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["neuron_params"]["leak_coefficient"] = \
            cortical_properties['neuron_leak_coefficient']

    if cortical_properties['neuron_psp_uniform_distribution'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] = \
            cortical_properties['neuron_psp_uniform_distribution']

    if cortical_properties['neuron_post_synaptic_potential'] is not None and not new_area:
        for neuron_id in runtime_data.brain[cortical_area]:
            for dst_neuron in runtime_data.brain[cortical_area][neuron_id]["neighbors"]:
                runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    cortical_properties['neuron_post_synaptic_potential']
        runtime_data.genome['blueprint'][cortical_area]['postsynaptic_current'] = \
            cortical_properties['neuron_post_synaptic_potential']

    if cortical_properties['neuron_refractory_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["refractory_period"] = \
            cortical_properties['neuron_refractory_period']

    if cortical_properties['neuron_snooze_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["snooze_length"] = \
            cortical_properties['neuron_snooze_period']

    if cortical_properties['neuron_degeneracy_coefficient'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
            cortical_properties['neuron_degeneracy_coefficient']

    if cortical_properties['neuron_plasticity_constant'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["plasticity_constant"] = \
            cortical_properties['neuron_plasticity_constant']

    if cortical_properties['neuron_post_synaptic_potential_max'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
            cortical_properties['neuron_post_synaptic_potential_max']

    if cortical_properties['neuron_consecutive_fire_count'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["consecutive_fire_cnt_max"] = \
            cortical_properties['neuron_consecutive_fire_count']

    if cortical_properties['cortical_visibility'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["visualization"] = \
            cortical_properties['cortical_visibility']

    if cortical_properties['cortical_destinations'] is not None:
        added_mappings, removed_mappings, modified_mappings = \
            mapping_change_report(cortical_area=cortical_area, new_mapping=cortical_properties['cortical_destinations'])

        print(added_mappings, removed_mappings, modified_mappings)

        # Handle new mappings
        for dst_cortical_area in added_mappings:
            neuroembryogenesis.synaptogenesis(cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)

            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = \
                cortical_properties['cortical_destinations'][dst_cortical_area]

        # Handle removed mappings
        for dst_cortical_area in removed_mappings:
            runtime_data.brain = synapse.synaptic_pruner(src_cortical_area=cortical_area,
                                                         dst_cortical_area=dst_cortical_area)

            if dst_cortical_area in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
                runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)

        # Handle modified mappings
        for dst_cortical_area in modified_mappings:
            runtime_data.brain = synapse.synaptic_pruner(src_cortical_area=cortical_area,
                                                         dst_cortical_area=dst_cortical_area)

            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)
            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = \
                cortical_properties['cortical_destinations'][dst_cortical_area]

            neuroembryogenesis.synaptogenesis(cortical_area=cortical_area,
                                              dst_cortical_area=dst_cortical_area)

    # ####################################################
    # Conditions that require cortical regeneration
    # ####################################################
    if cortical_properties['cortical_dimensions'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][0] != \
                cortical_properties['cortical_dimensions']["x"] and cortical_properties['cortical_dimensions']["x"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][0] = \
                cortical_properties['cortical_dimensions']["x"]
        if runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][1] != \
                cortical_properties['cortical_dimensions']["y"] and cortical_properties['cortical_dimensions']["y"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][1] = \
                cortical_properties['cortical_dimensions']["y"]
        if runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][2] != \
                cortical_properties['cortical_dimensions']["z"] and cortical_properties['cortical_dimensions']["z"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][2] = \
                cortical_properties['cortical_dimensions']["z"]

    if cortical_properties['cortical_neuron_per_vox_count'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] != \
                cortical_properties['cortical_neuron_per_vox_count']:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
                cortical_properties['cortical_neuron_per_vox_count']

    if cortical_properties['cortical_synaptic_attractivity'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] != \
                cortical_properties['cortical_synaptic_attractivity']:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
                cortical_properties['cortical_synaptic_attractivity']

    if regeneration_flag or new_area:
        cortical_regeneration(cortical_area=cortical_area)


def update_morphology_properties(morphology_properties):
    try:
        if morphology_properties['name'] in runtime_data.genome['neuron_morphologies']:
            runtime_data.genome['neuron_morphologies'].pop(morphology_properties['name'])
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']] = dict()
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']][morphology_properties['type']] = list()
            for entry in morphology_properties['morphology']:
                runtime_data.genome['neuron_morphologies'][morphology_properties['name']][morphology_properties['type']].append(entry)
            impacted_cortical_areas = synapse.cortical_areas_sharing_same_morphology(morphology_properties['name'])
            print("<><><><><>   <><><><> Impacted areas", impacted_cortical_areas)
            for impacted_area in impacted_cortical_areas:
                cortical_rewiring(src_cortical_area=impacted_area[0], dst_cortical_area=impacted_area[1])

    except Exception as e:
        print("Error during morphology update\n", e, traceback.print_exc())

    else:
        print("Error during processing morphology change request!")


def neighboring_cortical_areas(cortical_area):
    cortical_mappings = synapse.cortical_mapping()
    upstream_cortical_areas = set()
    downstream_cortical_areas = set(cortical_mappings[cortical_area])
    for area in cortical_mappings:
        if cortical_area in cortical_mappings[area]:
            upstream_cortical_areas.add(area)
    return upstream_cortical_areas, downstream_cortical_areas


def cortical_removal(cortical_area, genome_scrub=False):
    # cortical_area = cortical_id(cortical_name=cortical_name)
    upstream_cortical_areas, downstream_cortical_areas = neighboring_cortical_areas(cortical_area)

    # Prune affected synapses
    prune_cortical_synapses(cortical_area=cortical_area)
    
    # Clear connectome entries
    runtime_data.brain[cortical_area] = {}
    
    # Clear voxel indexes
    voxels.voxel_reset(cortical_area=cortical_area)

    # todo: plasticity dict
    
    # Optional genome scrub
    if genome_scrub:
        runtime_data.genome['blueprint'].pop(cortical_area)
        for upstream_area in upstream_cortical_areas:
            runtime_data.genome['blueprint'][upstream_area]['cortical_mapping_dst'].pop(cortical_area)


def prune_cortical_synapses(cortical_area):
    upstream_cortical_areas, downstream_cortical_areas = neighboring_cortical_areas(cortical_area)

    for src_cortical_area in upstream_cortical_areas:
        runtime_data.brain = synapse.synaptic_pruner(src_cortical_area=src_cortical_area,
                                                     dst_cortical_area=cortical_area)


def cortical_regeneration(cortical_area):
    upstream_cortical_areas, downstream_cortical_areas = neighboring_cortical_areas(cortical_area)
    
    # Reset effected areas
    cortical_removal(cortical_area=cortical_area)

    # Recreate neurons
    neuroembryogenesis.neurogenesis(cortical_area=cortical_area)

    # Recreate synapses
    for src_cortical_area in upstream_cortical_areas:
        neuroembryogenesis.synaptogenesis(cortical_area=src_cortical_area, dst_cortical_area=cortical_area)
    for dst_cortical_area in downstream_cortical_areas:
        neuroembryogenesis.synaptogenesis(cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)


def cortical_rewiring(src_cortical_area, dst_cortical_area):
    synapse.synaptic_pruner(src_cortical_area=src_cortical_area, dst_cortical_area=dst_cortical_area)
    neuroembryogenesis.synaptogenesis(cortical_area=src_cortical_area, dst_cortical_area=dst_cortical_area)


def cortical_id(cortical_name):
    for cortical_area in runtime_data.genome['blueprint']:
        if runtime_data.genome['blueprint'][cortical_area]["cortical_name"] == cortical_name:
            return cortical_area


def mapping_change_report(cortical_area, new_mapping):
    old_mapping = runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
    added = set()
    removed = set()
    modified = set()
    for area in new_mapping:
        if area not in old_mapping:
            added.add(area)
        else:
            if new_mapping[area] != old_mapping[area]:
                modified.add(area)
    for area in old_mapping:
        if area not in new_mapping:
            removed.add(area)
    return added, removed, modified
