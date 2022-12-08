
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
import random
import string
import traceback

from evo import neuron, synapse, stats, genetics, voxels, neuroembryogenesis, templates
from functools import partial
from multiprocessing import Pool, Process
from inf import disk_ops
from inf import settings
from inf import runtime_data
from evo.genome_processor import genome_1_cortical_list, genome_v1_v2_converter
from evo.genome_editor import save_genome
from inf.initialize import generate_cortical_dimensions


def x_neurogenesis():
    pass


def x_synaptogenesis():
    pass


def x_corticogenesis():
    pass


def x_cortical_resize():
    pass


def x_cortical_reposition(cortical_area, new_coordinates):
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][0] = new_coordinates['x']
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][1] = new_coordinates['y']
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][2] = new_coordinates['z']

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


def update_cortical_properties(cortical_properties):
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

    print("+++++++++++++++++++++   Cortical Change Request Received for %s ++++++++++++++++++++++++ " %
          cortical_properties['cortical_id'])
    cortical_area = cortical_properties['cortical_id']

    if cortical_properties['cortical_name'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["cortical_name"] = \
            cortical_properties['cortical_name']

    if cortical_properties['cortical_coordinates'] is not None:
        x_cortical_reposition(cortical_area=cortical_area,
                              new_coordinates=cortical_properties['cortical_coordinates'])

    if cortical_properties['neuron_fire_threshold'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["firing_threshold"] = \
            cortical_properties['neuron_fire_threshold']

        for neuron_ in runtime_data.brain[cortical_area]:
            runtime_data.brain[cortical_area][neuron_]['firing_threshold'] = cortical_properties['neuron_fire_threshold']

    if cortical_properties['neuron_leak_coefficient'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["leak_coefficient"] = \
            cortical_properties['neuron_leak_coefficient']

    if cortical_properties['neuron_psp_uniform_distribution'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] = \
            cortical_properties['neuron_psp_uniform_distribution']

    if cortical_properties['neuron_post_synaptic_potential'] is not None:
        for neuron_id in runtime_data.brain[cortical_area]:
            for dst_neuron in runtime_data.brain[cortical_area][neuron_id]["neighbors"]:
                runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    cortical_properties['neuron_post_synaptic_potential']
        runtime_data.genome['blueprint'][cortical_area]["postsynaptic_current"] = \
            cortical_properties['neuron_post_synaptic_potential']

    if cortical_properties['neuron_refractory_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["refractory_period"] = \
            cortical_properties['neuron_refractory_period']

    if cortical_properties['neuron_snooze_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["snooze_length"] = \
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
        runtime_data.genome["blueprint"][cortical_area]["consecutive_fire_cnt_max"] = \
            cortical_properties['neuron_consecutive_fire_count']

    if cortical_properties['cortical_visibility'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["visualization"] = \
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

            if dst_cortical_area in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
                runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)
            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = \
                cortical_properties['cortical_destinations'][dst_cortical_area]

            neuroembryogenesis.synaptogenesis(cortical_area=cortical_area,
                                              dst_cortical_area=dst_cortical_area)

    # ####################################################
    # Conditions that require cortical regeneration
    # ####################################################
    if cortical_properties['cortical_dimensions'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0] != \
                cortical_properties['cortical_dimensions']["x"] and cortical_properties['cortical_dimensions']["x"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0] = \
                cortical_properties['cortical_dimensions']["x"]
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][1] != \
                cortical_properties['cortical_dimensions']["y"] and cortical_properties['cortical_dimensions']["y"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][1] = \
                cortical_properties['cortical_dimensions']["y"]
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][2] != \
                cortical_properties['cortical_dimensions']["z"] and cortical_properties['cortical_dimensions']["z"] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][2] = \
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

    if regeneration_flag:
        cortical_regeneration(cortical_area=cortical_area)

    runtime_data.cortical_dimensions = generate_cortical_dimensions()
    save_genome(genome=genome_v1_v2_converter(runtime_data.genome), file_name="../runtime_genome.json")
    runtime_data.last_genome_modification_time = datetime.datetime.now()


def update_morphology_properties(morphology_properties):
    try:
        if morphology_properties['name'] in runtime_data.genome['neuron_morphologies']:
            if morphology_properties['name'] in runtime_data.genome['neuron_morphologies']:
                runtime_data.genome['neuron_morphologies'].pop(morphology_properties['name'])
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']] = dict()
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']][morphology_properties['type']] = \
                list()
            for entry in morphology_properties['morphology']:
                runtime_data.genome['neuron_morphologies'][morphology_properties['name']][morphology_properties['type']].append(entry)
            impacted_cortical_areas = synapse.cortical_areas_sharing_same_morphology(morphology_properties['name'])
            print("<><><><><>   <><><><> Impacted areas", impacted_cortical_areas)
            for impacted_area in impacted_cortical_areas:
                print(impacted_area[0], impacted_area[1])
                cortical_rewiring(src_cortical_area=impacted_area[0], dst_cortical_area=impacted_area[1])

        else:
            print("Error during processing morphology change request!\n Morphology name not found: ",
                  morphology_properties['name'])

    except Exception as e:
        print("Error during morphology update\n", e, traceback.print_exc())


def neighboring_cortical_areas(cortical_area):
    try:
        cortical_mappings = synapse.cortical_mapping()
        upstream_cortical_areas = set()
        downstream_cortical_areas = set(cortical_mappings[cortical_area])
        for area in cortical_mappings:
            if cortical_area in cortical_mappings[area]:
                upstream_cortical_areas.add(area)
        return upstream_cortical_areas, downstream_cortical_areas
    except KeyError:
        print("Error: Cortical area not found", traceback.print_exc())


def cortical_removal(cortical_area, genome_scrub=False):
    if cortical_area not in runtime_data.cortical_list:
        print("Error: Cortical area requested for removal does not exist:", cortical_area)
    else:
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
            if cortical_area in runtime_data.genome['blueprint']:
                runtime_data.genome['blueprint'].pop(cortical_area)
            if cortical_area in runtime_data.fire_candidate_list:
                runtime_data.fire_candidate_list.pop(cortical_area)
            if cortical_area in runtime_data.previous_fcl:
                runtime_data.previous_fcl.pop(cortical_area)
            if cortical_area in runtime_data.future_fcl:
                runtime_data.future_fcl.pop(cortical_area)
            runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
            runtime_data.cortical_dimensions = generate_cortical_dimensions()
            for upstream_area in upstream_cortical_areas:
                if upstream_area in runtime_data.genome['blueprint']:
                    if cortical_area in runtime_data.genome['blueprint'][upstream_area]['cortical_mapping_dst']:
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

    # Recreate voxels
    neuroembryogenesis.voxelogenesis(cortical_area=cortical_area)

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


def cortical_id_gen(seed='___'):
    while True:
        chars = string.ascii_uppercase + string.digits
        random_id = 'C' + str('').join(random.choice(chars) for _ in range(2)) + seed
        if random_id not in runtime_data.cortical_list:
            return random_id


def add_core_cortical_area(cortical_properties):
    try:
        template = templates.cortical_template.copy()
        cortical_type = cortical_properties['cortical_type']
        cortical_name = cortical_properties['cortical_name']
        cortical_area = \
            templates.cortical_types[cortical_type]["supported_devices"][cortical_name]['cortical_id']

        if cortical_area in runtime_data.genome['blueprint']:
            print("Warning! Cortical area already part of genome. Nothing got added.")
        else:
            neuroembryogenesis.reset_connectome_file(cortical_area=cortical_area)
            runtime_data.voxel_dict[cortical_area] = dict()
            runtime_data.genome['blueprint'][cortical_area] = dict()
            runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
            runtime_data.genome["blueprint"][cortical_area]["cortical_name"] = cortical_name
            runtime_data.genome['blueprint'][cortical_area] = dict()
            runtime_data.genome["blueprint"][cortical_area] = \
                template.copy()

            runtime_data.genome['blueprint'][cortical_area]["block_boundaries"] = \
                [cortical_properties['channel_count'] *
                 templates.cortical_types[cortical_type]['supported_devices'][cortical_name]['resolution'][0],
                 templates.cortical_types[cortical_type]['supported_devices'][cortical_name]['resolution'][1],
                 templates.cortical_types[cortical_type]['supported_devices'][cortical_name]['resolution'][2],
                 ]

            runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"] = \
                [cortical_properties['cortical_coordinates']['x'],
                 cortical_properties['cortical_coordinates']['y'],
                 cortical_properties['cortical_coordinates']['z']]
            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'] = dict()
            runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
                template['per_voxel_neuron_cnt']
            runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
                template['synapse_attractivity']
            runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current"] = \
                template['postsynaptic_current']
            runtime_data.genome["blueprint"][cortical_area]["plasticity_constant"] = \
                template['plasticity_constant']
            runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
                template['degeneration']
            runtime_data.genome["blueprint"][cortical_area]["psp_uniform_distribution"] = \
                template['psp_uniform_distribution']
            runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
                template['postsynaptic_current_max']

            runtime_data.genome["blueprint"][cortical_area]["group_id"] = cortical_properties['cortical_type']

            neuroembryogenesis.voxelogenesis(cortical_area=cortical_area)
            neuroembryogenesis.neurogenesis(cortical_area=cortical_area)
            runtime_data.cortical_dimensions = generate_cortical_dimensions()

            save_genome(genome=genome_v1_v2_converter(runtime_data.genome), file_name="../runtime_genome.json")
            runtime_data.last_genome_modification_time = datetime.datetime.now()

    except KeyError:
        print("Error: New cortical area was not added.", traceback.print_exc())


def add_custom_cortical_area(cortical_properties):
    # Generate Cortical ID
    # todo: instead of hard coding the length have the genome properties captured and reference instead
    cortical_area = cortical_id_gen(cortical_properties['cortical_name'][:3])

    cortical_names = neuroembryogenesis.cortical_name_list()
    template = templates.cortical_template.copy()

    if cortical_properties['cortical_name'] in cortical_names:
        print("Warning! Cortical area with same name already exists in genome. Nothing got added.")
    else:
        neuroembryogenesis.reset_connectome_file(cortical_area=cortical_area)
        runtime_data.voxel_dict[cortical_area] = {}
        runtime_data.genome['blueprint'][cortical_area] = {}
        runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)


        runtime_data.genome["blueprint"][cortical_area] = \
            template.copy()

        runtime_data.genome['blueprint'][cortical_area]['cortical_name'] = cortical_properties['cortical_name']

        runtime_data.genome['blueprint'][cortical_area]["block_boundaries"] = \
            [cortical_properties['cortical_dimensions']['x'],
             cortical_properties['cortical_dimensions']['y'],
             cortical_properties['cortical_dimensions']['z']]

        runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"] = \
            [cortical_properties['cortical_coordinates']['x'],
             cortical_properties['cortical_coordinates']['y'],
             cortical_properties['cortical_coordinates']['z']]

        runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'] = {}
        runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
            template['per_voxel_neuron_cnt']
        runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
            template['synapse_attractivity']
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current"] = \
            template['postsynaptic_current']
        runtime_data.genome["blueprint"][cortical_area]["plasticity_constant"] = \
            template['plasticity_constant']
        runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
            template['degeneration']
        runtime_data.genome["blueprint"][cortical_area]["psp_uniform_distribution"] = \
            template['psp_uniform_distribution']
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
            template['postsynaptic_current_max']

        runtime_data.genome["blueprint"][cortical_area]["group_id"] = "CUSTOM"

        neuroembryogenesis.voxelogenesis(cortical_area=cortical_area)
        neuroembryogenesis.neurogenesis(cortical_area=cortical_area)
        runtime_data.cortical_dimensions = generate_cortical_dimensions()

        save_genome(genome=genome_v1_v2_converter(runtime_data.genome), file_name="../runtime_genome.json")
        runtime_data.last_genome_modification_time = datetime.datetime.now()
