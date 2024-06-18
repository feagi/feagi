
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
# import concurrent.futures
import random
import string
import traceback

# from src.evo import neuron, synapse, stats, genetics, voxels, neuroembryogenesis, templates
from src.evo import voxels, neuroembryogenesis, templates
from src.evo.synapse import synaptic_pruner, morphology_usage_list, cortical_areas_sharing_same_morphology
# from functools import partial
# from multiprocessing import Pool, Process
# from src.inf import disk_ops
# from src.inf import settings
from src.inf import runtime_data
from src.evo.genome_processor import genome_1_cortical_list, genome_v1_v2_converter, genome_2_1_convertor, \
    is_memory_cortical_area
from src.evo.genome_editor import save_genome
from src.evo.voxels import generate_cortical_dimensions_by_id, generate_cortical_dimensions
from src.evo.connectome import reset_connectome_file
from src.evo.region import create_region
from src.evo.synapse import neighboring_cortical_areas
from src.evo.neuroembryogenesis import cortical_name_list, develop, generate_plasticity_dict
from src.inf.initialize import init_fcl, init_memory_register, init_cortical_cumulative_stats
from src.api.schemas import NewRegionProperties

# from src.evo.synaptogenesis_rules import syn_memory

logger = logging.getLogger(__name__)


class CustomError(Exception):
    def __init__(self, message, status_code):
        self.message = message
        self.status_code = status_code


def x_neurogenesis():
    pass


def x_synaptogenesis():
    pass


def x_corticogenesis(cortical_area):
    if cortical_area not in runtime_data.cortical_list:
        runtime_data.cortical_list.append(cortical_area)
    runtime_data.brain[cortical_area] = {}
    init_fcl(cortical_area)
    print(f"Corticogenesis of \"{cortical_area}\" completed.")


def x_cortical_resize():
    pass


def x_cortical_reposition(cortical_area, new_coordinates):
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][0] = new_coordinates[0]
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][1] = new_coordinates[1]
    runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"][2] = new_coordinates[2]


def x_cortical_reposition_2d(cortical_area, new_coordinates):
    runtime_data.genome['blueprint'][cortical_area]["2d_coordinate"][0] = new_coordinates[0]
    runtime_data.genome['blueprint'][cortical_area]["2d_coordinate"][1] = new_coordinates[1]


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
    changed_areas = set()
    regeneration_flag = False

    if 'cortical_id' not in cortical_properties:
        print("ERROR: Cortical change request did not include --cortical id--")
        return

    logger.info(f"+++++++++++++++++++++   Cortical Change Request Received for %s ++++++++++++++++++++++++"
                f"  {cortical_properties['cortical_id']}")
    cortical_area = cortical_properties['cortical_id']

    if cortical_properties['cortical_name'] is not None:
        runtime_data.genome['blueprint'][cortical_area]["cortical_name"] = \
            cortical_properties['cortical_name']
        changed_areas.add("name")

    if cortical_properties['coordinates_3d'] is not None:
        x_cortical_reposition(cortical_area=cortical_area,
                              new_coordinates=cortical_properties['coordinates_3d'])
        changed_areas.add("3d_loc")

    if cortical_properties['coordinates_2d'] is not None:
        x_cortical_reposition_2d(cortical_area=cortical_area,
                                 new_coordinates=cortical_properties['coordinates_2d'])
        changed_areas.add("2d_loc")

    if cortical_properties['neuron_fire_threshold'] is not None:
        runtime_data.genome['blueprint'][cortical_area]['firing_threshold'] = \
            cortical_properties['neuron_fire_threshold']

        for neuron_ in runtime_data.brain[cortical_area]:
            runtime_data.brain[cortical_area][neuron_]['firing_threshold'] = \
                cortical_properties['neuron_fire_threshold']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_post_synaptic_potential'] is not None:
        for neuron_id in runtime_data.brain[cortical_area]:
            for dst_neuron in runtime_data.brain[cortical_area][neuron_id]["neighbors"]:
                runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    cortical_properties['neuron_post_synaptic_potential']
        runtime_data.genome['blueprint'][cortical_area]["postsynaptic_current"] = \
            cortical_properties['neuron_post_synaptic_potential']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_refractory_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["refractory_period"] = \
            cortical_properties['neuron_refractory_period']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_excitability'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["neuron_excitability"] = \
            cortical_properties['neuron_excitability']
        runtime_data.genome["blueprint"][cortical_area]["neuron_excitability"] = \
            cortical_properties['neuron_excitability']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_snooze_period'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["snooze_length"] = \
            cortical_properties['neuron_snooze_period']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_degeneracy_coefficient'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["degeneration"] = \
            cortical_properties['neuron_degeneracy_coefficient']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_post_synaptic_potential_max'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["postsynaptic_current_max"] = \
            cortical_properties['neuron_post_synaptic_potential_max']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_consecutive_fire_count'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["consecutive_fire_cnt_max"] = \
            cortical_properties['neuron_consecutive_fire_count']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_mp_charge_accumulation'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["mp_charge_accumulation"] = \
            cortical_properties['neuron_mp_charge_accumulation']
        changed_areas.add("blueprint")

    if cortical_properties['neuron_mp_driven_psp'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["mp_driven_psp"] = \
            cortical_properties['neuron_mp_driven_psp']
        changed_areas.add("blueprint")

    if cortical_properties['cortical_visibility'] is not None:
        runtime_data.genome["blueprint"][cortical_area]["visualization"] = \
            cortical_properties['cortical_visibility']
        changed_areas.add("3d_viz")

    # ####################################################
    # Conditions that require cortical regeneration
    # ####################################################
    if cortical_properties['cortical_dimensions'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0] != \
                cortical_properties['cortical_dimensions'][0] and cortical_properties['cortical_dimensions'][0] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0] = \
                cortical_properties['cortical_dimensions'][0]
            changed_areas.add("3d_dimm")
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][1] != \
                cortical_properties['cortical_dimensions'][1] and cortical_properties['cortical_dimensions'][1] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][1] = \
                cortical_properties['cortical_dimensions'][1]
            changed_areas.add("3d_dimm")
        if runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][2] != \
                cortical_properties['cortical_dimensions'][2] and cortical_properties['cortical_dimensions'][2] > 0:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][2] = \
                cortical_properties['cortical_dimensions'][2]
            changed_areas.add("3d_dimm")

    if cortical_properties['cortical_neuron_per_vox_count'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] != \
                cortical_properties['cortical_neuron_per_vox_count']:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"] = \
                cortical_properties['cortical_neuron_per_vox_count']
            changed_areas.add("blueprint")

    if cortical_properties['cortical_synaptic_attractivity'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] != \
                cortical_properties['cortical_synaptic_attractivity']:
            regeneration_flag = True
            runtime_data.genome["blueprint"][cortical_area]["synapse_attractivity"] = \
                cortical_properties['cortical_synaptic_attractivity']
            changed_areas.add("blueprint")

    if cortical_properties['neuron_leak_variability'] is not None:
        if runtime_data.genome["blueprint"][cortical_area]["leak_variability"] != \
                cortical_properties['neuron_leak_variability']:
            regeneration_flag = True
            runtime_data.genome['blueprint'][cortical_area]["leak_variability"] = \
                cortical_properties['neuron_leak_variability']
            changed_areas.add("blueprint")

    if cortical_properties['neuron_leak_coefficient'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["leak_coefficient"] != \
                cortical_properties['neuron_leak_coefficient']:
            runtime_data.genome['blueprint'][cortical_area]["leak_coefficient"] = \
                cortical_properties['neuron_leak_coefficient']
            regeneration_flag = True
            changed_areas.add("blueprint")

    if cortical_properties['neuron_psp_uniform_distribution'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] != \
                cortical_properties['neuron_psp_uniform_distribution']:
            runtime_data.genome['blueprint'][cortical_area]["psp_uniform_distribution"] = \
                cortical_properties['neuron_psp_uniform_distribution']
            regeneration_flag = True
            changed_areas.add("blueprint")

    if cortical_properties['neuron_longterm_mem_threshold'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["longterm_mem_threshold"] != \
                cortical_properties['neuron_longterm_mem_threshold']:
            runtime_data.genome['blueprint'][cortical_area]["longterm_mem_threshold"] = \
                cortical_properties['neuron_longterm_mem_threshold']
            regeneration_flag = False
            changed_areas.add("blueprint")

    if cortical_properties['neuron_lifespan_growth_rate'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["lifespan_growth_rate"] != \
                cortical_properties['neuron_lifespan_growth_rate']:
            runtime_data.genome['blueprint'][cortical_area]["lifespan_growth_rate"] = \
                cortical_properties['neuron_lifespan_growth_rate']
            regeneration_flag = False
            changed_areas.add("blueprint")

    if cortical_properties['neuron_init_lifespan'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["init_lifespan"] != \
                cortical_properties['neuron_init_lifespan']:
            runtime_data.genome['blueprint'][cortical_area]["init_lifespan"] = \
                cortical_properties['neuron_init_lifespan']
            regeneration_flag = False
            changed_areas.add("blueprint")

    if cortical_properties['neuron_fire_threshold_increment'] is not None:
        genome_fire_threshold_increment = [
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_x"],
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_y"],
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_z"]
        ]
        if genome_fire_threshold_increment != \
                cortical_properties['neuron_fire_threshold_increment']:
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_x"] = \
                cortical_properties['neuron_fire_threshold_increment'][0]
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_y"] = \
                cortical_properties['neuron_fire_threshold_increment'][1]
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_increment_z"] = \
                cortical_properties['neuron_fire_threshold_increment'][2]
            regeneration_flag = True
            changed_areas.add("blueprint")

    if cortical_properties['neuron_firing_threshold_limit'] is not None:
        if runtime_data.genome['blueprint'][cortical_area]["firing_threshold_limit"] != \
                cortical_properties['neuron_firing_threshold_limit']:
            runtime_data.genome['blueprint'][cortical_area]["firing_threshold_limit"] = \
                cortical_properties['neuron_firing_threshold_limit']
            regeneration_flag = True
            changed_areas.add("blueprint")

    if regeneration_flag:
        print("@-----@ " * 10)
        logger.info(f"Cortical regeneration triggered for {cortical_area}")
        cortical_regeneration(cortical_area=cortical_area)

    runtime_data.cortical_dimensions = generate_cortical_dimensions()
    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
    save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                file_name=runtime_data.connectome_path + "genome.json")
    runtime_data.last_genome_modification_time = datetime.datetime.now()
    runtime_data.transforming_areas.remove(cortical_area)
    update_evo_change_register(change_area=changed_areas)


def update_evo_change_register(change_area: set):
    for change in change_area:
        if change in runtime_data.evo_change_register:
            runtime_data.evo_change_register[change] = runtime_data.burst_count


def update_cortical_mappings(cortical_mappings):
    cortical_area = cortical_mappings["src_cortical_area"]
    dst_cortical_area = cortical_mappings["dst_cortical_area"]
    mappings = cortical_mappings["mapping_data"]

    #  ------- Cleanup prior mappings ---------
    runtime_data.brain = synaptic_pruner(src_cortical_area=cortical_area,
                                                 dst_cortical_area=dst_cortical_area)

    if dst_cortical_area in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)

        # todo externalize this as a function
        # Clean Upstream neuron associations
        for neuron_ in runtime_data.brain[dst_cortical_area]:
            for upstream_neuron in runtime_data.brain[dst_cortical_area][neuron_]["upstream_neurons"].copy():
                if upstream_neuron[:6] == cortical_area:
                    runtime_data.brain[dst_cortical_area][neuron_]["upstream_neurons"].discard(upstream_neuron)

    # -------- Update supporting data structures -------
    src_is_mem = is_memory_cortical_area(cortical_area=cortical_area)
    dst_is_mem = is_memory_cortical_area(cortical_area=dst_cortical_area)

    if src_is_mem or dst_is_mem:
        # todo: only update impacted areas
        init_memory_register()

    #  ------- Add new mappings ---------
    runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = mappings

    if not mappings:
        runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)
        if is_memory_cortical_area(cortical_area=dst_cortical_area):
            if dst_cortical_area in runtime_data.memory_register:
                if cortical_area in runtime_data.memory_register[dst_cortical_area]:
                    runtime_data.memory_register[dst_cortical_area].remove(cortical_area)
    else:
        print("*****************              ************************        ********************")
        # todo: very inefficient. Need to only initiate synaptogenesis on changed links instead of all connections
        neuroembryogenesis.synaptogenesis(cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)

    # if src_is_mem and not dst_is_mem:
    #     for memory_neuron in runtime_data.brain[cortical_area]:
    #         synapse.memory_to_non_memory_synapse(memory_cortical_area=cortical_area, memory_neuron_id=memory_neuron)
    #     if mappings:
    #         syn_memory(src_cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)
    #
    # else:

    # todo: only update impacted areas
    generate_plasticity_dict()

    save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                file_name=runtime_data.connectome_path + "genome.json")
    update_evo_change_register(change_area={"mappings"})

    # added_mappings, removed_mappings, modified_mappings = \
    #     mapping_change_report(cortical_area=cortical_area, new_mapping=cortical_properties['cortical_destinations'])
    #
    # print(added_mappings, removed_mappings, modified_mappings)
    #
    # # Handle new mappings
    # for dst_cortical_area in added_mappings:
    #     neuroembryogenesis.synaptogenesis(cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)
    #
    #     runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = \
    #         cortical_properties['cortical_destinations'][dst_cortical_area]
    #
    # # Handle removed mappings
    # for dst_cortical_area in removed_mappings:
    #     runtime_data.brain = synapse.synaptic_pruner(src_cortical_area=cortical_area,
    #                                                  dst_cortical_area=dst_cortical_area)
    #
    #     if dst_cortical_area in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
    #         runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)
    #
    # # Handle modified mappings
    # for dst_cortical_area in modified_mappings:
    #     runtime_data.brain = synapse.synaptic_pruner(src_cortical_area=cortical_area,
    #                                                  dst_cortical_area=dst_cortical_area)
    #
    #     if dst_cortical_area in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
    #         runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'].pop(dst_cortical_area)
    #     runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][dst_cortical_area] = \
    #         cortical_properties['cortical_destinations'][dst_cortical_area]
    #
    #     neuroembryogenesis.synaptogenesis(cortical_area=cortical_area,
    #                                       dst_cortical_area=dst_cortical_area)


def update_morphology_properties(morphology_properties):
    try:
        if morphology_properties['name'] in runtime_data.genome['neuron_morphologies']:
            # if morphology_properties['name'] in runtime_data.genome['neuron_morphologies']:
            #     runtime_data.genome['neuron_morphologies'].pop(morphology_properties['name'])
            # runtime_data.genome['neuron_morphologies'][morphology_properties['name']] = dict()
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']]["type"] = \
                morphology_properties['type']
            runtime_data.genome['neuron_morphologies'][morphology_properties['name']]["parameters"] = \
                morphology_properties['parameters']
            impacted_cortical_areas = cortical_areas_sharing_same_morphology(morphology_properties['name'])
            print("<><><><><>   <><><><> Impacted areas", impacted_cortical_areas)
            for impacted_area in impacted_cortical_areas:
                print(impacted_area[0], impacted_area[1])
                cortical_rewiring(src_cortical_area=impacted_area[0], dst_cortical_area=impacted_area[1])
            save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                        file_name=runtime_data.connectome_path + "genome.json")
            update_evo_change_register(change_area={"morphology"})

        else:
            print("Error during processing morphology change request!\n Morphology name not found: ",
                  morphology_properties['name'])

    except Exception as e:
        print("Error during morphology update\n", e, traceback.print_exc())


def cortical_removal(cortical_area, genome_scrub=False):
    if cortical_area not in runtime_data.cortical_list:
        print("Error: Cortical area requested for removal does not exist:", cortical_area)

    elif cortical_area in templates.cortical_types["CORE"]["supported_devices"]:
        print("Error: Cortical areas of CORE type cannot be removed", cortical_area)
    else:
        print("Processing cortical removal for", cortical_area)
        msg = "Processing cortical removal request for" + cortical_area
        logger.info(msg=msg)

        # Update neuron count
        runtime_data.brain_stats["neuron_count"] -= len(runtime_data.brain[cortical_area])

        # cortical_area = cortical_id(cortical_name=cortical_name)
        upstream_cortical_areas, downstream_cortical_areas = \
            neighboring_cortical_areas(cortical_area, blueprint=runtime_data.genome["blueprint"])

        # Clean Upstream neuron associations
        if len(downstream_cortical_areas) > 0:
            for downstream_cortical_area in downstream_cortical_areas:
                if downstream_cortical_area:
                    for neuron in runtime_data.brain[downstream_cortical_area]:
                        for upstream_neuron in runtime_data.brain[downstream_cortical_area][neuron]["upstream_neurons"].copy():
                            if upstream_neuron[:6] == cortical_area:
                                runtime_data.brain[downstream_cortical_area][neuron]["upstream_neurons"].discard(upstream_neuron)

        # Prune affected synapses
        prune_cortical_synapses(cortical_area=cortical_area)

        # Clear connectome entries
        runtime_data.brain[cortical_area] = {}

        # Update memory register
        for memory_area in runtime_data.memory_register:
            if cortical_area in runtime_data.memory_register[memory_area]:
                runtime_data.memory_register[memory_area].remove(cortical_area)
        if cortical_area in runtime_data.memory_register:
            del runtime_data.memory_register[cortical_area]

        # Clear voxel indexes
        voxels.voxel_reset(cortical_area=cortical_area)

        # FCL cleanup
        if cortical_area in runtime_data.fire_candidate_list:
            runtime_data.fire_candidate_list.pop(cortical_area)
        if cortical_area in runtime_data.future_fcl:
            runtime_data.future_fcl.pop(cortical_area)
        if cortical_area in runtime_data.previous_fcl:
            runtime_data.previous_fcl.pop(cortical_area)

        # Update Plasticity Dict
        generate_plasticity_dict()

        # Update memory register
        if cortical_area in runtime_data.memory_register:
            runtime_data.memory_register.pop(cortical_area)

        # Optional genome scrub
        if genome_scrub:
            if cortical_area in runtime_data.genome['blueprint']:
                runtime_data.genome['blueprint'].pop(cortical_area)

            runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
            runtime_data.cortical_dimensions = generate_cortical_dimensions()
            runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
            for upstream_area in upstream_cortical_areas:
                if upstream_area in runtime_data.genome['blueprint']:
                    if cortical_area in runtime_data.genome['blueprint'][upstream_area]['cortical_mapping_dst']:
                        runtime_data.genome['blueprint'][upstream_area]['cortical_mapping_dst'].pop(cortical_area)

        runtime_data.manual_delete_list.add(cortical_area)
        save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                    file_name=runtime_data.connectome_path + "genome.json")


def prune_cortical_synapses(cortical_area):
    upstream_cortical_areas, downstream_cortical_areas = \
        neighboring_cortical_areas(cortical_area, blueprint=runtime_data.genome["blueprint"])

    for src_cortical_area in upstream_cortical_areas:
        print("++++++ 2")
        runtime_data.brain = synaptic_pruner(src_cortical_area=src_cortical_area,
                                                     dst_cortical_area=cortical_area)


def cortical_regeneration(cortical_area):
    # Clearing the burst engine from neuronal activities
    print("##### 1 #####")
    # Reset effected areas
    cortical_removal(cortical_area=cortical_area)
    print("##### 2 #####")
    x_corticogenesis(cortical_area)
    print("%%%%%%%     Brain:\n", runtime_data.brain[cortical_area])
    print("##### 3 #####")
    upstream_cortical_areas, downstream_cortical_areas = \
        neighboring_cortical_areas(cortical_area, runtime_data.genome["blueprint"])
    print("##### 4 #####")
    # Recreate voxels
    neuroembryogenesis.voxelogenesis(cortical_area=cortical_area)
    print("##### 5 #####")
    # Recreate neurons
    neuroembryogenesis.neurogenesis(cortical_area=cortical_area)
    print("##### 6 #####")

    print(runtime_data.genome["blueprint"][cortical_area])
    # Recreate synapses
    for src_cortical_area in upstream_cortical_areas:
        neuroembryogenesis.synaptogenesis(cortical_area=src_cortical_area, dst_cortical_area=cortical_area)
    print("##### 7 #####", downstream_cortical_areas)
    for dst_cortical_area in downstream_cortical_areas:
        if dst_cortical_area:
            neuroembryogenesis.synaptogenesis(cortical_area=cortical_area, dst_cortical_area=dst_cortical_area)


def cortical_rewiring(src_cortical_area, dst_cortical_area):
    print("++++++ 3")
    synaptic_pruner(src_cortical_area=src_cortical_area, dst_cortical_area=dst_cortical_area)
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


def cortical_id_gen(seed='___', is_memory=False):
    while True:
        chars = string.ascii_uppercase + string.digits
        if not is_memory:
            random_id = 'C' + str('').join(random.choice(chars) for _ in range(2)) + seed
            if random_id not in runtime_data.cortical_list:
                return random_id
        else:
            random_id = 'M' + str('').join(random.choice(chars) for _ in range(2)) + seed
            if random_id not in runtime_data.cortical_list:
                return random_id


def add_core_cortical_area(cortical_properties):
    try:
        template = templates.cortical_template.copy()
        cortical_type = cortical_properties['cortical_type']
        cortical_id_ = cortical_properties['cortical_id']
        if cortical_id_ in templates.cortical_types[cortical_type]["supported_devices"]:
            cortical_name = \
                templates.cortical_types[cortical_type]["supported_devices"][cortical_id_]['cortical_name']

            if cortical_id_ in runtime_data.genome['blueprint']:
                print("Warning! Cortical area already part of genome. Nothing got added.")
            else:
                reset_connectome_file(cortical_area=cortical_id_)
                runtime_data.voxel_dict[cortical_id_] = dict()
                runtime_data.genome['blueprint'][cortical_id_] = dict()
                runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
                runtime_data.genome["blueprint"][cortical_id_] = \
                    template.copy()
                runtime_data.genome["blueprint"][cortical_id_]["cortical_name"] = cortical_name
                runtime_data.genome['blueprint'][cortical_id_]["block_boundaries"] = \
                    [cortical_properties['channel_count'] *
                     templates.cortical_types[cortical_type]['supported_devices'][cortical_id_]['resolution'][0],
                     templates.cortical_types[cortical_type]['supported_devices'][cortical_id_]['resolution'][1],
                     templates.cortical_types[cortical_type]['supported_devices'][cortical_id_]['resolution'][2],
                     ]

                runtime_data.genome['blueprint'][cortical_id_]["relative_coordinate"] = \
                    [cortical_properties['coordinates_3d'][0],
                     cortical_properties['coordinates_3d'][1],
                     cortical_properties['coordinates_3d'][2]]

                runtime_data.genome['blueprint'][cortical_id_]["2d_coordinate"] = \
                    [cortical_properties['coordinates_2d'][0],
                     cortical_properties['coordinates_2d'][1]]

                runtime_data.genome['blueprint'][cortical_id_]['cortical_mapping_dst'] = dict()

                for parameter in template:
                    runtime_data.genome["blueprint"][cortical_id_][parameter] = template[parameter]

                # runtime_data.genome["blueprint"][cortical_id_]["per_voxel_neuron_cnt"] = \
                #     template['per_voxel_neuron_cnt']
                # runtime_data.genome["blueprint"][cortical_id_]["synapse_attractivity"] = \
                #     template['synapse_attractivity']
                # runtime_data.genome["blueprint"][cortical_id_]["postsynaptic_current"] = \
                #     template['postsynaptic_current']
                # runtime_data.genome["blueprint"][cortical_id_]["plasticity_constant"] = \
                #     template['plasticity_constant']
                # runtime_data.genome["blueprint"][cortical_id_]["degeneration"] = \
                #     template['degeneration']
                # runtime_data.genome["blueprint"][cortical_id_]["psp_uniform_distribution"] = \
                #     template['psp_uniform_distribution']
                # runtime_data.genome["blueprint"][cortical_id_]["postsynaptic_current_max"] = \
                #     template['postsynaptic_current_max']
                # runtime_data.genome["blueprint"][cortical_id_]["mp_charge_accumulation"] = \
                #     template['mp_charge_accumulation']
                # runtime_data.genome["blueprint"][cortical_id_]["mp_driven_psp"] = \
                #     template['mp_driven_psp']
                # runtime_data.genome["blueprint"][cortical_id_]["firing_threshold_increment"] = \
                #     template['firing_threshold_increment']
                # runtime_data.genome["blueprint"][cortical_id_]["firing_threshold_limit"] = \
                #     template['firing_threshold_limit']

                runtime_data.genome["blueprint"][cortical_id_]["group_id"] = cortical_properties['cortical_type']

                neuroembryogenesis.voxelogenesis(cortical_area=cortical_id_)
                neuroembryogenesis.neurogenesis(cortical_area=cortical_id_)
                init_fcl(cortical_id_)
                init_cortical_cumulative_stats(cortical_area=cortical_id_)
                runtime_data.cortical_dimensions = generate_cortical_dimensions()
                runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()

                save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                            file_name=runtime_data.connectome_path + "genome.json")
                runtime_data.last_genome_modification_time = datetime.datetime.now()
                return cortical_id_
        else:
            print(f"Warning! while adding core cortical area. {cortical_id_} is not defined as {cortical_type}, possibly a bad gene.")

    except KeyError:
        print("Error: New cortical area was not added.", traceback.print_exc())


def add_custom_cortical_area(cortical_name, coordinates_3d, coordinates_2d, cortical_dimensions,
                             parent_region_id="root", cortical_id_overwrite=None, is_memory=False, copy_of=None):
    # Generate Cortical ID
    # todo: instead of hard coding the length have the genome properties captured and reference instead
    temp_name = cortical_name
    if len(cortical_name) < 3:
        temp_name = cortical_name + "000"
    if cortical_id_overwrite:
        cortical_area = cortical_id_overwrite
    else:
        cortical_area = cortical_id_gen(temp_name[:3], is_memory=is_memory)

    cortical_names = neuroembryogenesis.cortical_name_list()
    if copy_of:
        if copy_of in runtime_data.genome["blueprint"]:
            template = runtime_data.genome["blueprint"][copy_of].copy()
            template["cortical_mapping_dst"] = {}
            template["cortical_name"] = cortical_name
        else:
            print("Source cortical area chosen for cloning is not part of Genome")
            raise CustomError
    else:
        template = templates.cortical_template.copy()

    if cortical_name in cortical_names:
        print("Warning! Cortical area with same name already exists in genome. Nothing got added.")
    else:
        reset_connectome_file(cortical_area=cortical_area)
        runtime_data.voxel_dict[cortical_area] = {}
        runtime_data.genome['blueprint'][cortical_area] = {}
        runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)

        cortical_template = template.copy()

        runtime_data.genome["blueprint"][cortical_area] = cortical_template

        for parameter in cortical_template:
            runtime_data.genome["blueprint"][cortical_area][parameter] = cortical_template[parameter]

        runtime_data.genome['blueprint'][cortical_area]['cortical_name'] = cortical_name

        new_cortical_dimensions = cortical_dimensions.copy()
        runtime_data.genome['blueprint'][cortical_area]["block_boundaries"] = new_cortical_dimensions

        new_3d_coordinates = coordinates_3d.copy()
        runtime_data.genome['blueprint'][cortical_area]["relative_coordinate"] = new_3d_coordinates

        new_coordinates_2d = coordinates_2d.copy()
        runtime_data.genome['blueprint'][cortical_area]["2d_coordinate"] = new_coordinates_2d

        runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'] = {}
        
        runtime_data.genome["blueprint"][cortical_area]["sub_group_id"] = ""

        runtime_data.cortical_area_region_association[cortical_area] = parent_region_id
        runtime_data.genome["brain_regions"][parent_region_id]["areas"].append(cortical_area)

        if is_memory:
            runtime_data.genome["blueprint"][cortical_area]["longterm_mem_threshold"] = \
                cortical_template['longterm_mem_threshold']
            runtime_data.genome["blueprint"][cortical_area]["lifespan_growth_rate"] = \
                cortical_template['lifespan_growth_rate']
            runtime_data.genome["blueprint"][cortical_area]["init_lifespan"] = \
                cortical_template['init_lifespan']
            runtime_data.genome["blueprint"][cortical_area]["sub_group_id"] = "MEMORY"
            runtime_data.memory_register[cortical_area] = set()

        runtime_data.genome["blueprint"][cortical_area]["group_id"] = "CUSTOM"

        neuroembryogenesis.voxelogenesis(cortical_area=cortical_area)
        neuroembryogenesis.neurogenesis(cortical_area=cortical_area)
        init_fcl(cortical_area)
        init_cortical_cumulative_stats(cortical_area=cortical_area)
        runtime_data.cortical_dimensions = generate_cortical_dimensions()
        runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()

        save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                    file_name=runtime_data.connectome_path + "genome.json")
        runtime_data.last_genome_modification_time = datetime.datetime.now()
        return cortical_area


def append_circuit(source_genome, circuit_origin, parent_brain_region):
    print("\n\n----------------------------------  Merging a new circuit  ----------------------------------------\n\n")
    try:
        if "genome_title" in source_genome:
            brain_region_title = source_genome["genome_title"]
        else:
            brain_region_title = "imported_genome"

        if "genome_description" in source_genome:
            brain_region_description = source_genome["genome_description"]
        else:
            brain_region_description = "No Description"

        region_data = NewRegionProperties
        region_data.title = brain_region_title
        region_data.region_description = brain_region_description
        region_data.parent_region_id = parent_brain_region
        region_data.coordinates_2d = [0, 0]
        region_data.coordinates_3d = circuit_origin

        region_data.areas = []
        region_data.regions = []

        new_region_id = create_region(region_data=region_data)

        converted_genome = genome_2_1_convertor(source_genome["blueprint"])
        source_genome["blueprint"] = converted_genome['blueprint']

        src_morphologies = source_genome['neuron_morphologies']
        dst_morphologies = runtime_data.genome['neuron_morphologies']

        src_blueprint = source_genome['blueprint']
        dst_blueprint = runtime_data.genome['blueprint']

        appended_cortical_areas = set()

        amalgamation_cortical_mapping = dict()

        # Amalgamate Blueprint
        for cortical_area_id in src_blueprint:
            print(f"-----Attempting to import cortical area {cortical_area_id}")
            try:
                src_cortical_area = src_blueprint[cortical_area_id]
                new_cortical_name = src_blueprint[cortical_area_id]["cortical_name"]
                new_cortical_area_id = cortical_area_id

                new_coordinates = [src_cortical_area["relative_coordinate"][0] + circuit_origin[0],
                                   src_cortical_area["relative_coordinate"][1] + circuit_origin[1],
                                   src_cortical_area["relative_coordinate"][2] + circuit_origin[2],
                                   ]

                if src_blueprint[cortical_area_id]['group_id'] not in ["IPU", "OPU", "CORE"]:

                    if cortical_area_id in dst_blueprint:
                        while new_cortical_area_id == cortical_area_id:
                            new_cortical_area_id = cortical_area_id[:-3] + \
                                                   "".join(random.choice(string.ascii_uppercase) for _ in range(3))

                    if new_cortical_name in cortical_name_list():
                        while new_cortical_name == src_blueprint[cortical_area_id]["cortical_name"]:
                            new_cortical_name = \
                                src_blueprint[cortical_area_id]["cortical_name"] + \
                                "".join(random.choice(string.ascii_uppercase) for _ in range(3))

                    upstream_cortical_areas, downstream_cortical_areas = \
                        neighboring_cortical_areas(cortical_area=cortical_area_id, blueprint=src_blueprint)

                    for upstream_area in upstream_cortical_areas:
                        placeholder = {}
                        for destination_mapping in src_blueprint[upstream_area]["cortical_mapping_dst"]:
                            if destination_mapping == cortical_area_id:
                                placeholder = src_blueprint[upstream_area]["cortical_mapping_dst"][destination_mapping]
                        src_blueprint[upstream_area]["cortical_mapping_dst"].pop(cortical_area_id)
                        src_blueprint[upstream_area]["cortical_mapping_dst"][new_cortical_area_id] = placeholder

                    # add_custom_cortical_area(cortical_name=new_cortical_name,
                    #                          cortical_dimensions=src_blueprint[cortical_area_id]["block_boundaries"],
                    #                          cortical_coordinates=new_coordinates,
                    #                          cortical_id_overwrite=new_cortical_area_id)

                    appended_cortical_areas.add(new_cortical_area_id)
                    amalgamation_cortical_mapping[cortical_area_id] = new_cortical_area_id

                    dst_blueprint[new_cortical_area_id] = src_cortical_area.copy()
                    dst_blueprint[new_cortical_area_id]["cortical_name"] = new_cortical_name
                    dst_blueprint[new_cortical_area_id]["relative_coordinate"][0] = new_coordinates[0]
                    dst_blueprint[new_cortical_area_id]["relative_coordinate"][1] = new_coordinates[1]
                    dst_blueprint[new_cortical_area_id]["relative_coordinate"][2] = new_coordinates[2]
                    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
                    print(f"---------------Successfully imported a new cortical area.  "
                          f"id:{new_cortical_area_id} name:{new_cortical_name}")
                else:
                    if cortical_area_id not in dst_blueprint:
                        add_core_cortical_area(cortical_properties={
                            "cortical_id": cortical_area_id,
                            "cortical_type": src_blueprint[cortical_area_id]['group_id'],
                            "cortical_name": src_blueprint[cortical_area_id]['cortical_name'],
                            "coordinates_3d": [new_coordinates[0], new_coordinates[1], new_coordinates[2]],
                            "channel_count": 1,
                            "coordinates_2d": [0, 0]
                        })
                        appended_cortical_areas.add(cortical_area_id)
                        print(f"---------------Successfully imported a built-in cortical area. id:{cortical_area_id}")
            except Exception as e:
                print("Exception during cortical import", e, traceback.print_exc())

        # Amalgamate Brain Regions
        if "brain_regions" in source_genome:
            incoming_genome_region_data = source_genome["brain_regions"]

            incoming_genome_region_data["root"]["title"] = \
                source_genome.get("genome_title", "No title")

            incoming_genome_region_data["root"]["description"] = \
                source_genome.get("genome_description", "No description")

            incoming_genome_region_data["root"]["parent_region_id"] = parent_brain_region

            for sub_region in incoming_genome_region_data["root"]["regions"]:
                incoming_genome_region_data[sub_region]["parent_region_id"] = new_region_id

            for sub_area in incoming_genome_region_data["root"]["areas"]:
                runtime_data.cortical_area_region_association[sub_area] = new_region_id

            incoming_genome_region_data[new_region_id] = incoming_genome_region_data.pop('root')
        else:
            incoming_genome_region_data = dict()

        # Swap cortical IDs
        for region in incoming_genome_region_data:
            incoming_area_set = set(incoming_genome_region_data[region]["areas"])
            for area in incoming_genome_region_data[region]["areas"]:
                if area in amalgamation_cortical_mapping:
                    incoming_area_set.remove(area)
                    incoming_area_set.add(amalgamation_cortical_mapping[area])
            incoming_genome_region_data[region]["areas"] = list(incoming_area_set)

        runtime_data.genome["brain_regions"] = {**runtime_data.genome["brain_regions"], **incoming_genome_region_data}

        # Amalgamate Morphologies
        # Create a hash table for source and destination morphologies
        dst_morphology_hash_table = dict()

        for dst_morphology in dst_morphologies:
            morphology_str = json.dumps(dst_morphologies[dst_morphology])
            morphology_hash = hash(morphology_str)
            if morphology_hash not in dst_morphology_hash_table:
                dst_morphology_hash_table[morphology_hash] = set()
            dst_morphology_hash_table[morphology_hash].add(dst_morphology)

        morphology_mapping_table = dict()

        for src_morphology in src_morphologies:
            print(f"-----Attempting to import morphology {src_morphology}")
            try:
                # Check if morphology is used or not
                morphology_usage = morphology_usage_list(morphology_name=src_morphology, genome=source_genome)

                if morphology_usage:
                    morphology_str = json.dumps(src_morphologies[src_morphology])
                    morphology_hash = hash(morphology_str)
                    if morphology_hash not in dst_morphology_hash_table:
                        if src_morphology in dst_morphologies:
                            src_morphology_ = src_morphology + "_" + \
                                             "".join(random.choice(string.ascii_uppercase) for _ in range(2))
                            runtime_data.genome["neuron_morphologies"][src_morphology_] = src_morphologies[
                                src_morphology].copy()
                        else:
                            runtime_data.genome["neuron_morphologies"][src_morphology] = \
                                src_morphologies[src_morphology].copy()
                        print(f"---------------Successfully imported a morphology. id:{src_morphology}")
                    else:
                        # Build a mapping table to be used while appending the Blueprint
                        morphology_association = list(dst_morphology_hash_table[morphology_hash])[0]
                        morphology_mapping_table[morphology_association] = src_morphology
            except Exception as e:
                print("Exception during morphology transfer", e, traceback.print_exc())

        print("++ ++ ++ " * 10)
        print("Merge phase 1 completed.. pending initialization")

        for cortical_area in appended_cortical_areas:
            x_corticogenesis(cortical_area)

        develop(appended_cortical_areas)

        runtime_data.cortical_dimensions = generate_cortical_dimensions()
        runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()

        save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                    file_name=runtime_data.connectome_path + "genome.json")
        runtime_data.last_genome_modification_time = datetime.datetime.now()

        print("++ ++ ++ " * 10)
        print("Merger of new genome completed successfully!")

    except Exception as e:
        print("Exception during morphology join", e, traceback.print_exc())
