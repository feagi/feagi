
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

This module is responsible for reading instructions from genome as a genotype and translating them into connectome as
a phenotype.

Instructions in this module are biologically inspired and a high level model after the neuroembryogenesis process that
translates genome to a fully developed brain starting from the embryo's neural tube.

In this framework, genome contains a list of all cortical layers and sub-layers for the brain. Each layer or sub-layer
has properties embedded within genome. Such properties are translated using this module to perform neurogenesis that
creates neurons followed by the process of synaptogenesis that is responsible for the creation of the connectivity
among neurons. All of the structures generated as a result of this process is stored in a data structure called
connectome.
"""

import logging
import json
import datetime
import shutil
import concurrent.futures
from evo import neuron, synapse, stats, genetics, voxels
from functools import partial
from multiprocessing import Pool, Process
from inf import disk_ops
from inf import settings
from inf import runtime_data
from inf.db_handler import InfluxManagement

# from igraph import *
# from igraph.directed_graph import DirectGraph

log = logging.getLogger(__name__)


# Reads the list of all Cortical areas defined in Genome
def cortical_list():
    blueprint = runtime_data.genome["blueprint"]
    cortical_list_ = []
    for key in blueprint:
        cortical_list_.append(key)
    return cortical_list_


def cortical_sub_group_members(group):
    members = []
    for item in runtime_data.cortical_list:
        if runtime_data.genome['blueprint'][item]['sub_group_id'] == group:
            members.append(item)
    return members


def synapse_count(cortical_area_src, cortical_area_dst):
    brain = runtime_data.brain
    synapse__count = 0
    for neuron in brain[cortical_area_src]:
        for synapse in brain[cortical_area_src][neuron]['neighbors']:
            if brain[cortical_area_src][neuron]['neighbors'][synapse]['cortical_area'] == cortical_area_dst:
                synapse__count += 1
    return synapse__count


def cortical_connectivity_eval():
    """
    Evaluates whether the cortical structure is meeting minimum expectations for basic functionality.

    Returns: True or False
    """


def build_cortical_map():
    """
    Develops a data structure suitable for graphing cortical connectivity using iGraph package

    Sample output structure
    graph_edges = [(0, 2), (0, 1), (0, 3), (1, 2), (1, 3), (2, 4), (3, 4), (3, 0)]
    graph_weights = [8, 6, 3, 5, 6, 4, 9, 50]
    graph_labels = ['v1', 'v2', 'v3', 'v4', 'v5']

    Returns: Cortical map in a ... format.

    """
    graph_edges, graph_weights, graph_labels = [], [], []

    labels = [label for label in runtime_data.genome['blueprint']]
    indexes = [index for index in range(len(labels))]
    graph_labels = zip(indexes, labels)
    graph_labels_dict = dict(zip(labels, indexes))

    for entry in runtime_data.intercortical_mapping:
        graph_weights.append(entry[2])
        graph_edges.append((graph_labels_dict[entry[0]], graph_labels_dict[entry[1]]))

    graph_labels_tmp = dict(graph_labels)
    graph_labels = []
    for key in graph_labels_tmp:
        graph_labels.append(graph_labels_tmp[key])
    print('graph_edges: ', graph_edges)
    print('graph_weights: ', graph_weights)
    print('graph_labels: ', graph_labels)
    return graph_edges, graph_weights, graph_labels


def ipu_opu_connectivity():
    """
    Evaluates synaptic connectivity between IPU and OPU

    Returns: True or False
    """


def connectome_structural_fitness():
    """
    To conduct a set of validations and calculate a structural fitness for the developed connectome. The returned value
    can be used to eliminate a premature structure.

    Returns: Structural fitness factor

    """
    #     vision_v2_it_synapse_cnt = synapse_count('vision_v2', 'vision_IT')
    #     vision_it_mem_synapse_cnt = synapse_count('vision_IT', 'vision_memory')
    #
    #     print("Synapse count vision_v2 >> vision_IT == ", vision_v2_it_synapse_cnt)
    #     print("Synapse count vision_IT >> vision_memory == ", vision_it_mem_synapse_cnt)
    #
    #     if vision_v2_it_synapse_cnt < 50 or vision_it_mem_synapse_cnt < 50:
    #         fitness = 0
    #     else:
    #         fitness = 1
    #     return fitness
    return


def develop_brain(reincarnation_mode=False):
    """
    This function operates in two modes. If run with reincarnation mode as false then it will develop a new brain by
    reading instructions from genome and developing it step by step. If run with reincarnation mode as True then the
    last available connectome data will be used to operate as a new brain. Reincarnation set to True will preserve the
    memories and learning from the previous generation while starting a new brain instance.

    Args:
        reincarnation_mode (bool): If true, an existing connectome will be used to reuse an existing brain structure.

    Returns:

    """
    if bool(reincarnation_mode):
        log.info('Reincarnating...')
        reuse()
    else:
        log.info('Developing a new brain...')
        genome_instructions = genetics.selection()
        start_time = datetime.datetime.now()
        develop()

        print("-----------------  Brain Development Completed Successfully -------------------")
        for _ in range(10):
            print("#" * 40)

        print("\nBrain development lasted %s\n" % (datetime.datetime.now()-start_time))


def voxelogenesis(cortical_area):
    """
    develop an empty dictionary structure for the entire brain that holds location for each voxel and neuron ids in it
    
    voxel_dict = {
        "cortical_area_1": {
            0-0-0: {"neuron_1_id", "neuron_2_id"},
            0-0-1: {"neuron_3_id"},
            0-0-2: {"neuron_4_id", "neuron_5_id"},
        },
        "cortical_area_2: {
            ...
        }
        ...
    }
    """
    runtime_data.voxel_dict[cortical_area] = {}

    x_dim = int(runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0])
    y_dim = int(runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][1])
    z_dim = int(runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][2])

    runtime_data.voxel_dict[cortical_area] = {}

    for x in range(x_dim):
        for y in range(y_dim):
            for z in range(z_dim):
                voxel_id = voxels.block_reference_builder([x, y, z])
                runtime_data.voxel_dict[cortical_area][voxel_id] = set()


def neurogenesis(cortical_area):
    """
    Develop Neurons for various cortical areas defined in Genome
    """

    timer = datetime.datetime.now()
    neuron_count = 0
    for voxel in runtime_data.voxel_dict[cortical_area]:
        neuron.create_neuron(cortical_area=cortical_area, voxel=voxel)
        neuron_count += runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"]
    if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
        duration = datetime.datetime.now() - timer
        try:
            print("Neuron creation completed for cortical area: \t%s   Count: \t%i  "
                  "Duration: \t%s  Per Neuron Avg.: \t%s"
                  % (cortical_area, neuron_count, duration, duration / neuron_count))
        except ZeroDivisionError:
            pass

    disk_ops.save_brain_to_disk(cortical_area=cortical_area,
                                brain=runtime_data.brain,
                                parameters=runtime_data.parameters)
    disk_ops.save_voxel_dict_to_disk(cortical_area=cortical_area,
                                     voxel_dict=runtime_data.voxel_dict)


def synaptogenesis(cortical_area, dst_cortical_area=None):

    build_synapses(genome=runtime_data.genome,
                   brain=runtime_data.brain,
                   voxel_dict=runtime_data.voxel_dict,
                   connectome_path=runtime_data.connectome_path,
                   src_cortical_area=cortical_area,
                   parameters=runtime_data.parameters,
                   dst_cortical_area=dst_cortical_area)


def build_synapses(genome, brain, parameters, voxel_dict, connectome_path, src_cortical_area, dst_cortical_area=None):
    """
    Develops all the synapses originated from neurons within a given cortical area which could be both internal and
    external.
    """
    runtime_data.parameters = parameters
    # if runtime_data.parameters["Database"]["influxdb_enabled"]:
    #     from inf import db_handler
    #     influxdb = db_handler.InfluxManagement()
    runtime_data.voxel_dict = voxel_dict
    intercortical_mapping = []
    runtime_data.connectome_path = connectome_path
    # Read Genome data
    cortical_genes = genome["blueprint"][src_cortical_area]

    if dst_cortical_area and dst_cortical_area in cortical_genes["cortical_mapping_dst"]:
        timer = datetime.datetime.now()
        synapse_count_, runtime_data.brain = \
            synapse.neighbor_builder(cortical_area=src_cortical_area, brain=brain, genome=genome, brain_gen=True,
                                     cortical_area_dst=dst_cortical_area)
        intercortical_mapping.append((src_cortical_area, dst_cortical_area, synapse_count_))
    else:
        for mapped_cortical_area in cortical_genes["cortical_mapping_dst"]:
            timer = datetime.datetime.now()
            synapse_count_, runtime_data.brain = \
                synapse.neighbor_builder(cortical_area=src_cortical_area, brain=brain, genome=genome, brain_gen=True,
                                         cortical_area_dst=mapped_cortical_area)

            intercortical_mapping.append((src_cortical_area, mapped_cortical_area, synapse_count_))

    disk_ops.save_brain_to_disk(cortical_area=src_cortical_area, brain=runtime_data.brain, parameters=parameters)
    return intercortical_mapping


def develop():
    print("-----------------------------------------------")
    print("-----------------------------------------------")
    print("------------  Brain generation has begun-------")
    print("-----------------------------------------------")
    print("-----------------------------------------------")

    parameters = runtime_data.parameters

    if parameters["Switches"]["folder_backup"]:
        # Backup the current folder
        connectome_backup('../Metis', '../Metis_archive/Metis_' + str(datetime.datetime.now()).replace(' ', '_'))

    print("Defined cortical areas: %s " % runtime_data.cortical_list)
    print("::::: connectome path is:", runtime_data.connectome_path)

    # --Corticogenesis-- Create definition of cortical areas in Connectome
    for cortical_area in runtime_data.cortical_list:
        runtime_data.brain[cortical_area] = {}

    # --Voxelogenesis-- Create an empty dictionary for each voxel
    for cortical_area in runtime_data.cortical_list:
        voxelogenesis(cortical_area=cortical_area)
    
    # --Neurogenesis-- Creation of all Neurons across all cortical areas
    for cortical_area in runtime_data.cortical_list:
        neurogenesis(cortical_area=cortical_area)

    print("=================================== Neurogenesis Completed ==================================")

    # --Synaptogenesis-- Build Synapses within all cortical areas
    for cortical_area in runtime_data.cortical_list:
        synaptogenesis(cortical_area=cortical_area)

    print("=================================== Synaptogenesis Completed ==================================")

    # Loading connectome data from disk to memory
    # runtime_data.brain = disk_ops.load_brain_in_memory()

    connectome_neuron_count, connectome_synapse_count = stats.brain_total_synapse_cnt()

    # The following formula was derived by curve fitting sample data
    connectome_size_on_disk = 3E-08 * connectome_neuron_count**2 + 0.0011 * connectome_neuron_count + 2.9073

    print("Neuronal mapping across all Cortical areas has been completed!!\n")
    print("Total brain neuron count:\t\t", connectome_neuron_count)
    print("Total brain synapse count:\t\t", connectome_synapse_count)
    print("Total brain est. size on disk:\t", connectome_size_on_disk, 'MB')

    brain_structural_fitness = connectome_structural_fitness()
    print("Brain structural fitness was evaluated as: ", brain_structural_fitness)

    # # Visualize the connectome
    # if runtime_data.parameters['Visualization']['connectome_visualizer']:
    #     graph_edges, graph_weights, graph_labels = build_cortical_map()
    #     graph = Graph(directed=True)
    #     graph_instance = DirectGraph(graph, edges=graph_edges, weights=graph_weights, labels=graph_labels)
    #     graph_instance.graph_in_color()

    return brain_structural_fitness


def generate_plasticity_dict():
    """
    Extracts data from the genome and creates a dictionary of source cortical areas 
    and their mapping destinations targeted for implementing neuroplasticity. 
        
        ex:    {
                    'motor_memory': {
                        'vision_memory',
                        'auditory_memory'
                    }
                }

    """

    cortical_areas = runtime_data.genome['blueprint']

    for area in cortical_areas:
        for mapping_dst in cortical_areas[area]['cortical_mapping_dst']:
            for morphology in cortical_areas[area]['cortical_mapping_dst'][mapping_dst]:
                if "plasticity_flag" in morphology:
                    if morphology['plasticity_flag']:
                        if area not in runtime_data.plasticity_dict:
                            runtime_data.plasticity_dict[area] = dict()
                        if mapping_dst not in runtime_data.plasticity_dict:
                            runtime_data.plasticity_dict[mapping_dst] = dict()

                        runtime_data.plasticity_dict[area][mapping_dst] = 'efferent'
                        runtime_data.plasticity_dict[mapping_dst][area] = 'afferent'

    print("Plasticity dict:", runtime_data.plasticity_dict)


def cortical_name_list():
    cortical_names = set()
    for cortical_area in runtime_data.genome['blueprint']:
        cortical_names.add(runtime_data.genome['blueprint'][cortical_area]['cortical_name'])
    return cortical_names


def cortical_name_to_id(cortical_name):
    for cortical_area in runtime_data.genome["blueprint"]:
        if runtime_data.genome['blueprint'][cortical_area]['cortical_name'] == cortical_name:
            return cortical_area
