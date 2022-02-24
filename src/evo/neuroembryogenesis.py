
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


# Resets the in-memory brain for each cortical area
def reset_connectome_in_mem():
    for item in runtime_data.cortical_list:
        runtime_data.brain[item] = {}


# Resets all connectome files
def reset_connectome_files():
    for key in runtime_data.genome['blueprint']:
        file_name = runtime_data.connectome_path + key + '.json'
        with open(file_name, "w") as connectome:
            connectome.write(json.dumps({}))
            connectome.truncate()
        if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
            print(settings.Bcolors.YELLOW + "Cortical area %s is has been cleared." % key
                  + settings.Bcolors.ENDC)
        runtime_data.brain[key] = {}


def reuse():
    """
    Placeholder for a function to reuse an existing connectome.

    Returns:

    """
    log.info("Reusing an old connectome")
    connectome_path = ''
    return


def connectome_backup(src, dst):
    """
    Backs up an existing connectome to preserve all the structures and associated memory data for future use.
    Args:
        src (String): Location of the source connectome folder
        dst (String): Destination folder for storing connectome backup

    Returns:

    """
    import shutil
    import errno
    try:
        shutil.copytree(src, dst)
    except OSError as exc:
        if exc.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else:
            raise


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
        print("\nBrain development lasted %s\n" % (datetime.datetime.now()-start_time))


def voxelogenesis():
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
    runtime_data.voxel_dict = {}
    for cortical_area in runtime_data.genome["blueprint"]:
        x_dim = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][0]
        y_dim = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][1]
        z_dim = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"][2]
        
        runtime_data.voxel_dict[cortical_area] = {}
        
        for x in range(x_dim):
            for y in range(y_dim):
                for z in range(z_dim):
                    voxel_id = voxels.block_reference_builder([x, y, z])
                    runtime_data.voxel_dict[cortical_area][voxel_id] = set()


def neurogenesis():
    """
    Develop Neurons for various cortical areas defined in Genome
    """
    for cortical_area in runtime_data.genome["blueprint"]:
        timer = datetime.datetime.now()
        neuron_count = 0
        for voxel in runtime_data.voxel_dict[cortical_area]:
            neuron.create_neuron(cortical_area=cortical_area, voxel=voxel)
            neuron_count += runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"]
        if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
            duration = datetime.datetime.now() - timer
            print("Neuron creation completed for cortical area: \t%s   Count: \t%i  "
                  "Duration: \t%s  Per Neuron Avg.: \t%s"
                  % (cortical_area, neuron_count, duration, duration / neuron_count))

    disk_ops.save_brain_to_disk(brain=runtime_data.brain, parameters=runtime_data.parameters)
    disk_ops.save_voxel_dict_to_disk(voxel_dict=runtime_data.voxel_dict)
    

# def neurogenesis_old():
#     # Develop Neurons for various cortical areas defined in Genome
#     for cortical_area in runtime_data.genome["blueprint"]:
#         timer = datetime.datetime.now()
#         neuron_count_ = neuron.neuron_genesis_3d(cortical_area=cortical_area)
#         if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
#             duration = datetime.datetime.now() - timer
#             if neuron_count_!= 0:
#                 print("Neuron creation completed for cortical area: \t%s   Count: \t%i  "
#                       "Duration: \t%s  Per Neuron Avg.: \t%s"
#                       % (cortical_area, neuron_count_, duration, duration / neuron_count_))
#             else:
#                 print("No neuron was created for cortical area: \t%s" % cortical_area)
#
#     disk_ops.save_brain_to_disk(brain=runtime_data.brain, parameters=runtime_data.parameters)
#     disk_ops.save_block_dic_to_disk(block_dic=runtime_data.block_dic, parameters=runtime_data.parameters)


def synaptogenesis():
    # synapse_creation_candidates = []
    # for key in runtime_data.genome["blueprint"]:
    #     if runtime_data.genome["blueprint"][key]["init_synapse_needed"]:
    #         synapse_creation_candidates.append(key)
    #     else:
    #         if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
    #             print("Synapse creation for Cortical area %s will be skipped." % key)

    # func1 = partial(build_synapse_intracortical, runtime_data.genome, runtime_data.brain, runtime_data.parameters)
    # pool1 = Pool(processes=int(runtime_data.parameters['System']['max_core']))
    #
    # pool1.map(func1, synapse_creation_candidates)
    # pool1.close()
    # pool1.join()
    #
    # print('All internal synapse creation has been completed.')
    # stats.brain_total_synapse_cnt()

    # Build Synapses across various Cortical areas
    func2 = partial(build_synapses, runtime_data.genome, runtime_data.brain, runtime_data.parameters,
                    runtime_data.voxel_dict, runtime_data.connectome_path)
    pool2 = Pool(processes=int(runtime_data.parameters['System']['max_core']))

    intercortical_mapping = pool2.map(func2, runtime_data.cortical_list)
    pool2.close()
    pool2.join()

    # Another implementation using concurrent.futures instead of Pool
    # with concurrent.futures.ProcessPoolExecutor() as executor:
    #     results = [executor.submit(build_synapses, runtime_data.genome, runtime_data.brain,
    #                runtime_data.parameters, runtime_data.block_dic, cortical_area)
    #                for cortical_area in runtime_data.cortical_list]

    # Building intercortical mapping data structure when connectome visualizer is enabled
    runtime_data.intercortical_mapping = []
    if runtime_data.parameters['Visualization']['connectome_visualizer']:
        for entry in intercortical_mapping:
            if len(entry) == 1:
                runtime_data.intercortical_mapping.append(entry[0])
            if len(entry) > 1:
                for _ in entry:
                    runtime_data.intercortical_mapping.append(_)
    print(runtime_data.intercortical_mapping)


# def build_synapse_intracortical(genome, brain, parameters, key):
#     # Read Genome data
#     timer = datetime.datetime.now()
#     synapse_count, runtime_data.brain = \
#         synapse.neighbor_builder(brain=brain, genome=genome, brain_gen=True, cortical_area=key, cortical_area_dst=key,
#                                  rule=genome["blueprint"][key]["neighbor_locator_rule_id"],
#                                  rule_param=genome["neighbor_locator_rule"]
#                                  [genome["blueprint"][key]["neighbor_locator_rule_id"]]
#                                  [genome["blueprint"][key]["neighbor_locator_rule_param_id"]],
#                                  postsynaptic_current=genome["blueprint"][key]["postsynaptic_current"])
#     if parameters["Logs"]["print_brain_gen_activities"]:
#         duration = datetime.datetime.now() - timer
#         if synapse_count != 0:
#             print("Synapse creation for Cortical area %s is now complete. Count: %i Duration: %s Per Synapse Avg.: %s"
#                   % (key, synapse_count, duration, duration / synapse_count))
#         else:
#             print("Synapse creation for Cortical area %s is now complete. Count: 0 Duration: 0 " % key)
#     disk_ops.save_brain_to_disk(cortical_area=key, brain=runtime_data.brain, parameters=parameters)


def build_synapses(genome, brain, parameters, block_dic, connectome_path, src_cortical_area):
    """
    Develops all the synapses originated from neurons within a given cortical area which could be both internal and
    external.
    """
    # if runtime_data.parameters["Database"]["influxdb_enabled"]:
    #     from inf import db_handler
    #     influxdb = db_handler.InfluxManagement()
    runtime_data.block_dic = block_dic
    intercortical_mapping = []
    runtime_data.connectome_path = connectome_path
    # Read Genome data
    cortical_genes = genome["blueprint"][src_cortical_area]

    for mapped_cortical_area in cortical_genes["cortical_mapping_dst"]:

        timer = datetime.datetime.now()
        synapse_count_, runtime_data.brain = \
            synapse.neighbor_builder(cortical_area=src_cortical_area, brain=brain, genome=genome, brain_gen=True,
                                     cortical_area_dst=mapped_cortical_area)
        # print("------------------------------Influxdb:", type(influxdb))
        # if parameters["Database"]["influx_brain_gen_stats"]:
        #     influxdb.insert_inter_cortical_stats(connectome_path=parameters["InitData"]["connectome_path"],
        #                                          cortical_area_src=src_cortical_area,
        #                                          cortical_area_dst=mapped_cortical_area,
        #                                          synapse_count=synapse_count_)
        if parameters["Logs"]["print_brain_gen_activities"]:
            morphology = cortical_genes['cortical_mapping_dst'][mapped_cortical_area]["morphology_id"]
            scalar = cortical_genes['cortical_mapping_dst'][mapped_cortical_area]["morphology_scalar"]
            duration = datetime.datetime.now() - timer
            print("Synaptogenesis: %s <> %s\t\t| Morphology:%s Scalar:%s | Synapse Count: %i | Duration: %s\t "
                  "| Per Synapse Avg.: %s"
                  % (src_cortical_area, mapped_cortical_area, morphology, scalar, synapse_count_, duration, duration /
                     (synapse_count_+1)))

        # Adding External Synapse counts to genome for future use
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
    runtime_data.brain = {}

    if parameters["Switches"]["folder_backup"]:
        # Backup the current folder
        connectome_backup('../Metis', '../Metis_archive/Metis_' + str(datetime.datetime.now()).replace(' ', '_'))

    # Reset in-memory brain data
    reset_connectome_in_mem()

    print("Defined cortical areas: %s " % runtime_data.cortical_list)
    print("::::: connectome path is:", runtime_data.connectome_path)

    # --Reset Connectome--
    reset_connectome_files()
    
    # --Voxelogenesis-- Create an empty dictionary for each voxel
    voxelogenesis()
    
    # --Neurogenesis-- Creation of all Neurons across all cortical areas
    neurogenesis()

    # --Synaptogenesis-- Build Synapses within all cortical areas
    synaptogenesis()

    # Loading connectome data from disk to memory
    runtime_data.brain = disk_ops.load_brain_in_memory()

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
            try:
                if cortical_areas[area]['cortical_mapping_dst'][mapping_dst][3]:
                    try:
                        runtime_data.plasticity_dict[area].add(mapping_dst)
                    except KeyError:
                        runtime_data.plasticity_dict[area] = set([mapping_dst])
            except KeyError:
                pass
