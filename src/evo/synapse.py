
#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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
This module covers needed tools for synapse creation.
"""
import random
import logging
from src.evo.synaptogenesis_rules import *
from src.inf import runtime_data
import traceback
from src.npu.physiology import post_synaptic_current_update
from src.evo.voxels import subregion_neurons
from src.evo.genome_processor import is_memory_cortical_area

logger = logging.getLogger(__name__)


def synapse(cortical_area, src_id, dst_cortical_area, dst_id, postsynaptic_current=0):
    """
    Function responsible for creating a synapse between a neuron and another one. In reality a single neuron can
    have many synapses with another individual neuron. Here we use synaptic strength to simulate the same.
    Note: Synapse association is captured on the Source Neuron side within Connectome

    # Input: The id for source and destination Neuron plus the parameter defining connection strength
    # Source provides the Axon and connects to Destination Dendrite
    # postsynaptic_current is intended to provide the level of synaptic strength
    """

    # Check to see if the source and destination ids are valid if not exit the function
    if dst_id not in runtime_data.brain[cortical_area][src_id]["neighbors"]:

        # Calculating the effective postSynapticCurrent(PSC) value
        # PSC with negative value will have an inhibitory effect

        runtime_data.brain[cortical_area][src_id]["neighbors"][dst_id] = \
            {"cortical_area": dst_cortical_area, "postsynaptic_current": postsynaptic_current}

        # Adding upstream neuron list to the brain
        if dst_id not in runtime_data.brain[dst_cortical_area]:
            print(cortical_area, src_id, dst_cortical_area, dst_id, "....not found")

        if "upstream_neurons" not in runtime_data.brain[dst_cortical_area][dst_id]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"] = set()

        if src_id not in runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"].add(src_id)

    else:
        # In the case of an existing synapse present, post synaptic current of the previous synapse will be added to
        # the new synapse
        existing_psc = runtime_data.brain[cortical_area][src_id]["neighbors"][dst_id]["postsynaptic_current"]
        new_psc = existing_psc + postsynaptic_current
        post_synaptic_current_update(cortical_area_src=cortical_area,
                                     cortical_area_dst=dst_cortical_area,
                                     neuron_id_src=src_id,
                                     neuron_id_dst=dst_id,
                                     post_synaptic_current=new_psc)


def bidirectional_synapse(cortical_area1, neuron1, cortical_area2, neuron2):
    synapse(cortical_area=cortical_area1, src_id=neuron1, dst_cortical_area=cortical_area2, dst_id=neuron2)
    synapse(cortical_area=cortical_area2, src_id=neuron2, dst_cortical_area=cortical_area1, dst_id=neuron1)

    return


def neighbor_reset(cortical_area):
    """
    This function deletes all the neighbor relationships in the connectome
    """

    for key in runtime_data.brain[cortical_area]:
        runtime_data.brain[cortical_area][key]["neighbors"] = {}

    return


def neighbor_candidate_generator(src_cortical_area, src_neuron_id, dst_cortical_area):
    """
    Identifies the list of candidate neurons in the destination cortical area that based on the rules defined by
    source cortical area are suitable fit for synapse creation.

    Args:
        src_cortical_area:
        src_neuron_id:
        dst_cortical_area:

    Returns:
        List of candidate Neurons

    """
    synapse_candidate_list = []

    return synapse_candidate_list


def neighbor_builder(cortical_area, brain, genome, brain_gen, cortical_area_dst):
    """
    Crawls thru a Cortical area/layer and builds Synapses with another Cortical area/layer

    todo: take advantage of multi processing building the synapses for a given cortical area
    todo: deficiency when brain gen is false
    """
    # to accommodate the new namespace used by multiprocessing
    if brain_gen:
        runtime_data.brain = brain
        runtime_data.genome = genome

    synapse_count = 0

    mappings = runtime_data.genome["blueprint"][cortical_area]['cortical_mapping_dst'][cortical_area_dst]
    for morphology in mappings:
        try:
            # subregion definition is the start and end points of a vector defining a subregion
            # Example: ((x1, y1, z1), (x2, y2, z2))
            # src_subregions with be a collection of all subregions associated with a select composite morphology
            morphology_properties = runtime_data.genome["neuron_morphologies"][morphology["morphology_id"]]
            mapper_morphology = None
            src_subregions = set()
            if morphology_properties["type"] == "composite":
                src_subregions = define_subregions(cortical_area=cortical_area,
                                                   parameters=morphology_properties["parameters"])

                mapper_morphology = morphology_properties["parameters"]["mapper_morphology"]
            else:
                src_subregion = (
                    (0, 0, 0),
                    tuple(runtime_data.genome["blueprint"][cortical_area]["block_boundaries"])
                )
                src_subregions.add(src_subregion)

            for src_subregion in src_subregions:
                subregion_neuron_list = subregion_neurons(src_cortical_area=cortical_area,
                                                          region_definition=src_subregion)

                for src_id in subregion_neuron_list:
                    # Cycle through the neighbor_candidate_list and establish Synapses
                    # neighbor_candidates contain the list of candidate connections along with associated
                    # postSynapticCurrent
                    # morphology_ = runtime_data.genome["neuron_morphologies"][morphology["morphology_id"]]
                    if runtime_data.brain[cortical_area][src_id]["immortal"]:
                        neighbor_candidates = neighbor_finder(cortical_area_src=cortical_area,
                                                              cortical_area_dst=cortical_area_dst,
                                                              src_neuron_id=src_id,
                                                              morphology_=morphology,
                                                              morphology_id_overwrite=mapper_morphology,
                                                              src_subregion=src_subregion)

                        if neighbor_candidates:
                            synapses_added = synapse_to_neighbor_candidates(src_area=cortical_area,
                                                                            src_neuron=src_id,
                                                                            dst_area=cortical_area_dst,
                                                                            candidate_list=neighbor_candidates)

                            synapse_count += synapses_added

        except Exception as e:
            print(f"Exception during mapping of {morphology}", e, traceback.print_exc())
    if brain_gen:
        brain = runtime_data.brain
    else:
        brain = {}
    return synapse_count, brain


def synapse_to_neighbor_candidates(src_area, src_neuron, dst_area, candidate_list):
    synapse_count = 0
    for dst_id, psc in candidate_list:
        # Throw a die to decide for synapse creation. This is to limit the amount of synapses.
        if random.randrange(1, 100) < \
                runtime_data.genome['blueprint'][dst_area]['synapse_attractivity']:
            # Connect the source and destination neuron via creating a synapse
            synapse(cortical_area=src_area,
                    src_id=src_neuron,
                    dst_cortical_area=dst_area,
                    dst_id=dst_id,
                    postsynaptic_current=psc
                    )
            synapse_count += 1
            # print("Made a Synapse between %s and %s" % (src_id, dst_id))
    return synapse_count


def memory_to_non_memory_synapse(memory_cortical_area, memory_neuron_id):
    # todo: add counters to track global synapse count
    synapse_count = 0
    # Check if memory neuron is a long-term-memory neuron
    if runtime_data.brain[memory_cortical_area][memory_neuron_id]["immortal"]:
        for dst_cortical_area in runtime_data.genome["blueprint"][memory_cortical_area]["cortical_mapping_dst"]:
            if not is_memory_cortical_area(cortical_area=dst_cortical_area):
                for dst_neuron_id in runtime_data.brain[dst_cortical_area]:
                    synapse(cortical_area=memory_cortical_area,
                            src_id=memory_neuron_id,
                            dst_cortical_area=dst_cortical_area,
                            dst_id=dst_neuron_id,
                            postsynaptic_current=
                            runtime_data.genome["blueprint"][memory_cortical_area]["postsynaptic_current"]
                            )
                    synapse_count += 1
    return synapse_count


def cortical_mapping(blueprint=None):
    """
    Generates a cortical mapping report of the connectome
    """
    mapping_dict = {}
    if not blueprint:
        blueprint = runtime_data.genome["blueprint"]
    for cortical_area in blueprint:
        if cortical_area not in mapping_dict:
            mapping_dict[cortical_area] = []
        for dst in blueprint[cortical_area]['cortical_mapping_dst']:
            mapping_dict[cortical_area].append(dst)

    mapping_str = ""
    for cortical_area in mapping_dict:
        for dst in mapping_dict[cortical_area]:
            mapping_str = mapping_str + "\n" + cortical_area + ' ' + dst
    return mapping_dict


def synaptic_pruner(src_cortical_area, dst_cortical_area):
    synapse_count = 0
    for neuron in runtime_data.brain[src_cortical_area].copy():
        for neighbor in runtime_data.brain[src_cortical_area][neuron]['neighbors'].copy():
            if runtime_data.brain[src_cortical_area][neuron]['neighbors'][neighbor]['cortical_area'] == \
                    dst_cortical_area:
                runtime_data.brain[src_cortical_area][neuron]['neighbors'].pop(neighbor)
                synapse_count += 1
    runtime_data.brain_stats["synapse_count"] -= synapse_count
    return runtime_data.brain


def cortical_areas_sharing_same_morphology(neuron_morphology):
    cortical_list = list()
    for cortical_area in runtime_data.genome['blueprint']:
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            for mapping in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][destination]:
                if mapping['morphology_id'] == neuron_morphology:
                    cortical_list.append([cortical_area, destination])
    return cortical_list


def morphology_usage_list(morphology_name, genome):
    usage_list = set()
    for cortical_area in genome['blueprint']:
        for destination in genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            for mapping in genome['blueprint'][cortical_area]['cortical_mapping_dst'][destination]:
                if mapping["morphology_id"] \
                        == morphology_name:
                    usage_list.add((cortical_area, destination))
    return usage_list


def synapse_memory_neuron(neuron_id):
    src_memory_cortical_area = neuron_id[:6]
    src_subregion = (
        (0, 0, 0),
        tuple(runtime_data.genome["blueprint"][src_memory_cortical_area]["block_boundaries"])
    )
    if runtime_data.brain[src_memory_cortical_area][neuron_id]["immortal"]:
        for dst_cortical_area in runtime_data.genome['blueprint'][src_memory_cortical_area]['cortical_mapping_dst']:
            for morphology in runtime_data.\
                    genome['blueprint'][src_memory_cortical_area]['cortical_mapping_dst'][dst_cortical_area]:
                neighbor_candidates = neighbor_finder(cortical_area_src=src_memory_cortical_area,
                                                      cortical_area_dst=dst_cortical_area,
                                                      src_neuron_id=neuron_id,
                                                      morphology_=morphology,
                                                      src_subregion=src_subregion)
                synapses_added = synapse_to_neighbor_candidates(src_area=src_memory_cortical_area,
                                                                src_neuron=neuron_id,
                                                                dst_area=dst_cortical_area,
                                                                candidate_list=neighbor_candidates)


def neighboring_cortical_areas(cortical_area, blueprint=None):
    try:
        if not blueprint:
            blueprint = runtime_data.genome["blueprint"]
        cortical_mappings = cortical_mapping(blueprint=blueprint)
        upstream_cortical_areas = set()
        downstream_cortical_areas = set(cortical_mappings[cortical_area])
        for area in cortical_mappings:
            if cortical_area in cortical_mappings[area]:
                upstream_cortical_areas.add(area)
        return upstream_cortical_areas, downstream_cortical_areas
    except KeyError:
        print("Exception in neighboring_cortical_areas: Cortical area not found", traceback.print_exc())
