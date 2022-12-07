
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
This module covers needed tools for synapse creation.
"""
import random
from evo.synaptogenesis_rules import *
from evo.voxels import block_reference_builder
from inf import runtime_data
import traceback
from math import prod
from npu.physiology import post_synaptic_current_update


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['geometric_boundaries'][_][
                1] -
            runtime_data.genome['blueprint'][cortical_area]['geometric_boundaries'][_][0])

    return length


def vector_processor(vector, src_voxel, src_cortical_area, dst_cortical_area):
    """
    Returns the
    """


def psc_calculator(cortical_area, dst_cortical_area):
    if dst_cortical_area not in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        psc_multiplier = 1
    else:
        psc_multiplier = runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][
            dst_cortical_area]["postSynapticCurrent_multiplier"]

    postsynaptic_current = \
        runtime_data.genome['blueprint'][cortical_area]["postsynaptic_current"] * psc_multiplier
    return postsynaptic_current


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
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"] = {}

        if cortical_area not in runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area] = list()
        if src_id not in runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area].append(src_id)

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


def match_vectors(src_voxel, cortical_area_dst, vector, morphology_scalar):
    scaled_vector = [prod(x) for x in zip(vector, morphology_scalar)]
    candidate_vector = [sum(x) for x in zip(src_voxel, scaled_vector)]
    for item in candidate_vector:
        if item < 0:
            return None
    within_limits = voxels.block_size_checker(cortical_area=cortical_area_dst,
                                              block=voxels.block_reference_builder(candidate_vector))
    if within_limits:
        return candidate_vector


def match_patterns(src_voxel, cortical_area_dst, pattern, morphology_scalar):
    """
    Matches source voxels to destination voxels

    Expected pattern format:    [source pattern, destination pattern] e.g. [["*", "?", 3], [2, "*", "?"]]

    """
    voxel_list = list()
    dst_block_boundaries = runtime_data.genome["blueprint"][cortical_area_dst]["block_boundaries"]

    if len(pattern) != 2:
        print("Error! Pattern was not defined correctly.. "
              "should be similar to e.g. [[\"*\", \"?\", 3], [2, \"*\", \"?\"]]\n Current is as:", pattern)

    src_pattern_x, src_pattern_y, src_pattern_z = pattern[0]
    dst_pattern_x, dst_pattern_y, dst_pattern_z = pattern[1]

    src_x, src_y, src_z = src_voxel

    for dst_x in range(dst_block_boundaries[0]):
        for dst_y in range(dst_block_boundaries[1]):
            for dst_z in range(dst_block_boundaries[2]):

                matching_condition_x = \
                    ((dst_pattern_x == "*" and
                      (src_pattern_x == "*" or src_pattern_x == "?" or src_pattern_x == src_x)) or
                     (dst_pattern_x == "?" and (src_pattern_x == "*" or src_pattern_x == "?" or src_x == dst_x)) or
                     (dst_pattern_x == dst_x and (src_pattern_x == "*" or
                                                  (src_pattern_x == "?" and src_x == dst_x) or
                                                  (src_pattern_x == src_x))))

                matching_condition_y = \
                    ((dst_pattern_y == "*" and
                      (src_pattern_y == "*" or src_pattern_y == "?" or src_pattern_y == src_y) or
                      (dst_pattern_y == "?" and (src_pattern_y == "*" or src_pattern_y == "?" or src_y == dst_y)) or
                      (dst_pattern_y == dst_y and (src_pattern_y == "*" or (src_pattern_y == "?" and src_y == dst_y) or
                                                  (src_pattern_y == src_y)))))

                matching_condition_z = \
                    ((dst_pattern_z == "*" and
                      (src_pattern_z == "*" or src_pattern_z == "?" or src_pattern_z == src_z) or
                      (dst_pattern_z == "?" and (src_pattern_z == "*" or src_pattern_z == "?" or src_z == dst_z)) or
                      (dst_pattern_z == dst_z and (src_pattern_z == "*" or (src_pattern_z == "?" and src_z == dst_z) or
                                                  (src_pattern_z == src_z)))))

                if matching_condition_x and matching_condition_y and matching_condition_z:
                    voxel_list.append([dst_x, dst_y, dst_z])

    # print("Matched voxel list based on pattern:", src_voxel, cortical_area_dst, voxel_list)

    # todo: account for morphology scalar

    return voxel_list


def voxel_list_to_neuron_list(cortical_area, voxel_list):
    neuron_list = list()

    for voxel in voxel_list:
        voxel_ref = voxels.block_reference_builder(voxel[0])
        neurons = voxels.neurons_in_the_block(cortical_area=cortical_area, block_ref=voxel_ref)
        for neuron in neurons:
            neuron_list.append([neuron, voxel[1]])

    return neuron_list


def neighbor_finder(cortical_area_src, cortical_area_dst, src_neuron_id):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    # Candidate_voxel_list includes a list of destination neuron and associated postSynapticCurrent pairs
    candidate_voxel_list = list()

    # rule_manager = SynaptogenesisRuleManager(src_neuron_id=src_neuron_id, src_cortical_area=cortical_area_src,
    #                                          dst_cortical_area=cortical_area_dst)
    # candidate_list = rule_manager.growth_rule_selector()

    src_voxel = runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location']

    morphologies = runtime_data.genome["blueprint"][cortical_area_src]['cortical_mapping_dst'][cortical_area_dst]

    for morphology_ in morphologies:
        neuron_morphology = morphology_['morphology_id']
        morphology_scalar = morphology_['morphology_scalar']
        psc_multiplier = morphology_['postSynapticCurrent_multiplier']
        psc_base = runtime_data.genome["blueprint"][cortical_area_src]['postsynaptic_current']
        postSynapticCurrent = psc_multiplier * psc_base

        try:
            for key in runtime_data.genome["neuron_morphologies"][neuron_morphology]:
                if key == "vectors":
                    for vector in runtime_data.genome["neuron_morphologies"][neuron_morphology]["vectors"]:
                        matching_vectors = match_vectors(src_voxel=src_voxel, cortical_area_dst=cortical_area_dst,
                                                         vector=vector, morphology_scalar=morphology_scalar)
                        if matching_vectors:
                            candidate_voxel_list.append([matching_vectors, postSynapticCurrent])

                elif key == "patterns":
                    for pattern in runtime_data.genome["neuron_morphologies"][neuron_morphology]["patterns"]:

                        matching_vectors = match_patterns(src_voxel=src_voxel, cortical_area_dst=cortical_area_dst,
                                                          pattern=pattern, morphology_scalar=morphology_scalar)
                        if matching_vectors:
                            for item in matching_vectors:
                                candidate_voxel_list.append([item, postSynapticCurrent])

                elif key == "functions":
                    if neuron_morphology == "expander_x":
                        candidate_list = expander_x(cortical_area_src, cortical_area_dst, src_neuron_id)
                        for candidate in candidate_list:
                            candidate_voxel_list.append([candidate, postSynapticCurrent])
                    elif neuron_morphology == "reducer_x":
                        candidate_list = reducer_x(cortical_area_src, cortical_area_dst, src_neuron_id)
                        for candidate in candidate_list:
                            candidate_voxel_list.append([candidate, postSynapticCurrent])
                    elif neuron_morphology == "randomizer":
                        candidate = randomizer(dst_cortical_area=cortical_area_dst)
                        candidate_voxel_list.append([candidate, postSynapticCurrent])
                    elif neuron_morphology == "lateral_pairs_x":
                        candidate = lateral_pairs_x(neuron_id=src_neuron_id, cortical_area=cortical_area_src)
                        candidate_voxel_list.append([candidate, postSynapticCurrent])
                    elif neuron_morphology == "block_connection":
                        candidate = block_connection(cortical_area_src, cortical_area_dst, src_neuron_id, s=10)
                        candidate_voxel_list.append([candidate, postSynapticCurrent])
                elif key == "placeholder":
                    pass

                else:
                    print("Warning! Morphology %s did not have any valid definition." % neuron_morphology)

        except Exception as e:
            print("Error during synaptogenesis of %s and %s" % (cortical_area_src, cortical_area_dst))
            print(traceback.format_exc())

    if candidate_voxel_list:
        candidate_neuron_list = \
            voxel_list_to_neuron_list(cortical_area=cortical_area_dst,
                                      voxel_list=candidate_voxel_list)
        return candidate_neuron_list


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

    for src_id in runtime_data.brain[cortical_area]:
        # Cycle through the neighbor_candidate_list and establish Synapses
        # neighbor_candidates contain the list of candidate connections along with associated postSynapticCurrent
        neighbor_candidates = neighbor_finder(cortical_area_src=cortical_area,
                                              cortical_area_dst=cortical_area_dst,
                                              src_neuron_id=src_id)

        if neighbor_candidates:
            for dst_id, psc in neighbor_candidates:
                # Throw a die to decide for synapse creation. This is to limit the amount of synapses.
                if random.randrange(1, 100) < \
                        runtime_data.genome['blueprint'][cortical_area_dst]['synapse_attractivity']:
                    # Connect the source and destination neuron via creating a synapse
                    synapse(cortical_area=cortical_area,
                            src_id=src_id,
                            dst_cortical_area=cortical_area_dst,
                            dst_id=dst_id,
                            postsynaptic_current=psc
                            )
                    synapse_count += 1
                    # print("Made a Synapse between %s and %s" % (src_id, dst_id))

    if brain_gen:
        brain = runtime_data.brain
    else:
        brain = {}
    return synapse_count, brain


def cortical_mapping():
    """
    Generates a cortical mapping report of the connectome
    """
    mapping_dict = {}
    for cortical_area in runtime_data.genome['blueprint']:
        if cortical_area not in mapping_dict:
            mapping_dict[cortical_area] = []
        for dst in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            mapping_dict[cortical_area].append(dst)

    mapping_str = ""
    for cortical_area in mapping_dict:
        for dst in mapping_dict[cortical_area]:
            mapping_str = mapping_str + "\n" + cortical_area + ' ' + dst
    print(mapping_str)
    return mapping_dict


def synaptic_pruner(src_cortical_area, dst_cortical_area):
    for neuron in runtime_data.brain[src_cortical_area].copy():
        for neighbor in runtime_data.brain[src_cortical_area][neuron]['neighbors'].copy():
            if runtime_data.brain[src_cortical_area][neuron]['neighbors'][neighbor]['cortical_area'] == \
                    dst_cortical_area:
                runtime_data.brain[src_cortical_area][neuron]['neighbors'].pop(neighbor)
    return runtime_data.brain


def cortical_areas_sharing_same_morphology(neuron_morphology):
    cortical_list = list()
    for cortical_area in runtime_data.genome['blueprint']:
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            for mapping in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][destination]:
                print("mapping----", mapping, neuron_morphology)
                if mapping['morphology_id'] == neuron_morphology:
                    cortical_list.append([cortical_area, destination])
    return cortical_list
