
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


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][
                1] -
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][0])

    return length


def vector_processor(vector, src_voxel, src_cortical_area, dst_cortical_area):
    """
    Returns the
    """


def synapse(cortical_area, src_id, dst_cortical_area, dst_id):
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
        postsynaptic_current = \
            runtime_data.genome['blueprint'][cortical_area]["postsynaptic_current"] * \
            runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst'][
                dst_cortical_area]["postSynapticCurrent_multiplier"]

        runtime_data.brain[cortical_area][src_id]["neighbors"][dst_id] = \
            {"cortical_area": dst_cortical_area, "postsynaptic_current": postsynaptic_current}

        # Adding upstream neuron list to the brain
        if cortical_area not in runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area] = list()
        if src_id not in runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area]:
            runtime_data.brain[dst_cortical_area][dst_id]["upstream_neurons"][cortical_area].append(src_id)

        return


def bidirectional_synapse(cortical_area1, neuron1, cortical_area2, neuron2):
    synapse(cortical_area1, neuron1, cortical_area2, neuron2)
    synapse(cortical_area2, neuron2, cortical_area1, neuron1)

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
    dst_block_boundaries = runtime_data.genome["blueprint"][cortical_area_dst]["neuron_params"]["block_boundaries"]

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
                    ((dst_pattern_x == "*" or
                      (src_pattern_x == "*" or src_pattern_x == "?" or src_pattern_x == src_x)) or
                     (dst_pattern_x == "?" and (src_pattern_x == "*" or src_pattern_x == "?" or src_x == dst_x)) or
                     (dst_pattern_x == dst_x and (src_pattern_x == "*" or (src_pattern_x == "?" and src_x == dst_x))))

                matching_condition_y = \
                    ((dst_pattern_y == "*" or
                      (src_pattern_y == "*" or src_pattern_y == "?" or src_pattern_y == src_y) or
                      (dst_pattern_y == "?" and (src_pattern_y == "*" or src_pattern_y == "?" or src_y == dst_y)) or
                      (dst_pattern_y == dst_y and (src_pattern_y == "*" or (src_pattern_y == "?" and src_y == dst_y)))))

                matching_condition_z = \
                    ((dst_pattern_z == "*" or
                      (src_pattern_z == "*" or src_pattern_z == "?" or src_pattern_z == src_z) or
                      (dst_pattern_z == "?" and (src_pattern_z == "*" or src_pattern_z == "?" or src_z == dst_z)) or
                      (dst_pattern_z == dst_z and (src_pattern_z == "*" or (src_pattern_z == "?" and src_z == dst_z)))))

                if matching_condition_x and matching_condition_y and matching_condition_z:
                    voxel_list.append([dst_x, dst_y, dst_z])

    # print("Matched voxel list based on pattern:", src_voxel, cortical_area_dst, voxel_list)

    # todo: account for morphology scalar

    return voxel_list


def voxel_list_to_neuron_list(cortical_area, voxel_list):
    neuron_list = set()

    for voxel in voxel_list:
        voxel_ref = voxels.block_reference_builder(voxel)
        neurons = voxels.neurons_in_the_block(cortical_area=cortical_area, block_ref=voxel_ref)
        for neuron in neurons:
            neuron_list.add(neuron)

    return neuron_list


def neighbor_finder(cortical_area_src, cortical_area_dst, src_neuron_id):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    candidate_voxel_list = list()

    # rule_manager = SynaptogenesisRuleManager(src_neuron_id=src_neuron_id, src_cortical_area=cortical_area_src,
    #                                          dst_cortical_area=cortical_area_dst)
    # candidate_list = rule_manager.growth_rule_selector()

    src_voxel = runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location']
    neuron_morphology = \
        runtime_data.genome["blueprint"][cortical_area_src]['cortical_mapping_dst'][
            cortical_area_dst]['morphology_id']
    morphology_scalar = \
        runtime_data.genome["blueprint"][cortical_area_src]['cortical_mapping_dst'][
            cortical_area_dst]['morphology_scalar']

    try:
        for key in runtime_data.genome["neuron_morphologies"][neuron_morphology]:
            if key == "vectors":
                for vector in runtime_data.genome["neuron_morphologies"][neuron_morphology]["vectors"]:
                    matching_vectors = match_vectors(src_voxel=src_voxel, cortical_area_dst=cortical_area_dst,
                                                     vector=vector, morphology_scalar=morphology_scalar)
                    if matching_vectors:
                        candidate_voxel_list.append(matching_vectors)

            elif key == "patterns":
                for pattern in runtime_data.genome["neuron_morphologies"][neuron_morphology]["patterns"]:

                    matching_vectors = match_patterns(src_voxel=src_voxel, cortical_area_dst=cortical_area_dst,
                                                      pattern=pattern, morphology_scalar=morphology_scalar)
                    if matching_vectors:
                        for item in matching_vectors:
                            candidate_voxel_list.append(item)

            elif key == "functions":
                if neuron_morphology == "expander_x":
                    candidate_list = expander_x(cortical_area_src, cortical_area_dst, src_neuron_id)
                    for candidate in candidate_list:
                        candidate_voxel_list.append(candidate)
                elif neuron_morphology == "reducer_x":
                    candidate_list = reducer_x(cortical_area_src, cortical_area_dst, src_neuron_id)
                    for candidate in candidate_list:
                        candidate_voxel_list.append(candidate)
                elif neuron_morphology == "randomizer":
                    candidate = randomizer(dst_cortical_area=cortical_area_dst)
                    candidate_voxel_list.append(candidate)
                elif neuron_morphology == "lateral_pairs_x":
                    candidate = lateral_pairs_x(neuron_id=src_neuron_id, cortical_area=cortical_area_src)
                    candidate_voxel_list.append(candidate)
            elif key == "placeholder":
                pass

            else:
                print("Warning! Morphology %s did not have any valid definition." % neuron_morphology)

    except Exception as e:
        print("Error during synaptogenesis of %s and %s" % (cortical_area_src, cortical_area_dst))
        print(traceback.format_exc())

    if candidate_voxel_list:
        candidate_neuron_list = \
            voxel_list_to_neuron_list(cortical_area=cortical_area_dst, voxel_list=candidate_voxel_list)
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
        neighbor_candidates = neighbor_finder(cortical_area_src=cortical_area,
                                              cortical_area_dst=cortical_area_dst,
                                              src_neuron_id=src_id)
        if neighbor_candidates:
            for dst_id in neighbor_candidates:
                # Throw a die to decide for synapse creation. This is to limit the amount of synapses.
                if random.randrange(1, 100) < \
                        runtime_data.genome['blueprint'][cortical_area_dst]['synapse_attractivity']:
                    # Connect the source and destination neuron via creating a synapse
                    synapse(cortical_area=cortical_area, src_id=src_id, dst_cortical_area=cortical_area_dst, dst_id=dst_id)
                    synapse_count += 1
                    # print("Made a Synapse between %s and %s" % (src_id, dst_id))

    if brain_gen:
        brain = runtime_data.brain
    else:
        brain = {}
    return synapse_count, brain
