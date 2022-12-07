
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


from evo import voxels
from inf import runtime_data
from random import randrange


# def rule_neuron_to_neuron(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
#     candidate_list = list()
#     # Input: neuron id of which we desire to find all candidate neurons for from another cortical region
#     src_data = runtime_data.brain[src_cortical_area]
#     dst_data = runtime_data.brain[dst_cortical_area]
#     for dst_neuron_id in dst_data:
#         if src_data[src_neuron_id]['soma_location'][0] == dst_data[dst_neuron_id]['soma_location'][0]:
#             candidate_list.append(dst_neuron_id)
#             break
#     return candidate_list


# def rule_block_distributor(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
#     """
#     This rule helps to take a set of unique inputs from one cortical area and develop synaptic projections that can
#     lead to a comprehensive set of unique connections that covers all the combinations of the input values.
#
#     Note: This function is designed for the corner case of the destination cortical area being 1 dimensional in z
#     direction
#     """
#
#     # todo: generalize this function so it takes the direction of the source and destination cortical areas as input
#     candidate_list = list()
#     block_list = blocks.z_block_refs(cortical_area=dst_cortical_area, x_ref=0, y_ref=0)
#     source_x_depth = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"][0]
#
#     for offset in range(source_x_depth):
#         for block_ref in block_list:
#             if blocks.block_ref_2_id(block_ref)[2] // (2 ** offset) % 2 == 0:
#                 for neuron in blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref):
#                     candidate_list.append(neuron)
#     return candidate_list


def syn_expander_x(src_cortical_area, dst_cortical_area, src_neuron_id, dst_y_index=0, dst_z_index=0):
    """
    This rule represents a unique combination of all blocks from the source cortical area on the destination side
    in x dim.
    """
    src_cortical_dim_x = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"][0]
    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0]

    cortical_length_binary = len(bin(dst_cortical_dim_x)) - 2

    print("***** *** ** * Expander initiated! ")

    # Note that the destination cortical area is expected to have at least 2 ^ (source block count) to be able to
    # address all the combinations
    if dst_cortical_dim_x < 2 ** src_cortical_dim_x:
        print("Warning: %s does not have enough blocks on x dim to support the needed synaptogenesis!"
              % dst_cortical_area)

    src_neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][0]

    candidate_list = list()

    for dst_x_index in range(dst_cortical_dim_x):
        print("dst_x_index", dst_x_index, src_neuron_block_index_x)
        # converting the destination cortical area x position to a binary number and taking away the nth position that
        # is the (src_neuron_block_index_x)th position

        position_length_binary = len(str(bin(dst_x_index))[2:])
        length_difference = cortical_length_binary - position_length_binary
        padding = "*" * length_difference

        processed_string = padding + str(bin(dst_x_index))[2:]

        if processed_string[src_neuron_block_index_x] == "1":
            voxel = [dst_x_index, dst_y_index, dst_z_index]
            candidate_list.append(voxel)
    return candidate_list


def syn_reducer_x(src_cortical_area, dst_cortical_area, src_neuron_id, dst_y_index=0, dst_z_index=0):
    """
    Acts in reverse of the expander rule. It reduces the combination of various blocks down to its building blocks
    representation through synaptic connections.
    """
    src_cortical_dim_x = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"][0]
    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0]

    # Note that the destination cortical area is expected to have at least 2 ^ (source block count) to be able to
    # address all the combinations
    if src_cortical_dim_x > 2 ** dst_cortical_dim_x:
        print("Warning: %s does not have enough blocks on x dim to support the needed synaptogenesis!"
              % dst_cortical_area)

    src_neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][0]

    # pad binary string with 0s if it's not long enough
    src_neuron_bin_str = str(bin(src_neuron_block_index_x))[2:]
    if len(src_neuron_bin_str) < dst_cortical_dim_x:
        src_neuron_bin_str = src_neuron_bin_str.rjust(dst_cortical_dim_x, '0')

    candidate_list = list()

    for dst_x_index in range(dst_cortical_dim_x):
        if int(src_neuron_bin_str[dst_x_index]):
            voxel = [dst_x_index, dst_y_index, dst_z_index]
            candidate_list.append(voxel)
    return candidate_list


def syn_randomizer(dst_cortical_area):
    """
    Identifies a random voxel from the destination cortical area.
    """

    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0]
    dst_cortical_dim_y = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][1]
    dst_cortical_dim_z = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][2]

    random_location = [randrange(0, dst_cortical_dim_x),
                       randrange(0, dst_cortical_dim_y),
                       randrange(0, dst_cortical_dim_z)]

    return random_location


def syn_lateral_pairs_x(neuron_id, cortical_area):
    """
    Identifies lateral pairs on x direction within the same cortical area

    0->1  2->3 ...
    0<-1  2<-3 ...
    """

    cortical_dim_x = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"][0]

    neuron_block_index_x = runtime_data.brain[cortical_area][neuron_id]['soma_location'][0]
    neuron_block_index_y = runtime_data.brain[cortical_area][neuron_id]['soma_location'][1]
    neuron_block_index_z = runtime_data.brain[cortical_area][neuron_id]['soma_location'][2]

    if neuron_block_index_x % 2 == 0:
        if neuron_block_index_x + 1 < cortical_dim_x:
            return [neuron_block_index_x + 1, neuron_block_index_y, neuron_block_index_z]
    else:
        if neuron_block_index_x - 1 >= 0:
            return [neuron_block_index_x - 1, neuron_block_index_y, neuron_block_index_z]


def syn_block_connection(src_cortical_area, dst_cortical_area, src_neuron_id, s=10):
    """
        voxel x to x+s from source connected to voxel x//s from destination on the axis x
    """
    src_cortical_dim_x = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"][0]
    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0]

    if src_cortical_dim_x != dst_cortical_dim_x * s:
        print("Warning: %s and %s do not have matching blocks on x dim to support the needed synaptogenesis!"
              % (src_cortical_area, dst_cortical_area))

    neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][0]
    neuron_block_index_y = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    neuron_block_index_z = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][2]

    return [neuron_block_index_x // s, neuron_block_index_y, neuron_block_index_z]
