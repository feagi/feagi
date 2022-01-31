
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


from evo import blocks
from inf import runtime_data


def rule_neuron_to_neuron(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    candidate_list = list()
    # Input: neuron id of which we desire to find all candidate neurons for from another cortical region
    src_data = runtime_data.brain[src_cortical_area]
    dst_data = runtime_data.brain[dst_cortical_area]
    for dst_neuron_id in dst_data:
        if src_data[src_neuron_id]['soma_location'][0] == dst_data[dst_neuron_id]['soma_location'][0]:
            candidate_list.append(dst_neuron_id)
            break
    return candidate_list


def rule_block_to_block(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Returns the list of neurons listed under the same block id as the source neuron
    todo: need to add ability to account for the sublayer mapping e.g. v1.1 > v2 layer 1 based on z offset
    todo: instead of a single destination block based on the rule pass a set of blocks to make it generalizable
    Returns:

    """
    candidate_list = list()
    src_neuron_block_ref = \
        blocks.block_reference_builder(runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1])

    src_neuron_block_ref = blocks.block_z_offset(block_ref=src_neuron_block_ref, offset=z_offset)
    src_neuron_block_ref = blocks.block_trimmer(dst_cortical_area, src_neuron_block_ref)

    if rule_param == 1:
        try:
            for neuron in runtime_data.block_dic[dst_cortical_area][src_neuron_block_ref]:
                candidate_list.append(neuron)
        except KeyError:
            pass
    elif rule_param in [3, 5, 7, 9]:
        candidate_list = blocks.neurons_in_block_neighborhood(cortical_area=dst_cortical_area,
                                                              block_ref=src_neuron_block_ref, kernel_size=rule_param)

    else:
        print(rule_param, "is an invalid parameter for block to block mapping")

    return candidate_list


def rule_block_one_to_all(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Uses all of the cortical blocks in the destination cortical area for synaptogenesis
    """
    candidate_list = list()

    # Get the list of all blocks from the destination cortical are
    block_ref_list = blocks.all_block_refs(dst_cortical_area)
    for block_ref in block_ref_list:
        block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref)
        for neuron in block_neurons:
            candidate_list.append(neuron)
    return candidate_list


def rule_block_distributor(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    This rule helps to take a set of unique inputs from one cortical area and develop synaptic projections that can
    lead to a comprehensive set of unique connections that covers all the combinations of the input values.

    Note: This function is designed for the corner case of the destination cortical area being 1 dimensional in z
    direction
    """

    # todo: generalize this function so it takes the direction of the source and destination cortical areas as input
    candidate_list = list()
    block_list = blocks.z_block_refs(cortical_area=dst_cortical_area, x_ref=0, y_ref=0)
    source_x_depth = runtime_data.genome['blueprint'][src_cortical_area]['neuron_params']['block_boundaries'][0]

    for offset in range(source_x_depth):
        for block_ref in block_list:
            if blocks.block_ref_2_id(block_ref)[2] // (2 ** offset) % 2 == 0:
                for neuron in blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref):
                    candidate_list.append(neuron)
    return candidate_list


def decrease_filter_diagonal(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Creates diagonal (upward, left to right) synapses between blocks within a cortical area
    (i.e. intracortically), where appropriate.

    ex: A neuron in block 0-0-0 of the source cortical area will have synapse candidates
    in block 0-1-1 of the same cortical area. If a block corresponding to source block x-y-z
    (i.e. x-(y+1)-(z+1)) does not exist in the cortical area, no synapses are created.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    candidate_list = list()
    dst_block_id = [
        src_neuron_block_id[0],
        min(src_neuron_block_id[1] + 1,
            runtime_data.genome['blueprint'][dst_cortical_area]["neuron_params"]["block_boundaries"][1] - 1),
        min(src_neuron_block_id[2] + 1,
            runtime_data.genome['blueprint'][dst_cortical_area]["neuron_params"]["block_boundaries"][2] - 1)
    ]
    print("%%% %%% %%%", src_neuron_block_id[1] + 1,
            runtime_data.genome['blueprint'][dst_cortical_area]["neuron_params"]["block_boundaries"][1] - 1)
    dst_block_ref = blocks.block_reference_builder(dst_block_id)
    dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=dst_block_ref)
    for neuron in dst_block_neurons:
        candidate_list.append(neuron)

    return candidate_list


def increase_filter_diagonal(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Creates diagonal (upward, right to left) synapses between blocks within a cortical area
    (i.e. intracortically), where appropriate.

    ex: A neuron in block 0-1-2 of the source cortical area will have synapse candidates
    in block 0-2-1 of the same cortical area. If a block corresponding to source block x-y-z
    (i.e. x-(y+1)-(z-1)) does not exist in the cortical area, no synapses are created.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    candidate_list = list()
    dst_block_id = [
        src_neuron_block_id[0],
        min(src_neuron_block_id[1] + 1,
            runtime_data.genome['blueprint'][dst_cortical_area]["neuron_params"]["block_boundaries"][1] - 1),
        max(src_neuron_block_id[1] - 1,
            runtime_data.genome['blueprint'][dst_cortical_area]["neuron_params"]["block_boundaries"][0])
    ]
    dst_block_ref = blocks.block_reference_builder(dst_block_id)
    dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=dst_block_ref)
    for neuron in dst_block_neurons:
        candidate_list.append(neuron)

    return candidate_list


def decrease_z_subregion(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Isolates all blocks in the Y plane of the source cortical area's Z axis maximum and
    creates synapse candidates in a destination cortical area.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    SRC_Z_MAX = runtime_data.genome['blueprint'][src_cortical_area]['neuron_params']['block_boundaries'][2]

    candidate_list = list()
    if src_neuron_block_id[2] == (SRC_Z_MAX - 1):
        dst_blocks = runtime_data.block_dic[dst_cortical_area]
        for block in dst_blocks:
            for dst_neuron in dst_blocks[block]:
                candidate_list.append(dst_neuron)

    return candidate_list


def increase_z_subregion(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Isolates all blocks in the Y plane of the source cortical area's Z axis minimum and
    creates synapse candidates in a destination cortical area.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    candidate_list = list()
    if src_neuron_block_id[2] == 0:
        dst_blocks = runtime_data.block_dic[dst_cortical_area]
        for block in dst_blocks:
            for dst_neuron in dst_blocks[block]:
                candidate_list.append(dst_neuron)

    return candidate_list


def expander_x(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset, dst_y_index=0, dst_z_index=0):
    """
    This rule represents a unique combination of all blocks from the source cortical area on the destination side
    in x dim.
    """
    src_cortical_dim_x = \
        len(runtime_data.genome['blueprint'][src_cortical_area]['neuron_params']['block_boundaries'][0])
    dst_cortical_dim_x = \
        len(runtime_data.genome['blueprint'][dst_cortical_area]['neuron_params']['block_boundaries'][0])

    # Note that the destination cortical area is expected to have at least 2 ^ (source block count) to be able to
    # address all the combinations
    if dst_cortical_dim_x < 2 ** src_cortical_dim_x:
        print("Warning: %s does not have enough blocks on x dim to support the needed synaptogenesis!"
              % dst_cortical_area)

    src_neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1][0]

    candidate_list = list()

    for dst_x_index in range(dst_cortical_dim_x):
        if str(bin(dst_x_index))[2:][src_neuron_block_index_x]:
            block_ref = blocks.block_reference_builder([dst_x_index, dst_y_index, dst_z_index])
            for dst_neuron in blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref):
                candidate_list.append(dst_neuron)
    return candidate_list


def reducer_x(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset, dst_y_index=0, dst_z_index=0):
    """
    Acts in reverse of the expander rule. It reduces the combination of various blocks down to its building blocks
    representation through synaptic connections.
    """
    src_cortical_dim_x = runtime_data.genome['blueprint'][src_cortical_area]['neuron_params']['block_boundaries'][0]
    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]['neuron_params']['block_boundaries'][0]

    # Note that the destination cortical area is expected to have at least 2 ^ (source block count) to be able to
    # address all the combinations
    if src_cortical_dim_x > 2 ** dst_cortical_dim_x:
        print("Warning: %s does not have enough blocks on x dim to support the needed synaptogenesis!"
              % dst_cortical_area)

    src_neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1][0]

    # pad binary string with 0s if it's not long enough
    src_neuron_bin_str = str(bin(src_neuron_block_index_x))[2:]
    if len(src_neuron_bin_str) < dst_cortical_dim_x:
        src_neuron_bin_str = src_neuron_bin_str.rjust(dst_cortical_dim_x, '0')

    candidate_list = list()

    for dst_x_index in range(dst_cortical_dim_x):
        if int(src_neuron_bin_str[dst_x_index]):
            block_ref = blocks.block_reference_builder([dst_x_index, dst_y_index, dst_z_index])
            for dst_neuron in blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref):
                candidate_list.append(dst_neuron)
    return candidate_list


# TODO: refactor to combine this with increase/decrease_filter_diagonal to generalize
# TODO: use rule_params to specify the type of adjacent synapsing desired...
def intracortical_adjacent(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Creates synapses between adjacent (x-axis) blocks.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    dst_block_id = [
        src_neuron_block_id[0] + 1,
        src_neuron_block_id[1],
        src_neuron_block_id[2]
    ]
    dst_block_ref = blocks.block_reference_builder(dst_block_id)
    dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=dst_block_ref)

    candidate_list = list()
    for neuron in dst_block_neurons:
        candidate_list.append(neuron)

    return candidate_list


def to_select_block(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset, dst_block_id=[0, 0, 0]):
    """
    Creates synapses to neurons in a specific block in dst_cortical_area. Default is the first (i.e. min x,y,z-axis)
    block.
    """
    dst_block_ref = blocks.block_reference_builder(dst_block_id)
    dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=dst_block_ref)

    candidate_list = list()
    for neuron in dst_block_neurons:
        candidate_list.append(neuron)

    return candidate_list


def from_last_block_only(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    Creates synapses to neurons in blocks in dst_cortical_area from src_cortical_area's last
    (i.e. max x-axis value) block.
    """
    SRC_X_MAX = runtime_data.genome['blueprint'][src_cortical_area]['neuron_params']['block_boundaries'][0]

    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    candidate_list = list()
    if src_neuron_block_id[0] == (SRC_X_MAX - 1):
        dst_blocks = runtime_data.block_dic[dst_cortical_area]
        for block in dst_blocks:
            for dst_neuron in dst_blocks[block]:
                candidate_list.append(dst_neuron)

    return candidate_list


def many_to_one(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset, bin_size=2):
    """
    Creates synapses between multiple (defined by bin_size) sequential (x-axis) blocks in src_cortical_area
    with neurons from a single block in dst_cortical_area.
    """
    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    dst_block_id = [
        src_neuron_block_id[0] // bin_size,
        src_neuron_block_id[1],
        src_neuron_block_id[2]
    ]
    dst_block_ref = blocks.block_reference_builder(dst_block_id)
    dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=dst_block_ref)

    candidate_list = list()
    for neuron in dst_block_neurons:
        candidate_list.append(neuron)

    return candidate_list
