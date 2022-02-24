
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

from math import floor
from inf import runtime_data
import traceback


# todo: rename block to voxel


def block_size_checker(cortical_area, block):
    """
    Tests if the given block fits inside the cortical area block boundary
    """
    block_boundary = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"]
    block_in_list = block_ref_2_id(block)

    for _ in range(3):
        if block_in_list[_] >= block_boundary[_]:
            return False
    return True


def block_trimmer(cortical_area, block):
    """
    limits the size of a block to stay within the block boundary of a cortical area
    """
    trimmed_block = [None, None, None]
    block_boundary = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"]
    if isinstance(block, list):
        for _ in range(3):
            trimmed_block[_] = min(block_boundary[_] - 1, block[_])
        return trimmed_block
    else:
        block_in_list = block_ref_2_id(block)
        for _ in range(3):
            trimmed_block[_] = min(block_boundary[_] - 1, block_in_list[_])
        return block_reference_builder(trimmed_block)


def block_reference_builder(block):
    return str(block[0]) + '-' + str(block[1]) + '-' + str(block[2])


def block_id_gen(cortical_area, coordinate):
    """
    Generating a block id so it can be used for faster neighbor detection
    """

    cortical_area_dim = []
    geometric_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']
    for axis in geometric_boundaries:
        cortical_area_dim.append(geometric_boundaries[axis][1] - geometric_boundaries[axis][0])
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']

    block_id = []
    index = 0
    for location in coordinate:
        if block_boundaries[index] != 0:
            block_number = int(location // (cortical_area_dim[index] / block_boundaries[index]))

        # block_number = floor(
        #     location / ((cortical_area_dim[index] / (block_boundaries[index] + 0.00001)) + 0.00001))

        block_id.append(block_number)
        index += 1
    if block_id[0] > 500:
        print("large block detected")
    return block_id


def neurons_in_the_block(cortical_area, block_ref):
    """
    Generates a list of Neurons in the given block
    block_id to be entered as [x,y,z]
    """
    try:
        return runtime_data.block_dic[cortical_area][block_ref]
    except Exception as e:
        print("Warning! No neuron was detected for ", cortical_area, block_ref)
        print("..... possibly too sparse of a cortical area.\n")
        return []


def block_z_offset(block_ref, offset):
    """
    Offsets the z coordinate of the block reference by the value defined by "offset"
    todo: There is a risk that the new offset value exceed defined block boundaries of a given cortical area

    Args:
        block_ref:
        offset:

    Returns: Adjusted block reference

    """
    block_id = block_ref_2_id(block_ref)
    block_id[2] += int(offset)
    if block_id[2] < 0:
        block_id[2] = 0
    return block_reference_builder(block_id)


def block_ref_2_id(block_ref):
    block_id_str = block_ref.split('-')
    block_id = [int(x) for x in block_id_str]
    return block_id


def neighboring_blocks(block_ref, kernel_size):
    """
    Returns the list of block ids who are neighbor of the given one
    Block_id is in form of [x,y,z]
    """
    block_id = block_ref_2_id(block_ref)

    block_id_list = list()

    kernel_half = floor(kernel_size / 2)
    seed_id = [block_id[0] - kernel_half, block_id[1] - kernel_half, block_id[2] - kernel_half]

    for i in range(0, kernel_size):
        for ii in range(0, kernel_size):
            for iii in range(0, kernel_size):
                neighbor_block_id = [seed_id[0] + i, seed_id[1] + ii, seed_id[2] + iii]
                if neighbor_block_id != block_id and \
                        neighbor_block_id[0] > 0 and \
                        neighbor_block_id[1] > 0 and \
                        neighbor_block_id[2] > 0:
                    block_id_list.append(neighbor_block_id)

    return block_id_list


def neurons_in_block_neighborhood(cortical_area, block_ref, kernel_size=3):
    """
    Provides the list of all neurons within the surrounding blocks given the kernel size with default being 3
    """
    candidate_list = list()
    block_list = neighboring_blocks(block_ref, kernel_size)
    # print("Block List: ", block_list)
    for _ in block_list:
        trimmed_block = block_trimmer(cortical_area, _)
        neurons_in_block = \
            neurons_in_the_block(cortical_area=cortical_area, block_ref=block_reference_builder(trimmed_block))
        for __ in neurons_in_block:
            candidate_list.append(__)
    return candidate_list


def all_block_refs(cortical_area):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    for x in range(block_boundaries[0]):
        for y in range(block_boundaries[1]):
            for z in range(block_boundaries[2]):
                block_ref_list.append(block_reference_builder([x, y, z]))
    return block_ref_list


def x_block_refs(cortical_area, y_ref, z_ref):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    for x in range(block_boundaries[0]):
        block_ref_list.append(block_reference_builder([x, y_ref, z_ref]))
    return block_ref_list


def y_block_refs(cortical_area, x_ref, z_ref):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    for y in range(block_boundaries[1]):
        block_ref_list.append(block_reference_builder([x_ref, y, z_ref]))
    return block_ref_list


def z_block_refs(cortical_area, x_ref, y_ref):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    for z in range(block_boundaries[2]):
        block_ref_list.append(block_reference_builder([x_ref, y_ref, z]))
    return block_ref_list


def percent_active_neurons_in_block(block_ref, cortical_area, current_fcl=True):
    """
    Returns a rounded, integer percentage of active (i.e. present in FCL at execution)
    neurons for a block in a cortical area

    Note: If the current_fcl flag is not True then the function returns the results against the previous FCL list
    """
    blocks_with_active_neurons = active_neurons_in_blocks(cortical_area, include_neurons=True)
    if block_ref not in blocks_with_active_neurons:
        return 0
    else:
        active_block_neurons = len(blocks_with_active_neurons[block_ref])
        total_block_neurons = len(runtime_data.block_dic[cortical_area][block_ref])
        percent_active_neurons = round(active_block_neurons / total_block_neurons * 100)
        return percent_active_neurons


def active_neurons_in_blocks(cortical_area, current_fcl=True, include_neurons=False):
    """
    Returns a dict of block_refs and their corresponding active (i.e. currently present
    in FCL) neurons for a given cortical area

    ex: {'1-0-1': [active_neuron_id1, active_neuron_id2, ...], '1-0-0': [ ... ]}

    Note: If the current_fcl flag is not True then the function returns the results against the previous FCL list
    """

    neuron_count = 1
    if current_fcl:
        fcl = runtime_data.fire_candidate_list
    else:
        fcl = runtime_data.previous_fcl

    blocks_with_active_neurons = {}
    for neuron in fcl[cortical_area]:
        neuron_block_ref = block_reference_builder(
            runtime_data.brain[cortical_area][neuron]['soma_location']
        )
        if include_neurons:
            if neuron_block_ref in blocks_with_active_neurons:
                blocks_with_active_neurons[neuron_block_ref].append(neuron)
            else:
                blocks_with_active_neurons[neuron_block_ref] = [neuron]
        else:
            if neuron_block_ref not in blocks_with_active_neurons:
                blocks_with_active_neurons[neuron_block_ref] = 1
            else:
                neuron_count += 1
                blocks_with_active_neurons[neuron_block_ref] = neuron_count
    return blocks_with_active_neurons
