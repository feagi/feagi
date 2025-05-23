
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


import json
from math import floor
from src.inf import runtime_data
import logging

from src.evo.cortical_area import cortical_area_type
from src.evo.templates import cortical_types


logger = logging.getLogger(__name__)

# todo: rename block to voxel


def voxel_list_to_neuron_list(cortical_area, voxel_list):
    neuron_list = list()

    for voxel in voxel_list:
        voxel_ref = voxel[0]
        neurons = neurons_in_the_block(cortical_area=cortical_area, block_ref=voxel_ref)
        for neuron in neurons:
            neuron_list.append([neuron, voxel[1]])

    return neuron_list


def block_size_checker(cortical_area, block):
    """
    Tests if the given block fits inside the cortical area block boundary
    """
    block_boundary = runtime_data.genome["blueprint"][cortical_area]["block_boundaries"]
    block_in_list = block

    for _ in range(3):
        if block_in_list[_] >= block_boundary[_]:
            return False
    return True


def block_trimmer(cortical_area, block):
    """
    limits the size of a block to stay within the block boundary of a cortical area
    """
    trimmed_block = (None, None, None)
    block_boundary = runtime_data.genome["blueprint"][cortical_area]["block_boundaries"]
    if isinstance(block, list):
        for _ in range(3):
            trimmed_block[_] = min(block_boundary[_] - 1, block[_])
        return trimmed_block
    else:
        block_in_list = block
        for _ in range(3):
            trimmed_block[_] = min(block_boundary[_] - 1, block_in_list[_])
        return trimmed_block


# def block_reference_builder(block):
#     return str(block[0]) + '-' + str(block[1]) + '-' + str(block[2])


def block_id_gen(cortical_area, coordinate):
    """
    Generating a block id so it can be used for faster neighbor detection
    """

    cortical_area_dim = []
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]

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
    if block_ref in runtime_data.voxel_dict[cortical_area]:
        return runtime_data.voxel_dict[cortical_area][block_ref]
    else:
        print(f"Warning! Voxel with reference {block_ref} does not exist in {cortical_area}.")
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
    # block_id = block_ref_2_id(block_ref)
    block_id = block_ref
    block_id[2] += int(offset)
    if block_id[2] < 0:
        block_id[2] = 0
    return block_id


# def block_ref_2_id(block_ref):
#     block_id_str = block_ref.split('-')
#     block_id = [int(x) for x in block_id_str]
#     return block_id


def neighboring_blocks(block_ref, kernel_size):
    """
    Returns the list of block ids who are neighbor of the given one
    Block_id is in form of [x,y,z]
    """
    block_id = block_ref

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
            neurons_in_the_block(cortical_area=cortical_area, block_ref=trimmed_block)
        for __ in neurons_in_block:
            candidate_list.append(__)
    return candidate_list


def all_block_refs(cortical_area):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]
    for x in range(block_boundaries[0]):
        for y in range(block_boundaries[1]):
            for z in range(block_boundaries[2]):
                block_ref_list.append((x, y, z))
    return block_ref_list


def x_block_refs(cortical_area, y_ref, z_ref):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]
    for x in range(block_boundaries[0]):
        block_ref_list.append((x, y_ref, z_ref))
    return block_ref_list


def y_block_refs(cortical_area, x_ref, z_ref):
    """
    Returns the list of all blocks in a given cortical area in a block_ref format
    """
    block_ref_list = list()
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]
    for y in range(block_boundaries[1]):
        block_ref_list.append((x_ref, y, z_ref))
    return block_ref_list


# def z_block_refs(cortical_area, x_ref, y_ref):
#     """
#     Returns the list of all blocks in a given cortical area in a block_ref format
#     """
#     block_ref_list = list()
#     block_boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]
#     for z in range(block_boundaries[2]):
#         block_ref_list.append((x_ref, y_ref, z))
#     return block_ref_list


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
        total_block_neurons = len(runtime_data.voxel_dict[cortical_area][block_ref])
        percent_active_neurons = active_block_neurons / total_block_neurons
        return percent_active_neurons


def cortical_activity_percentage_by_voxel(cortical_area):
    report = {}
    blocks_with_active_neurons = active_neurons_in_blocks(cortical_area=cortical_area)
    for block in blocks_with_active_neurons:
        report[block] = percent_active_neurons_in_block(block_ref=block, cortical_area=cortical_area)
    return report


def opu_percentage_report(cortical_area):
    report = cortical_activity_percentage_by_voxel(cortical_area=cortical_area)
    opu_data = {}
    for block in report:
        # block_index = block_ref_2_id(block)
        opu_data[block] = report[block]
    return opu_data


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
        neuron_block_ref = runtime_data.brain[cortical_area][neuron]['soma_location']

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


def voxel_reset(cortical_area):
    for voxel in runtime_data.voxel_dict[cortical_area]:
        runtime_data.voxel_dict[cortical_area][voxel] = set()


def subregion_voxels(src_cortical_area, region_definition):
    voxels = set()
    limits = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"]
    if limits[0] >= region_definition[1][0] > region_definition[0][0] and \
            limits[1] >= region_definition[1][1] > region_definition[0][1] and \
            limits[2] >= region_definition[1][2] > region_definition[0][2]:
        for x in range(region_definition[0][0], region_definition[1][0], 1):
            for y in range(region_definition[0][1], region_definition[1][1], 1):
                for z in range(region_definition[0][2], region_definition[1][2], 1):
                    voxels.add((x, y, z))
        return voxels


def subregion_neurons(src_cortical_area, region_definition):
    neurons = set()
    voxels = subregion_voxels(src_cortical_area=src_cortical_area,
                              region_definition=region_definition)
    try:
        for voxel in voxels:
            voxel_neurons = neurons_in_the_block(cortical_area=src_cortical_area,
                                                 block_ref=tuple(voxel))
            for neuron in voxel_neurons:
                neurons.add(neuron)
    except Exception as e:
        print("Exception while processing subregion neurons", e)
        pass
    return neurons


def generate_cortical_dimensions_by_id():
    """
    Generates the information needed to display cortical areas on Godot
    """
    cortical_information = {}

    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_information[cortical_area] = {}
        genes = runtime_data.genome["blueprint"][cortical_area]

        if "visualization" in genes:
            cortical_visibility = genes["visualization"]
            if not genes["visualization"]:
                cortical_visibility = False
        else:
            cortical_visibility = True

        cortical_information[cortical_area]["cortical_name"] = genes["cortical_name"]
        cortical_information[cortical_area]["cortical_group"] = genes["group_id"]
        cortical_information[cortical_area]["cortical_sub_group"] = genes["sub_group_id"]
        cortical_information[cortical_area]["visible"] = cortical_visibility

        cortical_information[cortical_area]["coordinates_2d"] = [
            genes["2d_coordinate"][0],
            genes["2d_coordinate"][1]
        ]

        cortical_information[cortical_area]["coordinates_3d"] = [
            genes["relative_coordinate"][0],
            genes["relative_coordinate"][1],
            genes["relative_coordinate"][2]
        ]

        dim_x = genes["block_boundaries"][0]
        dim_y = genes["block_boundaries"][1]
        dim_z = genes["block_boundaries"][2]

        cortical_information[cortical_area]["cortical_dimensions"] = [
            dim_x,
            dim_y,
            dim_z
        ]

        cortical_type = cortical_area_type(cortical_area=cortical_area)
        if cortical_type in ["IPU", "OPU"]:
            if "dev_count" not in runtime_data.genome["blueprint"][cortical_area]:
                runtime_data.genome["blueprint"][cortical_area]["dev_count"] = \
                    int(runtime_data.genome["blueprint"][cortical_area]["block_boundaries"][0] /
                        cortical_types[cortical_type]["supported_devices"][cortical_area]["resolution"][0])

            dev_count = runtime_data.genome["blueprint"][cortical_area]["dev_count"]

            cortical_information[cortical_area]["dev_count"] = dev_count

            cortical_information[cortical_area]["cortical_dimensions_per_device"] = [
                int(dim_x / dev_count),
                dim_y,
                dim_z
            ]

    with open(runtime_data.connectome_path+"cortical_data_by_id.json", "w") as data_file:
        data_file.seek(0)
        data_file.write(json.dumps(cortical_information, indent=3))
        data_file.truncate()

    return cortical_information


def generate_cortical_dimensions():
    """
    Generates the information needed to display cortical areas on Godot
    """
    cortical_information = {}

    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_name = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
        cortical_information[cortical_name] = []
        genes = runtime_data.genome["blueprint"][cortical_area]
        cortical_information[cortical_name].append(genes["relative_coordinate"][0])
        cortical_information[cortical_name].append(genes["relative_coordinate"][1])
        cortical_information[cortical_name].append(genes["relative_coordinate"][2])
        cortical_information[cortical_name].append(genes["visualization"])
        cortical_information[cortical_name].append(genes["block_boundaries"][0])
        cortical_information[cortical_name].append(genes["block_boundaries"][1])
        cortical_information[cortical_name].append(genes["block_boundaries"][2])
        cortical_information[cortical_name].append(cortical_area)

    with open(runtime_data.connectome_path+"cortical_data.json", "w") as data_file:
        data_file.seek(0)
        data_file.write(json.dumps(cortical_information, indent=3))
        data_file.truncate()

    return cortical_information
