
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


import logging
import traceback
from src.evo import voxels
from src.inf import runtime_data
from random import randrange
from math import prod


logger = logging.getLogger(__name__)


def define_subregions(cortical_area, parameters):
    subregions = set()
    boundaries = runtime_data.genome['blueprint'][cortical_area]["block_boundaries"]
    if "src_seed" in parameters and "src_pattern" in parameters:
        seed = parameters["src_seed"]
        # pattern format expected as [[c, s], [c, s], [c, s]] where c indicates choose and s as skip
        pattern = parameters["src_pattern"]
    
        seed_pointer = [0, 0, 0]
    
        while seed_pointer[0] <= boundaries[0]:
            for x_i in range(pattern[0][0]):
                while seed_pointer[1] <= boundaries[1]:
                    for y_i in range(pattern[1][0]):
                        while seed_pointer[2] <= boundaries[2]:
                            # Chosen regions
                            for z_i in range(pattern[2][0]):
                                if seed_pointer[0] + seed[0] <= boundaries[0] and \
                                        seed_pointer[1] + seed[1] <= boundaries[1] and \
                                        seed_pointer[2] + seed[2] <= boundaries[2]:
                                    subregions.add((tuple(seed_pointer),
                                                    (seed_pointer[0] + seed[0],
                                                     seed_pointer[1] + seed[1],
                                                     seed_pointer[2] + seed[2])))
                                seed_pointer[2] += seed[2]
                            # Skip regions
                            for z_j in range(pattern[2][1]):
                                seed_pointer[2] += seed[2]
                        seed_pointer[1] += seed[1]
                        seed_pointer[2] = 0
    
                    for y_j in range(pattern[1][1]):
                        seed_pointer[1] += seed[1]
                seed_pointer[0] += seed[0]
                seed_pointer[1] = 0
                seed_pointer[2] = 0
    
            for x_j in range(pattern[0][1]):
                seed_pointer[0] += seed[0]
    return subregions


def neighbor_finder(cortical_area_src, cortical_area_dst, src_neuron_id, morphology_, src_subregion,
                    morphology_id_overwrite=None):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    # Candidate_voxel_list includes a list of destination neuron and associated postSynapticCurrent pairs
    candidate_voxel_list = list()
    raw_candidate_list = set()

    # rule_manager = SynaptogenesisRuleManager(src_neuron_id=src_neuron_id, src_cortical_area=cortical_area_src,
    #                                          dst_cortical_area=cortical_area_dst)
    # candidate_list = rule_manager.growth_rule_selector()

    src_voxel = runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location']
    if morphology_id_overwrite:
        neuron_morphology = morphology_id_overwrite
    else:
        neuron_morphology = morphology_['morphology_id']
    morphology_scalar = morphology_['morphology_scalar']
    psc_multiplier = morphology_['postSynapticCurrent_multiplier']
    psc_base = runtime_data.genome["blueprint"][cortical_area_src]['postsynaptic_current']
    post_synaptic_current = psc_multiplier * psc_base

    src_block_boundaries = runtime_data.genome["blueprint"][cortical_area_src]["block_boundaries"]

    try:
        if runtime_data.genome["neuron_morphologies"][neuron_morphology]["type"] == "vectors":
            for vector in runtime_data.genome["neuron_morphologies"][neuron_morphology]["parameters"]["vectors"]:
                candidate_list = match_vectors(src_voxel=src_voxel, cortical_area_dst=cortical_area_dst,
                                               vector=vector, morphology_scalar=morphology_scalar
                                               , src_subregion=src_subregion)
                if candidate_list:
                    for candidate in candidate_list:
                        raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                    # candidate_voxel_list.append([matching_vectors, post_synaptic_current])
                candidate_list = None

        elif runtime_data.genome["neuron_morphologies"][neuron_morphology]["type"] == "patterns":
            for pattern in runtime_data.genome["neuron_morphologies"][neuron_morphology]["parameters"]["patterns"]:

                dst_block_boundaries = runtime_data.genome["blueprint"][cortical_area_dst]["block_boundaries"]

                source_pattern = pattern[0]
                destination_pattern = pattern[1]
                candidate_list = []
                # for src_candidate in src_generator:
                dst_generator = find_destination_coordinates(src_pattern=source_pattern,
                                                             dst_pattern=destination_pattern,
                                                             dst_cortical_boundary=dst_block_boundaries,
                                                             src_coordinate=src_voxel)
                for dst_candidate in dst_generator:
                    candidate_list.append(dst_candidate)

                if candidate_list:
                    for candidate in candidate_list:
                        raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                        # candidate_voxel_list.append([item, post_synaptic_current])
                candidate_list = None
        elif runtime_data.genome["neuron_morphologies"][neuron_morphology]["type"] == "functions":
            if neuron_morphology == "expander_x":
                candidate_list = syn_expander_x(cortical_area_src, cortical_area_dst,
                                                src_neuron_id, src_subregion=src_subregion)
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                    # candidate_voxel_list.append([candidate, post_synaptic_current])
            elif neuron_morphology == "reducer_x":
                candidate_list = syn_reducer_x(cortical_area_src, cortical_area_dst,
                                               src_neuron_id, src_subregion=src_subregion)
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                    # candidate_voxel_list.append([candidate, post_synaptic_current])
            elif neuron_morphology == "randomizer":
                candidate = syn_randomizer(dst_cortical_area=cortical_area_dst)
                raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                # candidate_voxel_list.append([candidate, post_synaptic_current])
            elif neuron_morphology == "lateral_pairs_x":
                candidate = syn_lateral_pairs_x(neuron_id=src_neuron_id, cortical_area=cortical_area_src,
                                                src_subregion=src_subregion)
                raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                # candidate_voxel_list.append([candidate, post_synaptic_current])
            elif neuron_morphology == "block_connection":
                candidate = syn_block_connection(cortical_area_src, cortical_area_dst, src_neuron_id,
                                                 s=10, src_subregion=src_subregion)
                raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))
                # candidate_voxel_list.append([candidate, post_synaptic_current])

            elif neuron_morphology == "last_to_first":
                src_cortical_dim = runtime_data.genome['blueprint'][cortical_area_src]["block_boundaries"]
                if src_voxel == [src_cortical_dim[0] - 1, src_cortical_dim[1] - 1, src_cortical_dim[2] - 1]:
                    raw_candidate_list.add((0, 0, 0))

            elif neuron_morphology == "projector":
                candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                               src_neuron_id, src_subregion=src_subregion)
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "projector_xy":
                candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                               src_neuron_id, src_subregion=src_subregion, transpose=("y", "x", "z"))
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "projector_xz":
                candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                               src_neuron_id, src_subregion=src_subregion, transpose=("z", "y", "x"))
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "projector_yz":
                candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                               src_neuron_id, src_subregion=src_subregion, transpose=("x", "z", "y"))
                for candidate in candidate_list:
                    raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "project_from_end_x":
                if src_voxel[0] == src_block_boundaries[0] - 1:
                    candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                                   src_neuron_id, src_subregion=src_subregion,
                                                   project_last_layer_of="x")
                    for candidate in candidate_list:
                        raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "project_from_end_y":
                if src_voxel[1] == src_block_boundaries[1] - 1:
                    candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                                   src_neuron_id, src_subregion=src_subregion,
                                                   project_last_layer_of="y")
                    for candidate in candidate_list:
                        raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "project_from_end_z":
                if src_voxel[2] == src_block_boundaries[2] - 1:
                    candidate_list = syn_projector(cortical_area_src, cortical_area_dst,
                                                   src_neuron_id, src_subregion=src_subregion,
                                                   project_last_layer_of="z")
                    for candidate in candidate_list:
                        raw_candidate_list.add((candidate[0], candidate[1], candidate[2]))

            elif neuron_morphology == "memory":
                syn_memory(src_cortical_area=cortical_area_src, dst_cortical_area=cortical_area_dst)

            candidate_list = None

        elif runtime_data.genome["neuron_morphologies"][neuron_morphology]["type"] == "placeholder":
            pass

        else:
            print("Warning! Morphology %s did not have any valid definition." % neuron_morphology)

    except Exception as e:
        print("Error during synaptogenesis of %s and %s" % (cortical_area_src, cortical_area_dst))
        print(traceback.format_exc())

    for candidate in raw_candidate_list:
        candidate_voxel_list.append([list(candidate), post_synaptic_current])

    if candidate_voxel_list:
        candidate_neuron_list = \
            voxels.voxel_list_to_neuron_list(cortical_area=cortical_area_dst,
                                             voxel_list=candidate_voxel_list)
        return candidate_neuron_list


# todo: use the following function
def validate_pattern(pattern):
    valid_patterns = {"*", "?", "!"}
    for p in pattern:
        if isinstance(p, list) and len(p) == 3:
            for axis_pattern in p:
                if axis_pattern not in valid_patterns:
                    try:
                        int(axis_pattern)
                    except ValueError:
                        raise ValueError(f"Invalid pattern value: {axis_pattern}")
        else:
            raise ValueError("Pattern must be a list of three elements per axis.")


def match_vectors(src_voxel, cortical_area_dst, vector, morphology_scalar, src_subregion):
    scaled_vector = [prod(x) for x in zip(vector, morphology_scalar)]
    candidate_vector = [sum(x) for x in zip(src_voxel, scaled_vector)]
    for item in candidate_vector:
        if item < 0:
            return None
    within_limits = voxels.block_size_checker(cortical_area=cortical_area_dst,
                                              block=voxels.block_reference_builder(candidate_vector))
    if within_limits:
        return [candidate_vector]


def check_pattern_validity(pattern):
    """
    Checks the validity of a pattern
    """
    valid_patterns = {"*", "?", "!"}
    for element in pattern:
        if element not in valid_patterns:
            try:
                value = int(element)
                if not (0 <= value):
                    return False
            except ValueError:
                return False
    return True


def find_source_coordinates(src_pattern, src_cortical_boundary):
    """
    Generator that yields coordinates within the cortical boundary that match the given pattern.

    :param src_pattern: A tuple (x, y, z) where each element can be an integer or "*".
                        "*" matches all positions along that axis.
    :param src_cortical_boundary: A list [x, y, z] defining the size of the cortical cube.
    :yield: Coordinates (as tuples) that match the pattern within the cortical boundaries.
    """
    # Generate ranges based on pattern and boundary
    x_range = range(src_cortical_boundary[0]) if src_pattern[0] == "*" else [src_pattern[0]]
    y_range = range(src_cortical_boundary[1]) if src_pattern[1] == "*" else [src_pattern[1]]
    z_range = range(src_cortical_boundary[2]) if src_pattern[2] == "*" else [src_pattern[2]]

    # Use a generator expression to yield each matching coordinate
    for x in x_range:
        for y in y_range:
            for z in z_range:
                yield [x, y, z]


def find_destination_coordinates(dst_cortical_boundary, src_coordinate, src_pattern, dst_pattern):
    """
    Generator that yields destination coordinates within the cortical boundary
    that match the given pattern based on the source coordinate.

    :param dst_cortical_boundary: A list [x, y, z] defining the size of the destination cube.
    :param src_coordinate: A list [int, int, int] representing the coordinate on the source object.
    :param dst_pattern: A tuple (x, y, z) where each element can be "*", "?", "!", or an int.
                        "*" matches any position along that axis.
                        "?" matches the corresponding source coordinate.
                        "!" matches any coordinate except the corresponding source coordinate.
                        int specifies an exact match on that axis.
    :yield: Destination coordinates (as tuples) that match the pattern within the destination boundaries.
    """
    # Generate ranges based on dst_pattern, dst_cortical_boundary, and src_coordinate
    x_range = (
        range(dst_cortical_boundary[0]) if dst_pattern[0] == "*"
        else [src_coordinate[0]] if (dst_pattern[0] == "?" and src_coordinate[0] < dst_cortical_boundary[0] and
                                     (src_coordinate[0] == src_pattern[0] or src_pattern[0] in ["*", "?"]))
        else [i for i in range(dst_cortical_boundary[0]) if i != src_coordinate[0]] if dst_pattern[0] == "!"
        else [dst_pattern[0]] if (isinstance(dst_pattern[0], int) and
                                  (src_pattern[0] == src_coordinate[0] or src_pattern[0] == "*" or
                                   (src_pattern[0] == "?" and dst_pattern[0] == src_coordinate[0]))) else []
    )

    y_range = (
        range(dst_cortical_boundary[1]) if dst_pattern[1] == "*"
        else [src_coordinate[1]] if (dst_pattern[1] == "?" and src_coordinate[1] < dst_cortical_boundary[1] and
                                     (src_coordinate[1] == src_pattern[1] or src_pattern[1] in ["*", "?"]))
        else [i for i in range(dst_cortical_boundary[1]) if i != src_coordinate[1]] if dst_pattern[1] == "!"
        else [dst_pattern[1]] if (isinstance(dst_pattern[1], int) and
                                  (src_pattern[1] == src_coordinate[1] or src_pattern[1] == "*" or
                                   (src_pattern[1] == "?" and dst_pattern[1] == src_coordinate[1]))) else []
    )

    z_range = (
        range(dst_cortical_boundary[2]) if dst_pattern[2] == "*"
        else [src_coordinate[2]] if (dst_pattern[2] == "?" and src_coordinate[2] < dst_cortical_boundary[2] and
                                     (src_coordinate[2] == src_pattern[2] or src_pattern[2] in ["*", "?"]))
        else [i for i in range(dst_cortical_boundary[2]) if i != src_coordinate[2]] if dst_pattern[2] == "!"
        else [dst_pattern[2]] if (isinstance(dst_pattern[2], int) and
                                  (src_pattern[2] == src_coordinate[2] or src_pattern[2] == "*" or
                                   (src_pattern[2] == "?" and dst_pattern[2] == src_coordinate[2]))) else []
    )

    # Use a generator expression to yield each matching destination coordinate
    for x in x_range:
        for y in y_range:
            for z in z_range:
                yield [x, y, z]


def syn_expander_x(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, dst_y_index=0, dst_z_index=0):
    """
    This rule represents a unique combination of all blocks from the source cortical area on the destination side
    in x dim.
    """
    src_cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]
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


def syn_reducer_x(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, dst_y_index=0, dst_z_index=0):
    """
    Acts in reverse of the expander rule. It reduces the combination of various blocks down to its building blocks
    representation through synaptic connections.
    """
    src_cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]
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


def syn_lateral_pairs_x(neuron_id, cortical_area, src_subregion):
    """
    Identifies lateral pairs on x direction within the same cortical area

    0->1  2->3 ...
    0<-1  2<-3 ...
    """

    cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]

    neuron_block_index_x = runtime_data.brain[cortical_area][neuron_id]['soma_location'][0]
    neuron_block_index_y = runtime_data.brain[cortical_area][neuron_id]['soma_location'][1]
    neuron_block_index_z = runtime_data.brain[cortical_area][neuron_id]['soma_location'][2]

    if neuron_block_index_x % 2 == 0:
        if neuron_block_index_x + 1 < cortical_dim_x:
            return [neuron_block_index_x + 1, neuron_block_index_y, neuron_block_index_z]
    else:
        if neuron_block_index_x - 1 >= 0:
            return [neuron_block_index_x - 1, neuron_block_index_y, neuron_block_index_z]


def syn_block_connection(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion, s=10):
    """
        voxel x to x+s from source connected to voxel x//s from destination on the axis x
    """
    src_cortical_dim_x = src_subregion[1][0] - src_subregion[0][0]
    dst_cortical_dim_x = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0]

    if src_cortical_dim_x != dst_cortical_dim_x * s:
        print("Warning: %s and %s do not have matching blocks on x dim to support the needed synaptogenesis!"
              % (src_cortical_area, dst_cortical_area))

    neuron_block_index_x = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][0]
    neuron_block_index_y = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    neuron_block_index_z = runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][2]

    return [neuron_block_index_x // s, neuron_block_index_y, neuron_block_index_z]


def syn_projector(src_cortical_area, dst_cortical_area, src_neuron_id, src_subregion,
                  transpose=None, project_last_layer_of=None):

    src_dimensions = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"]
    src_shape = [0, 0, 0]
    dst_shape = [0, 0, 0]
    candidate_list = list()

    # Apply transpose to src_subregion and neuron_location
    if transpose:
        axis_map = {"x": 0, "y": 1, "z": 2}  # Mapping of axes to indices
        transpose_indices = [axis_map[axis] for axis in transpose]  # Get new order indices

        # Transpose the source subregion
        src_subregion = [
            [src_subregion[0][transpose_indices[0]],
             src_subregion[0][transpose_indices[1]],
             src_subregion[0][transpose_indices[2]]],
            [src_subregion[1][transpose_indices[0]],
             src_subregion[1][transpose_indices[1]],
             src_subregion[1][transpose_indices[2]]],
        ]

        # Transpose the neuron location
        neuron_location = [
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][transpose_indices[0]],
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][transpose_indices[1]],
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][transpose_indices[2]],
        ]
    else:
        # If no transpose, use the default order
        neuron_location = [
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][0],
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1],
            runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][2],
        ]

    # If project_last_layer is enabled, handle that case
    if project_last_layer_of:

        if project_last_layer_of == "x":
            # Compute shapes of the source subregion and destination block boundaries
            src_shape = [
                src_dimensions[0] - 1,
                src_subregion[1][1] - src_subregion[0][1],
                src_subregion[1][2] - src_subregion[0][2],
            ]

            dst_shape = [
                1,
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][1],
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][2]
            ]

        if project_last_layer_of == "y":
            # Compute shapes of the source subregion and destination block boundaries
            src_shape = [
                src_subregion[1][0] - src_subregion[0][0],
                src_dimensions[1] - 1,
                src_subregion[1][2] - src_subregion[0][2],
            ]

            dst_shape = [
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0],
                1,
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][2]
            ]

        if project_last_layer_of == "z":
            # Compute shapes of the source subregion and destination block boundaries
            src_shape = [
                src_subregion[1][0] - src_subregion[0][0],
                src_subregion[1][1] - src_subregion[0][1],
                src_dimensions[2] - 1,
            ]

            dst_shape = [
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][0],
                runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"][1],
                1
            ]

    else:
        # Compute shapes of the source subregion and destination block boundaries
        src_shape = [
            src_subregion[1][0] - src_subregion[0][0],
            src_subregion[1][1] - src_subregion[0][1],
            src_subregion[1][2] - src_subregion[0][2],
        ]

        dst_shape = runtime_data.genome['blueprint'][dst_cortical_area]["block_boundaries"]

    dst_vox_dict = dict()

    try:
        for i in range(3):
            dst_vox_dict[i] = set()
            if src_shape[i] > dst_shape[i]:
                ratio = src_shape[i] / dst_shape[i]
                target_vox = int((neuron_location[i] - src_subregion[0][i]) / ratio)
                if (project_last_layer_of == "x" and i == 0) or \
                        (project_last_layer_of == "y" and i == 1) or \
                        (project_last_layer_of == "z" and i == 2):
                    target_vox -= 1
                dst_vox_dict[i].add(target_vox)

            elif src_shape[i] < dst_shape[i]:
                ratio = dst_shape[i] / src_shape[i]
                for vox in range(dst_shape[i]):
                    if int(vox / ratio) == (neuron_location[i] - src_subregion[0][i]):
                        target_vox = vox
                        dst_vox_dict[i].add(target_vox)
            elif src_shape[i] == dst_shape[i]:
                target_vox = (neuron_location[i] - src_subregion[0][i])
                dst_vox_dict[i].add(target_vox)
            else:
                pass
    except ZeroDivisionError:
        pass

    if dst_vox_dict[0] and dst_vox_dict[1] and dst_vox_dict[2]:
        for x in dst_vox_dict[0]:
            for y in dst_vox_dict[1]:
                for z in dst_vox_dict[2]:
                    candidate_list.append([x, y, z])

    return candidate_list


def syn_memory(src_cortical_area, dst_cortical_area):
    if dst_cortical_area not in runtime_data.memory_register:
        runtime_data.memory_register[dst_cortical_area] = set()
    runtime_data.memory_register[dst_cortical_area].add(src_cortical_area)


def last_to_first(src_cortical_area):
    src_cortical_dim = runtime_data.genome['blueprint'][src_cortical_area]["block_boundaries"]
    return [[src_cortical_dim[0] - 1, src_cortical_dim[1] - 1, src_cortical_dim[2] - 1]]

