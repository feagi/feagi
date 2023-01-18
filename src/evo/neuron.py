
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
A collection of functions related to Neurons
"""

import random
import string
import datetime
import logging
# import collections
# import numpy as np
from evo.voxels import *


logger = logging.getLogger(__name__)

# def neuron_location_gen(x1, y1, z1, x2, y2, z2):
#     """
#     Function responsible to generate a pseudo-random location for a Neuron given some constraints
#
#     """
#     # todo: update to leverage the Genome template
#     # todo: Would it be better to use relative locations in each cortical region instead?
#     neuron_location = [random.randrange(x1, x2, 1), random.randrange(y1, y2, 1), random.randrange(z1, z2, 1)]
#     return neuron_location


def neuron_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_') + '_' + (''.join(random.choice(chars)
                                                                                               for _ in
                                                                                               range(size))) + '_N'


def init_neuron(cortical_area, soma_location):
    """
    Responsible for adding a Neuron to connectome

    """

    genome = runtime_data.genome

    neuron_id = neuron_id_gen()

    runtime_data.brain[cortical_area][neuron_id] = {}
    runtime_data.brain[cortical_area][neuron_id]["neighbors"] = {}
    runtime_data.brain[cortical_area][neuron_id]["upstream_neurons"] = {}
    runtime_data.brain[cortical_area][neuron_id]["event_id"] = {}
    runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count_inst"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_intake_total"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_intake_count"] = 0
    runtime_data.brain[cortical_area][neuron_id]["consecutive_fire_cnt"] = 0
    runtime_data.brain[cortical_area][neuron_id]["snooze_till_burst_num"] = 0
    runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] = 0
    runtime_data.brain[cortical_area][neuron_id]["activity_history"] = []
    runtime_data.brain[cortical_area][neuron_id]["soma_location"] = soma_location
    # loc_blk is a two element list where first element being the location of the neuron and second being the block
    # runtime_data.brain[cortical_area][neuron_id]["dendrite_locations"] = dendrite_locations
    runtime_data.brain[cortical_area][neuron_id]["status"] = "Passive"
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_time"] = str(
        datetime.datetime.now())
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = 0

    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_update"] = 0
    # runtime_data.brain[cortical_area][neuron_id]["residual_membrane_potential"] = 0

    #   runtime_data.brain[cortical_area][neuron_id]["group_id"] = ""
    #  consider using the group name part of Genome instead
    # runtime_data.brain[cortical_area][neuron_id]["depolarization_threshold"] = \
    #     genome['blueprint'][cortical_area]['depolarization_threshold']
    runtime_data.brain[cortical_area][neuron_id]["firing_threshold"] = \
        genome['blueprint'][cortical_area]['firing_threshold']

    leak = genome['blueprint'][cortical_area]['leak_coefficient']
    leak_variability = genome['blueprint'][cortical_area]['leak_variability']
    if leak_variability:
        if abs(leak_variability) > 1:
            leak = leak + leak * random.randrange(1, leak_variability, 1) / 100

    runtime_data.brain[cortical_area][neuron_id]["leak_coefficient"] = leak

    return neuron_id


# def neuron_location_collector(cortical_area):
#     """
#     Generates a list of locations to be used for Neuron creation
#     :return:
#     """
#
#     #   neighbor_count = 9999
#     #   global max_density      # TBD: This value needs to be defied in a config file of some sort
#     #   while neighbor_count < max_density:
#     # Above condition will be met when enough neighbors has been created with following criteria
#     #     1. Density requirements has been met
#     #     2. TBD
#     # TBD:  Need to figure a way to pass in a 3d object formula and use that to contain neuronal growth
#     # Need to come up with an algorithm to populate the space within the object with random neurons given density
#     # Output is expected to be a N x 3 matrix containing dimensions for N neurons to be created
#
#     genome = runtime_data.genome
#
#     if genome["blueprint"].get(cortical_area) is None:
#         print("Cortical area %s not found!" % cortical_area)
#         return
#
#     neuron_loc_list = []
#     location_generation_type = genome["blueprint"][cortical_area]["location_generation_type"]
#
#     if location_generation_type == "random":
#         for _ in range(0, genome["blueprint"][cortical_area]["cortical_neuron_count"] *
#                        int(runtime_data.parameters['Brain_Development']['neuron_multiplier'])):
#             neuron_loc_list.append(neuron_location_gen(
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["x"][0],
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["y"][0],
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["z"][0],
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["x"][1],
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["y"][1],
#                 genome["blueprint"][cortical_area]["geometric_boundaries"]["z"][1]))
#     elif location_generation_type == "sequential":
#         # Following formula calculates the proper distance between neurons to be used to have n number of them
#         # evenly distributed within the given cortical area
#
#         # Note: Sequential cortical is assumed to be 1 dimensional
#
#         # Determine which block has the largest block count.
#         dominant_dimension = runtime_data.genome['blueprint'][cortical_area]['dimension_dominance']
#
#         dimension_map = {0: "x", 1: "y", 2: "z"}
#         dominant_index = dimension_map[dominant_dimension]
#         non_dominant_axis = list(filter(lambda x: x != dominant_dimension, dimension_map))
#         neuron_count = genome["blueprint"][cortical_area]["cortical_neuron_count"]
#         dominant_distance = \
#             runtime_data.genome['blueprint'][cortical_area]["geometric_boundaries"][dominant_index][1]\
#             - runtime_data.genome['blueprint'][cortical_area]["geometric_boundaries"][dominant_index][0]
#
#         neuron_gap = int(dominant_distance / neuron_count)
#
#         starting_point = int(neuron_gap / 2)
#         location_pointer = starting_point
#         # dominant_neuron_locations = []
#
#         for _ in range(neuron_count):
#             # dominant_neuron_locations.append(location_pointer)
#             location_pointer += neuron_gap
#             coordinate_value = [0, 0, 0]
#
#             for coordinate in dimension_map:
#                 if coordinate == dominant_dimension:
#                     coordinate_value[coordinate] = starting_point
#                     starting_point += neuron_gap
#                 else:
#                     # todo: store the midpoint values in a list once
#                     coordinate_value[coordinate] = \
#                         (genome["blueprint"][cortical_area]["geometric_boundaries"][dimension_map[coordinate]][1] -
#                          genome["blueprint"][cortical_area]["geometric_boundaries"][dimension_map[coordinate]][0]) / 2
#
#             neuron_loc_list.append([int(coordinate_value[0]), int(coordinate_value[1]), int(coordinate_value[2])])
#
#     else:
#         print("Warning: Unsupported location_generation_type was detected within Genome for %s as %s "
#               % (cortical_area, location_generation_type))
#         neuron_loc_list = []
#
#     return neuron_loc_list


def create_neuron(cortical_area, voxel):
    """
    Responsible for creating Neurons and updating the Voxel Dictionary
    """
    neuron_count = runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"]

    if neuron_count < 1 or not neuron_count:
        neuron_count = 1

    neuron_location = block_ref_2_id(voxel)

    # Create a new Neuron in target destination
    for _ in range(int(neuron_count)):
        neuron_id = init_neuron(cortical_area=cortical_area, soma_location=neuron_location)
        runtime_data.voxel_dict[cortical_area][voxel].add(neuron_id)


# def neuron_genesis_3d(cortical_area):
#     """
#     Function responsible for creating Neurons using the blueprint
#
#     1. location_collector function generates the list of all neuron locations for a given cortical area
#     2. for each location, create the list of dendrite locations and associated block
#
#     """
#     # This code segment creates a new Neuron per every location in neuron_loc_list
#     neuron_loc_list = neuron_location_collector(cortical_area=cortical_area)
#     neuron_count = 0
#
#     for candidate_neuron_location in neuron_loc_list:
#         dendrite_locations = neuron_dendrite_location_generator(cortical_area=cortical_area,
#                                                                 neuron_location=candidate_neuron_location)
#
#         candidate_neuron_location_block = block_id_gen(coordinate=candidate_neuron_location,
#                                                        cortical_area=cortical_area)
#         # Create a new Neuron in target destination
#         neuron_id = neuron_create(cortical_area=cortical_area,
#                                   soma_location=[candidate_neuron_location, candidate_neuron_location_block],
#                                   dendrite_locations=dendrite_locations)
#         neuron_count += 1
#         # Adding neuron id to the block dictionary
#         for dendrite in dendrite_locations:
#             block_reference = block_reference_builder(block=[dendrite[1][0], dendrite[1][1], dendrite[1][2]])
#             if cortical_area not in runtime_data.block_dic:
#                 runtime_data.block_dic[cortical_area] = {}
#             if block_reference not in runtime_data.block_dic[cortical_area]:
#                 runtime_data.block_dic[cortical_area][block_reference] = []
#             runtime_data.block_dic[cortical_area][block_reference].append(neuron_id)
#     return neuron_count


# def neuron_dendrite_template(direction):
#     # todo: use a mathematical model instead of manual templates
#     # todo: move templates to genome
#
#     if direction == '/':
#         template = np.array([
#             [0, 0, 0],
#             [1, 1, 0],
#             [2, 2, 0],
#             [-1, -1, 0],
#             [-2, -2, 0]
#         ])
#
#     elif direction == '\\':
#         template = np.array([
#             [0, 0, 0],
#             [-1, 1, 0],
#             [-2, 2, 0],
#             [1, -1, 0],
#             [2, -2, 0]
#         ])
#
#     elif direction == '-':
#         template = np.array([
#             [0, 0, 0],
#             [-1, 0, 0],
#             [-2, 0, 0],
#             [1, 0, 0],
#             [2, 0, 0]
#         ])
#
#     elif direction == '|':
#         template = np.array([
#             [0, 0, 0],
#             [0, 1, 0],
#             [0, 2, 0],
#             [0, -1, 0],
#             [0, -2, 0]
#         ])
#     else:
#         template = np.array([
#             [0, 0, 0]
#         ])
#
#     return template


# def neuron_dendrite_location_generator(cortical_area, neuron_location):
#     """
#     generates location and block information of the neuron dendrites
#     """
#     dendrite_location_blocks = list()
#
#     if runtime_data.genome["blueprint"][cortical_area]["location_generation_type"] == 'sequential':
#         dendrite_location = neuron_location
#         dendrite_location_block = block_id_gen(cortical_area=cortical_area, coordinate=neuron_location)
#         dendrite_location_blocks.append([dendrite_location, dendrite_location_block])
#     else:
#         neuron_location = np.array(neuron_location)
#
#         # dendrite_growth_rule = runtime_data.genome["blueprint"][cortical_area]["dendrite_growth_rule"]
#
#         # todo: build the function to generate dendrite locations based on dendrite_growth_rule
#         try:
#             direction_sensitivity = runtime_data.genome['blueprint'][cortical_area]['direction_sensitivity']
#         except KeyError:
#             direction_sensitivity = ''
#         dendrite_locations = abs(neuron_dendrite_template(direction_sensitivity) + neuron_location)
#
#         for dendrite_location in dendrite_locations.tolist():
#             dendrite_location_block = block_id_gen(cortical_area=cortical_area, coordinate=dendrite_location)
#             dendrite_location_blocks.append([dendrite_location, dendrite_location_block])
#
#     return dendrite_location_blocks


def neuron_apoptosis(cortical_area):
    """
    Responsible for programmed death of neuron
    """
    # todo: implement the function
    return


# # todo: Cythonize this
# # @jit
# def neuron_finder(cortical_area, location, radius):
#     """
#     Queries a given cortical area and returns a listed of Neuron IDs matching search criteria
#     """
#     brain = runtime_data.brain
#     location = np.array(location)
#
#     neuron_list = np.array([])
#
#     for key in runtime_data.brain[cortical_area]:
#         x = brain[cortical_area][key]['location'][0]
#         y = brain[cortical_area][key]['location'][1]
#         z = brain[cortical_area][key]['location'][2]
#
#         # Searching only the XY plane for candidate neurons         ????
#         if np.sqrt(
#             (x - location[0]) ** 2 + \
#             (y - location[1]) ** 2 + \
#             (z - location[2]) ** 2
#         ) <= (radius ** 2):
#             if collections.Counter(neuron_list)[key] == 0:
#                 neuron_list = np.append(neuron_list, key)
#
#     return list(neuron_list)
