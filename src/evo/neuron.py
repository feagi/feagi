"""
A collection of functions related to Neurons
"""

import random
import string
import datetime
import collections
import numpy as np
from math import floor
from inf import runtime_data


def block_reference_builder(block):
    return str(block[0]) + '-' + str(block[1]) + '-' + str(block[2])


def block_id_gen(cortical_area, coordinate):
    """
    Generating a block id so it can be used for faster neighbor detection

    Args:


    Returns:
        Something

    """
    cortical_area_dim = []
    geometric_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']
    for axis in geometric_boundaries:
        cortical_area_dim.append(geometric_boundaries[axis][1] - geometric_boundaries[axis][0])
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']

    block_id = []
    index = 0
    for location in coordinate:
        block_number = floor(
            location / ((cortical_area_dim[index] / (block_boundaries[index] + 0.00001)) + 0.00001))
        block_id.append(block_number)
        index += 1
    if block_id[0] > 100:
        print("large block detected")
    return block_id


def neuron_location_gen(x1, y1, z1, x2, y2, z2):
    """
    Function responsible to generate a pseudo-random location for a Neuron given some constraints

    """
    # todo: update to leverage the Genome template
    # todo: Would it be better to use relative locations in each cortical region instead?
    neuron_location = [random.randrange(x1, x2, 1), random.randrange(y1, y2, 1), random.randrange(z1, z2, 1)]
    return neuron_location


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


def neuron_create(cortical_area, soma_location, dendrite_locations):
    """
    Responsible for adding a Neuron to connectome

    """

    genome = runtime_data.genome

    neuron_id = neuron_id_gen()

    runtime_data.brain[cortical_area][neuron_id] = {}
    runtime_data.brain[cortical_area][neuron_id]["neighbors"] = {}
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
    runtime_data.brain[cortical_area][neuron_id]["dendrite_locations"] = dendrite_locations
    runtime_data.brain[cortical_area][neuron_id]["status"] = "Passive"
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_time"] = str(
        datetime.datetime.now())
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = 0

    #   runtime_data.brain[cortical_area][neuron_id]["group_id"] = ""
    #  consider using the group name part of Genome instead
    runtime_data.brain[cortical_area][neuron_id]["firing_pattern_id"] = \
        genome['blueprint'][cortical_area]['neuron_params']['firing_pattern_id']
    runtime_data.brain[cortical_area][neuron_id]["activation_function_id"] = \
        genome['blueprint'][cortical_area]['neuron_params']['activation_function_id']
    runtime_data.brain[cortical_area][neuron_id]["depolarization_threshold"] = \
        genome['blueprint'][cortical_area]['neuron_params']['depolarization_threshold']
    runtime_data.brain[cortical_area][neuron_id]["firing_threshold"] = \
        genome['blueprint'][cortical_area]['neuron_params']['firing_threshold']

    return neuron_id


def neuron_location_collector(cortical_area):
    """
    Generates a list of locations to be used for Neuron creation
    :return:
    """

    #   neighbor_count = 9999
    #   global max_density      # TBD: This value needs to be defied in a config file of some sort
    #   while neighbor_count < max_density:
    # Above condition will be met when enough neighbors has been created with following criteria
    #     1. Density requirements has been met
    #     2. TBD
    # TBD:  Need to figure a way to pass in a 3d object formula and use that to contain neuronal growth
    # Need to come up with an algorithm to populate the space within the object with random neurons given density
    # Output is expected to be a N x 3 matrix containing dimensions for N neurons to be created

    genome = runtime_data.genome

    if genome["blueprint"].get(cortical_area) is None:
        print("Cortical area %s not found!" % cortical_area)
        return

    neuron_loc_list = []

    if genome["blueprint"][cortical_area]["location_generation_type"] == "random":
        for _ in range(0, genome["blueprint"][cortical_area]["cortical_neuron_count"] *
                       int(runtime_data.parameters['Brain_Development']['neuron_multiplier'])):
            neuron_loc_list.append(neuron_location_gen(
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["x"][0],
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["y"][0],
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["z"][0],
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["x"][1],
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["y"][1],
                genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["z"][1]))
    else:
        x_lenght = (genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["x"][1] -
                    genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["x"][0])
        y_lenght = (genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["y"][1] -
                    genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["y"][0])
        z_lenght = (genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["z"][1] -
                    genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["z"][0])

        # Following formula calculates the proper distance between neurons to be used to have n number of them
        # evenly distributed within the given cortical area

        none_zero_axis = list(filter(lambda axis_width: axis_width > 1, [x_lenght, y_lenght, z_lenght]))

        dimension = len(none_zero_axis)

        neuron_count = genome["blueprint"][cortical_area]["cortical_neuron_count"]

        area = 1
        for _ in none_zero_axis:
            area = area * _

        neuron_gap = (area / neuron_count) ** (1 / dimension)

        # Number of neurons in each axis
        xn = int(x_lenght / neuron_gap)
        yn = int(y_lenght / neuron_gap)
        zn = int(z_lenght / neuron_gap)

        if xn == 0:
            xn = 1
        if yn == 0:
            yn = 1
        if zn == 0:
            zn = 1

        x_coordinate = genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["x"][0]
        y_coordinate = genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["y"][0]
        z_coordinate = genome["blueprint"][cortical_area]["neuron_params"]["geometric_boundaries"]["z"][0]

        for i in range(xn):
            for ii in range(yn):
                for iii in range(zn):
                    neuron_loc_list.append([int(x_coordinate), int(y_coordinate), int(z_coordinate)])
                    z_coordinate += neuron_gap
                y_coordinate += neuron_gap
            x_coordinate += neuron_gap

    return neuron_loc_list


def neuron_genesis_3d(cortical_area):
    """
    Function responsible for creating Neurons using the blueprint

    1. location_collector function generates the list of all neuron locations for a given cortical area
    2. for each location, create the list of dendrite locations and associated block

    """
    # This code segment creates a new Neuron per every location in neuron_loc_list
    neuron_loc_list = neuron_location_collector(cortical_area=cortical_area)
    neuron_count = 0

    for candidate_neuron_location in neuron_loc_list:
        dendrite_locations = neuron_dendrite_location_generator(cortical_area=cortical_area,
                                                                neuron_location=candidate_neuron_location)

        candidate_neuron_location_block = block_id_gen(coordinate=candidate_neuron_location,
                                                       cortical_area=cortical_area)
        # Create a new Neuron in target destination
        neuron_id = neuron_create(cortical_area=cortical_area,
                                  soma_location=[candidate_neuron_location, candidate_neuron_location_block],
                                  dendrite_locations=dendrite_locations)
        neuron_count += 1
        # Adding neuron id to the block dictionary
        for dendrite in dendrite_locations:
            block_reference = block_reference_builder(block=[dendrite[1][0], dendrite[1][1], dendrite[1][2]])
            if cortical_area not in runtime_data.block_dic:
                runtime_data.block_dic[cortical_area] = {}
            if block_reference not in runtime_data.block_dic[cortical_area]:
                runtime_data.block_dic[cortical_area][block_reference] = []
            runtime_data.block_dic[cortical_area][block_reference].append(neuron_id)

    return neuron_count


def neuron_dendrite_template(direction):
    # todo: use a mathematical model instead of manual templates
    # todo: move templates to genome

    if direction == '/':
        template = np.array([
            [0, 0, 0],
            [1, 1, 0],
            [2, 2, 0],
            [-1, -1, 0],
            [-2, -2, 0]
        ])

    elif direction == '\\':
        template = np.array([
            [0, 0, 0],
            [-1, 1, 0],
            [-2, 2, 0],
            [1, -1, 0],
            [2, -2, 0]
        ])

    elif direction == '-':
        template = np.array([
            [0, 0, 0],
            [-1, 0, 0],
            [-2, 0, 0],
            [1, 0, 0],
            [2, 0, 0]
        ])

    elif direction == '|':
        template = np.array([
            [0, 0, 0],
            [0, 1, 0],
            [0, 2, 0],
            [0, -1, 0],
            [0, -2, 0]
        ])
    else:
        template = np.array([
            [0, 0, 0]
        ])

    return template


def neuron_dendrite_location_generator(cortical_area, neuron_location):
    """
    generates location and block information of the neuron dendrites
    """
    dendrite_location_blocks = list()

    neuron_location = np.array(neuron_location)

    # dendrite_growth_rule = runtime_data.genome["blueprint"][cortical_area]["dendrite_growth_rule"]

    # todo: build the function to generate dendrite locations based on dendrite_growth_rule
    try:
        direction_sensitivity = runtime_data.genome['blueprint'][cortical_area]['direction_sensitivity']
    except KeyError:
        direction_sensitivity = ''
    dendrite_locations = neuron_dendrite_template(direction_sensitivity) + neuron_location

    for dendrite_location in dendrite_locations.tolist():
        dendrite_location_block = block_id_gen(cortical_area=cortical_area, coordinate=dendrite_location)
        dendrite_location_blocks.append([dendrite_location, dendrite_location_block])

    return dendrite_location_blocks


def neuron_apoptosis(cortical_area):
    """
    Responsible for programmed death of neuron
    """
    # todo: implement the function
    return


# todo: Cythonize this
# @jit
def neuron_finder(cortical_area, location, radius):
    """
    Queries a given cortical area and returns a listed of Neuron IDs matching search criteria
    """
    brain = runtime_data.brain
    location = np.array(location)

    neuron_list = np.array([])

    for key in runtime_data.brain[cortical_area]:
        x = brain[cortical_area][key]['location'][0]
        y = brain[cortical_area][key]['location'][1]
        # z = brain[cortical_area][key]['location'][2]

        # Searching only the XY plane for candidate neurons         ????
        if np.sqrt((x - location[0]) ** 2 + (y - location[1]) ** 2) <= (radius ** 2):
            if collections.Counter(neuron_list)[key] == 0:
                neuron_list = np.append(neuron_list, key)

    return list(neuron_list)
