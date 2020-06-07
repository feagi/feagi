# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
"""
set of algorithms designed to generate neurons and update the connectome

Architect will accept the following information as input:
        1. Growth boundaries: Driving the limits of Neuron creation
        2. Neuron Density:    Driving number of Neurons per unit of space
        3. Growth pattern:    Driving Neighbor relations and shape of connectivity
"""

import datetime
import string
import random
import numpy as np
import collections
# from numba import jit

from math import sqrt, ceil, floor
from configuration import runtime_data, settings


print(settings.Bcolors.YELLOW + "Module loaded: architect" + settings.Bcolors.ENDC)


def neuron_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                           for _ in range(size)))+'_N'


def event_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                           for _ in range(size)))+'_E'


def test_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                           for _ in range(size)))+'_T'


def run_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                           for _ in range(size)))+'_R'


def neuro_genesis(cortical_area, soma_location, dendrite_locations):
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
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_time"] = str(datetime.datetime.now())
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


def synapse(src_cortical_area, src_id, dst_cortical_area, dst_id, postsynaptic_current):
    """
    Function responsible for creating a synapse between a neuron and another one. In reality a single neuron can have
    many synapses with another individual neuron. Here we use synaptic strength to simulate the same.
    Note: Synapse association is captured on the Source Neuron side within Connectome
    
    # Input: The id for source and destination Neuron plus the parameter defining connection strength
    # Source provides the Axon and connects to Destination Dendrite
    # postsynaptic_current is intended to provide the level of synaptic strength
    """

    # # Check to see if the source and destination ids are valid if not exit the function
    # if src_id not in runtime_data.brain[src_cortical_area]:
    #     print("Source or Destination neuron not found")
    #     return

    runtime_data.brain[src_cortical_area][src_id]["neighbors"][dst_id] = \
        {"cortical_area": dst_cortical_area, "postsynaptic_current": postsynaptic_current}

    return


def random_location_generator(x1, y1, z1, x2, y2, z2):
    """
    Function responsible to generate a pseudo-random location for a Neuron given some constraints

    """
    # todo: update to leverage the Genome template
    # todo: Would it be better to use relative locations in each cortical region instead?
    neuron_location = [random.randrange(x1, x2, 1), random.randrange(y1, y2, 1), random.randrange(z1, z2, 1)]
    return neuron_location


def dendrite_template(direction):
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


def dendrite_location_generator(cortical_area, neuron_location):
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
    dendrite_locations = dendrite_template(direction_sensitivity) + neuron_location

    for dendrite_location in dendrite_locations.tolist():
        dendrite_location_block = block_id_gen(cortical_area=cortical_area, coordinate=dendrite_location)
        dendrite_location_blocks.append([dendrite_location, dendrite_location_block])

    return dendrite_location_blocks


def location_collector(cortical_area):
    """
    Function responsible to generate a list of locations to be used for Neuron creation
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
        for _ in range(0, genome["blueprint"][cortical_area]["cortical_neuron_count"]):
            neuron_loc_list.append(random_location_generator(
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


def three_dim_growth(cortical_area):
    """
    Function responsible for creating Neurons using the blueprint

    1. location_collector function generates the list of all neuron locations for a given cortical area
    2. for each location, create the list of dendrite locations and associated block

    """
    # This code segment creates a new Neuron per every location in neuron_loc_list
    neuron_loc_list = location_collector(cortical_area)
    neuron_count = 0

    for candidate_neuron_location in neuron_loc_list:
        dendrite_locations = dendrite_location_generator(cortical_area=cortical_area,
                                                         neuron_location=candidate_neuron_location)

        candidate_neuron_location_block = block_id_gen(cortical_area=cortical_area,
                                                       coordinate=candidate_neuron_location)
        # Create a new Neuron in target destination
        neuron_id = neuro_genesis(cortical_area=cortical_area,
                                  soma_location=[candidate_neuron_location, candidate_neuron_location_block],
                                  dendrite_locations=dendrite_locations)
        neuron_count += 1
        # Adding neuron id to the block dictionary
        for dendrite in dendrite_locations:
            block_reference = str(dendrite[1][0]) + '-' + str(dendrite[1][1]) + '-' + str(dendrite[1][2])
            if cortical_area not in runtime_data.block_dic:
                runtime_data.block_dic[cortical_area] = {}
            if block_reference not in runtime_data.block_dic[cortical_area]:
                runtime_data.block_dic[cortical_area][block_reference] = []
            runtime_data.block_dic[cortical_area][block_reference].append(neuron_id)

    return neuron_count


def neighbor_finder(cortical_area, neuron_id, rule, rule_param):
    """
    A set of math functions allowing to detect the eligibility of a Neuron to become neighbor
    :param neuron_id:
    :param rule:
    :param rule_param:
    :param cortical_area
    :return:
    """
    if cortical_area == 'utf8_memory':
        print('rule=', rule)
    # Input: neuron id of which we desire to find all candidate neurons for
    neighbor_candidates = []

    # Narrow down the search scope to only few blocks
    # neighbors_in_block = neurons_in_block_neighborhood(cortical_area, neuron_id, kernel_size=1)
    neuron_block = runtime_data.brain[cortical_area][neuron_id]['block']
    block_reference = str(neuron_block[0]) + '-' + str(neuron_block[1]) + '-' + str(neuron_block[2])
    neighbors_in_block = runtime_data.block_dic[cortical_area][block_reference]

    for dst_neuron_id in neighbors_in_block:
        if rule_matcher(rule_id=rule,
                        rule_param=rule_param,
                        cortical_area_src=cortical_area,
                        cortical_area_dst=cortical_area,
                        src_neuron_id=neuron_id,
                        dst_neuron_id=dst_neuron_id):
            neighbor_candidates.append(dst_neuron_id)

    return neighbor_candidates


def neighbor_builder(brain, genome, brain_gen, cortical_area, rule_id, rule_param, postsynaptic_current):
    """
    Function responsible for crawling through Neurons and deciding where to build Synapses
    """
    # Need to figure how to build a Neighbor relationship between Neurons
    #    1. Should it be based on the proximity?
    #    2. Should it be manually set? It wont be scalable
    #    3. How can it be based on a Guide function?
    # This function will utilize the Synapse function and based on an algorithm will create the relationships

    # Algorithm:
    #    1. Ask for a direction
    #    2. For each neuron in path find a list of eligible candidates
    #    3. Update connectome to the candidates become neighbors of the source neuron

    # todo: Warning: Need to watch for the side effects on this line to make sure its not overwriting values
    # to accommodate the new namespace used by multiprocessing
    if brain_gen:
        runtime_data.brain = brain
        runtime_data.genome = genome

    synapse_count = 0
    for src_id in runtime_data.brain[cortical_area]:
        # Cycle thru the neighbor_candidate_list and establish Synapses
        neighbor_candidates = neighbor_finder(cortical_area, src_id, rule_id, rule_param)
        for dst_id in neighbor_candidates:
            synapse(cortical_area, src_id, cortical_area, dst_id, postsynaptic_current)
            synapse_count += 1
            # print("Made a Synapse between %s and %s" % (src_id, dst_id))

    if brain_gen:
        brain = runtime_data.brain
        runtime_data.genome = genome

        synapse_count2 = 0
        for area in brain:
            for neuron in brain[area]:
                for connection in brain[area][neuron]['neighbors']:
                    synapse_count2 += 1
    else:
        brain = {}
    return synapse_count, brain


def dst_projection_center(cortical_area_src, neuron_id, cortical_area_dst):
    """
    Returns the coordinates of a neuron projected into a target Cortical layer
    """

    # Find relative coordinates on the source and destination side
    src_lengths = cortical_area_lengths(cortical_area_src)
    dst_lengths = cortical_area_lengths(cortical_area_dst)
    coordinate_scales = [a/b for a, b in zip(dst_lengths, src_lengths)]

    x_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][0]
    y_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][1]
    z_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][2]

    dst_projection_center = list()
    dst_projection_center.append(x_coordinate_src * coordinate_scales[0])
    dst_projection_center.append(y_coordinate_src * coordinate_scales[1])
    dst_projection_center.append(z_coordinate_src * coordinate_scales[2])

    return dst_projection_center


def neighbor_finder_ext(cortical_area_src, cortical_area_dst, src_neuron_id, rule, rule_param):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    # Input: neuron id of which we desire to find all candidate neurons for from another cortical region
    dst_data = runtime_data.brain[cortical_area_dst]
    neighbor_candidates = []

    if runtime_data.genome['blueprint'][cortical_area_dst]['location_generation_type'] == 'sequential':
        for dst_neuron_id in dst_data:
            if rule_matcher(rule_id=rule, rule_param=rule_param, cortical_area_src=cortical_area_src,
                            cortical_area_dst=cortical_area_dst, dst_neuron_id=dst_neuron_id,
                            src_neuron_id=src_neuron_id):
                neighbor_candidates.append(dst_neuron_id)
    else:
        # todo: rules are currently not being used for synapse creation. Would there be value? Assess!!
        neuron_block_src = runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location'][1]
        if 'layer_index' in runtime_data.genome['blueprint'][cortical_area_src]:
            block_index = runtime_data.genome['blueprint'][cortical_area_src]['layer_index'] - 1
        else:
            block_index = 0
        # todo: make the next line generic. It would become important when complex cortical areas are introduced
        if cortical_area_dst == 'vision_v2':
            block_reference = str(neuron_block_src[0]) + '-' + str(neuron_block_src[1]) + '-' + str(block_index)
            # print("block_reference:", block_reference)
        else:
            block_reference = str(neuron_block_src[0]) + '-' + str(neuron_block_src[1]) + '-' + str(neuron_block_src[2])

        z_block_boundary = runtime_data.genome['blueprint'][cortical_area_dst]['neuron_params']['block_boundaries'][2]

        if z_block_boundary > 1:
            if block_reference in runtime_data.block_dic[cortical_area_dst]:
                for dst_neuron in runtime_data.block_dic[cortical_area_dst][block_reference]:
                    if runtime_data.brain[cortical_area_dst][dst_neuron]['soma_location'][1][2] == block_index:
                        neighbor_candidates.append(dst_neuron)

        elif block_reference in runtime_data.block_dic[cortical_area_dst]:
            for neighbor in runtime_data.block_dic[cortical_area_dst][block_reference]:
                neighbor_candidates.append(neighbor)

    return neighbor_candidates


def neighbor_builder_ext(brain, genome, brain_gen, cortical_area_src, cortical_area_dst, rule, rule_param, postsynaptic_current=1.1):
    """
    Crawls thru a Cortical area and builds Synapses with External Cortical Areas
    """
    # to accommodate the new namespace used by multiprocessing
    if brain_gen:
        runtime_data.brain = brain
        runtime_data.genome = genome

    synapse_count = 0
    for src_id in runtime_data.brain[cortical_area_src]:
        # Cycle thru the neighbor_candidate_list and establish Synapses
        neighbor_candidates = neighbor_finder_ext(cortical_area_src, cortical_area_dst, src_id, rule, rule_param)
        for dst_id in neighbor_candidates:
            # Through a dice to decide for synapse creation. This is to limit the amount of synapses.
            if random.randrange(1, 100) < runtime_data.genome['blueprint'][cortical_area_dst]['synapse_attractivity']:
                # Connect the source and destination neuron via creating a synapse
                synapse(cortical_area_src, src_id, cortical_area_dst, dst_id, postsynaptic_current)
                synapse_count += 1
                # print("Made a Synapse between %s and %s" % (src_id, dst_id))

    if brain_gen:
        brain = runtime_data.brain
    else:
        brain = {}
    return synapse_count, brain


def field_set(cortical_area, field_name, field_value):
    """
    This function changes a field value in connectome 
    
    *** Incomplete ***
    
    """
    # runtime_data.brain[cortical_area] = runtime_data.brain[cortical_area]
    # for key in runtime_data.brain[cortical_area]:
    #     runtime_data.brain[cortical_area][key][field_name] = field_value
    #

    return


def neighbor_reset(cortical_area):
    """
    This function deletes all the neighbor relationships in the connectome
    """

    for key in runtime_data.brain[cortical_area]:
        runtime_data.brain[cortical_area][key]["neighbors"] = {}

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
        if np.sqrt((x-location[0]) ** 2 + (y-location[1]) ** 2) <= (radius ** 2):
            if collections.Counter(neuron_list)[key] == 0:
                neuron_list = np.append(neuron_list, key)

    return list(neuron_list)


def neuron_finder2(cortical_area, location, radius):

    block_id = block_id_gen(coordinate=location, cortical_area=cortical_area)

    neuron_list = neurons_in_the_block(cortical_area, block_id)

    return neuron_list


def connectome_location_data(cortical_area):
    """
    Extracts Neuron locations and neighbor relatioships from the connectome
    """

    neuron_locations = []
    for key in runtime_data.brain[cortical_area]:
        location_data = runtime_data.brain[cortical_area][key]["soma_location"][0]
        location_data.append(runtime_data.brain[cortical_area][key]["cumulative_fire_count"])
        neuron_locations.append(location_data)

    return neuron_locations


def neuron_eliminator():
    """
    Responsible for programmed neuron's death
    """
    # todo: implement the function
    return


def rule_matcher(rule_id, rule_param, cortical_area_src, cortical_area_dst, dst_neuron_id, src_neuron_id):
    # todo: locations to be updated and made generic allowing multiple locations
    src_coord = np.array(runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location'][0])
    dst_coord = np.array(runtime_data.brain[cortical_area_dst][dst_neuron_id]['dendrite_locations'][0][0])

    src_blk = np.array(runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location'][1])
    dst_blk = np.array(runtime_data.brain[cortical_area_dst][dst_neuron_id]['dendrite_locations'][0][1])

    # Find relative coordinates on the source and destination side
    src_lengths = cortical_area_lengths(cortical_area_src)
    dest_lengths = cortical_area_lengths(cortical_area_dst)
    coordinate_scales = [a/b for a, b in zip(dest_lengths, src_lengths)]

    x_coordinate_src = src_coord[0]
    y_coordinate_src = src_coord[1]
    z_coordinate_src = src_coord[2]
    x_coordinate_dst = dst_coord[0]
    y_coordinate_dst = dst_coord[1]
    z_coordinate_dst = dst_coord[2]

    projection_center = np.array([])
    projection_center = np.append(projection_center, x_coordinate_src * coordinate_scales[0])
    projection_center = np.append(projection_center, y_coordinate_src * coordinate_scales[1])
    projection_center = np.append(projection_center, z_coordinate_src * coordinate_scales[2])

    is_candidate = False

    # Rule 0: Selects all neurons within rule_param radius
    if rule_id == 'rule_0':
        radius = np.sqrt(((x_coordinate_src - x_coordinate_dst) ** 2) +
                         ((y_coordinate_src - y_coordinate_dst) ** 2) +
                         ((z_coordinate_src - z_coordinate_dst) ** 2))
        if radius < rule_param:
            print("This is the neuron id you were looking for:", src_neuron_id)
            is_candidate = True

    # Rule 1: Selects only neurons within rule_param unit limits forward of source Neuron in z direction
    if rule_id == 'rule_1':
        if (z_coordinate_src > z_coordinate_dst) and \
                np.sqrt(((x_coordinate_src - x_coordinate_dst) ** 2) +
                     ((y_coordinate_src - y_coordinate_dst) ** 2)) < rule_param:
            is_candidate = True

    # Rule 2: Selects neurons from the destination cortical region
    if rule_id == 'rule_2':
        if np.sqrt(((x_coordinate_src - x_coordinate_dst) ** 2) +
                   ((y_coordinate_src - y_coordinate_dst) ** 2)) < rule_param:
            is_candidate = True

    # Rule 3: Specific for narrow cortical regions specially built for computer interface
    if rule_id == 'rule_3':
        if abs(z_coordinate_src - z_coordinate_dst) == rule_param:
            is_candidate = True

    # Rule 4: Maps entire layer to another. Expands the xy plane and ignores the z location
    if rule_id == 'rule_4':
        if np.sqrt(((projection_center[0] - x_coordinate_dst) ** 2) +
                   ((projection_center[1] - y_coordinate_dst) ** 2)) < rule_param:
            is_candidate = True

    # Rule 5: Helps mapping multiple layers to a single layer
    if rule_id == 'rule_5':
        src_layer_index = runtime_data.genome['blueprint'][cortical_area_src]['layer_index']
        src_total_layer_count = runtime_data.genome['blueprint'][cortical_area_src]['total_layer_count']
        dst_layer_height = dest_lengths[2] / src_total_layer_count
        if src_blk[0] == dst_blk[0] and src_blk[1] == dst_blk[1] and \
                ((dst_layer_height * (src_layer_index - 1)) < z_coordinate_dst < (dst_layer_height * src_layer_index)):
            is_candidate = True
        # is_candidate = True

    # Rule 6: Maps XY blocks from one layer to another
    if rule_id == 'rule_6':
        # src_blk_x = runtime_data.brain[cortical_area_src][src_neuron_id]["block"][0]
        # src_blk_y = runtime_data.brain[cortical_area_src][src_neuron_id]["block"][1]
        # dst_blk_x = runtime_data.brain[cortical_area_dst][dst_neuron_id]["block"][0]
        # dst_blk_y = runtime_data.brain[cortical_area_dst][dst_neuron_id]["block"][1]
        # # print(src_blk_x, dst_blk_x, "---", src_blk_y, dst_blk_y)
        # if abs(src_blk_x - dst_blk_x) < rule_param and abs(src_blk_y - dst_blk_y) < rule_param:
        #     is_candidate = True
        is_candidate = True

    # Rule 7: Maps XY blocks from one layer to another
    if rule_id == 'rule_7':
        src_layer_index = runtime_data.genome['blueprint'][cortical_area_src]['layer_index']
        src_total_layer_count = runtime_data.genome['blueprint'][cortical_area_src]['total_layer_count']
        dst_layer_height = dest_lengths[2] / src_total_layer_count
        if (np.sqrt(((projection_center[0] - x_coordinate_src) ** 2) +
                    ((projection_center[1] - y_coordinate_src) ** 2)) < rule_param) and \
                (projection_center[2] > (src_layer_index * dst_layer_height)) and \
                (projection_center[2] < ((src_layer_index + 1) * dst_layer_height)):
            is_candidate = True

    return is_candidate


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][1] -
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][0])

    return length


def block_id_gen(cortical_area, coordinate):
    """
    Generating a block id so it can be used for faster neighbor detection
    """

    cortical_area_dim = []
    geometric_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']
    for axis in geometric_boundaries:
        cortical_area_dim.append(geometric_boundaries[axis][1]-geometric_boundaries[axis][0])
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']

    block_id = []
    index = 0
    for location in coordinate:
        block_number = floor(location / ((cortical_area_dim[index] / (block_boundaries[index] + 0.00001)) + 0.00001))
        block_id.append(block_number)
        index += 1
    if block_id[0] > 100:
        print("large block detected")
    return block_id


def neurons_in_same_block(cortical_area, neuron_id):
    """
    Generates a list of Neurons in the same block as the given one
    """
    neuron_list = []
    for _ in runtime_data.brain[cortical_area]:
        if runtime_data.brain[cortical_area][_]['block'] == \
                runtime_data.brain[cortical_area][neuron_id]['block']:
            if _ != neuron_id:
                neuron_list.append(_)
    return neuron_list


def neurons_in_the_block(cortical_area, block_id):
    """
    Generates a list of Neurons in the given block
    block_id to be entered as [x,y,z]
    """
    neuron_list = []
    for neuron_id in runtime_data.brain[cortical_area]:
        if runtime_data.brain[cortical_area][neuron_id]['block'] == block_id:
            neuron_list.append(neuron_id)
    return neuron_list


def neighboring_blocks(block_id, kernel_size):
    """
    Returns the list of block ids who are neighbor of the given one
    Block_id is in form of [x,y,z]
    """

    block_list = list()

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
                    block_list.append(neighbor_block_id)

    return block_list


def neurons_in_block_neighborhood(cortical_area, neuron_id, kernel_size=3):
    """
    Provides the list of all neurons within the surrounding blocks given the kernel size with default being 3
    """
    neuron_list = list()
    neuron_block_id = runtime_data.brain[cortical_area][neuron_id]['block']
    block_list = neighboring_blocks(neuron_block_id, kernel_size)
    for block in block_list:
        neurons_in_block = neurons_in_the_block(cortical_area, block)
        for neuron in neurons_in_block:
            neuron_list.append(neuron)
    return neuron_list


def neurons_in_block_neighborhood_2(cortical_area, block_id, kernel_size=3):
    """
    Provides the list of all neurons within the surrounding blocks given the kernel size with default being 3
    """
    neuron_list = list()
    block_list = neighboring_blocks(block_id, kernel_size)
    for _ in block_list:
        neurons_in_block = neurons_in_the_block(cortical_area, _)
        for __ in neurons_in_block:
            neuron_list.append(__)
    return neuron_list
