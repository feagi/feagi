
import random
import collections
import numpy as np
from math import floor
from inf import runtime_data
from evo.neuron import block_id_gen


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][
                1] -
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][0])

    return length


def dst_projection_center(cortical_area_src, neuron_id, cortical_area_dst):
    """
    Returns the coordinates of a neuron projected into a target Cortical layer
    """

    # Find relative coordinates on the source and destination side
    src_lengths = cortical_area_lengths(cortical_area_src)
    dst_lengths = cortical_area_lengths(cortical_area_dst)
    coordinate_scales = [a / b for a, b in zip(dst_lengths, src_lengths)]

    x_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][0]
    y_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][1]
    z_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][2]

    dst_projection_center = list()
    dst_projection_center.append(x_coordinate_src * coordinate_scales[0])
    dst_projection_center.append(y_coordinate_src * coordinate_scales[1])
    dst_projection_center.append(z_coordinate_src * coordinate_scales[2])

    return dst_projection_center


class IsCandidate:
    def __init__(self, rule_id, rule_param, cortical_area_src, cortical_area_dst, dst_neuron_id, src_neuron_id):

        # todo: locations to be updated and made generic allowing multiple locations
        self.src_coord = np.array(runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location'][0])
        self.dst_coord = np.array(runtime_data.brain[cortical_area_dst][dst_neuron_id]['dendrite_locations'][0][0])

        self.src_blk = np.array(runtime_data.brain[cortical_area_src][src_neuron_id]['soma_location'][1])
        self.dst_blk = np.array(runtime_data.brain[cortical_area_dst][dst_neuron_id]['dendrite_locations'][0][1])

        # Find relative coordinates on the source and destination side
        src_lengths = cortical_area_lengths(cortical_area_src)
        dest_lengths = cortical_area_lengths(cortical_area_dst)
        coordinate_scales = [a / b for a, b in zip(dest_lengths, src_lengths)]

        x_coordinate_src = self.src_coord[0]
        y_coordinate_src = self.src_coord[1]
        z_coordinate_src = self.src_coord[2]
        x_coordinate_dst = self.dst_coord[0]
        y_coordinate_dst = self.dst_coord[1]
        z_coordinate_dst = self.dst_coord[2]

        self.projection_center = np.array([])
        self.projection_center = np.append(self.projection_center, x_coordinate_src * coordinate_scales[0])
        self.projection_center = np.append(self.projection_center, y_coordinate_src * coordinate_scales[1])
        self.projection_center = np.append(self.projection_center, z_coordinate_src * coordinate_scales[2])

        self.is_candidate = False
        self.rule_id = rule_id
        self.rule_param = rule_param

    # Defining synapse growth rules
    def rule_block_to_block(self):
        return

    def rule_tbd_1(self):
        return

    def rule_tbd_2(self):
        return

    def growth_rule_applicator(self):
        switcher = {
            "rule_0": self.rule_block_to_block(),
            "rule_1": self.rule_tbd_1(),
            "rule_2": self.rule_tbd_2()
        }
        # Get the function from switcher dictionary
        func = switcher.get(self.rule_id, lambda: "Invalid month")
        # Execute the function
        return func()


def synapse(cortical_area, src_id, dst_cortical_area, dst_id, postsynaptic_current):
    """
    Function responsible for creating a synapse between a neuron and another one. In reality a single neuron can
    have many synapses with another individual neuron. Here we use synaptic strength to simulate the same.
    Note: Synapse association is captured on the Source Neuron side within Connectome

    # Input: The id for source and destination Neuron plus the parameter defining connection strength
    # Source provides the Axon and connects to Destination Dendrite
    # postsynaptic_current is intended to provide the level of synaptic strength
    """

    # # Check to see if the source and destination ids are valid if not exit the function
    # if src_id not in runtime_data.brain[src_cortical_area]:
    #     print("Source or Destination neuron not found")
    #     return

    runtime_data.brain[cortical_area][src_id]["neighbors"][dst_id] = \
        {"cortical_area": dst_cortical_area, "postsynaptic_current": postsynaptic_current}

    return


def neighbor_finder_intracortical(cortical_area, neuron_id, rule, rule_param):
    """
    A set of math functions allowing to detect the eligibility of a Neuron to become neighbor
    :param neuron_id:
    :param rule:
    :param rule_param:
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

    candidacy_check = IsCandidate(rule_id=rule,
                                  rule_param=rule_param,
                                  cortical_area_src=cortical_area,
                                  cortical_area_dst=cortical_area,
                                  src_neuron_id=neuron_id)

    for dst_neuron_id in neighbors_in_block:
        if candidacy_check(dst_neuron_id):
            neighbor_candidates.append(dst_neuron_id)

    return neighbor_candidates


def neighbor_reset(cortical_area):
    """
    This function deletes all the neighbor relationships in the connectome
    """

    for key in runtime_data.brain[cortical_area]:
        runtime_data.brain[cortical_area][key]["neighbors"] = {}

    return


def neighbor_builder_intracortical(cortical_area, brain, genome, brain_gen, rule_id, rule_param, postsynaptic_current):
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
        neighbor_candidates = neighbor_finder_intracortical(src_id, rule_id, rule_param)
        for dst_id in neighbor_candidates:
            synapse(src_id, cortical_area, dst_id, postsynaptic_current)
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


def neighbor_finder_intercortical(cortical_area, cortical_area_dst, src_neuron_id, rule, rule_param):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    # Input: neuron id of which we desire to find all candidate neurons for from another cortical region
    src_data = runtime_data.brain[cortical_area]
    dst_data = runtime_data.brain[cortical_area_dst]
    neighbor_candidates = []

    if runtime_data.genome['blueprint'][cortical_area_dst]['location_generation_type'] == 'sequential':
        for dst_neuron_id in dst_data:
            if src_data[src_neuron_id]['soma_location'][0] == dst_data[dst_neuron_id]['soma_location'][0]:
                neighbor_candidates.append(dst_neuron_id)

    # else:
    #     for dst_neuron_id in dst_data:
    #         if src_data[src_neuron_id]['soma_location'][1] == dst_data[dst_neuron_id]['soma_location'][1]:
    #             neighbor_candidates.append(dst_neuron_id)

    else:
        # todo: rules are currently not being used for synapse creation. Would there be value? Assess!!
        neuron_block_src = runtime_data.brain[cortical_area][src_neuron_id]['soma_location'][1]
        if 'layer_index' in runtime_data.genome['blueprint'][cortical_area]:
            block_index = runtime_data.genome['blueprint'][cortical_area]['layer_index'] - 1
        else:
            block_index = 0
        # todo: make the next line generic. It would become important when complex cortical areas are introduced
        if cortical_area_dst == 'vision_v2':
            block_reference = str(neuron_block_src[0]) + '-' + str(neuron_block_src[1]) + '-' + str(block_index)
            # print("block_reference:", block_reference)
        else:
            block_reference = str(neuron_block_src[0]) + '-' + str(neuron_block_src[1]) + '-' + str(
                neuron_block_src[2])

        z_block_boundary = runtime_data.genome['blueprint'][cortical_area_dst]['neuron_params']['block_boundaries'][
            2]

        if z_block_boundary > 1:
            if block_reference in runtime_data.block_dic[cortical_area_dst]:
                for dst_neuron in runtime_data.block_dic[cortical_area_dst][block_reference]:
                    if runtime_data.brain[cortical_area_dst][dst_neuron]['soma_location'][1][2] == block_index:
                        neighbor_candidates.append(dst_neuron)

        elif block_reference in runtime_data.block_dic[cortical_area_dst]:
            for neighbor in runtime_data.block_dic[cortical_area_dst][block_reference]:
                neighbor_candidates.append(neighbor)

    return neighbor_candidates


def neighbor_builder_intercortical(cortical_area, brain, genome, brain_gen, cortical_area_dst, rule, rule_param,
                                   postsynaptic_current=1.1):
    """
    Crawls thru a Cortical area and builds Synapses with External Cortical Areas
    """
    # to accommodate the new namespace used by multiprocessing
    if brain_gen:
        runtime_data.brain = brain
        runtime_data.genome = genome

    synapse_count = 0
    for src_id in runtime_data.brain[cortical_area]:
        # Cycle thru the neighbor_candidate_list and establish Synapses
        neighbor_candidates = neighbor_finder_intercortical(cortical_area=cortical_area,
                                                            cortical_area_dst=cortical_area_dst,
                                                            src_neuron_id=src_id, rule=rule, rule_param=rule_param)
        for dst_id in neighbor_candidates:
            # Through a dice to decide for synapse creation. This is to limit the amount of synapses.
            if random.randrange(1, 100) < \
                    runtime_data.genome['blueprint'][cortical_area_dst]['synapse_attractivity']:
                # Connect the source and destination neuron via creating a synapse
                synapse(cortical_area=cortical_area, src_id=src_id, dst_cortical_area=cortical_area_dst, dst_id=dst_id,
                        postsynaptic_current=postsynaptic_current)
                synapse_count += 1
                # print("Made a Synapse between %s and %s" % (src_id, dst_id))

    if brain_gen:
        brain = runtime_data.brain
    else:
        brain = {}
    return synapse_count, brain


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


def neuron_finder2(cortical_area, location):

    block_id = block_id_gen(coordinate=location, cortical_area=cortical_area)

    neuron_list = neurons_in_the_block(block_id)

    return neuron_list


def connectome_location_data(cortical_area):
    """
    Extracts Neuron locations and neighbor relationships from the connectome
    """

    neuron_locations = []
    for key in runtime_data.brain[cortical_area]:
        location_data = runtime_data.brain[cortical_area][key]["soma_location"][0]
        location_data.append(runtime_data.brain[cortical_area][key]["cumulative_fire_count"])
        neuron_locations.append(location_data)

    return neuron_locations


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


def neurons_in_block_neighborhood_2(block_id, kernel_size=3):
    """
    Provides the list of all neurons within the surrounding blocks given the kernel size with default being 3
    """
    neuron_list = list()
    block_list = neighboring_blocks(block_id, kernel_size)
    for _ in block_list:
        neurons_in_block = neurons_in_the_block(_)
        for __ in neurons_in_block:
            neuron_list.append(__)
    return neuron_list

