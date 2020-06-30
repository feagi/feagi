"""
This module covers needed tools for synapse creation.
"""
import random
import collections
import numpy as np
from math import floor
from inf import runtime_data
from evo.neuron import block_id_gen, block_reference_builder


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][
                1] -
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][0])

    return length


class SynaptogenesisRuleManager:
    """
    Manages the rules involved with neuron synaptogenesis process.

    Needed info: Rule and rule params
    return: the func for the rule

    """
    def __init__(self, src_neuron_id, src_cortical_area, dst_cortical_area):
        self.is_candidate = False
        self.src_neuron_id = src_neuron_id
        self.rule = runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]['neighbor_locator_rule_id']
        self.rule_param_key = runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]['neighbor_locator_rule_param_id']
        self.rule_param = runtime_data.genome['neighbor_locator_rule'][self.rule][self.rule_param_key]
        self.src_cortical_area = src_cortical_area
        self.dst_cortical_area = dst_cortical_area
        self.z_offset = 0
        # todo: this check is adding ~0.5s to neuroembryogenesis process
        if 'layer_index' in runtime_data.genome['blueprint'][src_cortical_area]:
            self.z_offset = int(runtime_data.genome['blueprint'][src_cortical_area]['layer_index'])
        else:
            self.z_offset = 0

    def growth_rule_selector(self):
        """
        Provides a mapping table between rules ids defined in genome and the functions driving the rules
        Args:
            rule_id: as referenced within Genome

        Returns:
            Function corresponding the rule_id

        """
        # todo: to fix the rules
        switcher = {
            "rule_0": self.rule_neuron_to_neuron,
            "rule_1": self.rule_block_to_block,
            "rule_5": self.rule_block_to_block,
            "rule_6": self.rule_block_to_block
        }
        # Get the function from switcher dictionary
        rule_func = switcher.get(self.rule, lambda: "Invalid rule")
        return rule_func()

    def rule_neuron_to_neuron(self):
        candidate_list = list()
        # Input: neuron id of which we desire to find all candidate neurons for from another cortical region
        src_data = runtime_data.brain[self.src_cortical_area]
        dst_data = runtime_data.brain[self.dst_cortical_area]
        for dst_neuron_id in dst_data:
            if src_data[self.src_neuron_id]['soma_location'][0] == dst_data[dst_neuron_id]['soma_location'][0]:
                candidate_list.append(dst_neuron_id)
                break
        return candidate_list

    def rule_block_to_block(self):
        """
        Returns the list of neurons listed under the same block id as the source neuron
        todo: need to add ability to account for the sublayer mapping e.g. v1.1 > v2 layer 1 based on z offset
        todo: instead of a single destincation block based on the rule pass a set of blocks to make it generalizable
        Returns:

        """
        candidate_list = list()
        src_neuron_block_ref = \
            block_reference_builder(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1])

        src_neuron_block_ref = block_z_offset(block_ref=src_neuron_block_ref, offset=self.z_offset)

        if self.rule_param == 1:
            try:
                for neuron in runtime_data.block_dic[self.dst_cortical_area][src_neuron_block_ref]:
                    candidate_list.append(neuron)
            except KeyError:
                pass
        elif self.rule_param in [3, 5, 7, 9]:
            candidate_list = neurons_in_block_neighborhood(cortical_area=self.dst_cortical_area,
                                                           block_ref=src_neuron_block_ref, kernel_size=self.rule_param)

        else:
            print(self.rule_param, "is an invalid parameter for block to block mapping")

        return candidate_list


def block_z_offset(block_ref, offset):
    """
    Offsets the z coordinate of the block reference by the value defined by "offset"
    Note: There is a risk that the new offset value exceed defined block boundaries of a given cortical area

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


def neurons_in_block_neighborhood(cortical_area, block_ref, kernel_size=3):
    """
    Provides the list of all neurons within the surrounding blocks given the kernel size with default being 3
    """
    candidate_list = list()
    block_list = neighboring_blocks(block_ref, kernel_size)
    # print("Block List: ", block_list)
    for _ in block_list:
        neurons_in_block = neurons_in_the_block(cortical_area=cortical_area, block_ref=block_reference_builder(_))
        for __ in neurons_in_block:
            candidate_list.append(__)
    return candidate_list


def neurons_in_the_block(cortical_area, block_ref):
    """
    Generates a list of Neurons in the given block
    block_id to be entered as [x,y,z]
    """
    try:
        return runtime_data.block_dic[cortical_area][block_ref]
    except KeyError:
        return []


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


def neighbor_finder_intercortical(cortical_area, cortical_area_dst, src_neuron_id):
    """
    Finds a list of candidate Neurons from another Cortical area to build Synapse with for a given Neuron
    """

    rule_manager = SynaptogenesisRuleManager(src_neuron_id=src_neuron_id, src_cortical_area=cortical_area,
                                             dst_cortical_area=cortical_area_dst)

    candidate_list = rule_manager.growth_rule_selector()
    return candidate_list


def neighbor_builder_intercortical(cortical_area, brain, genome, brain_gen, cortical_area_dst, rule, rule_param,
                                   postsynaptic_current=1.1):
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
        # Cycle thru the neighbor_candidate_list and establish Synapses
        neighbor_candidates = neighbor_finder_intercortical(cortical_area=cortical_area,
                                                            cortical_area_dst=cortical_area_dst,
                                                            src_neuron_id=src_id)
        for dst_id in neighbor_candidates:
            # Throw a dice to decide for synapse creation. This is to limit the amount of synapses.
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


# class UnusedFuncs:
#     def neighbor_builder_intracortical(cortical_area, brain, genome, brain_gen, rule_id, rule_param,
#                                        postsynaptic_current):
#         """
#         Function responsible for crawling through Neurons and deciding where to build Synapses
#         """
#         # Need to figure how to build a Neighbor relationship between Neurons
#         #    1. Should it be based on the proximity?
#         #    2. Should it be manually set? It wont be scalable
#         #    3. How can it be based on a Guide function?
#         # This function will utilize the Synapse function and based on an algorithm will create the relationships
#
#         # Algorithm:
#         #    1. Ask for a direction
#         #    2. For each neuron in path find a list of eligible candidates
#         #    3. Update connectome to the candidates become neighbors of the source neuron
#
#         # todo: Warning: Need to watch for the side effects on this line to make sure its not overwriting values
#         # to accommodate the new namespace used by multiprocessing
#         if brain_gen:
#             runtime_data.brain = brain
#             runtime_data.genome = genome
#
#         synapse_count = 0
#         for src_id in runtime_data.brain[cortical_area]:
#             # Cycle thru the neighbor_candidate_list and establish Synapses
#             neighbor_candidates = neighbor_finder_intracortical(src_id, rule_id, rule_param)
#             for dst_id in neighbor_candidates:
#                 synapse(src_id, cortical_area, dst_id, postsynaptic_current)
#                 synapse_count += 1
#                 # print("Made a Synapse between %s and %s" % (src_id, dst_id))
#         if brain_gen:
#             brain = runtime_data.brain
#             runtime_data.genome = genome
#
#             synapse_count2 = 0
#             for area in brain:
#                 for neuron in brain[area]:
#                     for connection in brain[area][neuron]['neighbors']:
#                         synapse_count2 += 1
#         else:
#             brain = {}
#         return synapse_count, brain
#
#     def connectome_location_data(cortical_area):
#         """
#         Extracts Neuron locations and neighbor relationships from the connectome
#         """
#
#         neuron_locations = []
#         for key in runtime_data.brain[cortical_area]:
#             location_data = runtime_data.brain[cortical_area][key]["soma_location"][0]
#             location_data.append(runtime_data.brain[cortical_area][key]["cumulative_fire_count"])
#             neuron_locations.append(location_data)
#
#         return neuron_locations
#
#     def neurons_in_same_block(cortical_area, neuron_id):
#         """
#         Generates a list of Neurons in the same block as the given one
#         """
#         neuron_list = []
#         for _ in runtime_data.brain[cortical_area]:
#             if runtime_data.brain[cortical_area][_]['block'] == \
#                     runtime_data.brain[cortical_area][neuron_id]['block']:
#                 if _ != neuron_id:
#                     neuron_list.append(_)
#         return neuron_list
#
#     def dst_projection_center(cortical_area_src, neuron_id, cortical_area_dst):
#         """
#         Returns the coordinates of a neuron projected into a target Cortical layer
#         """
#
#         # Find relative coordinates on the source and destination side
#         src_lengths = cortical_area_lengths(cortical_area_src)
#         dst_lengths = cortical_area_lengths(cortical_area_dst)
#         coordinate_scales = [a / b for a, b in zip(dst_lengths, src_lengths)]
#
#         x_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][0]
#         y_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][1]
#         z_coordinate_src = runtime_data.brain[cortical_area_src][neuron_id]["soma_location"][0][2]
#
#         dst_projection_center = list()
#         dst_projection_center.append(x_coordinate_src * coordinate_scales[0])
#         dst_projection_center.append(y_coordinate_src * coordinate_scales[1])
#         dst_projection_center.append(z_coordinate_src * coordinate_scales[2])
#
#         return dst_projection_center
#

#
#     def neuron_finder2(cortical_area, location):
#
#         block_id = block_id_gen(coordinate=location, cortical_area=cortical_area)
#
#         neuron_list = neurons_in_the_block(block_id)
#
#         return neuron_list
#
#
#     def projection_center(self):
#         # todo: locations to be updated and made generic allowing multiple locations
#         src_coord = np.array(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][0])
#         dst_coord = np.array(runtime_data.brain[self.dst_cortical_area][self.dst_neuron_id]['dendrite_locations'][0][0])
#
#         src_blk = np.array(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1])
#         dst_blk = np.array(runtime_data.brain[self.dst_cortical_area][self.dst_neuron_id]['dendrite_locations'][0][1])
#
#         # Find relative coordinates on the source and destination side
#         src_lengths = cortical_area_lengths(self.src_cortical_area)
#         dest_lengths = cortical_area_lengths(self.dst_cortical_area)
#         coordinate_scales = [a / b for a, b in zip(dest_lengths, src_lengths)]
#
#         x_coordinate_src = src_coord[0]
#         y_coordinate_src = src_coord[1]
#         z_coordinate_src = src_coord[2]
#         x_coordinate_dst = dst_coord[0]
#         y_coordinate_dst = dst_coord[1]
#         z_coordinate_dst = dst_coord[2]
#
#         projection_center = np.array([])
#         projection_center = np.append(projection_center, x_coordinate_src * coordinate_scales[0])
#         projection_center = np.append(projection_center, y_coordinate_src * coordinate_scales[1])
#         projection_center = np.append(projection_center, z_coordinate_src * coordinate_scales[2])
#         return projection_center
#
#     def neighbor_finder_intracortical(cortical_area, neuron_id, rule, rule_param):
#         """
#         A set of math functions allowing to detect the eligibility of a Neuron to become neighbor
#         :param cortical_area
#         :param neuron_id:
#         :param rule:
#         :param rule_param:
#         :return:
#         """
#         if cortical_area == 'utf8_memory':
#             print('rule=', rule)
#         # Input: neuron id of which we desire to find all candidate neurons for
#         neighbor_candidates = []
#
#         # Narrow down the search scope to only few blocks
#         # neighbors_in_block = neurons_in_block_neighborhood(cortical_area, neuron_id, kernel_size=1)
#         neuron_block = runtime_data.brain[cortical_area][neuron_id]['block']
#         block_reference = str(neuron_block[0]) + '-' + str(neuron_block[1]) + '-' + str(neuron_block[2])
#         neighbors_in_block = runtime_data.block_dic[cortical_area][block_reference]
#
#         candidacy_check = IsCandidate(rule_id=rule,
#                                       rule_param=rule_param,
#                                       cortical_area_src=cortical_area,
#                                       cortical_area_dst=cortical_area,
#                                       src_neuron_id=neuron_id)
#
#         for dst_neuron_id in neighbors_in_block:
#             if candidacy_check.growth_rule_applicator(dst_neuron_id):
#                 neighbor_candidates.append(dst_neuron_id)
#
#         return neighbor_candidates
#
#     class IsCandidate:
#         """
#         Identifies if a given neuron in the destination cortical area is a match for the source neuron based on a rule
#         """
#         def __init__(self, src_neuron_id, dst_neuron_id, rule, rule_param, src_cortical_area, dst_cortical_area):
#             self.is_candidate = False
#             self.src_neuron_id = src_neuron_id
#             self.dst_neuron_id = dst_neuron_id
#             self.rule = rule
#             self.rule_param = rule_param
#             self.src_cortical_area = src_cortical_area
#             self.dst_cortical_area = dst_cortical_area
#
#         def projection_center(self):
#             # todo: locations to be updated and made generic allowing multiple locations
#             src_coord = np.array(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][0])
#             dst_coord = np.array(runtime_data.brain[self.dst_cortical_area][self.dst_neuron_id]['dendrite_locations'][0][0])
#
#             src_blk = np.array(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1])
#             dst_blk = np.array(runtime_data.brain[self.dst_cortical_area][self.dst_neuron_id]['dendrite_locations'][0][1])
#
#             # Find relative coordinates on the source and destination side
#             src_lengths = cortical_area_lengths(self.src_cortical_area)
#             dest_lengths = cortical_area_lengths(self.dst_cortical_area)
#             coordinate_scales = [a / b for a, b in zip(dest_lengths, src_lengths)]
#
#             x_coordinate_src = src_coord[0]
#             y_coordinate_src = src_coord[1]
#             z_coordinate_src = src_coord[2]
#             x_coordinate_dst = dst_coord[0]
#             y_coordinate_dst = dst_coord[1]
#             z_coordinate_dst = dst_coord[2]
#
#             projection_center = np.array([])
#             projection_center = np.append(projection_center, x_coordinate_src * coordinate_scales[0])
#             projection_center = np.append(projection_center, y_coordinate_src * coordinate_scales[1])
#             projection_center = np.append(projection_center, z_coordinate_src * coordinate_scales[2])
#             return projection_center
#
#         # Defining synapse growth rules
#         def rule_block_2_block(self):
#             """
#             Older implementation of the block to block mapping/
#
#             todo: This function does not belong here. Planning to build a new class/method to return candidate neurons
#             """
#             neighbor_candidates = []
#             # todo: rules are currently not being used for synapse creation. Would there be value? Assess!!
#             neuron_block_src = runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1]
#             if 'layer_index' in runtime_data.genome['blueprint'][self.src_cortical_area]:
#                 src_block_index = runtime_data.genome['blueprint'][self.src_cortical_area]['layer_index'] - 1
#             else:
#                 src_block_index = 0
#             # todo: make the next line generic. It would become important when complex cortical areas are introduced
#             if self.dst_cortical_area == 'vision_v2':
#                 block_reference = block_reference_builder([neuron_block_src[0], neuron_block_src[1], src_block_index])
#                 # print("block_reference:", block_reference)
#             else:
#                 block_reference = block_reference_builder(neuron_block_src)
#
#             z_block_boundary = \
#                 runtime_data.genome['blueprint'][self.dst_cortical_area]['neuron_params']['block_boundaries'][2]
#
#             if z_block_boundary > 1:
#                 if block_reference in runtime_data.block_dic[self.dst_cortical_area]:
#                     for dst_neuron in runtime_data.block_dic[self.dst_cortical_area][block_reference]:
#                         if runtime_data.brain[self.dst_cortical_area][dst_neuron]['soma_location'][1][2] == src_block_index:
#                             neighbor_candidates.append(dst_neuron)
#
#             elif block_reference in runtime_data.block_dic[self.dst_cortical_area]:
#                 for neighbor in runtime_data.block_dic[self.dst_cortical_area][block_reference]:
#                     neighbor_candidates.append(neighbor)
#             return neighbor_candidates
#
#         def rule_block_to_block(self):
#             src_neuron_block = runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1]
#             src_neuron_block_ref = block_reference_builder(src_neuron_block)
#             try:
#                 if self.dst_neuron_id in runtime_data.block_dic[self.dst_cortical_area][src_neuron_block_ref]:
#                     return True
#                 return False
#             except KeyError:
#                 return False
#
#         def rule_tbd_2(self):
#             return
#
#         def growth_rule_applicator(self):
#             # todo: to fix the rules
#             switcher = {
#                 "rule_0": self.rule_block_to_block,
#                 "rule_1": self.rule_block_to_block,
#                 "rule_6": self.rule_block_to_block,
#                 "rule_5": self.rule_block_to_block
#             }
#             # Get the function from switcher dictionary
#             func = switcher.get(self.rule, lambda: "Invalid rule")
#             # Execute the function
#             is_candidate = func()
#             return is_candidate
#
