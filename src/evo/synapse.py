"""
This module covers needed tools for synapse creation.
"""
import random
from evo.synaptogenesis_rules import *
from evo.blocks import block_reference_builder
from inf import runtime_data


def cortical_area_lengths(cortical_area):
    length = []
    coordinates = ['x', 'y', 'z']
    for _ in coordinates:
        length.append(
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][
                1] -
            runtime_data.genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries'][_][0])

    return length


# todo: Externalize all the rules to a separate module
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
        self.src_neuron_block_ref = \
            block_reference_builder(runtime_data.brain[self.src_cortical_area][self.src_neuron_id]['soma_location'][1])

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
            "rule_0": rule_neuron_to_neuron,
            "rule_1": rule_block_to_block,
            "rule_5": rule_block_to_block,
            "rule_6": rule_block_to_block,
            "rule_7": rule_block_distributor
        }
        # Get the function from switcher dictionary
        rule_func = switcher.get(self.rule, lambda: "Invalid rule")
        return rule_func(rule_param=self.rule_param,
                         src_cortical_area=self.src_cortical_area,
                         dst_cortical_area=self.dst_cortical_area,
                         src_neuron_id=self.src_neuron_id,
                         z_offset=self.z_offset)


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


def neighbor_builder(cortical_area, brain, genome, brain_gen, cortical_area_dst, rule, rule_param, postsynaptic_current=1.1):
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

        # if cortical_area == "infrared_reducer":
        #     print(neighbor_candidates)

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
