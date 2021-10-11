
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
        candidate_list.append(blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block_ref))
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


def rule_selective_block_to_block(rule_param, src_cortical_area, dst_cortical_area, src_neuron_id, z_offset):
    """
    This ad hoc rule allows for selective synaptogenesis between block neurons in the IR IPU and motor OPU
    cortical areas. The IR IPU cortical area currently consists of 3 blocks (each containing 1 neuron). Each 
    IR IPU neuron stimulates a different subset of the blocks in the motor OPU to facilitate appropriate motor 
    activation for line-tracking purposes without using neuroplasticity.
    """

    src_neuron_block_ref = blocks.block_reference_builder(
        runtime_data.brain[src_cortical_area][src_neuron_id]['soma_location'][1]
    )
    src_neuron_block_id = blocks.block_ref_2_id(src_neuron_block_ref)

    candidate_list = list()
    if src_neuron_block_id[0] is 0:
        dst_block_refs = ["1-0-5", "3-0-5"]
        for block in dst_block_refs:
            dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block)
            for dst_neuron in dst_block_neurons:
                candidate_list.append(dst_neuron)

    elif src_neuron_block_id[0] is 1:
        dst_block_refs = ["0-0-2", "1-0-2", "2-0-2", "3-0-2"]
        for block in dst_block_refs:
            dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block)
            for dst_neuron in dst_block_neurons:
                candidate_list.append(dst_neuron)

    elif src_neuron_block_id[0] is 2:
        dst_block_refs = ["0-0-5", "2-0-5"]
        for block in dst_block_refs:
            dst_block_neurons = blocks.neurons_in_the_block(cortical_area=dst_cortical_area, block_ref=block)
            for dst_neuron in dst_block_neurons:
                candidate_list.append(dst_neuron)

    return candidate_list
