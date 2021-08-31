
from math import floor
from inf import runtime_data


# todo: rename block to voxel

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
    except KeyError:
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
        neurons_in_block = neurons_in_the_block(cortical_area=cortical_area, block_ref=block_reference_builder(_))
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
    for z in range(block_boundaries[0]):
        block_ref_list.append(block_reference_builder([x_ref, y_ref, z]))
    return block_ref_list
