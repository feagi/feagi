# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
"""
Provides functions performing statistical analysis on the Connectome and Cortical behavior
"""

# import numpy as np
# import pandas as pd

from configuration import runtime_data
from misc import disk_ops, db_handler


def cortical_area_neuron_count(cortical_area):
    """
    Returns number of Neurons in the connectome
    """
    data = runtime_data.brain[cortical_area]
    neuron_count = 0
    for key in data:
        neuron_count += 1
    return neuron_count


def connectome_neuron_count():
    total_neuron_count = 0
    for cortical_area in runtime_data.cortical_list:
        cortical_area_neuron_count(cortical_area)
        total_neuron_count += 1

    return total_neuron_count


def connectome_total_synapse_cnt(cortical_area):
    """
    Returns the total number of Neurons and Synapses for a given cortical area
    """
    data = runtime_data.brain[cortical_area]
    total_synapse_count = 0
    total_neuron_count = 0
    for neuron in data:
        total_neuron_count += 1
        for synapse in data[neuron]['neighbors']:
            total_synapse_count += 1
    return total_neuron_count, total_synapse_count


def brain_total_synapse_cnt(verbose=True):
    brain_synapse_cnt = 0
    brain_neuron_cnt = 0
    for cortical_area in runtime_data.brain:
        neuron_count, synapse_count = connectome_total_synapse_cnt(cortical_area)
        brain_neuron_cnt = brain_neuron_cnt + neuron_count
        brain_synapse_cnt = brain_synapse_cnt + synapse_count
        if verbose:
            print("For %s, the Neuron count is %i and  Synapse count is %i" % (cortical_area, neuron_count, synapse_count))
    if verbose:
        print("\nFor the entire brain, total Neuron count is %i and  total Synapse count is %i\n" % (brain_neuron_cnt, brain_synapse_cnt))
    return brain_neuron_cnt, brain_synapse_cnt


def connectome_neighbor_histogram(cortical_area):
    """
    Plots a Histogram of count of neighbor relationships per Neuron in a Cortical area
    """
    data = runtime_data.brain[cortical_area]
    for key in data:
        count = 0
        for y in data[key]['neighbors']:
            count += 1
        # raw.append([cortical_area, count])

    return


def print_cortical_stats():
    for cortical_area in runtime_data.cortical_list:
        print("%s total Neuron count: %i" % (cortical_area, connectome_neuron_count(cortical_area)))
        print("%s average synapse count per Neuron: %i" % (cortical_area, connectome_total_synapse_cnt(cortical_area)/connectome_neuron_count(cortical_area)))
    return


def cortical_xyz_range():
    cortical_list = runtime_data.cortical_list
    xyz_range = {}
    tmp_x = []
    tmp_y = []
    tmp_z = []
    for cortical_area in cortical_list:
        for neuron in runtime_data.brain[cortical_area]:
            tmp_x.append(runtime_data.brain[cortical_area][neuron]["location"][0])
            tmp_y.append(runtime_data.brain[cortical_area][neuron]["location"][1])
            tmp_z.append(runtime_data.brain[cortical_area][neuron]["location"][2])
        max_x = max(tmp_x)
        max_y = max(tmp_y)
        max_z = max(tmp_z)
        xyz_range[cortical_area] = [max_x, max_y, max_z]
    return xyz_range


def fcl_stats(genome_id):
    mongo = db_handler.MongoManagement()
    fcl_data_from_db = mongo.fcl_data(genome_id)
    fcl_stats_data = {}
    for burst in fcl_data_from_db:
        neuron_activities = burst['fcl_data']
        number_under_training = burst['number_under_training']
        if number_under_training != '':
            for activity in neuron_activities:
                cortical_area = activity[0]
                neuron_id = activity[1]
                if cortical_area not in fcl_stats_data:
                    fcl_stats_data[cortical_area] = {}
                if neuron_id not in fcl_stats_data[cortical_area]:
                    fcl_stats_data[cortical_area][neuron_id] = [0] * 10
                fcl_stats_data[cortical_area][neuron_id][number_under_training] += 1
    return fcl_stats_data


def print_fcl_stats(genome_id):
    print('*******************************************************************************************')
    print('*************************************************  ', genome_id)
    print('*******************************************************************************************')
    a = fcl_stats(genome_id)
    for _ in a:
        for __ in a[_]:
            print(_, __, a[_][__])
    print('*******************************************************************************************')
    print('*******************************************************************************************')
    print('*******************************************************************************************')



# def tbd():
#     for key in blueprint:
#         connectome_neighbor_histogram(key)
#
#     df = pd.DataFrame(raw)
#     print(df)
#     print(df[1])
#     # df.plot(kind='hist', stacked=True, bins=20)
#
#     plt.show()
#
#     return


# def cortical_synaptic_strengths(cortical_area):
#     """
#     list Neurons along with destination neuron and Synaptic strenght associated with them.
#
#     :param cortical_area:
#     :return:
#     """
#     synaptic_strengths = []
#     data = runtime_data.brain[cortical_area]
#     for key in data:
#         for neighbor in data[key]["neighbors"]:
#
#
#
#     return synaptic_strengths



