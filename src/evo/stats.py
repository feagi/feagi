
# Copyright 2019 The FEAGI Authors. All Rights Reserved.
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
Provides functions performing statistical analysis on the Connectome and Cortical behavior
"""
from evo.blocks import *
from inf import runtime_data, db_handler



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
    # mongo = db_handler.MongoManagement()
    fcl_data_from_db = runtime_data.mongo.fcl_data(genome_id)
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


def candidate_list_counter(candidate_list):
    count = 0
    for cortical_area in candidate_list:
        count += len(candidate_list[cortical_area])
        # print("&&$$%%>>", cortical_area, len(candidate_list[cortical_area]))
    return count


def list_upstream_neuron_count_for_digits(digit='all', mode=0):
    # function_start_time = datetime.now()
    results = []
    fcl_results = []

    if digit == 'all':
        for _ in range(10):
            # results.append([_, len(list_
            #                        upstream_neurons('utf8_memory', runtime_data.top_10_utf_memory_neurons[_][1]))])
            neuron_id = runtime_data.top_10_utf_memory_neurons[_][1]
            if 'utf8_memory' in runtime_data.upstream_neurons:
                if neuron_id in runtime_data.upstream_neurons['utf8_memory']:
                    if 'vision_memory' in runtime_data.upstream_neurons['utf8_memory'][neuron_id]:
                        results.append([_,
                                        len(runtime_data.upstream_neurons['utf8_memory'][neuron_id]['vision_memory'])])
                        if runtime_data.upstream_neurons['utf8_memory'][neuron_id]['vision_memory']:
                            counter = 0
                            for neuron in runtime_data.upstream_neurons['utf8_memory'][neuron_id]['vision_memory']:
                                if neuron in runtime_data.fire_candidate_list['vision_memory']:
                                    counter += 1
                            fcl_results.append([_, counter])
                        else:
                            fcl_results.append([_, 0])
                    else:
                        results.append([_, 0])
                        fcl_results.append([_, 0])
                else:
                    results.append([_, 0])
                    fcl_results.append([_, 0])
            else:
                results.append([_, 0])
                fcl_results.append([_, 0])
    else:
        neuron_id = runtime_data.top_10_utf_memory_neurons[digit][1]
        if 'utf8_memory' in runtime_data.upstream_neurons:
            if neuron_id in runtime_data.upstream_neurons['utf8_memory']:
                if 'vision_memory' in runtime_data.upstream_neurons['utf8_memory'][neuron_id]:
                    results.append([digit,
                                    len(runtime_data.upstream_neurons['utf8_memory'][neuron_id]['vision_memory'])])
                else:
                    results.append([digit, 0])
            else:
                results.append([digit, 0])
        else:
            results.append([digit, 0])
    # print("Timing : list_upstream_neuron_count_for_digits:", datetime.now()-function_start_time)

    if mode == 0:
        print("&& && & &&& && && & : The mode is == 0")
        return results
    else:
        print("&& && & &&& && && & : The mode is != 0")
        return results, fcl_results


def list_top_n_utf_memory_neurons(cortical_area, n):
    neuron_list = []
    counter = ord('0')
    the_other_counter = 0
    for neuron_id in runtime_data.brain[cortical_area]:
        if int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2]) == counter:
            neuron_list.append([int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])-48, neuron_id])
            counter += 1
            the_other_counter += 1
            if the_other_counter == n:
                return neuron_list
    print("ERROR: Something went wrong in list_top_n_utf_memory_neurons")


def list_common_upstream_neurons(neuron_a, neuron_b):
    common_neuron_list = []

    try:
        neuron_a_upstream_neurons = runtime_data.upstream_neurons['utf8_memory'][neuron_a]['vision_memory']
        neuron_b_upstream_neurons = runtime_data.upstream_neurons['utf8_memory'][neuron_b]['vision_memory']
        for neuron in neuron_a_upstream_neurons:
            if neuron in neuron_b_upstream_neurons:
                common_neuron_list.append(neuron)
        return common_neuron_list

    except:
        pass


def block_dict_summary(block_dict, cortical_area=[], verbose=False):
    """
    Returns a summary report of block_dict contents

    {
    cortical_area_1: ((block_ref_1, neuron_count), (block_ref_2, neuron_count), (block_ref_3, neuron_count),...),
    cortical_area_2: ((block_ref_1, neuron_count), (block_ref_2, neuron_count), (block_ref_3, neuron_count),...),
    ...
    }
    """

    stats = dict()

    for area in block_dict:
        if area in cortical_area or not cortical_area:
            stats[area] = set()
            for block in block_dict[area]:
                stats[area].add((block, len(block)))

    if verbose:
        for area in stats:
            print(area)
            for block in stats[area]:
                print("----", block[0], block[1])

    return stats


def opu_activity_report(cortical_area):
    """
    Returns percentage of neuronal activity in each block
    """
    block_boundaries = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    report = dict()
    # The X axis of the OPU cortical areas are designated as device indicators
    for device in range(block_boundaries[0]):
        device_activity_report = []
        # The Y axis of OPU is reserved for capturing second dimension of OPU property
        for secondary_variable in range(block_boundaries[1]):
            z_report = []
            # The Z axis is captures the primary property of the OPU such as speed, color, angle, etc
            for primary_variable in range(block_boundaries[2]):
                block_ref = block_reference_builder([device, secondary_variable, primary_variable])
                z_report.append(percent_active_neurons_in_block(block_ref=block_ref,
                                                              cortical_area=cortical_area))
                # device_activity_report.append(percent_active_neurons_in_block(block_ref=block_ref,
                #                                                 cortical_area=cortical_area))
            device_activity_report.append(z_report)
        report[device] = device_activity_report
    return report


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



