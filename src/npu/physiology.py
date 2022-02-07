
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

import json
from collections import deque
from evo.neuron import block_reference_builder
from evo.synapse import synapse
from evo.blocks import neurons_in_the_block
from inf import runtime_data, settings
from cython_lib import neuron_functions_cy as cy


def activation_function(postsynaptic_current):
    # print("PSC: ", postsynaptic_current)
    return postsynaptic_current


def reset_cumulative_counters(cortical_area, neuron_id):
    runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] = 0
    runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] = runtime_data.burst_count
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = runtime_data.burst_count
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count"] += 1
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count_inst"] += 1
    # Condition to increase the consecutive fire count
    if runtime_data.burst_count == runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] + 1:
        runtime_data.brain[cortical_area][neuron_id]["consecutive_fire_cnt"] += 1


def neuron_pre_fire_processing(cortical_area, neuron_id, degenerate=0):
    """This function initiate the firing of Neuron in a given cortical area"""

    neighbor_list = list()

    # Setting Destination to the list of Neurons connected to the firing Neuron
    try:
        neighbor_list = runtime_data.brain[cortical_area][neuron_id]["neighbors"]

    except KeyError:
        print(settings.Bcolors.RED + "KeyError on accessing neighbor_list while firing a neuron" +
              settings.Bcolors.ENDC)

    # After neuron fires all cumulative counters on source gets reset
    reset_cumulative_counters(cortical_area=cortical_area, neuron_id=neuron_id)

    neighbor_count = len(neighbor_list)

    # Updating downstream neurons
    for dst_neuron_id in neighbor_list:
        # Timing the update function
        # update_start_time = datetime.now()

        dst_cortical_area = \
            runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["cortical_area"]
        postsynaptic_current = \
            runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"]

        # TODO: create separate asynchronous process that performs maintenance-like operations
        """# (apoptosis, synaptic pruning) during brain IPU inactivity based on certain cortical area
        # conditions being met (ex: postsynaptic current degenerates to 0, exceeds bursting lifespan, etc.)
        # to avoid adding more conditional statements to this function (for performance reasons)"""

        # reduce neuron postsynaptic current by degeneration value defined in genome (if applicable)
        runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] -= degenerate

        neuron_output = activation_function(postsynaptic_current)

        # Update function
        # todo: (neuron_output/neighbor_count) needs to be moved outside the loop for efficiency
        dst_neuron_obj = runtime_data.brain[dst_cortical_area][dst_neuron_id]

        dst_neuron_obj["membrane_potential"] = \
            cy.neuron_update((neuron_output/neighbor_count),
                             runtime_data.burst_count,
                             max(dst_neuron_obj["last_membrane_potential_reset_burst"],
                             dst_neuron_obj["last_burst_num"]),
                             runtime_data.genome["blueprint"][dst_cortical_area]["neuron_params"]["leak_coefficient"],
                             dst_neuron_obj["membrane_potential"])

        # Update the fire_queue that holds a temporary list of all updated neurons across the brain during a burst
        if dst_cortical_area not in runtime_data.fire_queue:
            runtime_data.fire_queue[dst_cortical_area] = dict()
        if dst_neuron_id not in runtime_data.fire_queue[dst_cortical_area]:
            runtime_data.fire_queue[dst_cortical_area][dst_neuron_id] = list()
            runtime_data.fire_queue[dst_cortical_area][dst_neuron_id] = [None, None]
        # Storing the membrane potential of the updated neuron
        runtime_data.fire_queue[dst_cortical_area][dst_neuron_id][0] = dst_neuron_obj["membrane_potential"]
        # Storing the firing threshold of the updated neuron
        runtime_data.fire_queue[dst_cortical_area][dst_neuron_id][1] = dst_neuron_obj["firing_threshold"]


def neuron_prop(cortical_area, neuron_id):
    """This function accepts neuron id and returns neuron properties"""

    data = runtime_data.brain[cortical_area]

    if runtime_data.parameters["Switches"]["verbose"]:
        print('Listing Neuron Properties for %s:' % neuron_id)
        print(json.dumps(data[neuron_id], indent=3))
    return data[neuron_id]


def neuron_neighbors(cortical_area, neuron_id):
    """This function accepts neuron id and returns the list of Neuron neighbors"""

    data = runtime_data.brain[cortical_area]

    if runtime_data.parameters["Switches"]["verbose"]:
        print('Listing Neuron Neighbors for %s:' % neuron_id)
        print(json.dumps(data[neuron_id]["neighbors"], indent=3))
    return data[neuron_id]["neighbors"]


def snooze_till(cortical_area, neuron_id, burst_id):
    """ Acting as an inhibitory neurotransmitter to suppress firing of neuron till a later burst_manager

    *** This function instead of inhibitory behavior is more inline with Neuron Refractory period

    """
    runtime_data.brain[cortical_area][neuron_id]["snooze_till_burst_num"] \
        = burst_id + runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["snooze_length"]
    # print("%s : %s has been snoozed!" % (cortical_area, neuron_id))
    return


def exhibit_pain():
    if runtime_data.pain_flag:
        print("*******************************************************************")
        print("*******************************************************************")
        print("*********************                                 *************")
        print("*******************    Pain -- Pain -- Pain -- Pain     ***********")
        print("*********************                                 *************")
        print("*******************************************************************")
        print("*******************************************************************")


def trigger_pain():
    exhibit_pain()
    for neuron in runtime_data.brain['pain']:
        runtime_data.future_fcl['pain'].add(neuron)


def pruner(pruning_data):
    """
    Responsible for pruning unused connections between neurons
    """
    cortical_area_src, src_neuron_id, cortical_area_dst, dst_neuron_id = pruning_data
    runtime_data.brain[cortical_area_src][src_neuron_id]['neighbors'].pop(dst_neuron_id, None)

    runtime_data.upstream_neurons[cortical_area_dst][dst_neuron_id][cortical_area_src].remove(src_neuron_id)
    if dst_neuron_id in runtime_data.temp_neuron_list:
        runtime_data.temp_neuron_list.remove(dst_neuron_id)


# todo: performance bottleneck; cythonize
def average_postsynaptic_current(cortical_area):
    count = 0
    total = 0
    for neuron in runtime_data.brain[cortical_area]:
        for neighbor in runtime_data.brain[cortical_area][neuron]['neighbors']:
            count += 1
            total += runtime_data.brain[cortical_area][neuron]['neighbors'][neighbor]['postsynaptic_current']
    if count > 0:
        avg_postsynaptic_current = total / count
    else:
        avg_postsynaptic_current = 0
    return avg_postsynaptic_current


def prune_all_candidates():
    while runtime_data.prunning_candidates:
        prune_candidate = runtime_data.prunning_candidates.pop()
        pruner(prune_candidate)


def list_upstream_neurons(cortical_area, neuron_id):
    if cortical_area in runtime_data.upstream_neurons:
        if neuron_id in runtime_data.upstream_neurons[cortical_area]:
            return runtime_data.upstream_neurons[cortical_area][neuron_id]
    return {}


def update_upstream_db(src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id):
    # if dst_cortical_area not in runtime_data.upstream_neurons:
    #     runtime_data.upstream_neurons[dst_cortical_area] = {}
    if dst_neuron_id not in runtime_data.upstream_neurons[dst_cortical_area]:
        runtime_data.upstream_neurons[dst_cortical_area][dst_neuron_id] = {}
    if src_cortical_area not in runtime_data.upstream_neurons[dst_cortical_area][dst_neuron_id]:
        runtime_data.upstream_neurons[dst_cortical_area][dst_neuron_id][src_cortical_area] = set()
    if src_neuron_id not in runtime_data.upstream_neurons[dst_cortical_area][dst_neuron_id][src_cortical_area]:
        runtime_data.upstream_neurons[dst_cortical_area][dst_neuron_id][src_cortical_area].add(src_neuron_id)
