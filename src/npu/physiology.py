
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
import logging
# from collections import deque
# from evo.neuron import block_reference_builder
# from evo.synapse import synapse
# from evo.voxels import block_reference_builder
from inf import runtime_data, settings
# from cython_lib import neuron_functions_cy as cy


logger = logging.getLogger(__name__)


def activation_function(postsynaptic_current):
    # print("PSC: ", postsynaptic_current)
    return postsynaptic_current


def reset_cumulative_counters(cortical_area, neuron_id):
    runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] = runtime_data.burst_count
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = runtime_data.burst_count
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count"] += 1
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count_inst"] += 1
    # Condition to increase the consecutive fire count
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

    if cortical_area in runtime_data.neuron_mp_collection_scope:
        if monitor_filter(cortical_area=cortical_area, neuron_id=neuron_id,
                          filter_criteria=runtime_data.neuron_mp_collection_scope[cortical_area]):

            vox_x, vox_y, vox_z = [vox for vox in runtime_data.brain[cortical_area][neuron_id]['soma_location']]
            fire_threshold = runtime_data.genome["blueprint"][cortical_area]["firing_threshold"]

            mem_pot = runtime_data.brain[cortical_area][neuron_id]["membrane_potential"]

            # Note: dst_cortical_area is fed to the src_cortical_area field since the membrane potential of dst changes

            # To demonstrate a spike when a neuron is artificially stimulated to fire
            runtime_data.influxdb.insert_neuron_activity(connectome_path=runtime_data.connectome_path,
                                                         src_cortical_area=cortical_area,
                                                         src_neuron_id=neuron_id,
                                                         dst_voxel_x=vox_x,
                                                         dst_voxel_y=vox_y,
                                                         dst_voxel_z=vox_z,
                                                         membrane_potential=mem_pot / 1)
            runtime_data.influxdb.insert_neuron_activity(connectome_path=runtime_data.connectome_path,
                                                         src_cortical_area=cortical_area,
                                                         src_neuron_id=neuron_id,
                                                         dst_voxel_x=vox_x,
                                                         dst_voxel_y=vox_y,
                                                         dst_voxel_z=vox_z,
                                                         membrane_potential=fire_threshold / 1)
            runtime_data.influxdb.insert_neuron_activity(connectome_path=runtime_data.connectome_path,
                                                         src_cortical_area=cortical_area,
                                                         src_neuron_id=neuron_id,
                                                         dst_voxel_x=vox_x,
                                                         dst_voxel_y=vox_y,
                                                         dst_voxel_z=vox_z,
                                                         membrane_potential=0 / 1)

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

        if degenerate > 0:
            # reduce neuron postsynaptic current by degeneration value defined in genome (if applicable)
            new_psc = runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"]
            new_psc -= degenerate
            if new_psc < 0:
                new_psc = 0
            post_synaptic_current_update(cortical_area_src=cortical_area, cortical_area_dst=dst_cortical_area,
                                         neuron_id_src=neuron_id, neuron_id_dst=dst_neuron_id,
                                         post_synaptic_current=new_psc)
        neuron_output = activation_function(postsynaptic_current)

        # Update function
        # todo: (neuron_output/neighbor_count) needs to be moved outside the loop for efficiency
        dst_neuron_obj = runtime_data.brain[dst_cortical_area][dst_neuron_id]

        # Update membrane potential of the downstream neuron
        if runtime_data.genome['blueprint'][cortical_area]['psp_uniform_distribution']:
            membrane_potential_update(cortical_area=dst_cortical_area, neuron_id=dst_neuron_id,
                                      membrane_potential_change=neuron_output, bypass_db_log=False,
                                      src_cortical_area=cortical_area, src_neuron=neuron_id)
        else:
            membrane_potential_update(cortical_area=dst_cortical_area, neuron_id=dst_neuron_id,
                                      membrane_potential_change=neuron_output/neighbor_count, bypass_db_log=False,
                                      src_cortical_area=cortical_area, src_neuron=neuron_id)

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


def neuron_leak(cortical_area, neuron_id):
    """
    Calculates the amount of leak to be applied to the neuron during update

    Returns the membrane potential change caused by the leaky behavior
    """
    # Keeping track of the leak occurrences during a burst to prevent duplicate leaks across multiple update cycles
    # within the same burst
    leak_value = 0

    # Leaky behavior
    leak_coefficient = \
        runtime_data.genome["blueprint"][cortical_area]["leak_coefficient"]
    if leak_coefficient > 0:
        if runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_update"] and \
                runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_update"] > 5:

            last_membrane_potential_update = \
                runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"]

            leak_window = runtime_data.burst_count - last_membrane_potential_update
            leak_value = leak_window * leak_coefficient

            # Capping the leak to the max allowable membrane potential
            leak_value = min(leak_value, runtime_data.brain[cortical_area][neuron_id]["residual_membrane_potential"])

            if leak_value < 0:
                print("Warning! Leak less than 0 detected! ", leak_value)

    return leak_value


def monitor_filter(cortical_area, neuron_id, filter_criteria):
    """
    Assess if the neuron_id matches filter criteria

    filter_criteria data structure:

    {
        "voxels": [[0, 0, 0], [2, 0, 0]],
        "neurons": []
    }
    """
    if len(filter_criteria) == 0:
        return True
    if "voxels" in filter_criteria:
        neuron_voxel = runtime_data.brain[cortical_area][neuron_id]['soma_location']
        if neuron_voxel in filter_criteria["voxels"]:
            return True
    if "neurons" in filter_criteria:
        if neuron_id in filter_criteria["neurons"]:
            return True

    return False


def membrane_potential_update(cortical_area, neuron_id, membrane_potential_change, overwrite=False, overwrite_value=0,
                              bypass_db_log=False, src_cortical_area=None, src_neuron=None):
    """
    Responsible for updating the membrane potential of each neuron
    """

    runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] += membrane_potential_change

    if overwrite:
        runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] = overwrite_value

    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_update"] = runtime_data.burst_count

    if not bypass_db_log:
        # Assess the filter conditions set through the REST API
        if cortical_area in runtime_data.neuron_mp_collection_scope:
            if monitor_filter(cortical_area=cortical_area, neuron_id=neuron_id,
                              filter_criteria=runtime_data.neuron_mp_collection_scope[cortical_area]):
                vox_x, vox_y, vox_z = [vox for vox in runtime_data.brain[cortical_area][neuron_id]['soma_location']]
                dst_mp = runtime_data.brain[cortical_area][neuron_id]["membrane_potential"]
                # dst_cortical_area is fed to the src_cortical_area field since the membrane potential of dst changes
                runtime_data.influxdb.insert_neuron_activity(connectome_path=runtime_data.connectome_path,
                                                             src_cortical_area=cortical_area,
                                                             src_neuron_id=neuron_id,
                                                             dst_voxel_x=vox_x,
                                                             dst_voxel_y=vox_y,
                                                             dst_voxel_z=vox_z,
                                                             membrane_potential=dst_mp / 1)

        if cortical_area in runtime_data.neuron_psp_collection_scope and src_cortical_area and src_neuron:
            if monitor_filter(cortical_area=cortical_area, neuron_id=neuron_id,
                              filter_criteria=runtime_data.neuron_psp_collection_scope[cortical_area]):
                src_flag = False
                if "sources" not in runtime_data.neuron_psp_collection_scope[cortical_area]:
                    src_flag = True
                elif len(runtime_data.neuron_psp_collection_scope[cortical_area]["sources"]) == 0:
                    src_flag = True
                elif monitor_filter(cortical_area=src_cortical_area, neuron_id=src_neuron,
                                    filter_criteria=runtime_data.neuron_psp_collection_scope[cortical_area]
                                    ["sources"][src_cortical_area]):
                    src_flag = True

                if src_flag:
                    psc = runtime_data.brain[src_cortical_area][src_neuron]["neighbors"][neuron_id][
                        "postsynaptic_current"]
                    post_synaptic_current_update(cortical_area_src=src_cortical_area,
                                                 cortical_area_dst=cortical_area,
                                                 neuron_id_src=src_neuron, neuron_id_dst=neuron_id,
                                                 post_synaptic_current=psc)


def post_synaptic_current_update(cortical_area_src,
                                 cortical_area_dst,
                                 neuron_id_src,
                                 neuron_id_dst,
                                 post_synaptic_current):
    """
    Responsible for updating the post-synaptic-current between two neurons
    """
    runtime_data.brain[cortical_area_src][neuron_id_src]["neighbors"][neuron_id_dst]["postsynaptic_current"] = \
        post_synaptic_current

    # Assess the filter conditions set through the REST API
    if cortical_area_dst in runtime_data.neuron_psp_collection_scope:
        if monitor_filter(cortical_area=cortical_area_dst, neuron_id=neuron_id_dst,
                          filter_criteria=runtime_data.neuron_psp_collection_scope[cortical_area_dst]):
            src_flag = False
            if "sources" not in runtime_data.neuron_psp_collection_scope[cortical_area_dst]:
                src_flag = True
            elif len(runtime_data.neuron_psp_collection_scope[cortical_area_dst]["sources"]) == 0:
                src_flag = True
            elif monitor_filter(cortical_area=cortical_area_src, neuron_id=neuron_id_src,
                                filter_criteria=runtime_data.neuron_psp_collection_scope[cortical_area_dst]
                                ["sources"][cortical_area_src]):
                src_flag = True
            if src_flag:
                psc = runtime_data.brain[cortical_area_src][neuron_id_src]["neighbors"][
                    neuron_id_dst]["postsynaptic_current"]

                src_vox_x, src_vox_y, src_vox_z = \
                    [vox for vox in runtime_data.brain[cortical_area_src][neuron_id_src]['soma_location']]
                dst_vox_x, dst_vox_y, dst_vox_z = \
                    [vox for vox in runtime_data.brain[cortical_area_dst][neuron_id_dst]['soma_location']]
                runtime_data.influxdb.insert_synaptic_activity(connectome_path=runtime_data.connectome_path,
                                                               src_cortical_area=cortical_area_src,
                                                               dst_cortical_area=cortical_area_dst,
                                                               src_voxel_x=src_vox_x,
                                                               src_voxel_y=src_vox_y,
                                                               src_voxel_z=src_vox_z,
                                                               dst_voxel_x=dst_vox_x,
                                                               dst_voxel_y=dst_vox_y,
                                                               dst_voxel_z=dst_vox_z,
                                                               src_neuron_id=neuron_id_src,
                                                               dst_neuron_id=neuron_id_dst,
                                                               post_synaptic_current=psc)


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
        = burst_id + runtime_data.genome["blueprint"][cortical_area]["snooze_length"] - 1
    # print("%s : %s has been snoozed!" % (cortical_area, neuron_id))
    return


def pruner(pruning_data):
    """
    Responsible for pruning unused connections between neurons
    """
    cortical_area_src, src_neuron_id, cortical_area_dst, dst_neuron_id = pruning_data
    runtime_data.brain[cortical_area_src][src_neuron_id]['neighbors'].pop(dst_neuron_id, None)

    try:
        runtime_data.brain[cortical_area_dst][dst_neuron_id][
            "upstream_neurons"][cortical_area_src].remove(src_neuron_id)
        if dst_neuron_id in runtime_data.temp_neuron_list:
            runtime_data.temp_neuron_list.remove(dst_neuron_id)
    except KeyError as e:
        print("Pruner warning! Key not found", e)


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
    return runtime_data.brain[cortical_area][neuron_id]["upstream_neurons"]
