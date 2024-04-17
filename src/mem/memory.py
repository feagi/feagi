
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

"""This module is responsible for creating bidirectional synapses between neurons that are
firing together, while also strengthening/weakening (via LTP/LTD, respectively) a given active
neuron's synapses to upstream/downstream neurons firing in the previous fire candidate list (FCL).

The following code block was extracted from the now-deleted neuroplasticity function that used
LTP/LTD to strengthen/weaken synapses between vision memory and UTF8 memory neurons. It is likely
that this code will need to be generalized for use with any cortical area(s) when encountering a
similar scenario (where a neuron's synaptic connectivity to multiple sequential neurons needs to be
reduced) and incorporated in the new neuroplasticity function (below). 

    ####################################################################################
    #                                                                                  #
    # TODO: generalize the following code so it is applicable to other cortical areas  #
    # - consider filtering cortical areas where location_generation_type is sequential #
    #                                                                                  #
    ####################################################################################

    # Reducing synaptic connectivity when a single memory neuron is associated with more than one utf_memory one
    if utf8_memory_count >= 2:
        synapse_to_utf = 0
        runtime_data.temp_neuron_list = []
        neighbor_list = dict(runtime_data.brain['vision_memory'][source_neuron]['neighbors'])
        # print("<><><>")
        for synapse_ in neighbor_list:
            if runtime_data.brain['vision_memory'][source_neuron]['neighbors'][synapse_]['cortical_area'] \
                    == 'utf8_memory':
                synapse_to_utf += 1
                runtime_data.temp_neuron_list.append(synapse_)
            if synapse_to_utf >= 2:
                for dst_neuron in runtime_data.temp_neuron_list:
                    longterm_potentiation_depression(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
                                                        dst_cortical_area='utf8_memory', dst_neuron_id=dst_neuron,
                                                        long_term_depression=True, impact_multiplier=4)
"""
import traceback
import logging
import xxhash

from src.inf import runtime_data
from src.evo.neuron import init_neuron, increase_neuron_lifespan, neuron_apoptosis, convert_shortterm_to_longterm
from src.evo.synapse import synapse_memory_neuron
from src.npu.physiology import list_upstream_plastic_neurons, list_downstream_plastic_neurons, post_synaptic_current_update


logger = logging.getLogger(__name__)


def neuroplasticity():
    common_neurons = set()
    if runtime_data.plasticity_queue:
        common_neurons = set.intersection(*runtime_data.plasticity_queue)

    # todo: filter only plastic links
    for neuron in common_neurons:
        try:
            cortical_area = neuron[:6]
            # presynaptic_neurons = list_upstream_plastic_neurons(cortical_area=cortical_area, neuron_id=neuron)

            postsynaptic_neurons = list_downstream_plastic_neurons(cortical_area=cortical_area, neuron_id=neuron)
            postsynaptic_neurons_set = set()
            for item in postsynaptic_neurons:
                postsynaptic_neurons_set.add(item)

            # connected_neurons = presynaptic_neurons | postsynaptic_neurons_set

            for postsynaptic_neuron in postsynaptic_neurons:
                if postsynaptic_neuron in common_neurons:
                    # ------LTP------
                    longterm_potentiation_depression(
                        src_cortical_area=cortical_area,
                        src_neuron_id=neuron,
                        dst_cortical_area=postsynaptic_neuron[:6],
                        dst_neuron_id=postsynaptic_neuron
                    )
                else:
                    # ------LTD------
                    longterm_potentiation_depression(
                        src_cortical_area=cortical_area,
                        src_neuron_id=neuron,
                        dst_cortical_area=postsynaptic_neuron[:6],
                        dst_neuron_id=postsynaptic_neuron,
                        long_term_depression=True
                    )
        except Exception as e:
            print(f"Exception during neuroplasticity processing of {neuron}", e, traceback.print_exc())


def longterm_potentiation_depression(src_cortical_area, src_neuron_id, dst_cortical_area,
                                     dst_neuron_id, long_term_depression=False):
    if dst_cortical_area in runtime_data.genome["blueprint"][src_cortical_area]["cortical_mapping_dst"]:
        for mapping in runtime_data.genome["blueprint"][src_cortical_area]["cortical_mapping_dst"][dst_cortical_area]:
            plasticity_flag = mapping["plasticity_flag"]
            if plasticity_flag:
                ltp_multiplier = mapping["ltp_multiplier"]
                ltd_multiplier = mapping["ltd_multiplier"]
                plasticity_constant = mapping["plasticity_constant"]

                if long_term_depression:
                    # When long term depression flag is set, there will be negative synaptic influence caused
                    plasticity_constant = plasticity_constant * ltd_multiplier * -1
                    # print("<> <> <> <> <> <> <> <> <>  LTD  <> <> <> <> <> <> <> <> <>", src_neuron_id, dst_neuron_id, plasticity_constant)
                    try:
                        runtime_data.cumulative_stats[src_cortical_area]["LTD"] += 1
                    except Exception as e:
                        print("Exception during LTD:", e)

                else:
                    # print("<> <> <> <> <> <> <> <> <>  LTP  <> <> <> <> <> <> <> <>", src_neuron_id, dst_neuron_id, plasticity_constant)
                    plasticity_constant = plasticity_constant * ltp_multiplier
                    try:
                        runtime_data.cumulative_stats[src_cortical_area]["LTP"] += 1
                    except Exception as e:
                        print("Exception during LTP:", e)

                try:
                    new_psc = \
                        runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"]
                    new_psc += plasticity_constant

                    # Condition to cap the postsynaptic_current and provide prohibitory reaction
                    if new_psc > runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]:
                        new_psc = runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]

                    # Condition to prevent postsynaptic current to become negative
                    # todo: consider setting a postsynaptic_min in genome to be used instead of 0
                    # Condition to prune a synapse if its postsynaptic_current is zero
                    if new_psc < 0:
                        new_psc = 0
                        # runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id,
                        #                                       dst_cortical_area, dst_neuron_id))
                    post_synaptic_current_update(cortical_area_src=src_cortical_area,
                                                 cortical_area_dst=dst_cortical_area,
                                                 neuron_id_src=src_neuron_id, neuron_id_dst=dst_neuron_id,
                                                 post_synaptic_current=new_psc)

                except KeyError as e:
                    print("\n\n\nKey Error on longterm_potentiation_depression:", e, traceback.print_exc())
                    print("=============")
                    print(src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id, long_term_depression)
                    pass
    else:
        print(f"longterm_potentiation_depression did not find {dst_cortical_area} as a mapping in {src_cortical_area}")


def long_short_term_memory():
    # todo: build up the runtime_date.memory_register from synaptogenesis
    runtime_data.lstm_fire_queue = set()
    if runtime_data.memory_register:
        for memory_cortical_area in runtime_data.memory_register:
            neurogenesis_list = set()
            for upstream_cortical_area in runtime_data.memory_register[memory_cortical_area]:
                if upstream_cortical_area in runtime_data.fire_candidate_list:
                    if runtime_data.fire_candidate_list[upstream_cortical_area]:
                        # todo: performance: exclude non-immortal neurons from being processed
                        neurogenesis_list.update(runtime_data.fire_candidate_list[upstream_cortical_area])
            memory_hash = generate_mem_hash_cache(afferent_neuron_list=neurogenesis_list)

            mem_neuron_id = convert_hash_to_neuron_id(cortical_area=memory_cortical_area, memory_hash=memory_hash)
            if mem_neuron_id not in runtime_data.brain[memory_cortical_area] and memory_hash != "0x0":
                neuron_id = init_neuron(cortical_area=memory_cortical_area, soma_location=[0, 0, 0], mem_neuron_id=memory_hash)
                runtime_data.voxel_dict[memory_cortical_area]["0-0-0"].add(neuron_id)
                synapse_count = synapse_memory_neuron(neuron_id=neuron_id)
            else:
                increase_neuron_lifespan(cortical_area=memory_cortical_area, neuron_id=mem_neuron_id)

            if memory_hash != "0x0":
                if memory_cortical_area not in runtime_data.future_fcl:
                    runtime_data.future_fcl[memory_cortical_area] = set()

                runtime_data.lstm_fire_queue.add(mem_neuron_id)
                if memory_cortical_area in runtime_data.plasticity_dict:
                    runtime_data.plasticity_queue_candidates.add(mem_neuron_id)

            inject_lstm_fire_queue_to_fcl()


def lstm_lifespan_mgmt():
    """
    Handles lifecycle management for memory neurons
    """
    if runtime_data.memory_register:
        if runtime_data.burst_count > runtime_data.upcoming_lifesnap_mgmt:
            runtime_data.upcoming_lifesnap_mgmt += runtime_data.genome["physiology"]["lifespan_mgmt_interval"]

            # Wipe short-term memory neurons that has expired
            memory_cleanup()


def inject_lstm_fire_queue_to_fcl():
    for neuron in runtime_data.lstm_fire_queue:
        cortical_area = neuron[:6]
        if cortical_area not in runtime_data.fire_candidate_list:
            runtime_data.fire_candidate_list[cortical_area] = set()
        runtime_data.fire_candidate_list[cortical_area].add(neuron)


def memory_cleanup():
    for memory_cortical_area in runtime_data.memory_register:
        apoptosis_candidates = set()
        for neuron in runtime_data.brain[memory_cortical_area]:
            # Neuron lifespan management
            if not runtime_data.brain[memory_cortical_area][neuron]["immortal"]:
                # Neuron Apoptosis check
                if runtime_data.brain[memory_cortical_area][neuron]["lifespan"] < runtime_data.burst_count:
                    apoptosis_candidates.add(neuron)
                # Short-term Memory to Long-term Memory transformation check
                elif runtime_data.brain[memory_cortical_area][neuron]["lifespan"] > \
                        runtime_data.burst_count + \
                        runtime_data.genome["blueprint"][memory_cortical_area]["longterm_mem_threshold"]:
                    convert_shortterm_to_longterm(memory_area=memory_cortical_area, memory_neuron_id=neuron)
        for apoptosis_candidate in apoptosis_candidates:
            neuron_apoptosis(cortical_area=memory_cortical_area, neuron_id=apoptosis_candidate)


def generate_mem_hash_cache(afferent_neuron_list):
    combined_hash = 0
    hash_cache = {}

    for serial_number in afferent_neuron_list:
        if serial_number not in hash_cache:
            # Compute and cache the hash for new serial numbers
            hash_cache[serial_number] = xxhash.xxh32(serial_number).intdigest()

        # Retrieve the hash from the cache
        individual_hash = hash_cache[serial_number]

        # Combine hashes using XOR
        combined_hash ^= individual_hash

    return hex(combined_hash)


def convert_hash_to_neuron_id(cortical_area, memory_hash):
    neuron_id = str(cortical_area + '_' + memory_hash)
    return neuron_id
