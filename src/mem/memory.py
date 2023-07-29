
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
from inf import runtime_data
from evo.synapse import bidirectional_synapse, synapse
from npu.physiology import list_upstream_neurons, post_synaptic_current_update


logger = logging.getLogger(__name__)


def neuroplasticity():
    common_neurons = set()
    if runtime_data.plasticity_queue:
        common_neurons = set.intersection(*runtime_data.plasticity_queue)

    # todo: filter only plastic links
    for neuron in common_neurons:
        try:
            cortical_area = neuron[:6]
            presynaptic_neurons = list_upstream_neurons(cortical_area=cortical_area, neuron_id=neuron)

            postsynaptic_neurons = runtime_data.brain[cortical_area][neuron]['neighbors']

            postsynaptic_neurons_set = set()
            for item in postsynaptic_neurons:
                postsynaptic_neurons_set.add(item)

            connected_neurons = presynaptic_neurons | postsynaptic_neurons_set

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
                        long_term_depression=True,
                        impact_multiplier=1
                    )
        except Exception as e:
            print(f"Exception during neuroplasticity processing of {neuron}", e, traceback.print_exc())


def longterm_potentiation_depression(src_cortical_area, src_neuron_id, dst_cortical_area,
                                     dst_neuron_id, long_term_depression=False, impact_multiplier=1.0):

    plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"]

    if long_term_depression:
        # When long term depression flag is set, there will be negative synaptic influence caused
        plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"] * (-1) * \
                              impact_multiplier
        # print("<> <> <> <> <> <> <> <> <>     LTD     <> <> <> <> <> <> <> <> <> <>", src_neuron_id, dst_neuron_id)
        try:
            runtime_data.cumulative_stats[src_cortical_area]["LTD"] += 1
        except:
            pass

    else:
        # print("<> <> <> <> <> <> <> <> <>      LTP      <> <> <> <> <> <> <> <> <> <>", src_neuron_id, dst_neuron_id)
        try:
            runtime_data.cumulative_stats[src_cortical_area]["LTP"] += 1
        except:
            pass

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
            runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id,
                                                  dst_cortical_area, dst_neuron_id))

        post_synaptic_current_update(cortical_area_src=src_cortical_area, cortical_area_dst=dst_cortical_area,
                                     neuron_id_src=src_neuron_id, neuron_id_dst=dst_neuron_id,
                                     post_synaptic_current=new_psc)

    except KeyError as e:
        print("\n\n\nKey Error on longterm_potentiation_depression:", e, traceback.print_exc())
        pass
