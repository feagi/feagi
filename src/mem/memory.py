
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


from inf import runtime_data
from evo.synapse import bidirectional_synapse
from npu.physiology import list_upstream_neurons, update_upstream_db, synapse, post_synaptic_current_update


def form_memories(cortical_area, src_neuron, dst_neuron):
    """
    This function simulates neuron plasticity in a sense that when neurons in a given cortical area fire in the
     same burst_manager they wire together. This is done by increasing the postsynaptic_current associated with a link between
     two neuron. Additionally an event id is associated to the neurons who have fired together.
    """

    if runtime_data.parameters["Auto_injector"]["injector_status"]:
        genome = runtime_data.genome

        # Since this function only targets Memory regions and neurons in mem regions do not have neighbor relationship
        # by default hence here we first need to synapse the source and destination together
        # Build neighbor relationship between the source and destination if its not already in place

        neighbor_count = len(runtime_data.brain[cortical_area][src_neuron]["neighbors"])
        if neighbor_count < runtime_data.parameters["InitData"]["max_neighbor_count"]:
            # Check if source and destination have an existing synapse if not create one here
            if dst_neuron not in runtime_data.brain[cortical_area][src_neuron]["neighbors"]:
                synapse(cortical_area, src_neuron, cortical_area, dst_neuron,
                        genome["blueprint"][cortical_area]["postsynaptic_current"])
                update_upstream_db(cortical_area, src_neuron, cortical_area, dst_neuron)

            # Every time source and destination neuron is fired at the same time which in case of the code architecture
            # reside in the same burst_manager, the postsynaptic_current will be increased simulating the fire together,
            # wire together. This phenomenon is also considered as long term potentiation or LTP

            runtime_data.brain[cortical_area][src_neuron]["neighbors"][dst_neuron]["postsynaptic_current"] += \
                genome["blueprint"][cortical_area]["plasticity_constant"]

            # Condition to cap the postsynaptic_current and provide prohibitory reaction
            if runtime_data.brain[cortical_area][src_neuron]["neighbors"][dst_neuron]["postsynaptic_current"] > \
                    genome["blueprint"][cortical_area]["postsynaptic_current_max"]:
                runtime_data.brain[cortical_area][src_neuron]["neighbors"][dst_neuron]["postsynaptic_current"] = \
                    genome["blueprint"][cortical_area]["postsynaptic_current_max"]

            # print('<*> ', cortical_area, src_neuron[27:], dst_neuron[27:], 'PSC=',
            #       runtime_data.brain[cortical_area][src_neuron]["neighbors"][dst_neuron]["postsynaptic_current"])

            # Append a Group ID so Memory clusters can be uniquely identified
            if runtime_data.event_id:
                if runtime_data.event_id in runtime_data.brain[cortical_area][src_neuron]["event_id"]:
                    runtime_data.brain[cortical_area][src_neuron]["event_id"][runtime_data.event_id] += 1
                else:
                    runtime_data.brain[cortical_area][src_neuron]["event_id"][runtime_data.event_id] = 1

    return


def longterm_potentiation_depression(src_cortical_area, src_neuron_id, dst_cortical_area,
                                     dst_neuron_id, long_term_depression=False, impact_multiplier=1):

    plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"]

    if long_term_depression:
        # When long term depression flag is set, there will be negative synaptic influence caused
        plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"] * (-1) * \
                              impact_multiplier

    try:
        runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id][
            "postsynaptic_current"] += plasticity_constant

        # Condition to cap the postsynaptic_current and provide prohibitory reaction
        if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id][
            "postsynaptic_current"] > \
                runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]:
            runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id][
                "postsynaptic_current"] = \
                runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]

        # Condition to prevent postsynaptic current to become negative
        # todo: consider setting a postsynaptic_min in genome to be used instead of 0
        # Condition to prune a synapse if its postsynaptic_current is zero
        if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id][
            "postsynaptic_current"] < 0:
            runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id,
                                                  dst_cortical_area, dst_neuron_id))

    except KeyError:
        synapse(src_cortical_area,
                src_neuron_id,
                dst_cortical_area,
                dst_neuron_id,
                max(plasticity_constant, 0))
        update_upstream_db(src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id)

        runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] \
            += plasticity_constant

        # Condition to cap the postsynaptic_current and provide prohibitory reaction
        if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] > \
                runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]:
            new_postsynaptic_current = runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]
            post_synaptic_current_update(cortical_area_src=src_cortical_area, cortical_area_dst=dst_cortical_area,
                                         neuron_id_src=src_neuron_id, neuron_id_dst=dst_neuron_id,
                                         post_synaptic_current=new_postsynaptic_current)

        # Condition to prevent postsynaptic current to become negative
        # todo: consider setting a postsynaptic_min in genome to be used instead of 0
        # Condition to prune a synapse if its postsynaptic_current is zero
        if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] < 0:
            runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id))


def neuroplasticity(cfcl, pain_flag):
    """
    Creates bidirectional synapses between simultaneously-active neurons in connected 
    cortical areas (specified in genome and extracted into plasticity_dict). Also checks 
    for a given currently-active neuron's upstream/downstream neurons firing in the 
    previous FCL and applies LTP or LTD accordingly.
    """
    if runtime_data.parameters["Switches"]["neuroplasticity"]:

        plasticity_dict = runtime_data.plasticity_dict
        previous_fcl = runtime_data.previous_fcl

        for cortical_area in cfcl:
            if cortical_area in plasticity_dict:
                for mapping_dst in plasticity_dict[cortical_area]:
                    if mapping_dst in cfcl:
                        for neuron1 in cfcl[cortical_area]:
                            for neuron2 in cfcl[mapping_dst]:
                                bidirectional_synapse(
                                    cortical_area, 
                                    neuron1, 
                                    mapping_dst, 
                                    neuron2
                                )
                            if mapping_dst in previous_fcl:
                                neuron1_upstream_neurons = list_upstream_neurons(cortical_area, neuron1)
                                if mapping_dst in neuron1_upstream_neurons:
                                    for neuron in previous_fcl[mapping_dst]:
                                        if neuron in neuron1_upstream_neurons[mapping_dst]:
                                            longterm_potentiation_depression(
                                                src_cortical_area=cortical_area, 
                                                src_neuron_id=neuron1,
                                                dst_cortical_area=mapping_dst, 
                                                dst_neuron_id=neuron
                                            )
                            neuron1_downstream_neurons = runtime_data.brain[cortical_area][neuron1]['neighbors']
                            if mapping_dst in neuron1_downstream_neurons:
                                for neuron in previous_fcl[mapping_dst]:
                                    if neuron in neuron1_downstream_neurons[mapping_dst]:
                                        longterm_potentiation_depression(
                                            src_cortical_area=cortical_area,
                                            src_neuron_id=neuron1,
                                            dst_cortical_area=mapping_dst,
                                            dst_neuron_id=neuron,
                                            long_term_depression=True,
                                            impact_multiplier=4
                                        )
