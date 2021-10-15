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
from npu.physiology import form_memories, list_upstream_neurons, longterm_potentiation_depression


def neuroplasticity(cfcl, pain_flag):
    """
    Creates bidirectional synapses between simultaneously-active neurons in connected 
    cortical areas (specified in genome and extracted into plasticity_dict). Also checks 
    for a given currently-active neuron's upstream/downstream neurons firing in the 
    previous FCL and applies LTP or LTD accordingly.
    """
    if runtime_data.parameters["Switches"]["neuroplasticity"]:
        print("Memory is being formed....")

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
