from inf import runtime_data
from evo.synapse import bidirectional_synapse
from npu.physiology import form_memories, list_upstream_neurons, longterm_potentiation_depression


# def neuroplasticity_old(cfcl, pain_flag):
#     """
#     This function provides logics related to memory formation as follows:
#     - Logic to wire memory neurons together when they fire together
#     - Logic to reduce synaptic connectivity when one vision memory leads to activation of two UTF neurons
#     """
#     if runtime_data.parameters["Switches"]["memory_formation"]:
#         # todo: The following section to be generalized
#         """
#         Todo: Add a new key to genome under each mapping destination to set the plasticity on/off. Use this key to know 
#         where the plasticity needs to be applied. 
        
#         Example:
#         "cortical_mapping_dst": {
#             "motor_opu": {
#                     "neighbor_locator_rule_id": "rule_8",
#                     "neighbor_locator_rule_param_id": "param_1"
#                     "plasticity": True
#                 }
#         }
#         """
#         print("Memory is being formed....")

#         # print("+++++++++cfcl_utf8_memory_neurons:", cfcl_utf8_memory_neurons)
#         utf8_memory_count = len(cfcl['utf8_memory'])
#         if cfcl['vision_memory'] and runtime_data.parameters["Auto_injector"]["injector_status"]:
#             print("Number of vision memory neurons fired in this burst_manager:", len(cfcl['vision_memory']))
#             print("Number of UTF memory neurons fired in this burst_manager:", utf8_memory_count)
#             tmp_plasticity_list = []
#             # Wiring visual memory neurons who are firing together
#             for source_neuron in cfcl['vision_memory']:

#                 # Every visual memory neuron in CFCL is going to wire to evey other vision memory neuron
#                 # for destination_neuron in cfcl_vision_memory_neurons:
#                 #     if destination_neuron != source_neuron and destination_neuron not in tmp_plasticity_list:
#                 #         form_memories(cortical_area='vision_memory',
#                 #                       src_neuron=source_neuron,
#                 #                       dst_neuron=destination_neuron)

#                 # Wiring visual memory neurons to the utf_memory ones
#                 for destination_neuron in cfcl['utf8_memory']:
#                     if not pain_flag:
#                         longterm_potentiation_depression(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
#                                                          dst_cortical_area='utf8_memory', dst_neuron_id=destination_neuron)
#                         # print("wiring visual to utf:", source_neuron, destination_neuron)
#                     if pain_flag:
#                         longterm_potentiation_depression(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
#                                                          dst_cortical_area='utf8_memory', dst_neuron_id=destination_neuron,
#                                                          long_term_depression=True, impact_multiplier=4)
#                         # print("un-wiring visual to utf:", source_neuron, destination_neuron)

#                 ####################################################################################
#                 #                                                                                  #
#                 # TODO: generalize the following code so it is applicable to other cortical areas  #
#                 # - consider filtering cortical areas where location_generation_type is sequential #
#                 #                                                                                  #
#                 ####################################################################################

#                 # Reducing synaptic connectivity when a single memory neuron is associated with more than one utf_memory one
#                 if utf8_memory_count >= 2:
#                     synapse_to_utf = 0
#                     runtime_data.temp_neuron_list = []
#                     neighbor_list = dict(runtime_data.brain['vision_memory'][source_neuron]['neighbors'])
#                     # print("<><><>")
#                     for synapse_ in neighbor_list:
#                         if runtime_data.brain['vision_memory'][source_neuron]['neighbors'][synapse_]['cortical_area'] \
#                                 == 'utf8_memory':
#                             synapse_to_utf += 1
#                             runtime_data.temp_neuron_list.append(synapse_)
#                         if synapse_to_utf >= 2:
#                             for dst_neuron in runtime_data.temp_neuron_list:
#                                 longterm_potentiation_depression(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
#                                                                  dst_cortical_area='utf8_memory', dst_neuron_id=dst_neuron,
#                                                                  long_term_depression=True, impact_multiplier=4)
#                 tmp_plasticity_list.append(source_neuron)


def neuroplasticity(cfcl, pain_flag):

    if runtime_data.parameters["Switches"]["memory_formation"]:
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
                    # at this indentation level, neuron1 variable is now outside the scope of the following code...
                    # if mapping_dst in previous_fcl:
                    #     neuron1_upstream_neurons = list_upstream_neurons(cortical_area, neuron1)
                    #     if mapping_dst in neuron1_upstream_neurons:
                    #         if neuron2 in neuron1_upstream_neurons[mapping_dst]:
                    #             for neuron in previous_fcl[mapping_dst]:
                    #                 longterm_potentiation_depression(
                    #                     src_cortical_area=cortical_area, 
                    #                     src_neuron_id=neuron1,
                    #                     dst_cortical_area=mapping_dst, 
                    #                     dst_neuron_id=neuron
                    #                 )
                    # neuron1_downstream_neurons = runtime_data.brain[cortical_area][neuron1]['neighbors']
                    # if mapping_dst in neuron1_downstream_neurons:
                    #     if neuron2 in neuron1_downstream_neurons:
                    #         for neuron in previous_fcl[mapping_dst]:
                    #             longterm_potentiation_depression(
                    #                 src_cortical_area=cortical_area,
                    #                 src_neuron_id=neuron1,
                    #                 dst_cortical_area=mapping_dst,
                    #                 dst_neuron_id=neuron2,
                    #                 long_term_depression=True,
                    #                 impact_multiplier=4
                    #             )
