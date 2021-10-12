
from inf import runtime_data
from npu.physiology import apply_plasticity, apply_plasticity_ext


def form_memories(cfcl, pain_flag):
    """
    This function provides logics related to memory formation as follows:
    - Logic to wire memory neurons together when they fire together
    - Logic to reduce synaptic connectivity when one vision memory leads to activation of two UTF neurons
    """
    if runtime_data.parameters["Switches"]["memory_formation"]:
        # todo: The following section to be generalized
        """
        Todo: Add a new key to genome under each mapping destination to set the plasticity on/off. Use this key to know 
        where the plasticity needs to be applied. 
        
        Example:
        "cortical_mapping_dst": {
            "motor_opu": {
                    "neighbor_locator_rule_id": "rule_8",
                    "neighbor_locator_rule_param_id": "param_1"
                    "plasticity": True
                }
        }
        """

        print("Memory is being formed....")

        # print("+++++++++cfcl_utf8_memory_neurons:", cfcl_utf8_memory_neurons)
        utf8_memory_count = len(cfcl['utf8_memory'])
        if cfcl['vision_memory'] and runtime_data.parameters["Auto_injector"]["injector_status"]:
            print("Number of vision memory neurons fired in this burst_manager:", len(cfcl['vision_memory']))
            print("Number of UTF memory neurons fired in this burst_manager:", utf8_memory_count)
            tmp_plasticity_list = []
            # Wiring visual memory neurons who are firing together
            for source_neuron in cfcl['vision_memory']:

                # Every visual memory neuron in CFCL is going to wire to evey other vision memory neuron
                # for destination_neuron in cfcl_vision_memory_neurons:
                #     if destination_neuron != source_neuron and destination_neuron not in tmp_plasticity_list:
                #         apply_plasticity(cortical_area='vision_memory',
                #                          src_neuron=source_neuron,
                #                          dst_neuron=destination_neuron)

                # Wiring visual memory neurons to the utf_memory ones
                for destination_neuron in cfcl['utf8_memory']:
                    if not pain_flag:
                        apply_plasticity_ext(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
                                             dst_cortical_area='utf8_memory', dst_neuron_id=destination_neuron)
                        # print("wiring visual to utf:", source_neuron, destination_neuron)
                    if pain_flag:
                        apply_plasticity_ext(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
                                             dst_cortical_area='utf8_memory', dst_neuron_id=destination_neuron,
                                             long_term_depression=True, impact_multiplier=4)
                        # print("un-wiring visual to utf:", source_neuron, destination_neuron)

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
                                apply_plasticity_ext(src_cortical_area='vision_memory', src_neuron_id=source_neuron,
                                                     dst_cortical_area='utf8_memory', dst_neuron_id=dst_neuron,
                                                     long_term_depression=True, impact_multiplier=4)
                tmp_plasticity_list.append(source_neuron)

