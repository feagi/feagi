

def print_cortical_neuron_mappings(src_cortical_area, dst_cortical_area):
    print('Listing neuron mappings between %s and %s' % (src_cortical_area, dst_cortical_area))
    for neuron in runtime_data.brain[src_cortical_area]:
        for neighbor in runtime_data.brain[src_cortical_area][neuron]['neighbors']:
            if runtime_data.brain[src_cortical_area][neuron]['neighbors'][neighbor]['cortical_area'] == \
                    dst_cortical_area:
                print(settings.Bcolors.OKGREEN + '# ', neuron[27:], neighbor[27:],
                      str(runtime_data.brain[src_cortical_area][neuron]
                      ['neighbors'][neighbor]['postsynaptic_current']) + settings.Bcolors.ENDC)


def utf_neuron_id(n):
    # Returns the neuron id associated with a particular digit
    for neuron_id in runtime_data.brain['utf8_memory']:
        if int(runtime_data.brain['utf8_memory'][neuron_id]["soma_location"][0][2]) == n+ord('0'):
            return neuron_id


def utf_neuron_position(neuron_id):
    return int(runtime_data.brain['utf8_memory'][neuron_id]["soma_location"][0][2]) - ord('0')



def reset_cumulative_counter_instances():
    """
    To reset the cumulative counter instances
    """
    for cortical_area in runtime_data.brain:
        for neuron in runtime_data.brain[cortical_area]:
            runtime_data.brain[cortical_area][neuron]['cumulative_fire_count_inst'] = 0
    return


def stats_utf_memory_membrane_potentials():
    if runtime_data.top_10_utf_memory_neurons and runtime_data.top_10_utf_neurons:
        utf_neurons = []
        for neuron in runtime_data.top_10_utf_neurons:
            utf_neurons.append((neuron[0], neuron[1]))
        print('\n')
        for neuron in runtime_data.top_10_utf_memory_neurons:
            if runtime_data.brain['utf8_memory'][neuron[1]]['membrane_potential'] > \
                    runtime_data.genome['blueprint']['utf8_memory']['neuron_params']['firing_threshold']:
                print(settings.Bcolors.RED + '>>> ', neuron[0], '   >>> ',
                      str(runtime_data.brain['utf8_ipu'][utf_neurons[int(neuron[0])][1]]['membrane_potential']), '   >>> ',
                      str(runtime_data.brain['utf8_memory'][neuron[1]]['membrane_potential']),
                      '   >>>',
                      str(runtime_data.genome['blueprint']['utf8_memory']['neuron_params']['firing_threshold']) +
                      settings.Bcolors.ENDC)
            else:
                print('>>> ', neuron[0], '   >>> ',
                      str(runtime_data.brain['utf8_ipu'][utf_neurons[int(neuron[0])][1]]['membrane_potential']), '   >>> ',
                      runtime_data.brain['utf8_memory'][neuron[1]]['membrane_potential'],
                      '   >>>',
                      runtime_data.genome['blueprint']['utf8_memory']['neuron_params']['firing_threshold'])
        print('\n')


def print_basic_info():
    cp = mp.current_process()
    print("\rstarting", cp.name, cp.pid)
    print("\rConnectome database =                  %s" %
          runtime_data.parameters["InitData"]["connectome_path"])
    print("\rTotal neuron count in the connectome  %s  is: %i" %
          (runtime_data.parameters["InitData"]["connectome_path"] + 'vision_v1.json',
           stats.connectome_neuron_count(cortical_area='vision_v1')))
    print(' \rexiting', cp.name, cp.pid)
    return



def cortical_area_neuron_count(cortical_area):
    """
    Returns number of Neurons in the connectome
    """
    data = runtime_data.brain[cortical_area]
    neuron_count = 0
    for key in data:
        neuron_count += 1
    return neuron_count


def cortical_area_synapse_count(cortical_area):
    """
    Returns number of Neurons in the connectome
    """
    data = runtime_data.brain[cortical_area]
    synapse_count = 0
    for neuron in data:
        for _ in data[neuron]['neighbors']:
            synapse_count += 1
    return synapse_count



def connectome_neuron_count(self):
    total_neuron_count = 0
    for cortical_area in runtime_data.cortical_list:
        total_neuron_count += self.cortical_area_neuron_count(cortical_area)
    return total_neuron_count

def connectome_synapse_count(self):
    total_synapse_count = 0
    for cortical_area in runtime_data.cortical_list:
        total_synapse_count += self.cortical_area_synapse_count(cortical_area)

    return total_synapse_count


def terminate():
    """To terminate the brain activities without recording the genome in database or recording any stat.
    This function to be used when a brain instance is detected to be dysfunctional."""

    return


def pickler(data):
    id_ = runtime_data.brain_run_id
    with open("./pickle_jar/fcl-" + id_ + ".pkl", 'wb') as output:
        pickle.dump(data, output)


def unpickler(data_type, id_):
    if data_type == 'fcl':
        with open("./pickle_jar/fcl-" + id_ + ".pkl", 'rb') as input_data:
            data = pickle.load(input_data)
            return data
    else:
        print("Error: Type not found!")
