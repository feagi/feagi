
import json
from opu import utf8
from opu.processor import movement
from collections import deque
from evo.neuron import block_reference_builder
from evo.synapse import synapse
from evo.blocks import neurons_in_the_block
from inf import runtime_data, settings
from cython_lib import neuron_functions_cy as cy


def activation_function(postsynaptic_current):
    # print("PSC: ", postsynaptic_current)
    return postsynaptic_current


def neuron_fire(cortical_area, neuron_id):
    """This function initiate the firing of Neuron in a given cortical area"""

    # if runtime_data.parameters["Switches"]["logging_fire"]:
    #     print(datetime.now(), " Firing...", cortical_area, neuron_id, file=open("./logs/fire.log", "a"))
    neighbor_list = list()

    # if cortical_area == 'utf8_memory':
    #     print(">>> *** ... Firing...", neuron_id)
    # block_ref = block_reference_builder(runtime_data.brain[cortical_area][neuron_id]['soma_location'][1])
    # block_neurons = runtime_data.block_dic[cortical_area][block_ref]
    # print(">>> FIRING ID: ", neuron_id)
    # print(">>> BLOCK: ", block_ref)
    # print(">>> BLOCK_NEURONS: ", len(block_neurons))
    # print(">>> SRC_CORTICAL_AREA: ", cortical_area)
    # print(">>> SYNAPSES: ", len(runtime_data.brain[cortical_area][neuron_id]['neighbors']))

    print(">>>>>> >>>>>>>>> >>>>>>>> UPSTREAM NEURONS: ", list_upstream_neurons(cortical_area, neuron_id))

    # Setting Destination to the list of Neurons connected to the firing Neuron
    try:
        neighbor_list = runtime_data.brain[cortical_area][neuron_id]["neighbors"]

    except KeyError:
        print(settings.Bcolors.RED + "KeyError on accessing neighbor_list while firing a neuron" +
              settings.Bcolors.ENDC)

    # Condition to update neuron activity history currently only targeted for UTF-OPU
    # todo: move activity_history_span to genome
    activity_history_span = int(runtime_data.parameters["InitData"]["activity_history_span"])
    if cortical_area == 'utf8_memory':
        if not runtime_data.brain[cortical_area][neuron_id]["activity_history"]:
            zeros = deque([0] * activity_history_span)
            tmp_burst_list = []
            tmp_burst_count = runtime_data.burst_count
            for _ in range(activity_history_span):
                tmp_burst_list.append(tmp_burst_count)
                tmp_burst_count -= 1
            runtime_data.brain[cortical_area][neuron_id]["activity_history"] = deque(list(zip(tmp_burst_list, zeros)))
        else:
            runtime_data.brain[cortical_area][neuron_id]["activity_history"].append([runtime_data.burst_count,
                                                                                     runtime_data.brain[cortical_area]
                                                                                    [neuron_id]["membrane_potential"]])
            runtime_data.brain[cortical_area][neuron_id]["activity_history"].popleft()

    # After neuron fires all cumulative counters on Source gets reset
    runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] = 0
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = runtime_data.burst_count
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count"] += 1
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count_inst"] += 1

    # Transferring the signal from firing Neuron's Axon to all connected Neuron Dendrites
    # todo: Firing pattern to be accommodated here     <<<<<<<<<<  *****
    # neuron_update_list = []

    # if cortical_area == 'vision_memory':
    #     runtime_data.cumulative_neighbor_count += neighbor_count
    neighbor_count = len(neighbor_list)
    # Updating downstream neurons
    for dst_neuron_id in neighbor_list:
        # Timing the update function
        # update_start_time = datetime.now()

        dst_cortical_area = \
            runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["cortical_area"]
        postsynaptic_current = \
            runtime_data.brain[cortical_area][neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"]
        # if cortical_area == 'vision_memory':
        #    print("< %i >" %postsynaptic_current)

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

        # todo: Need to figure how to deal with activation function and firing threshold (belongs to fire func)
        # After destination neurons are updated, the following checks are performed to assess if the neuron should fire
        if dst_neuron_obj["membrane_potential"] > dst_neuron_obj["firing_threshold"]:
            # if dst_cortical_area == 'utf8_memory':
            # Refractory period check
            if dst_neuron_obj["last_burst_num"] + \
                    runtime_data.genome["blueprint"][dst_cortical_area]["neuron_params"]["refractory_period"] <= \
                    runtime_data.burst_count:
                # Inhibitory effect check
                if dst_neuron_obj["snooze_till_burst_num"] <= runtime_data.burst_count:
                    # Adding neuron to fire candidate list for firing in the next round
                    runtime_data.future_fcl[dst_cortical_area].add(dst_neuron_id)
                    # todo: not sure what's being done here. Why this is too generic on all cortical layers? !!
                    # todo: Why this needs to happen on each synapse update?? !! VERY EXPENSIVE OPERATION!!!!
                    # todo: Based on the initial test results, removing the following section can make the code run
                    # todo: 10 times faster but result in 1/2 fitness
                    # todo: add a condition to only LTP when cortical area source is upstream of cortical area dest.
                    # todo: adding an impact multiplier here could be beneficial
                    # This is an alternative approach to plasticity with hopefully less overhead
                    # LTP or Long Term Potentiation occurs here
                    upstream_data = list_upstream_neurons(dst_cortical_area, dst_neuron_id)

                    ltp_targets = ['vision_memory', 'utf8_memory']

                    if upstream_data and dst_cortical_area in ltp_targets:
                        for src_cortical_area in upstream_data:
                            for src_neuron in upstream_data[src_cortical_area]:
                                if src_cortical_area != dst_cortical_area and \
                                        src_neuron in runtime_data.previous_fcl[src_cortical_area]:
                                    apply_plasticity_ext(src_cortical_area=src_cortical_area,
                                                         src_neuron_id=src_neuron,
                                                         dst_cortical_area=dst_cortical_area,
                                                         dst_neuron_id=dst_neuron_id, impact_multiplier=1)

        # Resetting last time neuron was updated to the current burst_manager id
        runtime_data.brain[dst_cortical_area][dst_neuron_id]["last_burst_num"] = runtime_data.burst_count

        # Time overhead for the following function is about 2ms per each burst_manager cycle
        update_upstream_db(cortical_area, neuron_id, dst_cortical_area, dst_neuron_id)

        # Partial implementation of neuro-plasticity associated with LTD or Long Term Depression
        if cortical_area not in ['vision_memory']:
            if dst_neuron_id in runtime_data.previous_fcl[dst_cortical_area] and dst_cortical_area in ['vision_memory']:
                apply_plasticity_ext(src_cortical_area=cortical_area, src_neuron_id=neuron_id,
                                     dst_cortical_area=dst_cortical_area, dst_neuron_id=dst_neuron_id,
                                     long_term_depression=True)

                if runtime_data.parameters["Logs"]["print_plasticity_info"]:
                    print(settings.Bcolors.RED + "--------- Neuron Fire ---------"
                                                 "...........LTD between %s and %s occurred"
                          % (dst_cortical_area, dst_cortical_area)
                          + settings.Bcolors.ENDC)


        # Adding up all update times within a burst_manager span
        # total_update_time = datetime.now() - update_start_time
        # runtime_data.time_neuron_update = total_update_time + runtime_data.time_neuron_update

    # todo: not sure what's being done here. Why this is too generic on all cortical layers? !!
    # todo: Why this needs to happen on each synapse update?? !! VERY EXPENSIVE OPERATION!!!!
    # todo: Based on the initial test results, removing the following section can make the code run
    # todo: 10 times faster but result in 1/2 fitness
    # todo: add a condition to only LTP when cortical area source is upstream of cortical area dest.
    # todo: adding an impact multiplier here could be beneficial
    # This is an alternative approach to plasticity with hopefully less overhead
    # LTP or Long Term Potentiation occurs here
    upstream_data = list_upstream_neurons(cortical_area, neuron_id)

    if upstream_data:
        for src_cortical_area in upstream_data:
            for src_neuron in upstream_data[src_cortical_area]:
                if src_cortical_area != cortical_area and \
                        src_neuron in runtime_data.previous_fcl[src_cortical_area]:
                    apply_plasticity_ext(src_cortical_area=src_cortical_area,
                                         src_neuron_id=src_neuron,
                                         dst_cortical_area=cortical_area,
                                         dst_neuron_id=neuron_id, impact_multiplier=1)

    # Condition to snooze the neuron if consecutive fire count reaches threshold
    if runtime_data.brain[cortical_area][neuron_id]["consecutive_fire_cnt"] > \
            runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["consecutive_fire_cnt_max"]:
        snooze_till(cortical_area, neuron_id, runtime_data.burst_count +
                    runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["snooze_length"])

    # Condition to increase the consecutive fire count
    if runtime_data.burst_count == runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] + 1:
        runtime_data.brain[cortical_area][neuron_id]["consecutive_fire_cnt"] += 1

    # todo: rename last_burst_num to last_firing_burst
    runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] = runtime_data.burst_count

    # Condition to translate activity in utf8_out region as a character comprehension
    if cortical_area == 'utf8_memory':
        detected_item, activity_rank = utf8.convert_neuron_activity_to_utf8_char(cortical_area, neuron_id)
        # todo: burst_manager detection list could be a set instead
        if detected_item not in runtime_data.burst_detection_list:
            runtime_data.burst_detection_list[detected_item] = {}
            runtime_data.burst_detection_list[detected_item]['count'] = 1
        else:
            runtime_data.burst_detection_list[detected_item]['count'] += 1
        runtime_data.burst_detection_list[detected_item]['rank'] = activity_rank

    # # Removing the fired neuron from the FCL
    # runtime_data.fire_candidate_list[cortical_area].remove(neuron_id)

    # todo: add a check that if the firing neuron is part of OPU to perform an action
    if cortical_area == 'direction_opu':
        movement.convert_neuronal_activity_to_directions(cortical_area, neuron_id)
        # print('Movement OPU Neuron fired *** ** *** ** *** **** *')

    return


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

    if runtime_data.parameters["Auto_injector"]["injector_status"]:
        plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"]

        if long_term_depression:
            # When long term depression flag is set, there will be negative synaptic influence caused
            plasticity_constant = runtime_data.genome["blueprint"][src_cortical_area]["plasticity_constant"] * (-1) * \
                                  impact_multiplier

        try:
            runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] \
                += plasticity_constant

            # Condition to cap the postsynaptic_current and provide prohibitory reaction
            if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] > runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]:
                runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] = \
                    runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]

            # Condition to prevent postsynaptic current to become negative
            # todo: consider setting a postsynaptic_min in genome to be used instead of 0
            # Condition to prune a synapse if its postsynaptic_current is zero
            if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] < 0:
                runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id))

        except KeyError:
            synapse(src_cortical_area,
                    src_neuron_id,
                    dst_cortical_area,
                    dst_neuron_id,
                    max(plasticity_constant, 0))
            update_upstream_db(src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id)

            runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] += \
                plasticity_constant

            # Condition to cap the postsynaptic_current and provide prohibitory reaction
            if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] > \
                    runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]:
                runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] = \
                    runtime_data.genome["blueprint"][src_cortical_area]["postsynaptic_current_max"]

            # Condition to prevent postsynaptic current to become negative
            # todo: consider setting a postsynaptic_min in genome to be used instead of 0
            # Condition to prune a synapse if its postsynaptic_current is zero
            if runtime_data.brain[src_cortical_area][src_neuron_id]["neighbors"][dst_neuron_id]["postsynaptic_current"] < 0:
                runtime_data.prunning_candidates.add((src_cortical_area, src_neuron_id, dst_cortical_area, dst_neuron_id))


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
