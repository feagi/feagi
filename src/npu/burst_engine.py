
import os
import csv
import glob
import random
import string
from datetime import datetime
from inf import runtime_data, disk_ops, db_handler
from time import sleep
from npu.physiology import *
from mem.memory import form_memories
from npu.comprehension import utf_detection_logic
from npu.injector import Injector
from evo.stats import connectome_total_synapse_cnt
from collections import deque
from evo.stats import candidate_list_counter, list_upstream_neuron_count_for_digits
from inf.initialize import burst_exit_process
from inf.disk_ops import save_fcl_to_disk
def save_fcl_to_disk():
    with open("./fcl_repo/fcl-" + runtime_data.brain_run_id + ".json", 'w') as fcl_file:
        # Saving changes to the connectome
        fcl_file.seek(0)  # rewind
        fcl_file.write(json.dumps(runtime_data.fcl_history, indent=3))
        fcl_file.truncate()

    print("Brain activities has been preserved!")

def run_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                  for _ in range(size)))+'_R'


def cortical_group_members(group):
    # members = []
    # for item in runtime_data.cortical_list:
    #     if runtime_data.genome['blueprint'][item]['group_id'] == group:
    #         members.append(item)
    return [item for item in runtime_data.cortical_list if runtime_data.genome['blueprint'][item]['group_id'] == group]


def burst():
    """This function behaves as instance of Neuronal activities"""
    print("\n\n")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** ****       Starting the burst engine...      **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("\n\n")

    print(runtime_data.parameters['Switches']['use_static_genome'])
    disk_ops.genome_handler(runtime_data.parameters['InitData']['connectome_path'])

    # todo: Move comprehension span to genome that is currently in parameters
    comprehension_span = int(runtime_data.parameters["InitData"]["comprehension_span"])

    # Initializing the comprehension queue
    comprehension_queue = deque(['-'] * comprehension_span)

    runtime_data.parameters["Auto_injector"]["injector_status"] = False
    runtime_data.termination_flag = False
    runtime_data.top_10_utf_memory_neurons = list_top_n_utf_memory_neurons("utf8_memory", 10)
    runtime_data.top_10_utf_neurons = list_top_n_utf_memory_neurons("utf8", 10)

    runtime_data.v1_members = []
    for item in runtime_data.cortical_list:
        if runtime_data.genome['blueprint'][item]['sub_group_id'] == "vision_v1":
            runtime_data.v1_members.append(item)

    injector = Injector()
    mongo = db_handler.MongoManagement()
    influxdb = db_handler.InfluxManagement()

    if not runtime_data.brain_is_running:
        toggle_brain_status()
        runtime_data.brain_run_id = run_id_gen()
        if runtime_data.parameters["Switches"]["capture_brain_activities"]:
            print(settings.Bcolors.HEADER + " *** Warning!!! *** Brain activities are being recorded!!" +
                  settings.Bcolors.ENDC)

    runtime_data.event_id = runtime_data.event_id

    print('runtime_data.genome_id = ', runtime_data.genome_id)
    cortical_list = []

    for cortical_area in runtime_data.genome['blueprint']:
        cortical_list.append(cortical_area)
        runtime_data.fire_candidate_list[cortical_area] = set()
        runtime_data.future_fcl[cortical_area] = set()
        runtime_data.previous_fcl[cortical_area] = set()
        runtime_data.upstream_neurons[cortical_area] = {}

    runtime_data.cortical_list = cortical_list
    runtime_data.memory_list = cortical_group_members('Memory')
    print("Memory list is: ", runtime_data.memory_list)

    verbose = runtime_data.parameters["Switches"]["verbose"]

    if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        runtime_data.fcl_history = {}

    if runtime_data.parameters["Switches"]["capture_neuron_mp"]:
        with open(runtime_data.parameters['InitData']['connectome_path'] + '/neuron_mp.csv', 'w') as neuron_mp_file:
            neuron_mp_writer = csv.writer(neuron_mp_file, delimiter=',')
            neuron_mp_writer.writerow(('burst_number', 'cortical_layer', 'neuron_id', 'membrane_potential'))

    print("** ** ** Live mode, live mode status: ",
          runtime_data.parameters["Switches"]["live_mode"],
          runtime_data.live_mode_status)

    # Live mode condition
    if runtime_data.parameters["Switches"]["live_mode"] and runtime_data.live_mode_status == 'idle':
        runtime_data.live_mode_status = 'learning'
        print(
            settings.Bcolors.RED + "Starting an automated learning process...<> <> <> <>" + settings.Bcolors.ENDC)
        injector.injection_manager(injection_mode="l1", injection_param="")

    print("\n\n >> >> >> Ready to exist burst engine flag:", runtime_data.parameters["Switches"]["ready_to_exit_burst"])

    connectome_path = runtime_data.parameters['InitData']['connectome_path']
    while not runtime_data.parameters["Switches"]["ready_to_exit_burst"]:
        if runtime_data.parameters["Switches"]["influx_stat_logger"]:
            influxdb.insert_burst_checkpoints(connectome_path, runtime_data.burst_count)
        burst_start_time = datetime.now()
        runtime_data.pain_flag = False
        now = datetime.now()

        # print(datetime.now(), "Burst count = ", runtime_data.burst_count, file=open("./logs/burst.log", "a"))

        for cortical_area in runtime_data.fire_candidate_list:
            runtime_data.previous_fcl[cortical_area] = \
                set([item for item in runtime_data.fire_candidate_list[cortical_area]])

        runtime_data.burst_count += 1

        # logging neuron activities to the influxdb
        if runtime_data.parameters["Switches"]["influx_stat_logger"]:
            connectome_path = runtime_data.parameters['InitData']['connectome_path']
            for cortical_area in runtime_data.fire_candidate_list:
                for neuron in runtime_data.fire_candidate_list[cortical_area]:
                    influxdb.insert_neuron_activity(connectome_path=connectome_path,
                                                    cortical_area=cortical_area,
                                                    neuron_id=neuron,
                                                    membrane_potential=
                                                    runtime_data.brain[cortical_area][neuron]["membrane_potential"] /1)

        # Fire all neurons within fire_candidate_list (FCL) or add a delay if FCL is empty
        # time_firing_activities = datetime.now()
        # todo: replace the hardcoded vision memory statement
        if candidate_list_counter(runtime_data.fire_candidate_list) == \
                0 and not runtime_data.parameters["Auto_injector"]["injector_status"]:
            sleep(runtime_data.parameters["Timers"]["idle_burst_timer"])
            runtime_data.empty_fcl_counter += 1
            print("FCL is empty!")
        else:
            # Capture cortical activity stats
            for cortical_area in runtime_data.fire_candidate_list:
                if cortical_area in runtime_data.activity_stats:
                    cortical_neuron_count = len(runtime_data.fire_candidate_list[cortical_area])
                    runtime_data.activity_stats[cortical_area] = max(runtime_data.activity_stats[cortical_area],
                                                                     cortical_neuron_count)

                    if runtime_data.parameters["Switches"]["influx_stat_logger"]:
                        influxdb.insert_burst_activity(connectome_path=connectome_path,
                                                       burst_id=runtime_data.burst_count,
                                                       cortical_area=cortical_area,
                                                       neuron_count=cortical_neuron_count)

                    if runtime_data.parameters["Switches"]["global_logger"] and \
                            runtime_data.parameters["Logs"]["print_cortical_activity_counters"] and \
                            runtime_data.parameters["Auto_injector"]["injector_status"]:
                        print(settings.Bcolors.YELLOW + '    %s : %i  '
                              % (cortical_area, cortical_neuron_count)
                              + settings.Bcolors.ENDC)
                    if runtime_data.parameters["Switches"]["global_logger"] and \
                            runtime_data.parameters["Logs"]["print_cortical_activity_counters"] and \
                            runtime_data.parameters["Auto_tester"]["tester_status"]:
                        print(settings.Bcolors.OKGREEN + '    %s : %i  '
                              % (cortical_area, cortical_neuron_count)
                              + settings.Bcolors.ENDC)

                else:
                    runtime_data.activity_stats[cortical_area] = len(runtime_data.fire_candidate_list[cortical_area])

            # todo: Look into multi-threading for Neuron neuron_fire and wire_neurons function
            # Firing all neurons in the Fire Candidate List
            # Fire all neurons in FCL
            time_actual_firing_activities = datetime.now()
            # now = datetime.now()
            # runtime_data.time_neuron_update = datetime.now() - now
            # runtime_data.plasticity_time_total = datetime.now() - datetime.now()
            # runtime_data.plasticity_time_total_p1 = datetime.now() - datetime.now()
            # stats_utf_memory_membrane_potentials()
            # Firing all neurons in the fire_candidate_list

            for cortical_area in runtime_data.fire_candidate_list:
                while runtime_data.fire_candidate_list[cortical_area]:
                    neuron_to_fire = runtime_data.fire_candidate_list[cortical_area].pop()
                    neuron_fire(cortical_area, neuron_to_fire)

            # stats_utf_memory_membrane_potentials()
            # pfcl_total_neuron_count = candidate_list_counter(runtime_data.previous_fcl)
            # cfcl_total_neuron_count = candidate_list_counter(runtime_data.fire_candidate_list)

            # print("PFCL:", pfcl_total_neuron_count,
            #       "\nCFCL:", cfcl_total_neuron_count,
            #       "\nFFCL:", candidate_list_counter(runtime_data.future_fcl))

            # Transferring future_fcl to current one and resetting the future one in process
            for cortical_area in runtime_data.future_fcl:
                runtime_data.fire_candidate_list[cortical_area] = \
                    set([item for item in runtime_data.future_fcl[cortical_area]])
                runtime_data.future_fcl[cortical_area] = set()

            # try:
            #     print("Timing : .__________ Firing ops...........:",
            #           (datetime.now() - time_actual_firing_activities - runtime_data.time_neuron_update) /
            #           pfcl_total_neuron_count)
            #     print("Timing : |___________Neuron updates.......:",
            #           runtime_data.time_neuron_update / pfcl_total_neuron_count)
            #     print("Timing :             |__Ext plasticity....:",
            #           runtime_data.plasticity_time_total / pfcl_total_neuron_count)
            #     print("Timing :                |___Ext plast. P1.:",
            #           runtime_data.plasticity_time_total_p1 / pfcl_total_neuron_count)
            #
            #     print("\nTiming : Average time per fire ....................:",
            #           (datetime.now() - time_firing_activities) /
            #           pfcl_total_neuron_count)
            #     print("\nTiming : Total firing time per FCL.................:", datetime.now() - time_firing_activities)
            #
            # except ZeroDivisionError:
            #     pass

            if verbose:
                print(settings.Bcolors.YELLOW + 'Current fire_candidate_list is %s'
                      % runtime_data.fire_candidate_list + settings.Bcolors.ENDC)

            # print_cortical_neuron_mappings('vision_memory', 'utf8_memory')



        # todo: need to break down the training function into pieces with one feeding a stream of data
        # Auto-inject if applicable
        if runtime_data.parameters["Auto_injector"]["injector_status"]:
            # injection_time = datetime.now()
            # print("-------------------------++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ auto_injector")
            injector.auto_injector()
            # print("Timing : Injection:", datetime.now() - injection_time)

        # Auto-test if applicable
        if runtime_data.parameters["Auto_tester"]["tester_status"]:
            # test_time = datetime.now()
            injector.auto_tester()
            # print("Timing : Test:", datetime.now() - test_time)

        # todo: The following is to have a check point to assess the perf of the in-use genome and make on the fly adj.
        if runtime_data.burst_count % runtime_data.genome['evolution_burst_count'] == 0:
            print('Evolution phase reached...')
            for area in runtime_data.cortical_list:
                neuron_count, synapse_count = connectome_total_synapse_cnt(area)
                if runtime_data.parameters["Switches"]["influx_stat_logger"]:
                    influxdb.insert_connectome_stats(connectome_path=connectome_path,
                                                     cortical_area=area,
                                                     neuron_count=neuron_count,
                                                     synapse_count=synapse_count)
            # genethesizer.generation_assessment()

        # Saving FCL to disk for post-processing and running analytics
        if runtime_data.parameters["Switches"]["save_fcl_to_db"]:
            disk_ops.save_fcl_in_db(runtime_data.burst_count,
                                    runtime_data.fire_candidate_list,
                                    injector.injector_num_to_inject)

        detected_char = utf_detection_logic(runtime_data.burst_detection_list)
        comprehension_queue.append(detected_char)
        comprehension_queue.popleft()

        def training_quality_test():
            upstream_general_stats_ = list_upstream_neuron_count_for_digits()
            for entry in upstream_general_stats_:
                if entry[1] == 0:
                    print(upstream_general_stats_, "This entry was zero >", entry)
                    return False

        # Monitor cortical activity levels and terminate brain if not meeting expectations
        # time_monitoring_cortical_activity = datetime.now()
        if runtime_data.parameters["Switches"]["evaluation_based_termination"]:
            if runtime_data.parameters["Auto_injector"]["injector_status"] and \
                    runtime_data.burst_count > int(runtime_data.parameters["InitData"]["kill_trigger_burst_count"]):
                if 'vision_memory' not in runtime_data.activity_stats:
                    runtime_data.activity_stats['vision_memory'] = 0
                elif runtime_data.activity_stats['vision_memory'] < \
                        int(runtime_data.parameters["InitData"]["kill_trigger_vision_memory_min"]):
                    print(settings.Bcolors.RED +
                          "\n\n\n\n\n\n!!!!! !! !Terminating the brain due to low performance! !! !!!" +
                          settings.Bcolors.ENDC)
                    print("vision_memory max activation was:", runtime_data.activity_stats['vision_memory'])
                    runtime_data.termination_flag = True
                    burst_exit_process()

        # print("Timing : Monitoring cortical activity:", datetime.now()-time_monitoring_cortical_activity)

        # Pain check
        if runtime_data.pain_flag:
            exhibit_pain()

        # Comprehension check
        # time_comprehension_check = datetime.now()
        counter_list = {}
        print("~~~~~~..... Burst detection list: ", runtime_data.burst_detection_list)
        if runtime_data.parameters["Logs"]["print_comprehension_queue"]:
            if runtime_data.burst_detection_list != {}:
                print(settings.Bcolors.RED + "<><><><><><><><><><><><><><>"
                                             "<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>"
                      + settings.Bcolors.ENDC)
            print(">>comprehension_queue  ", comprehension_queue, "  <<")

        for item in comprehension_queue:
            if item in counter_list:
                counter_list[item] += 1
            else:
                counter_list[item] = 1
        list_length = len(counter_list)

        if runtime_data.parameters["Logs"]["print_comprehension_queue"]:
            print("+++++++This is the counter list", counter_list)
        for item in counter_list:
            if list_length == 1 and item != '-':
                runtime_data.parameters["Input"]["comprehended_char"] = item[0]
                print(settings.Bcolors.HEADER + "UTF8 out was stimulated with the following character: <<< %s >>>"
                      % runtime_data.parameters["Input"]["comprehended_char"] + settings.Bcolors.ENDC)
                # In the event that the comprehended UTF character is not matching the injected one pain is triggered
                if runtime_data.parameters["Input"]["comprehended_char"] != str(runtime_data.labeled_image[1]):
                    trigger_pain()
                    runtime_data.pain_flag = True
            else:
                if list_length >= 2:
                    runtime_data.parameters["Input"]["comprehended_char"] = ''

        # Forming memories through creation of cell assemblies
        if runtime_data.parameters["Switches"]["memory_formation"]:
            # memory_formation_start_time = datetime.now()
            # todo: instead of passing a pain flag simply detect of pain neuron is activated
            form_memories(runtime_data.fire_candidate_list, runtime_data.pain_flag)
            # print("    Memory formation took--",datetime.now()-memory_formation_start_time)
        # print("Timing : Comprehension check:", datetime.now() - time_comprehension_check)

        # Burst stats
        if runtime_data.parameters["Logs"]["print_burst_stats"]:
            for area in runtime_data.brain:
                print("### Average postSynaptic current in --- %s --- was: %i"
                      % (area, average_postsynaptic_current(area)))

        if runtime_data.parameters["Logs"]["print_upstream_neuron_stats"]:
            # Listing the number of neurons activating each UTF memory neuron
            upstream_report_time = datetime.now()
            upstream_general_stats, upstream_fcl_stats = \
                list_upstream_neuron_count_for_digits(mode=1)
            print("list_upstream_neuron_count_for_digits:", upstream_general_stats)
            print("list_upstream___FCL__count_for_digits:", upstream_fcl_stats)

            print("Timing : Upstream + common neuron report:", datetime.now() - upstream_report_time)

        if runtime_data.parameters["Logs"]["print_common_neuron_report"]:
            # todo: investigate the efficiency of the common neuron report
            print("The following is the common neuron report:")
            common_neuron_report()

        # Resetting burst detection list
        runtime_data.burst_detection_list = {}

        # todo: *** Danger *** The following section could cause considerable memory expansion. Need to add limitations.
        # # Condition to save FCL data to disk
        # user_input_processing(user_input, user_input_param)
        # if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        #     runtime_data.fcl_history[runtime_data.burst_count] = runtime_data.fire_candidate_list
        #
        # if runtime_data.parameters["Switches"]["save_fcl_to_disk"]:
        #     with open('./fcl_repo/fcl.json', 'w') as fcl_file:
        #         fcl_file.write(json.dumps(runtime_data.fire_candidate_list))
        #         fcl_file.truncate()
        #     sleep(0.5)

        # todo: This is the part to capture the neuron membrane potential values in a file, still need to figure how
        if runtime_data.parameters["Switches"]["capture_neuron_mp"]:
            with open(runtime_data.parameters['InitData']['connectome_path'] + '/neuron_mp.csv', 'a') as neuron_mp_file:
                neuron_mp_writer = csv.writer(neuron_mp_file,  delimiter=',')
                new_data = []
                for cortical_area in runtime_data.fire_candidate_list:
                    for neuron in runtime_data.fire_candidate_list[cortical_area]:
                        new_content = (runtime_data.burst_count, cortical_area, neuron,
                                       runtime_data.brain[cortical_area][neuron]["membrane_potential"])
                        new_data.append(new_content)
                neuron_mp_writer.writerows(new_data)

        if runtime_data.parameters["Switches"]["capture_neuron_mp_db"]:
            new_data = []
            for cortical_area in runtime_data.fire_candidate_list:
                for neuron in runtime_data.fire_candidate_list[cortical_area]:
                    new_content = (runtime_data.burst_count, cortical_area, neuron,
                                   runtime_data.brain[cortical_area][neuron]["membrane_potential"])
                    new_data.append(new_content)
                    mongo.inset_membrane_potentials(new_content)

        # Prune all prune candidate synapses
        pruning_start_time = datetime.now()
        prune_all_candidates()
        print("Timing : Pruning:", datetime.now() - pruning_start_time)

        burst_duration = datetime.now() - burst_start_time
        if runtime_data.parameters["Logs"]["print_burst_info"]:
            print(settings.Bcolors.YELLOW +
                  ">>> Burst duration: %s %i --- ---- ---- ---- ---- ---- ----"
                  % (burst_duration, runtime_data.burst_count) + settings.Bcolors.ENDC)


def fire_candidate_locations(fire_cnd_list):
    """Extracts Neuron locations from the fire_candidate_list"""

    # print('***')
    # print(fire_cnd_list)

    neuron_locations = {}
    # Generate a dictionary of cortical areas in the fire_candidate_list
    for item in runtime_data.cortical_list:
        neuron_locations[item] = []

    # Add neuron locations under each cortical area
    for cortical_area in fire_cnd_list:
        for neuron in fire_cnd_list[cortical_area]:
            neuron_locations[cortical_area].append([runtime_data.brain[cortical_area][neuron]["location"][0],
                                                    runtime_data.brain[cortical_area][neuron]["location"][1],
                                                    runtime_data.brain[cortical_area][neuron]["location"][2]])

    return neuron_locations


def list_top_n_utf_memory_neurons(cortical_area, n):
    neuron_list = []
    counter = ord('0')
    the_other_counter = 0
    for neuron_id in runtime_data.brain[cortical_area]:
        if int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2]) == counter:
            neuron_list.append([int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])-48, neuron_id])
            counter += 1
            the_other_counter += 1
            if the_other_counter == n:
                return neuron_list
    print("ERROR: Something went wrong in list_top_n_utf_memory_neurons")


def list_common_upstream_neurons(neuron_a, neuron_b):
    common_neuron_list = []

    try:
        neuron_a_upstream_neurons = runtime_data.upstream_neurons['utf8_memory'][neuron_a]['vision_memory']
        neuron_b_upstream_neurons = runtime_data.upstream_neurons['utf8_memory'][neuron_b]['vision_memory']
        for neuron in neuron_a_upstream_neurons:
            if neuron in neuron_b_upstream_neurons:
                common_neuron_list.append(neuron)
        return common_neuron_list

    except:
        pass


def load_fcl_in_memory(file_name):
    with open(file_name, 'r') as fcl_file:
        fcl_data = json.load(fcl_file)
    return fcl_data


def latest_fcl_file():
    list_of_files = glob.glob('./fcl_repo/*.json')  # * means all if need specific format then *.csv
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file


def toggle_verbose_mode():
    if runtime_data.parameters["Switches"]["verbose"]:
        runtime_data.parameters["Switches"]["verbose"] = False
        print("Verbose mode is Turned OFF!")
    else:
        runtime_data.parameters["Switches"]["verbose"] = True
        print("Verbose mode is Turned On!")


def toggle_brain_status():
    if runtime_data.brain_is_running:
        runtime_data.brain_is_running = False
        print("Brain is not running!")
    else:
        runtime_data.brain_is_running = True
        print("Brain is now running!!!")


def inject_to_fcl(fire_list, fcl_queue):
    # print("Injecting to FCL.../\/\/\/")
    # Update FCL with new input data. FCL is read from the Queue and updated
    flc = fcl_queue.get()
    for item in fire_list:
        flc.append(item)
    fcl_queue.put(flc)
    # print("Injected to FCL.../\/\/\/")
    return


def utf_neuron_id(n):
    # Returns the neuron id associated with a particular digit
    for neuron_id in runtime_data.brain['utf8_memory']:
        if int(runtime_data.brain['utf8_memory'][neuron_id]["soma_location"][0][2]) == n+ord('0'):
            return neuron_id


def common_neuron_report():
    digits = range(10)
    number_matrix = []
    for _ in digits:
        for __ in digits:
            if _ != __ and [_, __] not in number_matrix and [__, _] not in number_matrix:
                number_matrix.append([_, __])
    for item in number_matrix:
        neuron_a = utf_neuron_id(item[0])
        neuron_b = utf_neuron_id(item[1])
        common_neuron_list = list_common_upstream_neurons(neuron_a, neuron_b)

        if common_neuron_list:
            overlap_amount = len(common_neuron_list)
            # print(item, '> ', overlap_amount)

            if overlap_amount > runtime_data.parameters["InitData"]["overlap_prevention_constant"]:
                # The following action is taken to eliminate the overlap
                for neuron in common_neuron_list:
                    runtime_data.prunning_candidates.add(('vision_memory', neuron, 'utf8_memory', neuron_a))
                    runtime_data.prunning_candidates.add(('vision_memory', neuron, 'utf8_memory', neuron_b))

