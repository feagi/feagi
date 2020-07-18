"""
Burst engine is responsible for the event driven behavior of the artificial brain. It facilitates the firing all the
contents of the Fire Candidate List (FCL) at the same instant.

todo: Engine modes; one the super mode used for automated training and evaluations, second one with the speed of life.
todo: figure how to calibrate the speed of engine with IPU feeder
todo: assess viability of having a GIL like behavior for burst engine
todo: implement autopilot mode suitable for automated training and testing vs speed of life
todo: need a higher level mechanism to switch between life mode and autopilot mode something like falling sleep
        - let the lack of IPU activity for a period to trigger the education mode. IPU activity to wake up in life mode
        - Question: Where does IPU activity monitor belong? Thalamus? IPU controller? FCL injector?
        - Solution: FCL Injector can keep track of IPU activities in a variable and set a flag accordingly
"""
import os
import csv
import glob
import random
import string
from datetime import datetime
from inf import disk_ops
from time import sleep
from npu.physiology import *
from mem.memory import form_memories
from npu.comprehension import utf_detection_logic
from npu.feeder import Feeder
from evo.stats import *
from inf.initialize import init_burst_engine, exit_burst_process
from edu.trainer import Trainer
from edu.evaluator import Tester


def cortical_group_members(group):
    # members = []
    # for item in runtime_data.cortical_list:
    #     if runtime_data.genome['blueprint'][item]['group_id'] == group:
    #         members.append(item)
    return [item for item in runtime_data.cortical_list if runtime_data.genome['blueprint'][item]['group_id'] == group]


def burst_manager():
    """This function behaves as instance of Neuronal activities"""

    def init_fcl(cortical_area_):
        runtime_data.fire_candidate_list[cortical_area_] = set()
        runtime_data.future_fcl[cortical_area_] = set()
        runtime_data.previous_fcl[cortical_area_] = set()
        runtime_data.upstream_neurons[cortical_area_] = {}

    def save_fcl_2_dsk():
        if runtime_data.parameters["Switches"]["save_fcl_to_db"]:
            disk_ops.save_fcl_in_db(runtime_data.burst_count,
                                    runtime_data.fire_candidate_list,
                                    injector.injector_num_to_inject)

    def capture_neuron_mp():
        if runtime_data.parameters["Switches"]["capture_neuron_mp"]:
            with open(runtime_data.parameters['InitData']['connectome_path'] + '/neuron_mp.csv', 'w') as neuron_mp_file:
                neuron_mp_writer = csv.writer(neuron_mp_file, delimiter=',')
                neuron_mp_writer.writerow(('burst_number', 'cortical_layer', 'neuron_id', 'membrane_potential'))

    # def save_fcl_2_disk():
    #     # todo: *** Danger *** The following section could cause considerable memory expansion. Need to add limitations.
    #     # Condition to save FCL data to disk
    #     user_input_processing(user_input, user_input_param)
    #     if runtime_data.parameters["Switches"]["capture_brain_activities"]:
    #         runtime_data.fcl_history[runtime_data.burst_count] = runtime_data.fire_candidate_list
    #
    #     if runtime_data.parameters["Switches"]["save_fcl_to_disk"]:
    #         with open('./fcl_repo/fcl.json', 'w') as fcl_file:
    #             fcl_file.write(json.dumps(runtime_data.fire_candidate_list))
    #             fcl_file.truncate()
    #         sleep(0.5)

    def print_cortical_activity_stats():
        influxdb = db_handler.InfluxManagement()
        for _ in runtime_data.fire_candidate_list:
            if _ in runtime_data.activity_stats:
                cortical_neuron_count = len(runtime_data.fire_candidate_list[_])
                runtime_data.activity_stats[_] = max(runtime_data.activity_stats[_], cortical_neuron_count)

                if runtime_data.parameters["Switches"]["influx_stat_logger"]:
                    influxdb.insert_burst_activity(
                        connectome_path=runtime_data.parameters['InitData']['connectome_path'],
                        burst_id=runtime_data.burst_count,
                        cortical_area=_,
                        neuron_count=cortical_neuron_count)

                if runtime_data.parameters["Switches"]["global_logger"] and \
                        runtime_data.parameters["Logs"]["print_cortical_activity_counters"] and \
                        runtime_data.parameters["Auto_injector"]["injector_status"]:
                    print(settings.Bcolors.YELLOW + '    %s : %i  '
                          % (_, cortical_neuron_count)
                          + settings.Bcolors.ENDC)
                if runtime_data.parameters["Switches"]["global_logger"] and \
                        runtime_data.parameters["Logs"]["print_cortical_activity_counters"] and \
                        runtime_data.parameters["Auto_tester"]["tester_status"]:
                    print(settings.Bcolors.OKGREEN + '    %s : %i  '
                          % (_, cortical_neuron_count)
                          + settings.Bcolors.ENDC)

            else:
                runtime_data.activity_stats[cortical_area] = len(runtime_data.fire_candidate_list[cortical_area])

    def training_quality_test():
        upstream_general_stats_ = list_upstream_neuron_count_for_digits()
        for entry in upstream_general_stats_:
            if entry[1] == 0:
                print(upstream_general_stats_, "This entry was zero >", entry)
                return False

    def terminate_on_low_perf():
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
                    exit_burst_process()

    def comprehension_check():
        detected_char = utf_detection_logic(runtime_data.burst_detection_list)
        runtime_data.comprehension_queue.append(detected_char)
        runtime_data.comprehension_queue.popleft()

        counter_list = {}
        print("~~~~~~..... Burst detection list: ", runtime_data.burst_detection_list)
        if runtime_data.parameters["Logs"]["print_comprehension_queue"]:
            if runtime_data.burst_detection_list != {}:
                print(settings.Bcolors.RED + "<><><><><><><><><><><><><><>"
                                             "<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>"
                      + settings.Bcolors.ENDC)
            print(">>comprehension_queue  ", runtime_data.comprehension_queue, "  <<")

        for item in runtime_data.comprehension_queue:
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

    def capture_mem_potential():
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

    def burst_stats(burst_start_time):
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

        burst_duration = datetime.now() - burst_start_time
        if runtime_data.parameters["Logs"]["print_burst_info"]:
            print(settings.Bcolors.YELLOW +
                  ">>> Burst duration: %s %i --- ---- ---- ---- ---- ---- ----"
                  % (burst_duration, runtime_data.burst_count) + settings.Bcolors.ENDC)

    def evolutionary_checkpoint():
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

    def fire_fcl_contents():
        # time_firing_activities = datetime.now()
        # todo: replace the hardcoded vision memory statement
        if candidate_list_counter(runtime_data.fire_candidate_list) == \
                0 and not runtime_data.parameters["Auto_injector"]["injector_status"]:
            sleep(runtime_data.parameters["Timers"]["idle_burst_timer"])
            runtime_data.empty_fcl_counter += 1
            print("FCL is empty!")
        else:
            # Capture cortical activity stats
            print_cortical_activity_stats()

            for _ in runtime_data.fire_candidate_list:
                while runtime_data.fire_candidate_list[_]:
                    neuron_to_fire = runtime_data.fire_candidate_list[_].pop()
                    neuron_fire(_, neuron_to_fire)

            # Transferring future_fcl to current one and resetting the future one in process
            for _ in runtime_data.future_fcl:
                runtime_data.fire_candidate_list[_] = \
                    set([item for item in runtime_data.future_fcl[_]])
                runtime_data.future_fcl[_] = set()

    def log_neuron_activity_influx():
        if runtime_data.parameters["Switches"]["influx_stat_logger"]:
            for _ in runtime_data.fire_candidate_list:
                for neuron in runtime_data.fire_candidate_list[_]:
                    influxdb.insert_neuron_activity(connectome_path=connectome_path,
                                                    cortical_area=_,
                                                    neuron_id=neuron,
                                                    membrane_potential=
                                                    runtime_data.brain[_][neuron]["membrane_potential"] /1)

    def log_burst_activity_influx():
        if runtime_data.parameters["Switches"]["influx_stat_logger"]:
            influxdb.insert_burst_checkpoints(connectome_path, runtime_data.burst_count)

    def burst():
        sleep(1)

        burst_start_time = datetime.now()
        log_burst_activity_influx()
        runtime_data.pain_flag = False
        runtime_data.burst_count += 1

        # A deep copy of the FCL to previous FCL
        for _ in runtime_data.fire_candidate_list:
            runtime_data.previous_fcl[_] = set([item for item in runtime_data.fire_candidate_list[_]])

        # logging neuron activities to the influxdb
        log_neuron_activity_influx()

        # Fire all neurons within fire_candidate_list (FCL) or add a delay if FCL is empty
        fire_fcl_contents()

        # Auto-inject/test if applicable
        # todo: move the following functionality to the life.controller to run as a thread
        # Trainer.auto_train()
        # Tester.auto_tester()

        # The following is to have a check point to assess the perf of the in-use genome and make on the fly adj.
        evolutionary_checkpoint()

        # Saving FCL to disk for post-processing and running analytics
        save_fcl_2_dsk()

        # Monitor cortical activity levels and terminate brain if not meeting expectations
        terminate_on_low_perf()

        # Pain check
        exhibit_pain()

        # Comprehension check
        comprehension_check()

        # Forming memories through creation of cell assemblies
        # todo: instead of passing a pain flag simply detect of pain neuron is activated
        form_memories(runtime_data.fire_candidate_list, runtime_data.pain_flag)

        # Resetting burst_manager detection list
        runtime_data.burst_detection_list = {}

        # Capture Neuron Membrane Potential Stats
        capture_mem_potential()

        # Prune all prune candidate synapses
        prune_all_candidates()

        # Burst stats
        burst_stats(burst_start_time)

    print('runtime_data.genome_id = ', runtime_data.genome_id)

    # Initializing the burst_manager engine parameters
    init_burst_engine()

    injector = Feeder()
    mongo = db_handler.MongoManagement()
    influxdb = db_handler.InfluxManagement()
    connectome_path = runtime_data.parameters['InitData']['connectome_path']

    if not runtime_data.brain_is_running:
        toggle_brain_status()
        if runtime_data.parameters["Switches"]["capture_brain_activities"]:
            print(settings.Bcolors.HEADER + " *** Warning!!! *** Brain activities are being recorded!!" +
                  settings.Bcolors.ENDC)

    cortical_list = []
    for cortical_area in runtime_data.genome['blueprint']:
        cortical_list.append(cortical_area)
        init_fcl(cortical_area)
    runtime_data.cortical_list = cortical_list
    runtime_data.memory_list = cortical_group_members('Memory')

    if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        runtime_data.fcl_history = {}
    capture_neuron_mp()

    # Live mode condition
    print("live mode status: ", runtime_data.parameters["Switches"]["live_mode"], runtime_data.live_mode_status)
    if runtime_data.parameters["Switches"]["live_mode"] and runtime_data.live_mode_status == 'idle':
        runtime_data.live_mode_status = 'learning'
        print(settings.Bcolors.RED + "Starting an automated learning process..." + settings.Bcolors.ENDC)
        injector.injection_manager(injection_mode="l1", injection_param="")

    print("\n\nReady to exit burst_manager engine flag:", runtime_data.parameters["Switches"]["ready_to_exit_burst"])

    # This loop runs for the entirety of brain active life
    while not runtime_data.parameters["Switches"]["ready_to_exit_burst"]:
        burst()


def fcl_injector(fire_list, fcl_queue):
    # print("Injecting to FCL.../\/\/\/")
    # Update FCL with new input data. FCL is read from the Queue and updated
    flc = fcl_queue.get()
    for item in fire_list:
        flc.append(item)
    fcl_queue.put(flc)

    print("Injected to FCL.../\/\/\/")

    # todo: add the check so if the there is limited IPU activity for multiple consequtive rounds to set a flag

    return


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

