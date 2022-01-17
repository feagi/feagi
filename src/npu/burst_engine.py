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
from datetime import datetime
from inf import disk_ops
from time import sleep
from npu.physiology import *
from mem.memory import neuroplasticity
from npu.comprehension import utf_detection_logic
from ipu.ipu_controller import ipu_handler
from opu.opu_controller import opu_handler
from evo.stats import *
from inf.initialize import init_burst_engine, exit_burst_process
from inf.messenger import Pub, Sub, PubBrainActivities


def cortical_group_members(group):
    # members = []
    # for item in runtime_data.cortical_list:
    #     if runtime_data.genome['blueprint'][item]['group_id'] == group:
    #         members.append(item)
    return [item for item in runtime_data.cortical_list if runtime_data.genome['blueprint'][item]['group_id'][:1]
            == group]


def burst_manager():
    """This function behaves as instance of Neuronal activities"""

    def consciousness_manager():
        """responsible for start and stop of all non-main threads based on various conditions"""
        # Check flags for IPU activities
        # todo: need mechanism to set the ipu_idle flag if there is no IPU activity for a period
        # Alert condition checks to ensure brain is not in Alert mode which can be triggered via fear or cautiousness
        elapsed_time = datetime.now() - runtime_data.last_alertness_trigger
        alert_condition = elapsed_time.seconds > int(runtime_data.parameters['Timers']['alert_mode_duration'])
        if alert_condition:
            time_delta = datetime.now() - runtime_data.last_ipu_activity
            if time_delta.seconds > int(runtime_data.parameters['IPU']['idle_threshold']):
                # Go to sleep by stopping IPU/OPU threads
                # todo: instead of turning off the IPU, reduce IPU responsiveness so via an trigger brain can awake
                print(">> >> Brain going to sleep..")

                # todo: adjust burst frequency

        # todo: implementation of coming out of sleep
        # one trigger to be large activity on IPU and another to be time-bound
        ready_to_wake = False
        if ready_to_wake:
            print(">> >> Brain waking up from sleep..")

            # todo: adjust burst frequency

    def init_fcl(cortical_area_):
        runtime_data.fire_candidate_list[cortical_area_] = set()
        runtime_data.future_fcl[cortical_area_] = set()
        runtime_data.previous_fcl[cortical_area_] = set()
        runtime_data.upstream_neurons[cortical_area_] = {}

    def save_fcl_2_dsk():
        if runtime_data.parameters["Switches"]["save_fcl_to_db"]:
            disk_ops.save_fcl_in_db(runtime_data.burst_count,
                                    runtime_data.fire_candidate_list,
                                    feeder.injector_num_to_inject)

    def capture_neuron_mp():
        if runtime_data.parameters["Switches"]["capture_neuron_mp"]:
            with open(runtime_data.parameters['InitData']['connectome_path'] + '/neuron_mp.csv', 'w') as neuron_mp_file:
                neuron_mp_writer = csv.writer(neuron_mp_file, delimiter=',')
                neuron_mp_writer.writerow(('burst_number', 'cortical_layer', 'neuron_id', 'membrane_potential'))

    # def save_fcl_2_disk():
    #     # todo: * Danger * The following section could cause considerable memory expansion. Need to add limitations.
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

    def capture_cortical_activity_stats():
        # print('@@@--- Activity Stats:', runtime_data.activity_stats)
        for cortical_area_ in runtime_data.fire_candidate_list:
            if cortical_area_ in runtime_data.activity_stats:
                cortical_neuron_count = len(runtime_data.fire_candidate_list[cortical_area_])
                runtime_data.activity_stats[cortical_area_] = \
                    max(runtime_data.activity_stats[cortical_area_], cortical_neuron_count)

                if (runtime_data.parameters["Database"]["influxdb_enabled"] and 
                        runtime_data.influxdb and 
                        runtime_data.parameters["Database"]["influx_stat_logger"]):
                    runtime_data.influxdb.insert_burst_activity(
                        connectome_path=runtime_data.parameters['InitData']['connectome_path'],
                        burst_id=runtime_data.burst_count,
                        cortical_area=cortical_area_,
                        neuron_count=cortical_neuron_count)

                if runtime_data.parameters["Switches"]["global_logger"] and \
                        runtime_data.parameters["Logs"]["print_cortical_activity_counters"]:
                    if cortical_neuron_count > 0:
                        print(settings.Bcolors.RED + '    %s : %i  '
                              % (cortical_area_, cortical_neuron_count)
                              + settings.Bcolors.ENDC)
                    elif runtime_data.parameters["Logs"]["print_cortical_activity_counters_all"]:
                        print(settings.Bcolors.YELLOW + '    %s : %i  '
                              % (cortical_area_, cortical_neuron_count)
                              + settings.Bcolors.ENDC)

            else:
                try:
                    runtime_data.activity_stats[cortical_area_] = len(runtime_data.fire_candidate_list[cortical_area_])
                except KeyError:
                    print("Error: Cortical Area not found:", cortical_area_)

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
        # print("~~~~~~..... Burst detection list: ", runtime_data.burst_detection_list)
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
                try:
                    if runtime_data.parameters["Input"]["comprehended_char"] != str(runtime_data.labeled_image[1]):
                        trigger_pain()
                        runtime_data.pain_flag = True
                finally:
                    pass
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
                    runtime_data.mongodb.inset_membrane_potentials(new_content)

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
            if runtime_data.genome_ver == "2.0":
                print(settings.Bcolors.UPDATE +
                      ">>> Burst duration: %s %i --- ---- ---- ---- ---- ---- ----"
                      % (burst_duration, runtime_data.burst_count) + settings.Bcolors.ENDC)
            else:
                print(settings.Bcolors.YELLOW +
                      ">>> Burst duration: %s %i --- ---- ---- ---- ---- ---- ----"
                      % (burst_duration, runtime_data.burst_count) + settings.Bcolors.ENDC)

    def evolutionary_checkpoint():
        if runtime_data.burst_count % runtime_data.genome['evolution_burst_count'] == 0:
            print('Evolution phase reached...')
            for area in runtime_data.cortical_list:
                neuron_count, synapse_count = connectome_total_synapse_cnt(area)
                if (runtime_data.parameters["Database"]["influxdb_enabled"] and 
                        runtime_data.influxdb and 
                        runtime_data.parameters["Database"]["influx_stat_logger"]):
                    runtime_data.influxdb.insert_connectome_stats(connectome_path=connectome_path,
                                                     cortical_area=area,
                                                     neuron_count=neuron_count,
                                                     synapse_count=synapse_count)
            # genethesizer.generation_assessment()

    def fire_fcl_contents():
        # time_firing_activities = datetime.now()
        # todo: replace the hardcoded vision memory statement
        if candidate_list_counter(runtime_data.fire_candidate_list) == \
                0 and not runtime_data.parameters["Auto_injector"]["injector_status"]:
            # sleep(float(runtime_data.parameters["Timers"]["idle_burst_timer"]))
            runtime_data.empty_fcl_counter += 1
            print("FCL is empty!")
        else:
            # Capture cortical activity stats
            capture_cortical_activity_stats()

            for _ in runtime_data.fire_candidate_list:
                if runtime_data.genome['blueprint'][_].get('degeneration'):
                    degeneration_val = runtime_data.genome['blueprint'][_]['degeneration']
                    while runtime_data.fire_candidate_list[_]:
                        neuron_to_fire = runtime_data.fire_candidate_list[_].pop()
                        neuron_fire(_, neuron_to_fire, degenerate=degeneration_val)
                else:
                    while runtime_data.fire_candidate_list[_]:
                        neuron_to_fire = runtime_data.fire_candidate_list[_].pop()
                        neuron_fire(_, neuron_to_fire)
            # Transferring future_fcl to current one and resetting the future one in process
            for _ in runtime_data.future_fcl:
                runtime_data.fire_candidate_list[_] = \
                    set([item for item in runtime_data.future_fcl[_]])
                runtime_data.future_fcl[_] = set()

    def log_neuron_activity_influx():
        if (runtime_data.parameters["Database"]["influxdb_enabled"] and 
                runtime_data.influxdb and 
                runtime_data.parameters["Database"]["influx_stat_logger"]):
            for _ in runtime_data.fire_candidate_list:
                for neuron in runtime_data.fire_candidate_list[_]:
                    runtime_data.influxdb.insert_neuron_activity(connectome_path=connectome_path,
                                                    cortical_area=_,
                                                    neuron_id=neuron,
                                                    membrane_potential=
                                                    runtime_data.brain[_][neuron]["membrane_potential"] /1)

    def log_burst_activity_influx():
        if (runtime_data.parameters["Database"]["influxdb_enabled"] and 
                runtime_data.influxdb and 
                runtime_data.parameters["Database"]["influx_stat_logger"]):
            runtime_data.influxdb.insert_burst_checkpoints(connectome_path, runtime_data.burst_count)

    def fake_cortical_stimulation(input_instruction, burst_count):
        """
        It fakes cortical stimulation for the purpose of testing

        The following data format is used for input_instruction as the function input:

        input_instructions receives a dictionary as input with keys as the name of the ipu cortical name and the value
        being a list of block locations that needs to be activated in the block-ref format e.g. xBlock-yBlock-zBlock.

        Note: all of the blocks outlined in the data structure will be activated at the same time during the same
        burst.

        input_instruction_example = {
            ir_ipu: ["0-0-0", "1-0-0"],
            proximity_ipu: ["0-0-0", "0-0-3", "0-0-10", "0-0-20"]
            led_opu: ["5-0-0"]
        }

        # todo: Currently we can only inject data from the first index on each burst. change it so it goes thru all
        """
        neuron_list = []

        for cortical_area_ in input_instruction[burst_count]:
            if cortical_area_ in runtime_data.block_dic:
                for block_ref in input_instruction[burst_count][cortical_area_]:
                    if block_ref in runtime_data.block_dic[cortical_area_]:
                        for neuron in runtime_data.block_dic[cortical_area_][block_ref]:
                            neuron_list.append(neuron)
                    else:
                        print("Warning: Block ref %s was not found for %s" % (block_ref, cortical_area_))
                # print("neuron list:", cortical_area_, neuron_list)
                runtime_data.fcl_queue.put({cortical_area_: set(neuron_list)})
                neuron_list = []
            else:
                print("Warning: Cortical area %s not found within the block_dic" % cortical_area_)

    def sensory_message_router():
        # Broadcasts a TCP message on each burst
        if runtime_data.parameters['Switches']['burst_beacon']:
            # Limiting the broadcast messages to one in every 10 burst
            # todo: externalize this parameter to ini
            if runtime_data.burst_count % 10 == 0:
                broadcast_message = dict()
                broadcast_message['burst_counter'] = runtime_data.burst_count
                broadcast_message['sockets'] = runtime_data.parameters['Sockets']
                broadcast_message['burst_frequency'] = runtime_data.burst_timer
                burst_beacon.send(message=broadcast_message)

        # IPU listener: Receives IPU data through ZMQ channel
        if runtime_data.router_address is not None:
            ipu_data = ipu_listener.receive()
            if ipu_data:
                ipu_handler(ipu_data)
                if runtime_data.parameters["Logs"]["print_burst_info"]:
                    print("FEAGI received message from router as:", ipu_data)

    def brain_activity_message_router():
        # Broadcasts a TCP message on each burst
        if runtime_data.parameters['Switches']['zmq_activity_publisher']:
            # Limiting the broadcast messages to one in every 1 burst
            # todo: externalize this parameter to ini
            if runtime_data.burst_count % 1 == 0:
                broadcast_message = brain_activity_voxelizer()
                brain_activity_beacon.send(message=broadcast_message)

    def brain_activity_voxelizer():
        """
        Convert FCL activities to a set of voxel locations and sends out through the ZMQ publisher
        """
        broadcast_message = set()

        for _ in runtime_data.fire_candidate_list:
            fire_list = set(runtime_data.fire_candidate_list[_])
            if runtime_data.genome['blueprint'][_]['neuron_params'].get('visualization'):
                while fire_list:
                    firing_neuron = fire_list.pop()
                    firing_neuron_loc = runtime_data.brain[_][firing_neuron]['soma_location'][1]
                    relative_coords = runtime_data.genome['blueprint'][_]['neuron_params'].get('relative_coordinate')
                    broadcast_message.add(
                        (
                            runtime_data.burst_count,
                            firing_neuron_loc[0] + relative_coords[0],
                            firing_neuron_loc[1] + relative_coords[1],
                            firing_neuron_loc[2] + relative_coords[2]
                        )
                    )

        return broadcast_message

    def burst():
        # todo: the following sleep value should be tied to Autopilot status
        sleep(float(runtime_data.burst_timer))

        burst_start_time = datetime.now()
        log_burst_activity_influx()
        runtime_data.pain_flag = False
        runtime_data.burst_count += 1

        # A deep copy of the FCL to previous FCL
        for _ in runtime_data.fire_candidate_list:
            runtime_data.previous_fcl[_] = set([item for item in runtime_data.fire_candidate_list[_]])

        fcl_tmp = set()

        # Manage ZMQ communication from and to FEAGI
        sensory_message_router()

        # Feeding FCL queue content into the FCL
        while not runtime_data.fcl_queue.empty():
            fcl_tmp = runtime_data.fcl_queue.get()

            for _ in fcl_tmp:
                runtime_data.fire_candidate_list[_] = \
                    set([item for item in fcl_tmp[_]])
                fcl_tmp = set()

        # logging neuron activities to the influxdb
        log_neuron_activity_influx()
        
        # Fake Stimuli
        if runtime_data.parameters['Switches']['fake_stimulation_flag']:
            if runtime_data.burst_count in runtime_data.stimulation_data:
                fake_cortical_stimulation(input_instruction=runtime_data.stimulation_data,
                                          burst_count=runtime_data.burst_count)

        # Process neuron stimulation that ties to OPU
        opu_handler()

        # Publish brain activities on ZMQ
        if runtime_data.parameters['Switches']['zmq_activity_publisher']:
            brain_activity_message_router()

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
        neuroplasticity(runtime_data.fire_candidate_list, runtime_data.pain_flag)

        # Resetting burst_manager detection list
        runtime_data.burst_detection_list = {}

        # Capture Neuron Membrane Potential Stats
        capture_mem_potential()

        # Prune all prune candidate synapses
        prune_all_candidates()

        # Burst stats
        burst_stats(burst_start_time)

        # Manage Threads
        # For performance reasons, running this function not on every single burst
        if runtime_data.burst_count % 10 == 0:
            consciousness_manager()

    print('runtime_data.genome_id = ', runtime_data.genome_id)

    # Initializing the burst_manager engine parameters
    init_burst_engine()

    # Initialize a broadcaster
    if runtime_data.parameters["Switches"]["burst_beacon"]:
        burst_engine_pub_address = 'tcp://0.0.0.0:' + runtime_data.parameters['Sockets']['burst_engine_pub']
        burst_beacon = Pub(address=burst_engine_pub_address)
    if runtime_data.parameters["Switches"]["zmq_activity_publisher"]:
        brain_activity_pub_address = 'tcp://0.0.0.0:' + runtime_data.parameters['Sockets']['brain_activities_pub']
        brain_activity_beacon = PubBrainActivities(address=brain_activity_pub_address)

    # Initialize IPU listener
    if runtime_data.router_address is not None:
        print("subscribing to ", runtime_data.router_address)
        ipu_listener = Sub(address=runtime_data.router_address)

    # todo: need to figure how to incorporate FCL injection
    # feeder = Feeder()
    # mongo = db_handler.MongoManagement()
    # influxdb = db_handler.InfluxManagement()
    connectome_path = runtime_data.parameters['InitData']['connectome_path']

    if not runtime_data.brain_is_running:
        toggle_brain_status()
        if runtime_data.parameters["Switches"]["capture_brain_activities"]:
            print(settings.Bcolors.HEADER + " *** Warning!!! *** Brain activities are being recorded!!" +
                  settings.Bcolors.ENDC)

    # cortical_list = []
    for cortical_area in runtime_data.cortical_list:
        # cortical_list.append(cortical_area)
        init_fcl(cortical_area)
        print("%%#$%@$%@$#%@#$% @# $% @#$ % @ % #$% @#$ %@#$ %@ $       ", cortical_area)
    # runtime_data.cortical_list = cortical_list

    runtime_data.memory_list = cortical_group_members('m')
    print("runtime_data.memory_list=", runtime_data.memory_list)

    if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        runtime_data.fcl_history = {}
    capture_neuron_mp()

    # Live mode condition
    # todo: This segment to be replaced with auto-pilot code
    # print("live mode status: ", runtime_data.parameters["Switches"]["live_mode"], runtime_data.live_mode_status)
    # if runtime_data.parameters["Switches"]["live_mode"] and runtime_data.live_mode_status == 'idle':
    #     runtime_data.live_mode_status = 'learning'
    #     print(settings.Bcolors.RED + "Starting an automated learning process..." + settings.Bcolors.ENDC)
    #     feeder.injection_manager(injection_mode="l1", injection_param="")

    print("\n\nReady to exit burst_manager engine flag:", runtime_data.parameters["Switches"]["ready_to_exit_burst"])

    # This loop runs for the entirety of brain active life
    while not runtime_data.parameters["Switches"]["ready_to_exit_burst"]:
        burst()


def fcl_feeder(fire_list, fcl_queue):
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
            neuron_locations[cortical_area].append([runtime_data.brain[cortical_area][neuron]["soma_location"][0],
                                                    runtime_data.brain[cortical_area][neuron]["soma_location"][1],
                                                    runtime_data.brain[cortical_area][neuron]["soma_location"][2]])

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


def eval(self):
    # Effectiveness check
    if runtime_data.parameters["Switches"]["evaluation_based_termination"]:
        upstream_neuron_count_for_digits = \
            list_upstream_neuron_count_for_digits(digit=self.injector_utf_counter_actual)
        print('## ## ###:', upstream_neuron_count_for_digits)
        if upstream_neuron_count_for_digits[0][1] == 0:
            print(settings.Bcolors.RED +
                  "\n\n\n\n\n\n!!!!! !! !Terminating the brain due to low training capability! !! !!!" +
                  settings.Bcolors.ENDC)
            runtime_data.termination_flag = True
            exit_burst_process()
            self.injector_exit_flag = True
