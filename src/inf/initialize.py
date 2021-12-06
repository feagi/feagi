import logging
import os
import psutil
import string
import random
from queue import Queue
from tempfile import gettempdir
from threading import Thread
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from datetime import datetime
from collections import deque
from inf import runtime_data, disk_ops, settings
from configparser import ConfigParser
from shutil import copyfile
from evo.stats import list_top_n_utf_memory_neurons, block_dict_summary
from inf.messenger import Pub
from evo.neuroembryogenesis import generate_plasticity_dict

log = logging.getLogger(__name__)


# def init_hw_controller():
#     """
#     Loads the proper controller module based on the robot species defined within genome
#     """
#     import sys
#     print("controller path:", runtime_data.hw_controller_path)
#     sys.path.insert(1, runtime_data.hw_controller_path)
#     import controller


def detect_hardware():
    """
    Identifies the type of hardware the brain is running on so the right capabilities can be utilized
    """
    # todo

    try:
        with open('/sys/firmware/devicetree/base/model', "r") as file:
            if "Raspberry" in file.read():
                runtime_data.hardware = "raspberry_pi"
    except:
        print("Need to figure how other platforms can be detected")

    runtime_data.hw_controller_path = '../third_party/' + \
                                      runtime_data.genome['species']['brand'] + '/' +\
                                      runtime_data.genome['species']['model'] + '/controller.py'
    print("Hardware controller path: ", runtime_data.hw_controller_path)

def init_container_variables():
    """
    Identifies variables set by containers and sets them in FEAGI runtime parameters
    """

    if os.environ.get('CONTAINERIZED', False):
        runtime_data.running_in_container = True
    if os.environ.get('influxdb', False):
        runtime_data.influxdb = True
    if os.environ.get('mongodb', False):
        runtime_data.mongodb = True
    if os.environ.get('gazebo', False):
        runtime_data.gazebo = True


def running_in_container():
    """
    Identifies if FEAGI is running in a container or not based on the ENV variable set during the container creation

    Warning: This method of detection is not reliable as it will fail if during container formation ENV is not set

    """
    container_check = os.environ.get('CONTAINERIZED', False)

    if container_check:
        print("FEAGI is running in a Docker container")
    else:
        print("FEAGI is not running outside container")
    return container_check


def assess_max_thread_count():
    """
    FEAGI requires approxiately 1GB of memory per process. This function determines the proper number of max threads
    used by FEAGI by taking into consideration the number of CPU core count as well as available memory on the system.
    """

    cpu_core_count = psutil.cpu_count()
    print("Device CPU Core Count = ", cpu_core_count)

    free_mem = psutil.virtual_memory().available
    print("Device Free Memory = ", free_mem)

    max_thread_count = min(int(free_mem / 1024 ** 3), cpu_core_count)

    if max_thread_count == 0:
        max_thread_count = 1

    return max_thread_count


def run_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:

    Rand gen source partially from:
    http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    """
    runtime_data.brain_run_id = \
        (str(datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                               for _ in range(size)))+'_R'


def init_parameters(ini_path='./feagi_configuration.ini'):
    """To load all the key configuration parameters"""
    feagi_config = ConfigParser()
    feagi_config.read(ini_path)
    runtime_data.parameters = {s: dict(feagi_config.items(s)) for s in feagi_config.sections()}
    if not runtime_data.parameters["InitData"]["working_directory"]:
        runtime_data.parameters["InitData"]["working_directory"] = gettempdir()
    log.info("All parameters have been initialized.")


def init_working_directory():
    """
    Creates needed folder structure as the working directory for FEAGI. This includes a folder to house connectome
    as well as folders needed for IPU/OPU to ingress/egress data accordingly.

    """
    runtime_data.working_directory = runtime_data.parameters["InitData"]["working_directory"] + '/' + runtime_data.brain_run_id

    # Create connectome directory if needed
    # todo: need to consolidate the connectome path as currently captured in two places
    runtime_data.connectome_path = runtime_data.working_directory + '/connectome/'
    runtime_data.parameters["InitData"]["connectome_path"] = runtime_data.connectome_path
    if not os.path.exists(runtime_data.connectome_path):
        os.makedirs(runtime_data.connectome_path)
    # copyfile(runtime_data.parameters["InitData"]["static_genome_path"], runtime_data.connectome_path + '/')

    # Create IPU directories if needed
    # todo: figure best way to obtain the following list. possibly from genome
    directory_list = ['ipu', 'opu_vision', 'opu_utf', 'opu_auditory']
    for _ in directory_list:
        ipu_path = runtime_data.working_directory + '/' + _
        if not os.path.exists(ipu_path):
            os.makedirs(ipu_path)
        runtime_data.paths[_] = runtime_data.working_directory + _
    # print(runtime_data.paths)


def init_genome():
    print("\nInitializing genome...\n")
    # The following stages the genome in the proper connectome path and loads it into the memory
    disk_ops.genome_handler(runtime_data.connectome_path)


def init_cortical_list():
    cortical_list = []
    for key in runtime_data.genome['blueprint']:
        cortical_list.append(key)
    runtime_data.cortical_list = cortical_list


def init_genome_post_processes():
    """
    Augments genome with details that can improve the run-time performance by reducing frequent operations
    """
    # Augment cortical dimension dominance e.g. is it longer in x dimension or z
    for cortical_area in runtime_data.cortical_list:
        block_boundaries = runtime_data.genome["blueprint"][cortical_area]["neuron_params"]["block_boundaries"]
        dominance = block_boundaries.index(max(block_boundaries))
        runtime_data.genome['blueprint'][cortical_area]['dimension_dominance'] = dominance


def init_timeseries_db():
    """
    Conducts needed checks to ensure the time-series database is ready

    Utilizing InfluxDb as the time-series database
    """
    from inf import db_handler
    runtime_data.influxdb = db_handler.InfluxManagement()
    runtime_data.influxdb.test_influxdb()
    return


def init_genome_db():
    print("- Starting MongoDb initialization...")
    from inf import db_handler
    runtime_data.mongodb = db_handler.MongoManagement()
    runtime_data.mongodb.test_mongodb()
    print("+ MondoDb has been successfully initialized")
    return


def init_data_sources():
    """To validate and initialize all data sources and databases"""
    print("\nInitializing databases...")
    # Light mode is a switch to help globally turn off some of the resource intensive processes
    # todo: Light mode needs better definition as currently it is more db centric and not serving the key purpose
    if not runtime_data.parameters['Switches']['light_mode']:
        if runtime_data.parameters['Database']['mongodb_enabled']:
            init_genome_db()
        else:
            print("    MongoDb:", settings.Bcolors.RED + "Disabled" + settings.Bcolors.ENDC)

        if runtime_data.parameters['Database']['influxdb_enabled']:
            init_timeseries_db()
        else:
            print("    InfluxDb:", settings.Bcolors.RED + "Disabled" + settings.Bcolors.ENDC)

        log.info("All data sources have been initialized.")
    else:
        print("FEAGI is operating in LIGHT MODE where no database is utilized.")

    return


def init_resources():

    if runtime_data.parameters['System']['max_core']:
        print("Max thread count was overwritten to:", runtime_data.parameters['System']['max_core'])
    else:
        runtime_data.parameters['System']['max_core'] = assess_max_thread_count()
        print("Max thread count was set to ", runtime_data.parameters['System']['max_core'])


def init_fake_stimulation():
    if runtime_data.parameters['Switches']['fake_stimulation_flag']:
        import inf.fake_stimulation as fake_stimulation
        runtime_data.stimulation_data = fake_stimulation.stimulation_data


def initialize():
    runtime_data.last_alertness_trigger = datetime.now()
    run_id_gen()
    init_parameters()
    init_io_channels()
    init_working_directory()
    init_container_variables()
    init_data_sources()
    init_genome()
    detect_hardware()
    init_cortical_list()
    init_genome_post_processes()
    init_resources()
    init_fake_stimulation()
    generate_plasticity_dict()
    runtime_data.fcl_queue = Queue()


def init_burst_engine():
    print("\n\n")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** ****       Starting the burst_manager engine...        **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("\n\n")

    print(runtime_data.parameters['Switches']['use_static_genome'])

    # Initializing the comprehension queue
    disk_ops.genome_handler(runtime_data.parameters['InitData']['connectome_path'])
    # todo: Move comprehension span to genome that is currently in parameters
    comprehension_span = int(runtime_data.parameters["InitData"]["comprehension_span"])
    runtime_data.comprehension_queue = deque(['-'] * comprehension_span)
    runtime_data.parameters["Auto_injector"]["injector_status"] = False
    runtime_data.termination_flag = False
    runtime_data.top_10_utf_memory_neurons = list_top_n_utf_memory_neurons("utf8_memory", 10)
    runtime_data.top_10_utf_neurons = list_top_n_utf_memory_neurons("utf8_ipu", 10)
    runtime_data.v1_members = []
    runtime_data.burst_timer = runtime_data.parameters["Timers"]["burst_timer"]

    if runtime_data.parameters["Logs"]["print_block_dict_report"]:
        print("Block Dictionary Report:")
        block_dict_summary(runtime_data.block_dic, verbose=True)

    for item in runtime_data.cortical_list:
        if runtime_data.genome['blueprint'][item]['sub_group_id'] == "vision_v1":
            runtime_data.v1_members.append(item)


def exit_burst_process():
    print(settings.Bcolors.YELLOW + '>>>Burst Exit criteria has been met!   <<<' + settings.Bcolors.ENDC)
    runtime_data.live_mode_status = 'idle'
    runtime_data.burst_count = 0
    runtime_data.parameters["Switches"]["ready_to_exit_burst"] = True
    runtime_data.parameters["Auto_injector"]["injector_status"] = False
    if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        disk_ops.save_fcl_to_disk()


def init_io_channels():
    # Initialize ZMQ connections
    try:
        opu_socket = 'tcp://0.0.0.0:' + runtime_data.parameters['Sockets']['opu_port']
        print("OPU socket is:", opu_socket)
        runtime_data.opu_pub = Pub(opu_socket)
        print("OPU channel as been successfully established at ",
              runtime_data.parameters['Sockets']['opu_port'])

        runtime_data.router_address = 'tcp://' + runtime_data.parameters['Sockets']['sensory_router_ip'] + ':' + \
                                      runtime_data.parameters['Sockets']['sensory_router_port']
        print("Router address is set to:", runtime_data.router_address)
    except KeyError as e:
        print('ERROR: OPU socket is not properly defined as part of feagi_configuration.ini\n', e)


