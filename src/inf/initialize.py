

import logging
import os
from collections import deque
from inf import runtime_data, disk_ops, settings
from configparser import ConfigParser
from shutil import copyfile
from evo.stats import list_top_n_utf_memory_neurons

log = logging.getLogger(__name__)


def init_parameters():
    """To load all the key configuration parameters"""
    feagi_config = ConfigParser()
    feagi_config.read('./feagi_configuration.ini')
    runtime_data.parameters = {s: dict(feagi_config.items(s)) for s in feagi_config.sections()}
    log.info("All parameters have been initialized.")


def init_connectome():
    connectome_path = runtime_data.parameters["InitData"]["connectome_path"]
    if not os.path.exists(connectome_path):
        os.makedirs(connectome_path)
        copyfile(runtime_data.parameters["InitData"]["static_genome_path"], connectome_path)


def init_genome():
    # The following stages the genome in the proper connectome path and loads it into the memory
    disk_ops.genome_handler(runtime_data.parameters["InitData"]["connectome_path"])


def init_cortical_list():
    cortical_list = []
    for key in runtime_data.genome['blueprint']:
        cortical_list.append(key)
    runtime_data.cortical_list = cortical_list


def init_data_sources():
    """To validate and initialize all data sources and databases"""
    log.info("All data sources have been initialized.")
    return


def init_ipu():
    """To validate and initialize all the Input Processing Units"""
    log.info("All IPUs have been initialized.")
    return


def init_opu():
    """To validate and initialize all the Output Processing Units"""
    log.info("All OPUs have been initialized.")
    return


def initialize():
    init_parameters()
    init_connectome()
    init_genome()
    init_cortical_list()
    init_data_sources()
    init_ipu()
    init_opu()


def init_burst_engine():
    print("\n\n")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** **** ****")
    print("**** **** **** **** ****       Starting the burst_manager engine...      **** **** **** **** ****")
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
    runtime_data.top_10_utf_neurons = list_top_n_utf_memory_neurons("utf8", 10)
    runtime_data.v1_members = []

    for item in runtime_data.cortical_list:
        if runtime_data.genome['blueprint'][item]['sub_group_id'] == "vision_v1":
            runtime_data.v1_members.append(item)


def burst_exit_process():
    print(settings.Bcolors.YELLOW + '>>>Burst Exit criteria has been met!   <<<' + settings.Bcolors.ENDC)
    runtime_data.live_mode_status = 'idle'
    runtime_data.burst_count = 0
    runtime_data.parameters["Switches"]["ready_to_exit_burst"] = True
    runtime_data.parameters["Auto_injector"]["injector_status"] = False
    if runtime_data.parameters["Switches"]["capture_brain_activities"]:
        disk_ops.save_fcl_to_disk()

