#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import json
import os
import platform

import psutil

from src.inf import runtime_data
from src.inf import settings
from src.inf.helpers import id_gen

from queue import Queue
from src.evo.autopilot import update_generation_dict

from threading import Thread
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from datetime import datetime, timedelta
from collections import deque
from shutil import copyfile
from src.evo.connectome import reset_connectome
from src.evo.voxels import generate_cortical_dimensions_by_id, generate_cortical_dimensions
from src.evo.cortical_area import cortical_area_type
from src.evo.stats import voxel_dict_summary
from src.evo.genome_editor import save_genome
from src.inf.messenger import Pub
from src.evo.neuroembryogenesis import generate_plasticity_dict, develop_brain
from src.evo.genome_processor import genome_1_cortical_list, genome_ver_check
from src.evo.genome_validator import *
from src.evo.region import region_id_gen
from src.evo.templates import cortical_types


logger = logging.getLogger(__name__)


def utc_time():
    current_time = datetime.utcnow()
    return current_time


def deploy_genome(neuroembryogenesis_flag=False, reset_runtime_data_flag=False, genome_data=None):
    print("=======================    Genome Staging Initiated        =======================")
    if neuroembryogenesis_flag:
        print("cortical_list:", runtime_data.cortical_list)
        reset_connectome()
    if reset_runtime_data_flag:
        reset_runtime_data()
    runtime_data.genome_counter += 1
    runtime_data.genome_reset_flag = False
    runtime_data.genome_ver = None
    runtime_data.last_genome_modification_time = datetime.now()

    if not genome_data:
        try:
            with open(runtime_data.connectome_path + "genome.json", "r") as genome_file:
                genome_data = json.load(genome_file)
                runtime_data.genome_file_name = "genome.json"
                print("Genome loaded from connectome folder")
        except Exception as e:
            print("Exception while loading Genome from connectome folder", traceback.print_exc(), e)
            print("Could not stage genome. No genome data available")

    runtime_data.genome_orig = genome_data.copy()
    runtime_data.genome = genome_data
    runtime_data.genome = genome_ver_check(runtime_data.genome)
    runtime_data.genome_ver = "2.0"

    # todo temp check to find a better solution
    if "physiology" not in runtime_data.genome:
        runtime_data.genome["physiology"] = {}

    if "lifespan_mgmt_interval" not in runtime_data.genome["physiology"]:
        runtime_data.genome["physiology"]["lifespan_mgmt_interval"] = 10

    for _ in runtime_data.genome["blueprint"]:
        if "mp_charge_accumulation" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["mp_charge_accumulation"] = True

        if "mp_driven_psp" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["mp_driven_psp"] = False

        if "firing_threshold_increment" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["firing_threshold_increment"] = 0

        if "firing_threshold_limit" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["firing_threshold_limit"] = 0

        if "visualization" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["visualization"] = True

        if "leak_variability" not in runtime_data.genome["blueprint"][_]:
            runtime_data.genome["blueprint"][_]["leak_variability"] = 0

    if "plasticity_queue_depth" not in runtime_data.genome["physiology"]:
        runtime_data.genome["physiology"]["plasticity_queue_depth"] = 3

    runtime_data.plasticity_queue_depth = runtime_data.genome["physiology"]["plasticity_queue_depth"]
    init_fcl()
    init_brain()
    init_brain_regions()

    if 'genome_id' not in runtime_data.genome:
        runtime_data.genome['genome_id'] = id_gen(signature="_G")
    runtime_data.genome_id = runtime_data.genome['genome_id']
    print("brain_run_id", runtime_data.brain_run_id)
    if runtime_data.autopilot:
        update_generation_dict(genome_id=runtime_data.genome_id,
                               robot_id=runtime_data.robot_id,
                               env_id=runtime_data.environment_id)
    # Process of artificial neuroembryogenesis that leads to connectome development
    if neuroembryogenesis_flag:
        develop_brain(reincarnation_mode=runtime_data.parameters[
            'Brain_Development']['reincarnation_mode'])
    print("=======================    Genome Staging Completed        =======================")
    runtime_data.brain_readiness = True


# def init_hw_controller():
#     """
#     Loads the proper controller module based on the robot species defined within genome
#     """
#     import sys
#     print("controller path:", runtime_data.hw_controller_path)
#     sys.path.insert(1, runtime_data.hw_controller_path)
#     import controller


# def detect_hardware():
#     """
#     Identifies the type of hardware the brain is running on so the right capabilities can be utilized
#     """
#     # todo
#
#     try:
#         with open('/sys/firmware/devicetree/base/model', "r") as file:
#             if "Raspberry" in file.read():
#                 runtime_data.hardware = "raspberry_pi"
#     except:
#         print("Need to figure how other platforms can be detected")
#
#     runtime_data.hw_controller_path = '../third_party/' + \
#                                       runtime_data.genome['species']['brand'] + '/' +\
#                                       runtime_data.genome['species']['model'] + '/controller.py'
#     print("Hardware controller path: ", runtime_data.hw_controller_path)
#


def init_brain_regions():
    if "brain_regions" not in runtime_data.genome:
        runtime_data.genome["brain_regions"] = {}

    if not runtime_data.genome["brain_regions"]:
        runtime_data.genome["brain_regions"] = {}

    if "root" not in runtime_data.genome["brain_regions"]:
        runtime_data.genome["brain_regions"]["root"] = {}
        runtime_data.genome["brain_regions"]["root"]["title"] = "Genome's root brain region"
        runtime_data.genome["brain_regions"]["root"]["description"] = None
        runtime_data.genome["brain_regions"]["root"]["parent_region_id"] = None
        runtime_data.genome["brain_regions"]["root"]["coordinate_2d"] = [0, 0]
        runtime_data.genome["brain_regions"]["root"]["coordinate_3d"] = [0, 0, 0]
        runtime_data.genome["brain_regions"]["root"]["areas"] = []
        runtime_data.genome["brain_regions"]["root"]["regions"] = []
        runtime_data.genome["brain_regions"]["root"]["inputs"] = []
        runtime_data.genome["brain_regions"]["root"]["outputs"] = []

        for cortical_area in runtime_data.cortical_list:
            runtime_data.genome["brain_regions"]["root"]["areas"].append(cortical_area)
            runtime_data.cortical_area_region_association[cortical_area] = "root"

    for cortical_area in runtime_data.cortical_list:
        if cortical_area not in runtime_data.cortical_area_region_association:
            for region in runtime_data.genome["brain_regions"]:
                if cortical_area in runtime_data.genome["brain_regions"][region]["areas"]:
                    runtime_data.cortical_area_region_association[cortical_area] = region
            if cortical_area not in runtime_data.cortical_area_region_association:
                runtime_data.cortical_area_region_association[cortical_area] = "root"

        # if runtime_data.genome["blueprint"][cortical_area]["group_id"] == "IPU":
        #     runtime_data.genome["brain_regions"]["root"]["inputs"][cortical_area] = []
        # if runtime_data.genome["blueprint"][cortical_area]["group_id"] == "OPU":
        #     runtime_data.genome["brain_regions"]["root"]["outputs"][cortical_area] = []


def init_container_variables():
    """
    Identifies variables set by containers and sets them in FEAGI runtime parameters
    """

    if os.environ.get('CONTAINERIZED', False):
        runtime_data.running_in_container = True
    if os.environ.get('INFLUXDB', None):
        init_timeseries_db()
        # runtime_data.influxdb = True
    if os.environ.get('mongodb', False):
        runtime_data.mongodb = True
    if os.environ.get('gazebo', False):
        runtime_data.gazebo = True


def init_memory_register():
    runtime_data.memory_register = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        if "sub_group_id" in runtime_data.genome["blueprint"][cortical_area]:
            if runtime_data.genome["blueprint"][cortical_area]["sub_group_id"] == "MEMORY":
                runtime_data.memory_register[cortical_area] = set()

    for cortical_area in runtime_data.genome["blueprint"]:
        for dst_cortical_area in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            if dst_cortical_area in runtime_data.genome["blueprint"]:
                if "sub_group_id" in runtime_data.genome["blueprint"][dst_cortical_area]:
                    if runtime_data.genome["blueprint"][dst_cortical_area]["sub_group_id"] == "MEMORY":
                        if dst_cortical_area not in runtime_data.memory_register:
                            runtime_data.memory_register[dst_cortical_area] = set()
                        runtime_data.memory_register[dst_cortical_area].add(cortical_area)
            else:
                print(f"Warning!! {dst_cortical_area} not found in blueprint while running init_memory_register func.")


def running_in_container():
    """
    Identifies if FEAGI is running in a container or not based on the ENV variable set during the container creation

    Warning: This method of detection is not reliable as it will fail if during container formation ENV is not set

    """
    container_check = os.environ.get('CONTAINERIZED', False)

    if container_check:
        print("FEAGI is running in a Docker container")
    else:
        print("FEAGI is running outside container")
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


def init_working_directory():
    """
    Creates needed folder structure as the working directory for FEAGI. This includes a folder to house connectome
    as well as folders needed for IPU/OPU to ingress/egress data accordingly.

    """
    if platform.system() == 'Windows':
        runtime_data.working_directory = runtime_data.parameters["InitData"]["working_directory"] + '\\' + \
                                         runtime_data.brain_run_id

        # Create connectome directory if needed
        # todo: need to consolidate the connectome path as currently captured in two places
        runtime_data.connectome_path = runtime_data.working_directory + '\\connectome\\'
        runtime_data.parameters["InitData"]["connectome_path"] = runtime_data.connectome_path

        if not os.path.exists(runtime_data.connectome_path):
            os.makedirs(runtime_data.connectome_path)

        # Create IPU directories if needed
        # todo: figure best way to obtain the following list. possibly from genome
        directory_list = ['ipu', 'opu_vision', 'opu_utf', 'opu_auditory']
        for _ in directory_list:
            ipu_path = runtime_data.working_directory + '\\' + _
            if not os.path.exists(ipu_path):
                os.makedirs(ipu_path)
            runtime_data.paths[_] = runtime_data.working_directory + _
    else:
        runtime_data.working_directory = runtime_data.parameters["InitData"]["working_directory"] + '/' + \
                                         runtime_data.brain_run_id
        runtime_data.connectome_path = runtime_data.working_directory + 'connectome/'
        runtime_data.parameters["InitData"]["connectome_path"] = runtime_data.connectome_path

        if not os.path.exists(runtime_data.connectome_path):
            os.makedirs(runtime_data.connectome_path)
        # copyfile(runtime_data.parameters["InitData"]["static_genome_path"], runtime_data.connectome_path + '/')

        directory_list = ['ipu', 'opu_vision', 'opu_utf', 'opu_auditory']
        for _ in directory_list:
            ipu_path = runtime_data.working_directory + '/' + _
            if not os.path.exists(ipu_path):
                os.makedirs(ipu_path)
            runtime_data.paths[_] = runtime_data.working_directory + _
        # print(runtime_data.paths)


# # Todo: Check of the following init is needed anymore
# def init_genome(genome):
#     logger.info("Initializing genome...")
#     # The following stages the genome in the proper connectome path and loads it into the memory
#     disk_ops.genome_handler(runtime_data.connectome_path)
#
#     try:
#         if runtime_data.genome['version'] == "2.0":
#             print("\n\n\n"
#                   ""
#                   "******* ***** Genome Version 2.0 has been detected **************\n\n\n")
#             runtime_data.genome_ver = "2.0"
#             runtime_data.cortical_list = genome_2_cortical_list(runtime_data.genome['blueprint'])
#             genome2 = genome_2_1_convertor(flat_genome=runtime_data.genome['blueprint'])
#             genome_2_hierarchifier(flat_genome=runtime_data.genome['blueprint'])
#             runtime_data.genome['blueprint'] = genome2['blueprint']
#             blueprint_validator(runtime_data.genome)
#         else:
#             print("ERROR! Genome is not compatible with 2.0 standard")
#     except KeyError as e:
#         print("Error:", e)
#         print("Genome version not available; assuming Genome 1.0 procedures.")
#         runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome['blueprint'])
#         pass


def init_genome_post_processes():
    """
    Augments genome with details that can improve the run-time performance by reducing frequent operations
    """
    # Augment cortical dimension dominance e.g. is it longer in x dimension or z
    for cortical_area in runtime_data.cortical_list:
        block_boundaries = runtime_data.genome["blueprint"][cortical_area]["block_boundaries"]
        dominance = block_boundaries.index(max(block_boundaries))
        runtime_data.genome['blueprint'][cortical_area]['dimension_dominance'] = dominance


def init_timeseries_db():
    """
    Conducts needed checks to ensure the time-series database is ready

    Utilizing InfluxDb as the time-series database
    """
    from ..inf import db_handler
    runtime_data.influxdb = db_handler.InfluxManagement()
    runtime_data.influxdb.test_influxdb()

    # # Setup message queues
    # runtime_data.influx_mp_queue = deque(maxlen=10)
    # runtime_data.influx_psp_queue = deque(maxlen=10)
    #
    # # Instantiate a new process thread to read messages from the queue and save in db

    return  


def init_cortical_info():
    genome = runtime_data.genome
    runtime_data.ipu_list = set()
    runtime_data.opu_list = set()
    runtime_data.mem_list = set()
    runtime_data.core_list = set()

    for cortical_area in genome['blueprint']:
        try:
            if genome['blueprint'][cortical_area]['group_id'] == 'IPU':
                runtime_data.ipu_list.add(cortical_area)
            if genome['blueprint'][cortical_area]['group_id'] == 'OPU':
                runtime_data.opu_list.add(cortical_area)
            if genome['blueprint'][cortical_area]['group_id'] == 'MEMORY':
                runtime_data.mem_list.add(cortical_area)
            if genome['blueprint'][cortical_area]['group_id'] == 'CORE':
                runtime_data.core_list.add(cortical_area)

        except KeyError:
            print("Error: Cortical area %s missing cortical definition" % cortical_area)

    print("IPU list:", runtime_data.ipu_list)
    print("OPU list:", runtime_data.opu_list)
    print("Mem list:", runtime_data.mem_list)
    print("Core list:", runtime_data.core_list)


def init_genome_db():
    print("- Starting MongoDb initialization...")
    from ..inf import db_handler
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

        logger.info("All data sources have been initialized.")
    else:
        print("FEAGI is operating in LIGHT MODE where no database is utilized.")

    return


def init_resources():
    if runtime_data.parameters['System']['max_core']:
        print("Max thread count was overwritten to:", runtime_data.parameters['System']['max_core'])
    else:
        runtime_data.parameters['System']['max_core'] = assess_max_thread_count()
        print("Max thread count was set to ", runtime_data.parameters['System']['max_core'])


def init_infrastructure():
    if not runtime_data.parameters:
        from src.configuration import init_parameters
        runtime_data.parameters = init_parameters()
    # init_io_channels()
    init_cortical_defaults()
    init_working_directory()
    init_container_variables()
    init_data_sources()
    # detect_hardware()
    init_resources()
    runtime_data.fcl_queue = Queue()


def reset_runtime_data():
    print("\n\n\n\n----------------- Resetting the brain -----------------------------\n\n\n")
    runtime_data.genome = {}
    runtime_data.stats = {}
    runtime_data.brain = {}
    runtime_data.cortical_list = []
    runtime_data.plasticity_dict = {}
    runtime_data.memory_register = {}
    runtime_data.cortical_dimensions = {}
    runtime_data.cortical_dimensions_by_id = {}
    runtime_data.stimulation_script = {}
    runtime_data.plasticity_queue_candidates = set()
    runtime_data.shock_admin = False
    runtime_data.shock_scenarios = tuple
    runtime_data.intercortical_mapping = []

    # Clear brain activities
    runtime_data.fire_candidate_list = {}
    runtime_data.previous_fcl = {}
    runtime_data.future_fcl = {}

    # Reset brain age
    runtime_data.current_age = 0


def init_fcl(cortical_area_=None):
    print("\n\n=========================  Initializing the FCL ===================================\n\n")
    runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
    if not cortical_area_:
        runtime_data.fire_candidate_list = {}
        runtime_data.future_fcl = {}
        runtime_data.previous_fcl = {}
        for area in runtime_data.cortical_list:
            runtime_data.fire_candidate_list[area] = set()
            runtime_data.future_fcl[area] = set()
            runtime_data.previous_fcl[area] = set()
    else:
        runtime_data.fire_candidate_list[cortical_area_] = set()
        runtime_data.future_fcl[cortical_area_] = set()
        runtime_data.previous_fcl[cortical_area_] = set()
        # runtime_data.upstream_neurons[cortical_area_] = {}
    print("\n\n=========================  FCL Initializing Completed ===================================\n\n")


def init_brain():
    print("\n\n=========================   Brain Initialization Started ===================================\n\n")
    runtime_data.last_alertness_trigger = datetime.now()
    runtime_data.brain_run_id = id_gen(signature='_R')
    init_cortical_info()
    init_io_areas()
    init_memory_register()
    runtime_data.cortical_list = genome_1_cortical_list(runtime_data.genome)
    runtime_data.cortical_dimensions = generate_cortical_dimensions()
    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
    # genome2 = genome_2_1_convertor(flat_genome=runtime_data.genome['blueprint'])
    # genome_2_hierarchifier(flat_genome=runtime_data.genome['blueprint'])
    # runtime_data.genome['blueprint'] = genome2['blueprint']
    init_genome_post_processes()
    generate_plasticity_dict()
    runtime_data.new_genome = True
    if 'burst_delay' in runtime_data.genome["physiology"]:
        runtime_data.burst_timer = float(runtime_data.genome['physiology']['burst_delay'])

    print("\n\n=========================   Brain Initialization Complete ===================================\n\n")
    runtime_data.cumulative_stats = {}
    for area in runtime_data.cortical_list:
        init_cortical_cumulative_stats(area)


def init_cortical_cumulative_stats(cortical_area):
    runtime_data.cumulative_stats[cortical_area] = {}
    runtime_data.cumulative_stats[cortical_area]["LTP"] = 0
    runtime_data.cumulative_stats[cortical_area]["LTD"] = 0
    runtime_data.cumulative_stats[cortical_area]["Bursts"] = 0


def init_cortical_defaults():
    runtime_data.cortical_defaults = cortical_types.copy()


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
    runtime_data.burst_duration = timedelta(seconds=0.0001)
    runtime_data.death_flag = False

    try:
        runtime_data.burst_timer = float(runtime_data.genome['physiology']['burst_delay'])
        print("Burst time has been set to:", runtime_data.burst_timer)
    except KeyError:
        print("\n\n==============================================")
        print("Warning! Genome is missing the \"burst_delay\" gene")
        runtime_data.burst_timer = float(runtime_data.parameters["Timers"]["default_burst_delay"])
        print("Burst_delay has been set to its default value %f " %
              float(runtime_data.parameters["Timers"]["default_burst_delay"]))
        print("==============================================\n\n")


def init_io_areas():
    for area in runtime_data.genome["blueprint"]:
        if cortical_area_type(area) in ["IPU", "OPU"]:
            if "dev_count" not in runtime_data.genome["blueprint"][area]:
                runtime_data.genome["blueprint"][area]["dev_count"] = 1


# def init_io_channels():
#     # Initialize ZMQ connections
#     try:
        # opu_socket = 'tcp://0.0.0.0:' + runtime_data.parameters['Sockets']['feagi_opu_port']
        # print("OPU socket is:", opu_socket)
        # runtime_data.opu_pub = Pub(opu_socket)
        # print("OPU channel as been successfully established at ",
        #       runtime_data.parameters['Sockets']['feagi_opu_port'])

        # if runtime_data.parameters['Switches']['zmq_activity_publisher']:
        #     runtime_data.brain_activity_pub = True
        #     brain_activities_socket = 'tcp://0.0.0.0:' + runtime_data.parameters['Sockets']['brain_activities_pub']
        #     print("Brain activity publisher socket is:", brain_activities_socket)
        #     runtime_data.brain_activity_pub = PubBrainActivities(brain_activities_socket)

        # if runtime_data.parameters['Sockets']['feagi_inbound_port_godot']:
        #     runtime_data.router_address_godot = "tcp://" + runtime_data.parameters['Sockets']['godot_host_name'] + ':' + runtime_data.parameters['Sockets'][
        #         'feagi_inbound_port_godot']
        #
        # if runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo']:
        #     runtime_data.router_address_embodiment = "tcp://" + runtime_data.parameters['Sockets']['gazebo_host_name'] + ':' + runtime_data.parameters['Sockets'][
        #         'feagi_inbound_port_gazebo']

        # if runtime_data.parameters['Sockets']['feagi_inbound_port_embodiment']:
        #     runtime_data.router_address_embodiment = "tcp://" + runtime_data.parameters['Sockets']['embodiment_host_name'] \
        #                                          + ':' + runtime_data.parameters['Sockets']['feagi_inbound_port_embodiment']

    #     print("Router addresses has been set")
    # except KeyError as e:
    #     print('ERROR: OPU socket is not properly defined as part of feagi_configuration.ini\n', e)

