# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
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

import logging.config
import json
import os
import platform
import tempfile
from inf import initialize
from evo import neuroembryogenesis, death, genome_processor
from npu import burst_engine
from inf import runtime_data, disk_ops
from trn import edu_controller
from inf.initialize import init_parameters
from evo.genome_editor import save_genome
from configparser import ConfigParser
from tempfile import gettempdir
import logging

# log = logging.getLogger(__name__)


init_parameters()


def splash_screen():
    print("""\n
          _________           _________               __                ______               _____    
         |_   ___  |         |_   ___  |             /  \             .' ___  |             |_   _|   
           | |_  \_|           | |_  \_|            / /\ \           / .'   \_|               | |     
           |  _|               |  _|  _            / ____ \          | |    ____              | |     
          _| |_               _| |___/ |         _/ /    \ \_        \ `.___]  _|            _| |_    
         |_____|             |_________|        |____|  |____|        `._____.'             |_____|   
    """)


def start_feagi(api_queue=None):
    """
    Starts a dedicated thread for FEAGI operations

    start_mode options: genome, connectome
    mode values: genome[file, string]  connectome[path, upload]
    """

    splash_screen()

    runtime_data.api_queue = api_queue
    if platform.system() != 'Windows':
        logging_config_file = './logging_config.json'

        with open(logging_config_file, 'r') as data_file:
            LOGGING_CONFIG = json.load(data_file)
            if platform.system() == 'Windows':
                win_temp_dir = os.path.join(tempfile.gettempdir(), os.urandom(24).hex())
                LOGGING_CONFIG['handlers']['file']['filename'] = win_temp_dir
            logging.config.dictConfig(LOGGING_CONFIG)

    runtime_data.exit_condition = False

    # This while loop simulates a single cycle of life for the artificial brain
    while not runtime_data.exit_condition:
        # Initialize the environment
        initialize.init_infrastructure()

        # if mode == 'genome':
        #     # Process of artificial neuroembryogenesis that leads to connectome development
        #     neuroembryogenesis.develop_brain(reincarnation_mode=runtime_data.parameters[
        #         'Brain_Development']['reincarnation_mode'])

        # Staring the burst_manager engine
        burst_engine.burst_manager()

        # Starting the edu controller responsible for learning and evaluations
        edu_controller.initialize()

        # A set of experiences will be outlined under life adventures that leads to learning
        # adventures.tbd()

        # Death process eliminates the brain instance and captures associated performance details
        death.register()
        runtime_data.exit_condition = True

    print('FEAGI instance has been terminated!')

    # def genome_ver_check():
    #     try:
    #         if runtime_data.genome['version'] == "2.0":
    #             print("\n\n\n************ Genome Version 2.0 has been detected **************\n\n\n")
    #             runtime_data.genome_ver = "2.0"
    #             save_genome(genome=runtime_data.genome, file_name="../runtime_genome.py")
    #             runtime_data.cortical_list = genome_processor.genome_2_cortical_list(runtime_data.genome['blueprint'])
    #             genome2 = genome_processor.genome_2_1_convertor(flat_genome=runtime_data.genome['blueprint'])
    #             genome_processor.genome_2_hierarchifier(flat_genome=runtime_data.genome['blueprint'])
    #             runtime_data.genome['blueprint'] = genome2['blueprint']
    #         else:
    #             print("ERROR! Genome is not compatible with 2.0 standard")
    #     except KeyError as e:
    #         print("Error:", e)
    #         print("Genome version not available; assuming Genome 1.0 procedures.")
    #         runtime_data.cortical_list = genome_processor.genome_1_cortical_list(runtime_data.genome['blueprint'])
    #         pass

    # if mode == 'genome':
    #     print("Genome string option selected")
    #     genome_json = mode_value
    #     runtime_data.genome = genome_json
    #     genome_ver_check()
    #     start(mode='genome')
    #
    # elif mode == 'connectome':
    #     print("Starting FEAGI with loading a <Connectome>")
    #     if mode_option == 'path':
    #         print("Genome file option selected")
    #         connectome_path = mode_value
    #         print("connectome_path:", connectome_path)
    #         start(mode='connectome')
    #     elif mode_option == 'upload':
    #         print("Genome string option selected")
    #         connectome_files = mode_value
    #
    #         start()
    # else:
    #     print("ERROR! start_feagi operation mode is not defined properly.. exiting FEAGI")

