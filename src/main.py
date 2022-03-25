# -*- coding: utf-8 -*-


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

"""
FEAGI is a brain inspired evolutionary framework capable of growing an artificial brain from a
genome and helping it evolve over generations.

This main module is responsible for driving the lifecycle of a single generation of an
artificial brain at a time. To scale up the system to many parallel generations, FEAGI
is intended to run within a container and scale up to many container instances.
"""

from fastapi import FastAPI
from pydantic import BaseModel
from typing import Optional


def splash_screen():
    print("""\n
          _________           _________               __                ______               _____    
         |_   ___  |         |_   ___  |             /  \             .' ___  |             |_   _|   
           | |_  \_|           | |_  \_|            / /\ \           / .'   \_|               | |     
           |  _|               |  _|  _            / ____ \          | |    ____              | |     
          _| |_               _| |___/ |         _/ /    \ \_        \ `.___]  _|            _| |_    
         |_____|             |_________|        |____|  |____|        `._____.'             |_____|   
    """)


splash_screen()


app = FastAPI()


class FEAGI(BaseModel):
    begin: Optional[bool]


@app.api_route("/v1/feagi", methods=['GET', 'POST', 'PUT', 'DELETE'])
async def feagi_management(feagi: FEAGI):
    function_status = False
    if feagi.begin:
        start()
        print("This is where we call the function to start robot container and have it associated with environment_id",
              feagi.begin)
        # placeholder for collecting the robot instance creation status
        function_status = True
    if function_status:
        return {"FEAGI was successfully started."}
    else:
        return {"FEAGI start failed ... error details to be provided here"}


def start():
    import logging.config
    import json
    import os
    import platform
    import tempfile
    from inf import initialize
    from evo import neuroembryogenesis, death
    from npu import burst_engine
    from inf import runtime_data
    from edu import edu_controller

    logging_config_file = './logging_config.json'

    with open(logging_config_file, 'r') as data_file:
        LOGGING_CONFIG = json.load(data_file)
        if platform.system() == 'Windows':
            win_temp_dir = os.path.join(tempfile.gettempdir(), os.urandom(24).hex())
            LOGGING_CONFIG['handlers']['file']['filename'] = win_temp_dir
        logging.config.dictConfig(LOGGING_CONFIG)


    exit_condition = False

    # This while loop simulates a single cycle of life for the artificial brain
    while not runtime_data.exit_condition:

        # Initialize the environment
        initialize.initialize()

        # Process of artificial neuroembryogenesis that leads to connectome development
        neuroembryogenesis.develop_brain(reincarnation_mode=runtime_data.parameters[
            'Brain_Development']['reincarnation_mode'])

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
