
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
Drives the evolutionary process.
"""

import os
import random
import json
import traceback
import logging
from inf import runtime_data
from inf.initialize import id_gen
from api import message_processor


logger = logging.getLogger(__name__)


def init_autopilot_folders():

    if not os.path.isdir('./evo/autopilot/generations'):
        os.mkdir('./evo/autopilot/generations')

    if not os.path.isdir('./evo/autopilot/embodiments'):
        os.mkdir('./evo/autopilot/embodiments')

    if not os.path.isdir('./evo/autopilot/environments'):
        os.mkdir('./evo/autopilot/environments')


def process_manager():
    pass


def load_new_genome():
    genome_file_name = pick_a_random_genome()

    try:
        with open("./evo/autopilot/brains/" + genome_file_name, "r") as data_file:
            genome_str = json.load(data_file)
            # todo: refactor the genome handling to not have to use the message processor for evo purpose
            api_message = {"genome": genome_str}
            message_processor.api_message_processor(api_message)

    except Exception:
        print("Error while loading genome file\n", traceback.print_exc())


def pick_a_random_genome():
    genome_file_name = random.choice(os.listdir("./evo/autopilot/brains"))
    # todo: validate the choice
    return genome_file_name


def save_genome():
    # save genome to brain folder
    # add an entry to generation dict
    pass


def log_generation():
    pass


def set_default(obj):
    if isinstance(obj, set):
        return list(obj)
    raise TypeError


def init_generation_dict():
    """
    Generation dictionary holds details about the composition of every single generation that has gone through the
    evolutionary system.

    Template example:

    {
        "generation_id_1": {
            "genome_id" : genome_id,
            "robot_id" : robot_id,
            "environment_id": env_id
        },
        "generation_id_2": {
            "genome_id" : genome_id,
            "robot_id" : robot_id,
            "environment_id": env_id
        },
        ...,
    }

    """
    init_autopilot_folders()
    runtime_data.current_generation_dict_id = id_gen(signature='_C')  # C for generation collection
    with open('./evo/autopilot/generations/' + runtime_data.current_generation_dict_id + '.json', "w") as data_file:
        data = {}
        data_file.seek(0)  # rewind
        data_file.write(json.dumps(data, indent=3, default=set_default))
        data_file.truncate()

    runtime_data.generation_dict = {}


def update_generation_dict(genome_id=runtime_data.genome_id,
                           robot_id=runtime_data.robot_id,
                           env_id=runtime_data.environment_id,
                           fitness=None):

    print(">>> brain_run_id", runtime_data.brain_run_id)
    runtime_data.generation_dict[runtime_data.brain_run_id] = {
        'genome_id': genome_id,
        'robot_id': robot_id,
        'environment_id': env_id,
        'fitness': fitness
    }
    update_generation_dict_file()


def update_generation_dict_file():
    """
    Saves the updated version of the generation_dict from memory to disk
    """
    with open('./evo/autopilot/generations/' + runtime_data.current_generation_dict_id + '.json', "w+") as data_file:
        data = runtime_data.generation_dict
        data_file.seek(0)  # rewind
        data_file.write(json.dumps(data, indent=3, default=set_default))
        data_file.truncate()
