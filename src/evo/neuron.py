
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
A collection of functions related to Neurons
"""

import random
import string
import datetime
import logging
# import collections
# import numpy as np
from evo.voxels import *


logger = logging.getLogger(__name__)

# def neuron_location_gen(x1, y1, z1, x2, y2, z2):
#     """
#     Function responsible to generate a pseudo-random location for a Neuron given some constraints
#
#     """
#     # todo: update to leverage the Genome template
#     # todo: Would it be better to use relative locations in each cortical region instead?
#     neuron_location = [random.randrange(x1, x2, 1), random.randrange(y1, y2, 1), random.randrange(z1, z2, 1)]
#     return neuron_location


def neuron_id_gen(cortical_id=None, size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :param cortical_id
    :return:
    """
    now = datetime.datetime.now()
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return str(cortical_id + '_' + now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_' + (''.join(random.choice(chars) for _ in range(size))) + '_N'


def init_neuron(cortical_area, soma_location):
    """
    Responsible for adding a Neuron to connectome

    """

    genome = runtime_data.genome

    neuron_id = neuron_id_gen(cortical_id=cortical_area)

    runtime_data.brain[cortical_area][neuron_id] = {}
    runtime_data.brain[cortical_area][neuron_id]["neighbors"] = {}
    runtime_data.brain[cortical_area][neuron_id]["upstream_neurons"] = set()
    runtime_data.brain[cortical_area][neuron_id]["event_id"] = {}
    runtime_data.brain[cortical_area][neuron_id]["membrane_potential"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_fire_count_inst"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_intake_total"] = 0
    runtime_data.brain[cortical_area][neuron_id]["cumulative_intake_count"] = 0
    runtime_data.brain[cortical_area][neuron_id]["consecutive_fire_cnt"] = 0
    runtime_data.brain[cortical_area][neuron_id]["snooze_till_burst_num"] = 0
    runtime_data.brain[cortical_area][neuron_id]["last_burst_num"] = 0
    runtime_data.brain[cortical_area][neuron_id]["activity_history"] = []
    runtime_data.brain[cortical_area][neuron_id]["soma_location"] = soma_location
    # loc_blk is a two element list where first element being the location of the neuron and second being the block
    # runtime_data.brain[cortical_area][neuron_id]["dendrite_locations"] = dendrite_locations
    runtime_data.brain[cortical_area][neuron_id]["status"] = "Passive"
    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_reset_burst"] = 0

    runtime_data.brain[cortical_area][neuron_id]["last_membrane_potential_update"] = 0
    # runtime_data.brain[cortical_area][neuron_id]["residual_membrane_potential"] = 0

    #   runtime_data.brain[cortical_area][neuron_id]["group_id"] = ""
    #  consider using the group name part of Genome instead
    # runtime_data.brain[cortical_area][neuron_id]["depolarization_threshold"] = \
    #     genome['blueprint'][cortical_area]['depolarization_threshold']
    if genome['blueprint'][cortical_area]['firing_threshold_increment']:
        runtime_data.brain[cortical_area][neuron_id]["firing_threshold"] = \
            genome['blueprint'][cortical_area]['firing_threshold'] + \
            (genome['blueprint'][cortical_area]['firing_threshold_increment'] *
             (soma_location[0] + soma_location[1] + soma_location[2]))
    else:
        runtime_data.brain[cortical_area][neuron_id]["firing_threshold"] = \
            genome['blueprint'][cortical_area]['firing_threshold']

    leak = genome['blueprint'][cortical_area]['leak_coefficient']
    leak_variability = genome['blueprint'][cortical_area]['leak_variability']
    if leak_variability:
        if abs(leak_variability) > 2:
            leak = leak + leak * random.randrange(1, int(leak_variability), 1) / 100

    runtime_data.brain[cortical_area][neuron_id]["leak_coefficient"] = leak

    return neuron_id


def create_neuron(cortical_area, voxel):
    """
    Responsible for creating Neurons and updating the Voxel Dictionary
    """
    neuron_count = runtime_data.genome["blueprint"][cortical_area]["per_voxel_neuron_cnt"]

    if neuron_count < 1 or not neuron_count:
        neuron_count = 1

    neuron_location = block_ref_2_id(voxel)

    # Create a new Neuron in target destination
    for _ in range(int(neuron_count)):
        neuron_id = init_neuron(cortical_area=cortical_area, soma_location=neuron_location)
        runtime_data.voxel_dict[cortical_area][voxel].add(neuron_id)


def neuron_apoptosis(cortical_area):
    """
    Responsible for programmed death of neuron
    """
    # todo: implement the function
    return
