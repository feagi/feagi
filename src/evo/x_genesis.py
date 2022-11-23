
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
This module is responsible for updating the commectome structure during brain life and post neuroembryogenesis.
"""

import logging
import json
import datetime
import concurrent.futures
from evo import neuron, synapse, stats, genetics, voxels
from functools import partial
from multiprocessing import Pool, Process
from inf import disk_ops
from inf import settings
from inf import runtime_data


def x_neurogenesis():
    pass


def x_synaptogenesis():
    pass


def x_corticogenesis():
    pass


def x_cortical_resize():
    pass


def x_cortical_reposition(cortical_area, new_coordinates):
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][0] = new_coordinates[0]
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][1] = new_coordinates[1]
    runtime_data.genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][2] = new_coordinates[2]


def x_cortical_rewire():
    pass


def x_update_neuronal_properties():
    pass


def x_cortical_manipulator(data):
    cortical_id = data["cortical_id"]


    pass


def x_synaptic_pruning():
    pass


def x_apoptosis():
    pass


def x_neurodegeneration():
    pass


