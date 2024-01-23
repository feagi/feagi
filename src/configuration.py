# -*- coding: utf-8 -*-
# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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

import os
import logging

from configparser import ConfigParser
from tempfile import gettempdir
# from inf.runtime_data import parameters as feagi_runtime_params

logger = logging.getLogger(__name__)


def update_ini_variables_from_environment(var_dict):
    """
    Function to update ini variables from environment
    Any ini variable starting with $ will be referenced from environment
    """
    new_var_dict = {}
    for key, val in var_dict.items():
        new_var_dict[key] = val
        if val.startswith('$'):
            new_val = os.environ.get(val.lstrip('$'))
            print(f"Fetching {key} from environment with ENV {val} - NEW VAL - {new_val}")
            new_var_dict[key] = new_val
    return new_var_dict


def init_parameters(ini_path='./feagi_configuration.ini'):
    """To load all the key configuration parameters"""
    print("\n_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+")
    print("Initializing FEAGI parameters...")
    print("_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+\n")
    feagi_config = ConfigParser()
    feagi_config.read(ini_path)
    feagi_runtime_params = {
        s: update_ini_variables_from_environment(dict(feagi_config.items(s))) for s in feagi_config.sections()
    }
    # print("runtime_data.parameters ", feagi_runtime_params.parameters)
    if not feagi_runtime_params["InitData"]["working_directory"]:
        feagi_runtime_params["InitData"]["working_directory"] = gettempdir()
    logger.info("All parameters have been initialized.")
    return feagi_runtime_params
