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

import uvicorn
import json
import logging.config
from inf.initialize import init_parameters, runtime_data

with open("logging_config.json", "r") as config_file:
    logging_config_data = json.load(config_file)

# setup loggers
logging.config.dictConfig(logging_config_data)

# get root logger
logger = logging.getLogger(__name__)

init_parameters()

if __name__ == "__main__":
    print("Starting FEAGI API on port ", runtime_data.parameters['Sockets']['feagi_api_port'])
    if runtime_data.parameters['Sockets']['feagi_api_port'] is None:
        runtime_data.parameters['Sockets']['feagi_api_port'] = 8000  # Default port
    uvicorn.run("src.api.api:app", host="0.0.0.0", port=int(runtime_data.parameters['Sockets']['feagi_api_port']),
                reload=False, log_level="debug", debug=True)
