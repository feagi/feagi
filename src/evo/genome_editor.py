
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


"""
A tool to help add custom keys to genome

Todo: Make improvements to this tool as it will have further use-cases.
"""

import logging
import traceback

import xxhash
from src.inf import runtime_data
from datetime import datetime
from time import time
from time import sleep
import json


logger = logging.getLogger(__name__)


def set_default(obj):
    if isinstance(obj, set):
        return list(obj)
    return obj


def add_gene():
    for cortical_area in runtime_data.genome['blueprint']:
        for _ in runtime_data.genome['blueprint'][cortical_area]:
            if _ == "cortical_mapping_dst":
                for __ in runtime_data.genome['blueprint'][cortical_area]["cortical_mapping_dst"]:
                    if "excitatory" not in runtime_data.genome['blueprint'][cortical_area]["cortical_mapping_dst"][__]:
                        runtime_data.genome['blueprint'][cortical_area]["cortical_mapping_dst"][__]["excitatory"] = True


def save_genome(genome, file_name=''):
    if file_name != '':
        genome_file = file_name
    else:
        genome_file = "genome" + datetime.now().isoformat() + ".json"

    try:
        with open(genome_file, "w") as data_file:
            data = genome
            if "signatures" not in data:
                data["signatures"] = {}
            data["timestamp"] = time()

            # host_info = runtime_data.host_info.copy()
            # data["hosts"] = clean_host_info(host_info)
            # print("@----" * 20)
            # print(data["hosts"])

            if "signatures" not in data:
                data["signatures"] = {}
            data["timestamp"] = time()
            data["signatures"]["genome"] = generate_hash(genome_signature_payload(data))
            data["signatures"]["blueprint"] = generate_hash(data["blueprint"])
            data["signatures"]["physiology"] = generate_hash(data["physiology"])

            data_file.seek(0)  # rewind
            data_file.write(json.dumps(data, indent=3, default=set_default))
            data_file.truncate()
            # todo: Identify the cause of errors when sleep is eliminated
            sleep(0.5)  # Elimination of sleep causes issues with Uvicorn
            print("genome is saved")
            runtime_data.changes_saved_externally = False
    except Exception as e:
        print(f"Warning: Genome could not be saved! {e}", traceback.print_exc())


def clean_host_info(host_info):
    for host in host_info:
        if "listener" in host_info[host]:
            host_info[host].pop("listener")
    return host_info


def generate_hash(payload):
    serialized_genome = json.dumps(payload, sort_keys=True)
    signature = xxhash.xxh64(serialized_genome).hexdigest()
    return signature


def genome_signature_payload(genome):
    payload = dict()
    payload["blueprint"] = genome["blueprint"]
    payload["physiology"] = genome["physiology"]
    return payload
