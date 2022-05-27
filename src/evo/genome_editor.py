
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
A tool to help add custom keys to genome

Todo: Make improvements to this tool as it will have further use-cases.
"""

from inf import runtime_data
from datetime import datetime
from time import sleep
import json


def set_default(obj):
    if isinstance(obj, set):
        return list(obj)
    raise TypeError


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
        genome_file = "genome" + datetime.now().isoformat() + ".py"

    try:
        with open(genome_file, "w") as data_file:
            data = genome
            data_file.seek(0)  # rewind
            data_file.write(json.dumps(data, indent=3, default=set_default))
            data_file.truncate()
            # todo: Identify the cause of errors when sleep is eliminated
            sleep(0.5)  # Elimination of sleep causes issues with Uvicorn
            print("genome is saved")

    except KeyError:
        print("Warning: Genome could not be saved!")


# def validate_genome():
#
#
#

if __name__ == "__main__":
    add_gene()
    save_genome()
