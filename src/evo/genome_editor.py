
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

from static_genome import genome
from datetime import datetime
from pprint import pformat


new_genome_file_name = "genome" + datetime.now().isoformat() + ".py"


def add_gene():
    for cortical_area in genome['blueprint']:
        for _ in genome['blueprint'][cortical_area]:
            if _ == "cortical_mapping_dst":
                for __ in genome['blueprint'][cortical_area]["cortical_mapping_dst"]:
                    if "excitatory" not in genome['blueprint'][cortical_area]["cortical_mapping_dst"][__]:
                        genome['blueprint'][cortical_area]["cortical_mapping_dst"][__]["excitatory"] = True


def save_genome(file_name='./tmp.py'):
    with open('./' + new_genome_file_name, "w") as data_file:
        try:
            data = genome
            print("genome = " + pformat(data, indent=3), file=data_file)

        except KeyError:
            print("Warning: %s was not present in the block_dic")


# def validate_genome():
#
#
#

if __name__ == "__main__":
    add_gene()
    save_genome()
