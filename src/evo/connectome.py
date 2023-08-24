
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

import os
import logging
import json
import shutil
import errno

from inf import runtime_data, settings


log = logging.getLogger(__name__)


# Resets the in-memory brain for each cortical area
def reset_connectome_in_mem():
    # for item in runtime_data.cortical_list:
    #     runtime_data.brain[item] = {}
    runtime_data.brain = {}
    # runtime_data.cortical_list = {}


def reset_connectome_file(cortical_area):
    file_name = runtime_data.connectome_path + cortical_area + '.json'
    with open(file_name, "w") as connectome:
        connectome.write(json.dumps({}))
        connectome.truncate()
    if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
        print(settings.Bcolors.YELLOW + "Cortical area %s is has been cleared." % cortical_area
              + settings.Bcolors.ENDC)
    runtime_data.brain[cortical_area] = {}


# Resets all connectome files
def reset_connectome_files():
    for cortical_area in runtime_data.genome['blueprint']:
        reset_connectome_file(cortical_area=cortical_area)


def reset_connectome():
    print("\n" * 5, "@__@--" * 20)
    print("Clearing the entire Connectome!")
    connectome_path = runtime_data.connectome_path
    shutil.rmtree(connectome_path)
    os.mkdir(connectome_path)
    print("All files associated with Connectome has been cleared.")
    reset_connectome_in_mem()
    print("All memory units associated with Connectome has been cleared.", "\n" * 5)


def reuse():
    """
    Placeholder for a function to reuse an existing connectome.

    Returns:

    """
    log.info("Reusing an old connectome")
    connectome_path = ''
    return


def connectome_backup(src, dst):
    """
    Backs up an existing connectome to preserve all the structures and associated memory data for future use.
    Args:
        src (String): Location of the source connectome folder
        dst (String): Destination folder for storing connectome backup

    Returns:

    """
    try:
        shutil.copytree(src, dst)
    except OSError as exc:
        if exc.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else:
            raise

