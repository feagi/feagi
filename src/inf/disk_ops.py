
# Copyright 2016-2022 Neuraville Inc. Authors. All Rights Reserved.
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

import logging
import os.path
import json
import pickle
import traceback
import gzip
import sys

from src.inf import runtime_data
from src.evo.genome_processor import genome_v1_v2_converter

logger = logging.getLogger(__name__)


def load_brain_in_memory(connectome_path=None, cortical_list=None):
    # todo: Need error handling added so if there is a corruption in brain data it can regenerate
    if not connectome_path:
        connectome_path = runtime_data.connectome_path
    if not cortical_list:
        cortical_list = runtime_data.cortical_list
    brain = {}
    print("cortical_list:", cortical_list)
    for item in cortical_list:
        if os.path.isfile(connectome_path + item + '.json'):
            with open(connectome_path + item + '.json', "r") as data_file:
                data = json.load(data_file)
                brain[item] = data
                print(f"++++++++++++Cortical area {item} is loaded")
        else:
            print(runtime_data.connectome_path)
            print(os.listdir(runtime_data.connectome_path))
            print(f"------------------Cortical area {item} data not found")

    print("$-" * 40)
    runtime_data.brain = brain
    print("Brain has been successfully loaded into memory...")
    return brain


def load_genome_in_memory(connectome_path=None, cortical_list=None):
    # todo: Need error handling added so if there is a corruption in brain data it can regenerate
    if not connectome_path:
        connectome_path = runtime_data.connectome_path
    if not cortical_list:
        cortical_list = runtime_data.cortical_list
    genome = {}
    if os.path.isfile(connectome_path + 'genome.json'):
        with open(connectome_path + 'genome.json', "r") as data_file:
            data = json.load(data_file)
            runtime_data.genome = data
    print("Genome has been successfully loaded into memory...")


def serialize_brain_data(brain):
    for cortical_area in brain:
        for neuron_id in brain[cortical_area]:
            runtime_data.brain[cortical_area][neuron_id]["activity_history"] = \
                list(runtime_data.brain[cortical_area][neuron_id]["activity_history"])
    return brain


def load_voxel_dict_in_memory():
    # todo: Need error handling added so if there is a corruption in voxel_dict data it can regenerate
    connectome_path = runtime_data.parameters["InitData"]["connectome_path"]
    voxel_dict = {}
    for item in runtime_data.cortical_list:
        if os.path.isfile(connectome_path + item + '_vox_dict.json'):
            with open(connectome_path + item + '_vox_dict.json', "r") as data_file:
                data = json.load(data_file)
                voxel_dict[item] = data
    return voxel_dict


def set_default(obj):
    if isinstance(obj, set):
        return list(obj)
    raise TypeError


def save_voxel_dict_to_disk(cortical_area='all', voxel_dict=runtime_data.voxel_dict, backup=False):
    connectome_path = runtime_data.connectome_path
    if voxel_dict == {}:
        print(">> >> Error: Could not save the brain contents to disk as >> voxel_dict << was empty!")
        return

    if cortical_area != 'all':
        with open(connectome_path+cortical_area+'_vox_dict.json', "w") as data_file:
            data = voxel_dict[cortical_area]
            data_file.seek(0)  # rewind
            data_file.write(json.dumps(data, indent=3, default=set_default))
            data_file.truncate()
    elif backup:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_backup_vox_dict.json', "w") as data_file:
                data = voxel_dict[cortical_area]
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3, default=set_default))
                data_file.truncate()
    else:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_vox_dict.json', "w") as data_file:
                try:
                    data = voxel_dict[cortical_area]
                    data_file.seek(0)  # rewind
                    data_file.write(json.dumps(data, indent=3, default=set_default))
                    data_file.truncate()
                except KeyError:
                    print("Warning: %s was not present in the voxel_dict")
    return


def save_fcl_to_disk():
    with open("./fcl_repo/fcl-" + runtime_data.brain_run_id + ".json", 'w') as fcl_file:
        # Saving changes to the connectome
        fcl_file.seek(0)  # rewind
        fcl_file.write(json.dumps(runtime_data.fcl_history, indent=3))
        fcl_file.truncate()

    print("Brain activities has been preserved!")


def save_brain_to_disk(cortical_area='all', brain=runtime_data.brain,
                       connectome_path=runtime_data.connectome_path,
                       parameters=runtime_data.parameters, type=None):
    print("Saving cortical area to disk:", runtime_data.connectome_path, cortical_area)
    if not connectome_path:
        connectome_path = runtime_data.connectome_path
    if not brain:
        brain = runtime_data.brain
    if brain == {}:
        print(">> >> Error: Could not save the brain contents to disk as brain was empty!")
        return
    brain = serialize_brain_data(brain)

    if cortical_area != 'all':
        with open(connectome_path+cortical_area+'.json', "w") as data_file:
            data = brain[cortical_area]
            # print("...All data related to Cortical area %s is saved in connectome\n" % cortical_area)
            # Saving changes to the connectome
            data_file.seek(0)  # rewind
            data_file.write(json.dumps(data, indent=3))
            data_file.truncate()

    elif type == 'backup':
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_backup.json', "w") as data_file:
                data = brain[cortical_area]
                # if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
                    # print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)
                # Saving changes to the connectome
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
                print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)

    elif type == 'snapshot':
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'.json', "w") as data_file:
                data = brain[cortical_area]
                # if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
                    # print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)
                # Saving changes to the connectome
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
                print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)

    else:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'.json', "r+") as data_file:
                data = brain[cortical_area]
                # if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
                    # print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)
                # Saving changes to the connectome
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
    return


def load_rules_in_memory():
    with open(runtime_data.parameters["InitData"]["rules_path"], "r") as data_file:
        rules = json.load(data_file)
    # print("Rules has been successfully loaded into memory...")
    return rules


def save_fcl_in_db(burst_number, fire_candidate_list, number_under_training):
    # mongo = db_handler.MongoManagement()
    fcl_data = {}
    fcl_data['genome_id'] = runtime_data.genome_id
    fcl_data['burst_id'] = burst_number
    fcl_data['number_under_training'] = number_under_training
    fcl_data['fcl_data'] = fire_candidate_list
    runtime_data.mongodb.insert_neuron_activity(fcl_data=fcl_data)


def get_deep_size(obj, seen=None):
    # Create a set to store objects that have been seen
    if seen is None:
        seen = set()

    # Get object's memory size
    size = sys.getsizeof(obj)

    # If object is a dictionary, add sizes of its keys and values
    if isinstance(obj, dict):
        for key, value in obj.items():
            if id(key) not in seen:
                seen.add(id(key))
                size += get_deep_size(key, seen)
            if id(value) not in seen:
                seen.add(id(value))
                size += get_deep_size(value, seen)

    # If object is a list, add sizes of its items
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        for item in obj:
            if id(item) not in seen:
                seen.add(id(item))
                size += get_deep_size(item, seen)

    return size


# Convert to human-readable format
def format_bytes(size):
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024.0:
            return f"{size:.2f} {unit}"
        size /= 1024.0
    return f"{size:.2f} TB"


def preserve_brain():
    # Combine Brain Data
    brain = dict()
    brain["connectome"] = runtime_data.brain
    brain["voxel_dict"] = runtime_data.voxel_dict
    brain["genome"] = genome_v1_v2_converter(runtime_data.genome)
    brain["plasticity_dict"] = runtime_data.plasticity_dict

    brain_size = get_deep_size(brain)
    print("Brain size (pre compression):", format_bytes(brain_size))

    # Convert Brain to JSON
    try:
        json_brain = json.dumps(brain, default=set_default)
        # Compress Brain
        gzip_brain = gzip.compress(json_brain.encode())
        size_of_compressed_data = len(gzip_brain)
        compression_rate = round(brain_size / size_of_compressed_data * 100)
        print(f"Compression rate: {compression_rate} %")
        print("Brain size (post compression):", format_bytes(size_of_compressed_data))
        print("_-" * 20)
        return gzip_brain
    except Exception as e:
        print("Exception during brain jasonification", e, traceback.print_exc())


def revive_brain(brain_data):
    try:
        print("\n\n$$$$$$$$$$$$$$$$$      Brain Revival Begun     $$$$$$$$$$$$$$$$$")
        # Decompress brain data
        decompressed_brain = gzip.decompress(brain_data)

        # Convert bytes back to string.
        json_string = decompressed_brain.decode()

        # Load the string as a dictionary.
        revived_brain = json.loads(json_string)

        for type_ in revived_brain:
            print(f"!!!!!!!  {type_}")

        # Unpack the brain
        if "connectome" in revived_brain:
            runtime_data.pending_brain = revived_brain["connectome"]
        if "voxel_dict" in revived_brain:
            runtime_data.pending_voxel_dict = revived_brain["voxel_dict"]
        if "genome" in revived_brain:
            runtime_data.pending_genome = revived_brain["genome"]
        if "plasticity_dict" in revived_brain:
            runtime_data.pending_plasticity_dict = revived_brain["plasticity_dict"]
        print("$$$$$$$$$$$$$$$$$      Brain Revival Completed       $$$$$$$$$$$$$$$$$\n\n")

    except Exception as e:
        print("Exception during brain revival!", e, traceback.print_exc())
