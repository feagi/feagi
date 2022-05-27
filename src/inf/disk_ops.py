
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


import os.path
import json
import pickle
from inf import runtime_data


def load_brain_in_memory(connectome_path=None, cortical_list=None):
    # todo: Need error handling added so if there is a corruption in brain data it can regenerate
    if not connectome_path:
        connectome_path = runtime_data.connectome_path
    if not cortical_list:
        cortical_list = runtime_data.cortical_list
    brain = {}
    for item in cortical_list:
        if os.path.isfile(connectome_path + item + '.json'):
            with open(connectome_path + item + '.json', "r") as data_file:
                data = json.load(data_file)
                brain[item] = data
    print("Brain has been successfully loaded into memory...")
    return brain


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
    print("connectome_path / runtime_connectome_path:", connectome_path, runtime_data.connectome_path)
    if not connectome_path:
        connectome_path = runtime_data.connectome_path
    if not brain:
        brain = runtime_data.brain
    if brain == {}:
        print(">> >> Error: Could not save the brain contents to disk as brain was empty!")
        return
    brain = serialize_brain_data(brain)

    if cortical_area != 'all':
        with open(connectome_path+cortical_area+'.json', "r+") as data_file:
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


def load_processed_mnist_from_disk(mnist_type, kernel_size):
    with open("./PUs/mnist_processed_" +
              mnist_type + "_k" + str(kernel_size) + ".pkl", 'rb') as pickled_data:
        data = pickle.load(pickled_data)
    return data


def load_mnist_data_in_memory(mnist_type, kernel_size):
    if mnist_type == 'training':
        runtime_data.mnist_training[str(kernel_size)] = \
            load_processed_mnist_from_disk(mnist_type=mnist_type, kernel_size=kernel_size)
        print("MNIST %s data has been loaded into memory." % mnist_type)
    if mnist_type == 'test':
        runtime_data.mnist_testing[str(kernel_size)] = \
            load_processed_mnist_from_disk(mnist_type=mnist_type, kernel_size=kernel_size)
        print("MNIST %s data has been loaded into memory." % mnist_type)


def save_processed_mnist_to_disk(data_type, data):
    if data_type == 'training':
        # with open('mnist_processed_training.json', "w") as data_file:
        #     data_file.seek(0)  # rewind
        #     data_file.write(json.dumps(data, indent=3))
        #     data_file.truncate()
        with open("mnist_processed_training.pkl", 'wb') as output:
            pickle.dump(data, output)
    elif data_type == 'test':
        # with open('mnist_processed_test.json', "w") as data_file:
        #     data_file.seek(0)  # rewind
        #     data_file.write(json.dumps(data, indent=3))
        #     data_file.truncate()
        with open("mnist_processed_test.pkl", 'wb') as output:
            pickle.dump(data, output)
    else:
        print("ERROR: Invalid type provided to save_processed_mnist_to_disk function")


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
