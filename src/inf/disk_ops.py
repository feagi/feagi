# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
# import random
# import string
from datetime import datetime
import os.path
import json
import pickle
from inf import db_handler
from evo import stats
from inf import runtime_data, settings
from evo.genome import genome_id_gen
from configparser import ConfigParser


# # Reads the list of all Cortical areas defined in Genome
# def cortical_list():
#     blueprint = runtime_data.genome["blueprint"]
#     cortical_list_ = []
#     for key in blueprint:
#         cortical_list_.append(key)
#     return cortical_list_


# def load_parameters_in_memory():
#     runtime_data.parameters = ConfigParser.get()
#     with open("./feagi/configuration/parameters.json", "r") as data_file:
#         runtime_data.parameters = json.load(data_file)
#         # print("Parameters has been read from file")


def load_genome_in_memory(connectome_path, static=False):
    if not static:
        print("Genome from local connectome folder was chosen: ", connectome_path)
        with open(connectome_path+'genome_tmp.json', "r") as genome_file:
            genome_data = json.load(genome_file)
            runtime_data.genome = genome_data
    else:
        print("Static genome from the following file was loaded in memory: ",
              runtime_data.parameters["InitData"]["static_genome_path"])
        with open(runtime_data.parameters["InitData"]["static_genome_path"], "r") as genome_file:
            genome_data = json.load(genome_file)
            runtime_data.genome = genome_data
    # todo: The following is suitable for the main_auto but needs to be adjusted for the main_manual
    runtime_data.genome_id = genome_id_gen()


def save_genome_to_disk():
    from evo.genome import calculate_brain_cognitive_fitness
    mongo = db_handler.MongoManagement()
    influxdb = db_handler.InfluxManagement()
    genome = runtime_data.genome
    genome_id = runtime_data.genome_id

    updated_genome_test_stats = []
    for item in runtime_data.genome_test_stats:
        item["genome_id"] = genome_id
        updated_genome_test_stats.append(item)

    # print(updated_genome_test_stats)
    print("*** @@@ *** @@@ *** Genome_id:  \n ", genome_id)

    genome_db = {}
    genome_db["genome_id"] = genome_id
    genome_db["generation_date"] = str(datetime.now())
    genome_db["properties"] = genome
    genome_db["parameters"] = runtime_data.parameters
    genome_db["activity_stats"] = runtime_data.activity_stats

    if not runtime_data.termination_flag:
        brain_fitness = calculate_brain_cognitive_fitness(runtime_data.genome_test_stats)
        genome_db["fitness"] = brain_fitness
        print("\n\n\n")
        print(" **********")
        print(" ********** ********")
        print(" ********** ******** ********")
        print(" ********** ******** ******** ***** *** ** * Brain fitness: ", brain_fitness)
        print(" ********** ******** ********")
        print(" ********** ********")
        print(" **********")
        print("\n\n\n")
        influxdb.insert_evolutionary_fitness_stats(connectome_path=runtime_data.parameters["InitData"]
        ["connectome_path"],
                                                   fitness_score=brain_fitness/1,
                                                   training_sets=runtime_data.parameters["Auto_injector"]
                                                   ["variation_default"],
                                                   test_sets=runtime_data.parameters["Auto_tester"]
                                                   ["variation_default"],
                                                   training_exposure=runtime_data.parameters["Auto_injector"]
                                                   ["exposure_default"],
                                                   test_exposure=runtime_data.parameters["Auto_tester"]
                                                   ["exposure_default"]
                                                   )
    else:
        brain_fitness = ''
        print("\n\n\n")
        print("       ****")
        print(" **********")
        print(" **** Brain was terminated prematurely ******")
        print(" **********")
        print("       ****")

    # Logging cortical stats in the InfluxDb
    for cortical_area in runtime_data.brain:
        neuron_count, synapse_count = stats.connectome_total_synapse_cnt(cortical_area)
        influxdb.insert_evolutionary_connectome_stats(connectome_path=runtime_data.parameters["InitData"]["connectome_path"],
                                                      cortical_area=cortical_area,
                                                      neuron_count=neuron_count,
                                                      synapse_count=synapse_count)

    # print("*** @@@ *** @@@ *** \n ", genome_db)

    mongo.insert_genome(genome_db)

    mail_body = "Genome " + str(genome_id) + " has been evaluated to have a fitness of " + str(brain_fitness)

    # Sending out email
    # if brain_fitness > runtime_data.parameters["Alerts"]["email_fitness_threshold"]:
    #     alerts.send_email(mail_body)

    print(">>>Genome test stats: >", runtime_data.genome_test_stats)
    print(">>>Genome_id: >", runtime_data.genome_id)

    for stat in runtime_data.genome_test_stats:
        stat_to_save = stat
        # todo: The following is leading to duplicate db record ---> Investigate
        # mongo.insert_test_stats(stat_to_save)


    print("Genome %s has been preserved for future generations!" % genome_id)
    stats.print_fcl_stats(genome_id)

    return


def stage_genome(connectome_path, dynamic_selection_mode=True):
    from evo.genome import select_a_genome
    if dynamic_selection_mode:
        genome_data, original_genome_id = select_a_genome()
        runtime_data.original_genome_id = original_genome_id
    else:
        # load_genome_in_memory(connectome_path, static=True)
        genome_data = runtime_data.genome
        runtime_data.original_genome_id = ["static"]

    print("Staged genome had the following genome id:", runtime_data.original_genome_id)
    genome_data['genome_id_source'] = runtime_data.original_genome_id
    with open(connectome_path+'genome_tmp.json', "w") as staged_genome:
        # Saving changes to the connectome
        staged_genome.seek(0)  # rewind
        staged_genome.write(json.dumps(genome_data, indent=3))
        staged_genome.truncate()
        print("\n*\n**\n***\ngenome_tmp.json was just staged...vvv ^^^ vvv\n***\n**\n*", connectome_path)

    print("<< << Genome has been staged in runtime repo >> >>")


def load_brain_in_memory():
    # todo: Need error handling added so if there is a corruption in brain data it can regenerate
    connectome_path = runtime_data.parameters["InitData"]["connectome_path"]
    brain = {}
    for item in runtime_data.cortical_list:
        if os.path.isfile(connectome_path + item + '.json'):
            with open(connectome_path + item + '.json', "r") as data_file:
                data = json.load(data_file)
                brain[item] = data
    print("Brain has been successfully loaded into memory...")
    return brain


def genome_handler(connectome_path):
    # Calling function to regenerate the Brain from the Genome
    if runtime_data.parameters["InitData"]["regenerate_brain"]:
        print("use_static_genome:", runtime_data.parameters["Switches"]["use_static_genome"])
        if runtime_data.parameters["Switches"]["use_static_genome"]:
            print("** ** ** ** ** ** ** ** **")
            stage_genome(connectome_path, dynamic_selection_mode=False)
            load_genome_in_memory(connectome_path, static=True)
            print(settings.Bcolors.RED + ">> >> >> A static genome was used to generate the brain."
                  + settings.Bcolors.ENDC)
        else:
            stage_genome(connectome_path)
            load_genome_in_memory(connectome_path)
    else:
        # Using the existing genome previously staged in the connectome_path
        load_genome_in_memory(connectome_path)


def serialize_brain_data(brain):
    for cortical_area in brain:
        for neuron_id in brain[cortical_area]:
            runtime_data.brain[cortical_area][neuron_id]["activity_history"] = \
                list(runtime_data.brain[cortical_area][neuron_id]["activity_history"])
    return brain


def load_block_dic_in_memory():
    # todo: Need error handling added so if there is a corruption in block_dic data it can regenerate
    connectome_path = runtime_data.parameters["InitData"]["connectome_path"]
    block_dic = {}
    for item in runtime_data.cortical_list:
        if os.path.isfile(connectome_path + item + '_blk_dic.json'):
            with open(connectome_path + item + '_blk_dic.json', "r") as data_file:
                data = json.load(data_file)
                block_dic[item] = data
    return block_dic


def save_block_dic_to_disk(cortical_area='all', block_dic=runtime_data.block_dic,
                           parameters=runtime_data.parameters, backup=False):
    connectome_path = parameters["InitData"]["connectome_path"]
    if block_dic == {}:
        print(">> >> Error: Could not save the brain contents to disk as >> block_dic << was empty!")
        return

    if cortical_area != 'all':
        with open(connectome_path+cortical_area+'_blk_dic.json', "w") as data_file:
            data = block_dic[cortical_area]
            data_file.seek(0)  # rewind
            data_file.write(json.dumps(data, indent=3))
            data_file.truncate()
    elif backup:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_backup_blk_dic.json', "w") as data_file:
                data = block_dic[cortical_area]
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
    else:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_blk_dic.json', "w") as data_file:
                data = block_dic[cortical_area]
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
    return


def save_brain_to_disk(cortical_area='all', brain=runtime_data.brain, parameters=runtime_data.parameters, backup=False):
    connectome_path = parameters["InitData"]["connectome_path"]
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
    elif backup:
        for cortical_area in runtime_data.cortical_list:
            with open(connectome_path+cortical_area+'_backup.json', "w") as data_file:
                data = brain[cortical_area]
                # if runtime_data.parameters["Logs"]["print_brain_gen_activities"]:
                    # print(">>> >>> All data related to Cortical area %s is saved in connectome" % cortical_area)
                # Saving changes to the connectome
                data_file.seek(0)  # rewind
                data_file.write(json.dumps(data, indent=3))
                data_file.truncate()
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
    mongo = db_handler.MongoManagement()
    fcl_data = {}
    fcl_data['genome_id'] = runtime_data.genome_id
    fcl_data['burst_id'] = burst_number
    fcl_data['number_under_training'] = number_under_training
    fcl_data['fcl_data'] = fire_candidate_list
    mongo.insert_neuron_activity(fcl_data=fcl_data)
