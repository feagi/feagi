
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

import json
import copy


# def load_genome_in_memory(connectome_path, static=False):
#     if not static:
#         print("Genome from local connectome folder was chosen: ", connectome_path)
#         with open(runtime_data.working_directory + '/genome_tmp.json', "r") as genome_file:
#             genome_data = json.load(genome_file)
#             runtime_data.genome = genome_data
#     else:
#         runtime_data.genome = genome
#         print("Static genome was loaded in memory")
#     # todo: The following is suitable for the main_auto but needs to be adjusted for the main_manual
#     runtime_data.genome_id = genome_id_gen()
#

# def save_genome_to_db():
#     from evo.genetics import calculate_brain_cognitive_fitness
#     # mongo = db_handler.MongoManagement()
#     # influxdb = db_handler.InfluxManagement()
#     genome = runtime_data.genome
#     genome_id = runtime_data.genome_id
#
#     updated_genome_test_stats = []
#     for item in runtime_data.genome_test_stats:
#         item["genome_id"] = genome_id
#         updated_genome_test_stats.append(item)
#
#     # print(updated_genome_test_stats)
#     print("*** @@@ *** @@@ *** Genome_id:  \n ", genome_id)
#
#     genome_db = {}
#     genome_db["genome_id"] = genome_id
#     genome_db["generation_date"] = str(datetime.now())
#     genome_db["properties"] = genome
#     genome_db["parameters"] = runtime_data.parameters
#     genome_db["activity_stats"] = runtime_data.activity_stats
#
#     if not runtime_data.termination_flag:
#         brain_fitness = calculate_brain_cognitive_fitness(runtime_data.genome_test_stats)
#         genome_db["fitness"] = brain_fitness
#         print("\n\n\n")
#         print(" **********")
#         print(" ********** ********")
#         print(" ********** ******** ********")
#         print(" ********** ******** ******** ***** *** ** * Brain fitness: ", brain_fitness)
#         print(" ********** ******** ********")
#         print(" ********** ********")
#         print(" **********")
#         print("\n\n\n")
#         runtime_data.influxdb.insert_evolutionary_fitness_stats(connectome_path=runtime_data.parameters["InitData"]
#         ["connectome_path"],
#                                                                 fitness_score=brain_fitness / 1,
#                                                                 training_sets=runtime_data.parameters["Auto_injector"]
#                                                                 ["variation_default"],
#                                                                 test_sets=runtime_data.parameters["Auto_tester"]
#                                                                 ["variation_default"],
#                                                                 training_exposure=
#                                                                 runtime_data.parameters["Auto_injector"]
#                                                                 ["exposure_default"],
#                                                                 test_exposure=runtime_data.parameters["Auto_tester"]
#                                                                 ["exposure_default"]
#                                                                 )
#     else:
#         brain_fitness = ''
#         print("\n\n\n")
#         print("       ****")
#         print(" **********")
#         print(" **** Brain was terminated prematurely ******")
#         print(" **********")
#         print("       ****")
#
#     # Logging cortical stats in the InfluxDb
#     for cortical_area in runtime_data.brain:
#         neuron_count, synapse_count = stats.connectome_total_synapse_cnt(cortical_area)
#         runtime_data.influxdb.insert_evolutionary_connectome_stats(
#             connectome_path=runtime_data.parameters["InitData"]["connectome_path"],
#             cortical_area=cortical_area,
#             neuron_count=neuron_count,
#             synapse_count=synapse_count)
#
#     print("*** @@@ *** @@@ *** \n ", genome_db)
#
#     runtime_data.mongodb.insert_genome(genome_db)
#
#     mail_body = "Genome " + str(genome_id) + " has been evaluated to have a fitness of " + str(brain_fitness)
#
#     # Sending out email
#     # if brain_fitness > runtime_data.parameters["Alerts"]["email_fitness_threshold"]:
#     #     alerts.send_email(mail_body)
#
#     print(">>>Genome test stats: >", runtime_data.genome_test_stats)
#     print(">>>Genome_id: >", runtime_data.genome_id)
#
#     for stat in runtime_data.genome_test_stats:
#         stat_to_save = stat
#         # todo: The following is leading to duplicate db record ---> Investigate
#         # runtime_data.mongodb.insert_test_stats(stat_to_save)
#
#     print("Genome %s has been preserved for future generations!" % genome_id)
#     stats.print_fcl_stats(genome_id)
#
#     return
#
#
# def stage_genome(connectome_path, dynamic_selection_mode=True):
#     from evo.genetics import select_a_genome
#     if dynamic_selection_mode:
#         genome_data, original_genome_id = select_a_genome()
#         runtime_data.original_genome_id = original_genome_id
#     else:
#         # load_genome_in_memory(connectome_path, static=True)
#         genome_data = runtime_data.genome
#         runtime_data.original_genome_id = ["static"]
#
#     print("Staged genome had the following genome id:", runtime_data.original_genome_id)
#     genome_data['genome_id_source'] = runtime_data.original_genome_id
#     with open(runtime_data.working_directory + '/genome_tmp.json', "w") as staged_genome:
#         # Saving changes to the connectome
#         staged_genome.seek(0)  # rewind
#         try:
#             staged_genome.write(json.dumps(genome_data, indent=3))
#         except TypeError:
#             bson_to_json = json_util.dumps(genome_data)
#             genome_data = json.loads(bson_to_json)
#             staged_genome.write(json.dumps(genome_data, indent=3))
#         staged_genome.truncate()
#
#         # print("\n*\n**\n***\ngenome_tmp.json was just staged...vvv ^^^ vvv\n***\n**\n*", connectome_path)
#
#     print("<< << Genome has been staged in runtime repo >> >>")
#
#
# def genome_handler(connectome_path):
#     # Calling function to regenerate the Brain from the Genome
#     if runtime_data.parameters["InitData"]["regenerate_brain"]:
#         print("use_static_genome:", runtime_data.parameters["Switches"]["use_static_genome"])
#         if runtime_data.parameters["Switches"]["use_static_genome"]:
#             stage_genome(connectome_path, dynamic_selection_mode=False)
#             load_genome_in_memory(connectome_path, static=True)
#             print(settings.Bcolors.RED + ">> >> >> A static genome was used to generate the brain."
#                   + settings.Bcolors.ENDC)
#         else:
#             stage_genome(connectome_path)
#             load_genome_in_memory(connectome_path)
#     else:
#         # Using the existing genome previously staged in the connectome_path
#         load_genome_in_memory(connectome_path)
#


def genome_2_print(genome):
    for cortical_area in genome:
        print(cortical_area)
        for gene in genome[cortical_area]:
            try:
                print("      ", genome_2_to_1[gene], "\n\t\t\t", genome[cortical_area][gene])
            except:
                pass


def genome_2_validator(genome_2):
    """
    Conducts various test to ensure the stability of the Genome 2.0
    """
    standard_gene_length = 27

    def structure_test_gene_lengths():
        """
        Check length requirements for each gene
        """
        gene_anomalies = 0
        for key in genome_2:
            if len(key) != standard_gene_length:
                print("Warning! Key did not meet length requirement:", key)
                gene_anomalies += 1
        if gene_anomalies == 0:
            print("\nGene length verification...... PASSED!")
        else:
            print("\nGene length verification...... Failed!   ", gene_anomalies, " anomalies detected")
        return gene_anomalies


def genome_2_hierarchifier(flat_genome):
    """
    Converts Genome 2.0 to a hierarchical data structure
    """
    hierarchical_genome = dict()
    for key in flat_genome:
        cortical_id = key[9:15]
        exon = key[16:]
        if key[7] == "c":
            if cortical_id not in hierarchical_genome:
                hierarchical_genome[cortical_id] = dict()
            if exon not in hierarchical_genome[cortical_id]:
                hierarchical_genome[cortical_id][exon] = flat_genome[key]
    # genome_2_print(hierarchical_genome)
    return hierarchical_genome


def genome_1_cortical_list(genome):
    cortical_list = list()
    for cortical_area in genome:
        cortical_list.append(cortical_area)
    return cortical_list


def genome_2_cortical_list(flat_genome):
    """
    Generates a list of cortical areas inside genome
    """
    cortical_list = list()
    for key in flat_genome:
        cortical_id = key[9:15]
        if cortical_id not in cortical_list and key[7] == "c":
            cortical_list.append(cortical_id)
    return cortical_list


def genome_2_1_convertor(flat_genome):
    genome = dict()
    genome['blueprint'] = dict()

    cortical_list = genome_2_cortical_list(flat_genome)
    # Assign a blank template to each cortical area
    for cortical_area in cortical_list:
        genome['blueprint'][cortical_area] = copy.deepcopy(genome_1_template)

    # Populate each cortical area with
    for cortical_area in genome['blueprint']:
        for gene in flat_genome:
            cortical_id = gene[9:15]
            exon = gene[16:]
            gene_type = gene[16:18]
            if cortical_id == cortical_area:
                try:
                    if gene_type == 'cx':
                        if genome_2_to_1[exon] == "cortical_name":
                            genome['blueprint'][cortical_area][genome_2_to_1[exon]] = flat_genome[gene]
                        elif genome_2_to_1[exon] == "location_generation_type":
                            if flat_genome[gene]:
                                    genome['blueprint'][cortical_area][genome_2_to_1[exon]] = "random"
                            else:
                                genome['blueprint'][cortical_area][genome_2_to_1[exon]] = "sequential"
                        elif genome_2_to_1[exon] == "cortical_mapping_dst":
                            for destination in flat_genome[gene]:
                                for mapping_recipe in flat_genome[gene][destination]:
                                    if destination not in genome['blueprint'][cortical_area][genome_2_to_1[exon]]:
                                        genome['blueprint'][cortical_area][genome_2_to_1[exon]][destination] = list()

                                    temp_dict = dict()
                                    temp_dict["morphology_id"] = mapping_recipe[0]
                                    temp_dict["morphology_scalar"] = mapping_recipe[1]
                                    temp_dict["postSynapticCurrent_multiplier"] = mapping_recipe[2]
                                    temp_dict["plasticity_flag"] = mapping_recipe[3]

                                    genome['blueprint'][
                                        cortical_area][genome_2_to_1[exon]][destination].append(temp_dict)
                        else:
                            try:
                                genome['blueprint'][cortical_area][genome_2_to_1[exon]] = flat_genome[gene]
                            except:
                                print("Key not processed: ", cortical_area)
                    elif gene_type == 'nx':
                        if genome_2_to_1[exon] == "block_boundaries":
                            if gene[24] == 'x':
                                genome['blueprint'][cortical_area]["neuron_params"]["block_boundaries"][0] = \
                                    flat_genome[gene]
                            elif gene[24] == 'y':
                                genome['blueprint'][cortical_area]["neuron_params"]["block_boundaries"][1] = \
                                    flat_genome[gene]
                            elif gene[24] == 'z':
                                genome['blueprint'][cortical_area]["neuron_params"]["block_boundaries"][2] = \
                                    flat_genome[gene]
                            else:
                                pass

                        elif genome_2_to_1[exon] == "relative_coordinate":
                            if gene[24] == 'x':
                                genome['blueprint'][cortical_area]["neuron_params"]["relative_coordinate"][0] = \
                                    flat_genome[gene]
                            elif gene[24] == 'y':
                                genome['blueprint'][cortical_area]["neuron_params"]["relative_coordinate"][1] = \
                                    flat_genome[gene]
                            elif gene[24] == 'z':
                                genome['blueprint'][cortical_area]["neuron_params"]["relative_coordinate"][2] = \
                                    flat_genome[gene]
                            else:
                                pass

                        else:
                            genome['blueprint'][cortical_area]["neuron_params"][genome_2_to_1[exon]] = flat_genome[gene]
                    else:
                        pass
                except KeyError as e:
                    print("Error while converting a gene:", e, cortical_area, gene)
    # print(genome)
    return genome


gene_decoder = {
    "_______b-_____s-__-__name-t": "species_name",
    "_______c-______-cx-__name-t": "cortical_name",
    "_______c-______-cx-_n_cnt-i": "cortical_neuron_count",
    "_______c-______-cx-gd_vis-b": "godot_visualization",
    "_______c-______-cx-rcordx-i": "relative_coordinate_x",
    "_______c-______-cx-rcordy-i": "relative_coordinate_y",
    "_______c-______-cx-rcordz-i": "relative_coordinate_z",
    "_______c-______-cx-___bbx-i": "block_boundary_x",
    "_______c-______-cx-___bby-i": "block_boundary_y",
    "_______c-______-cx-___bbz-i": "block_boundary_z",
    "_______c-______-cx-synatt-i": "synapse_attractivity",
    "_______c-______-nx-pstcr_-f": "postsynaptic_current",
    "_______c-______-nx-pstcrm-f": "postsynaptic_current_max",
    "_______c-______-nx-plst_c-f": "plasticity_constant",
    "_______c-______-nx-fire_t-f": "firing_threshold",
    "_______c-______-nx-refrac-i": "refractory_period",
    "_______c-______-nx-leak_c-f": "leak_coefficient",
    "_______c-______-nx-c_fr_c-i": "consecutive_fire_cnt_max",
    "_______c-______-nx-snooze-f": "snooze_length",
    "_______c-______-cx-__rand-b": "location_generation_type",
    "_______c-______-cs-dstmap-d": "cortical_mapping_dst"
}

genome_1_template = {
          "per_voxel_neuron_cnt": None,
          "location_generation_type": None,
          "synapse_attractivity": None,
          "postsynaptic_current": None,
          "plasticity_constant": None,
          "degeneration": None,
          "postsynaptic_current_max": None,
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": None,
              "orientation_selectivity_id": None,
              "depolarization_threshold": None,
              "firing_threshold": None,
              "firing_pattern_id": None,
              "refractory_period": None,
              "axon_avg_length": None,
              "leak_coefficient": None,
              "axon_avg_connections": None,
              "axon_orientation function": None,
              "consecutive_fire_cnt_max": None,
              "snooze_length": None,
              "block_boundaries": [
                  None,
                  None,
                  None
              ],
              "relative_coordinate": [
                  None,
                  None,
                  None
              ],
              "visualization": None,
          }
      }

genome_2_to_1 = {
    "cx-_n_cnt-i": "per_voxel_neuron_cnt",
    "nx-gd_vis-b": "visualization",
    "cx-__name-t": "cortical_name",
    "nx-rcordx-i": "relative_coordinate",
    "nx-rcordy-i": "relative_coordinate",
    "nx-rcordz-i": "relative_coordinate",
    "nx-___bbx-i": "block_boundaries",
    "nx-___bby-i": "block_boundaries",
    "nx-___bbz-i": "block_boundaries",
    "cx-__rand-b": "location_generation_type",
    "cx-synatt-i": "synapse_attractivity",
    "cx-pstcr_-f": "postsynaptic_current",
    "cx-pstcrm-f": "postsynaptic_current_max",
    "cx-plst_c-f": "plasticity_constant",
    "nx-fire_t-f": "firing_threshold",
    "nx-refrac-i": "refractory_period",
    "nx-leak_c-f": "leak_coefficient",
    "nx-c_fr_c-i": "consecutive_fire_cnt_max",
    "nx-snooze-f": "snooze_length",
    "cx-_group-t": "group_id",
    "cx-dstmap-d": "cortical_mapping_dst",
    "cx-de_gen-f": "degeneration"
}

genome_1_to_2 = {
    "per_voxel_neuron_cnt": "cx-_n_cnt-i",
    "visualization": "cx-gd_vis-b",
    "relative_coordinate": "cx-rcord_-i",
    "block_boundaries": "cx-___bb_-i",
    "location_generation_type": "cx-__rand-b",
    "synapse_attractivity": "cx-synatt-i",
    "postsynaptic_current": "nx-pstcr_-f",
    "postsynaptic_current_max": "nx-pstcrm-f",
    "plasticity_constant": "nx-plst_c-f",
    "neighbor_locator_rule_id": "nx-locr__-t",
    "neighbor_locator_rule_param_id": "nx-locrp_-t",
    "firing_threshold": "nx-fire_t-f",
    "refractory_period": "nx-refrac-i",
    "leak_coefficient": "nx-leak_c-f",
    "consecutive_fire_cnt_max": "nx-c_fr_c-i",
    "snooze_length": "nx-snooze-f",
    "group_id": "cx-_group-t",
    "cortical_mapping_dst": "cs-dstmap-d",
    "degeneration": "cx-de_gen-f"
}
