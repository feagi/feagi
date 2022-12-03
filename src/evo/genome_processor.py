
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

import copy
import traceback
from evo.genome_editor import save_genome
from evo.genome_validator import *


def genome_ver_check(genome):
    try:
        if genome['version'] == "2.0":
            print("\n\n\n************ Genome Version 2.0 has been detected **************\n\n\n")
            try:
                blueprint_validator(genome)
            except Exception:
                print("Error during genome validation!!\n", traceback.print_exc())
            save_genome(genome=genome, file_name="../runtime_genome.json")
            genome2 = genome_2_1_convertor(flat_genome=genome['blueprint'])
            genome_2_hierarchifier(flat_genome=genome['blueprint'])
            genome['blueprint'] = genome2['blueprint']
            return genome
        else:
            print("ERROR! Genome is not compatible with 2.0 standard")
    except KeyError as e:
        print("Error:", e)
        print("Genome version not available; assuming Genome 1.0 procedures.")
        pass


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
        if json_comment_catcher(key):
            cortical_id = key[9:15]
            if cortical_id not in cortical_list and key[7] == "c":
                cortical_list.append(cortical_id)
    return cortical_list


def genome_1_cortical_list(genome):
    cortical_list = list()
    for cortical_area in genome['blueprint']:
        cortical_list.append(cortical_area)

    return cortical_list


def json_comment_catcher(key):
    if key[:1] == '/':
        return False
    else:
        return True


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
            if json_comment_catcher(gene):
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
                                    if json_comment_catcher(flat_genome[gene][destination]) and \
                                            json_comment_catcher(destination):
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
    print(genome)
    return genome


def genome_v1_v2_converter(genome_v1):
    genome_v2 = genome_v1.copy()
    genome_v2.pop('blueprint')
    genome_v2['blueprint'] = {}

    for cortical_area in genome_v1['blueprint']:
        for key in genome_v1['blueprint'][cortical_area]:
            if type(key) is not dict and key not in ["cortical_mapping_dst"]:
                if key in genome_1_to_2:
                    gene = "_____10c-" + cortical_area + "-" + genome_1_to_2[key]
                    genome_v2['blueprint'][gene] = genome_v1['blueprint'][cortical_area][key]
                elif key == "neuron_params":
                    for subkey in genome_v1['blueprint'][cortical_area]["neuron_params"]:
                        print(">>->>", subkey)
                        if subkey not in ["block_boundaries", "relative_coordinate"]:
                            if subkey in genome_1_to_2:
                                gene = "_____10c-" + cortical_area + "-" + genome_1_to_2[subkey]
                                genome_v2['blueprint'][gene] = genome_v1['blueprint'][cortical_area]['neuron_params'][subkey]
                        elif subkey == "block_boundaries":
                            genex = "_____10c-" + cortical_area + "-" + "cx-___bbx-i"
                            geney = "_____10c-" + cortical_area + "-" + "cx-___bby-i"
                            genez = "_____10c-" + cortical_area + "-" + "cx-___bbz-i"

                            genome_v2['blueprint'][genex] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['block_boundaries'][0]
                            genome_v2['blueprint'][geney] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['block_boundaries'][1]
                            genome_v2['blueprint'][genez] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['block_boundaries'][2]
                        elif subkey == "relative_coordinate":
                            genex = "_____10c-" + cortical_area + "-" + "cx-rcordx-i"
                            geney = "_____10c-" + cortical_area + "-" + "cx-rcordy-i"
                            genez = "_____10c-" + cortical_area + "-" + "cx-rcordz-i"

                            genome_v2['blueprint'][genex] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][0]
                            genome_v2['blueprint'][geney] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][1]
                            genome_v2['blueprint'][genez] = \
                                genome_v1['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][2]

            elif key == "cortical_mapping_dst":
                gene = "_____10c-" + cortical_area + "-dstmap-d"
                destination_map = {}
                for destination in genome_v1['blueprint'][cortical_area]["cortical_mapping_dst"]:
                    destination_map[destination] = list()
                    for entry in genome_v1['blueprint'][cortical_area]["cortical_mapping_dst"][destination]:
                        morphology_id = entry["morphology_id"]
                        morphology_scalar = entry["morphology_scalar"]
                        postSynapticCurrent_multiplier = entry["postSynapticCurrent_multiplier"]
                        plasticity_flag = entry["plasticity_flag"]

                        destination_map[destination].append([morphology_id,
                                                            morphology_scalar,
                                                            postSynapticCurrent_multiplier,
                                                            plasticity_flag])

                genome_v2['blueprint'][gene] = destination_map
            else:
                print("Warning! ", key, " not found in genome_1_template!")

    return genome_v2


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
          "synapse_attractivity": None,
          "postsynaptic_current": None,
          "plasticity_constant": None,
          "degeneration": None,
          "psp_uniform_distribution": False,
          "postsynaptic_current_max": None,
          "cortical_mapping_dst": {},
          "neuron_params": {
              "firing_threshold": None,
              "refractory_period": None,
              "leak_coefficient": None,
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
    "cx-de_gen-f": "degeneration",
    "cx-pspuni-b": "psp_uniform_distribution"
}

genome_1_to_2 = {
    "cortical_name": "cx-__name-t",
    "group_id": "cx-_group-t",
    "per_voxel_neuron_cnt": "cx-_n_cnt-i",
    "visualization": "cx-gd_vis-b",
    "relative_coordinate": "cx-rcord_-i",
    "block_boundaries": "cx-___bb_-i",
    "location_generation_type": "cx-__rand-b",
    "synapse_attractivity": "cx-synatt-i",
    "postsynaptic_current": "nx-pstcr_-f",
    "postsynaptic_current_max": "nx-pstcrm-f",
    "plasticity_constant": "nx-plst_c-f",
    "firing_threshold": "nx-fire_t-f",
    "refractory_period": "nx-refrac-i",
    "leak_coefficient": "nx-leak_c-f",
    "consecutive_fire_cnt_max": "nx-c_fr_c-i",
    "snooze_length": "nx-snooze-f",
    "degeneration": "cx-de_gen-f",
    "psp_uniform_distribution": "cx-pspuni-b",
    "cortical_mapping_dst": "cs-dstmap-d"
}
