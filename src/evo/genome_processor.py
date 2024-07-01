# Copyright 2016-2023 The FEAGI Authors. All Rights Reserved.
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
import copy
import traceback
import datetime
from src.evo.genome_editor import save_genome
from src.evo.genome_validator import genome_validator
from src.evo.templates import core_morphologies, cortical_types
from src.inf import runtime_data


logger = logging.getLogger(__name__)


def merge_core_morphologies(genome):
    for core_morphology in core_morphologies:
        genome["neuron_morphologies"][core_morphology] = core_morphologies[core_morphology].copy()
    return genome


def genome_ver_check(genome):
    try:
        if genome['version'] == "2.0":
            print("\n\n\n************ Genome Version 2.0 has been detected **************\n\n\n")
            try:
                runtime_data.genome_validity = genome_validator(genome)
                print("Genome validity=", runtime_data.genome_validity)
            except Exception as e:
                print("Error during genome validation!!\n", traceback.print_exc(), e)
            genome = merge_core_morphologies(genome)
            genome = genome_morphology_updator(genome)
            genome = genome_physiology_updator(genome=genome)
            genome = genome_stat_updator(genome=genome)
            save_genome(genome=genome, file_name=runtime_data.connectome_path + "genome.json")
            genome1 = genome_2_1_convertor(flat_genome=genome['blueprint'])
            genome_2_hierarchifier(flat_genome=genome['blueprint'])
            genome['blueprint'] = genome1['blueprint']
            return genome
        else:
            print("ERROR! Genome is not compatible with 2.0 standard")
    except KeyError as e:
        print("Exception during genome version check", e, traceback.print_exc())
        pass


def genome_2_print(genome):
    for cortical_area in genome:
        print(cortical_area)
        for gene in genome[cortical_area]:
            try:
                print("       ", genome_2_to_1[gene], "\n\t\t\t", genome[cortical_area][gene])
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
            print("\nGene length verification ...... PASSED!")
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
    try:
        cortical_list = list()
        for key in flat_genome:
            if json_comment_catcher(key):
                cortical_id = key[9:15]
                if cortical_id not in cortical_list and key[7] == "c":
                    cortical_list.append(cortical_id)
        return cortical_list
    except Exception as e:
        print("Exception during genome_2_cortical_list", e, traceback.print_exc())


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


def cortical_area_id_update_checker(cortical_id):
    """
    Responsible for updating deprecated cortical names to new ones
    """
    if cortical_id == "i__v0C":
        return "iv00_C"
    elif cortical_id == "i_v0BL":
        return "iv00BL"
    elif cortical_id == "i_v0BR":
        return "iv00BR"
    elif cortical_id == "i_v0BM":
        return "iv00BM"
    elif cortical_id == "i_v0ML":
        return "iv00ML"
    elif cortical_id == "i_v0MR":
        return "iv00MR"
    elif cortical_id == "i_v0TL":
        return "iv00TL"
    elif cortical_id == "i_v0TR":
        return "iv00TR"
    elif cortical_id == "i_v0TM":
        return "iv00TM"
    else:
        return cortical_id


def genome_2_1_convertor(flat_genome):
    genome = dict()
    genome['blueprint'] = dict()
    cortical_list = genome_2_cortical_list(flat_genome)
    print("%" * 30)
    print(cortical_list)
    # Assign a blank template to each cortical area
    for cortical_area in cortical_list:
        genome['blueprint'][cortical_area_id_update_checker(cortical_id=cortical_area)] = \
            copy.deepcopy(genome_1_template)

    # Populate each cortical area with
    for cortical_area in genome['blueprint']:
        try:
            for gene in flat_genome:
                if json_comment_catcher(gene):
                    cortical_id = cortical_area_id_update_checker(cortical_id=gene[9:15])
                    exon = gene[19:]
                    gene_type = gene[16:18]
                    if exon in genome_2_to_1:
                        if cortical_id == cortical_area:
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
                                                genome['blueprint'][cortical_area][genome_2_to_1[exon]][destination] = \
                                                    list()

                                            temp_dict = dict()

                                            temp_dict["morphology_id"] = mapping_recipe[0]
                                            temp_dict["morphology_scalar"] = mapping_recipe[1]
                                            temp_dict["postSynapticCurrent_multiplier"] = mapping_recipe[2]
                                            temp_dict["plasticity_flag"] = mapping_recipe[3]
                                            if mapping_recipe[3]:
                                                try:
                                                    temp_dict["plasticity_constant"] = mapping_recipe[4]
                                                    temp_dict["ltp_multiplier"] = mapping_recipe[5]
                                                    temp_dict["ltd_multiplier"] = mapping_recipe[6]
                                                except Exception as e:
                                                    temp_dict["plasticity_constant"] = 1
                                                    temp_dict["ltp_multiplier"] = 1
                                                    temp_dict["ltd_multiplier"] = 1
                                            else:
                                                temp_dict["plasticity_flag"] = False
                                                temp_dict["plasticity_constant"] = 1
                                                temp_dict["ltp_multiplier"] = 1
                                                temp_dict["ltd_multiplier"] = 1

                                            genome['blueprint'][
                                                cortical_area][genome_2_to_1[exon]][destination].append(temp_dict)

                            elif genome_2_to_1[exon] == "block_boundaries":
                                if gene[24] == 'x':
                                    genome['blueprint'][cortical_area]["block_boundaries"][0] = \
                                        flat_genome[gene]
                                elif gene[24] == 'y':
                                    genome['blueprint'][cortical_area]["block_boundaries"][1] = \
                                        flat_genome[gene]
                                elif gene[24] == 'z':
                                    genome['blueprint'][cortical_area]["block_boundaries"][2] = \
                                        flat_genome[gene]
                                else:
                                    pass

                            elif genome_2_to_1[exon] == "relative_coordinate":
                                if gene[24] == 'x':
                                    genome['blueprint'][cortical_area]["relative_coordinate"][0] = \
                                        flat_genome[gene]
                                elif gene[24] == 'y':
                                    genome['blueprint'][cortical_area]["relative_coordinate"][1] = \
                                        flat_genome[gene]
                                elif gene[24] == 'z':
                                    genome['blueprint'][cortical_area]["relative_coordinate"][2] = \
                                        flat_genome[gene]
                                else:
                                    pass
                            elif genome_2_to_1[exon] == "2d_coordinate":
                                if gene[24] == 'x':
                                    genome['blueprint'][cortical_area]["2d_coordinate"][0] = \
                                        flat_genome[gene]
                                elif gene[24] == 'y':
                                    genome['blueprint'][cortical_area]["2d_coordinate"][1] = \
                                        flat_genome[gene]
                                else:
                                    pass

                            else:
                                try:
                                    genome['blueprint'][cortical_area][genome_2_to_1[exon]] = flat_genome[gene]
                                except Exception as e:
                                    print("Key not processed: ", cortical_area, e, traceback.print_exc())

        except Exception as e:
            print(f"Exception during gene translation of {cortical_area}", e, traceback.print_exc())
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
                else:
                    if key not in ["block_boundaries", "relative_coordinate", "2d_coordinate"]:
                        if key in genome_1_to_2:
                            gene = "_____10c-" + cortical_area + "-" + genome_1_to_2[key]
                            genome_v2['blueprint'][gene] = genome_v1['blueprint'][cortical_area][key]
                    if key == "block_boundaries":
                        genex = "_____10c-" + cortical_area + "-" + "cx-___bbx-i"
                        geney = "_____10c-" + cortical_area + "-" + "cx-___bby-i"
                        genez = "_____10c-" + cortical_area + "-" + "cx-___bbz-i"

                        genome_v2['blueprint'][genex] = \
                            genome_v1['blueprint'][cortical_area]["block_boundaries"][0]
                        genome_v2['blueprint'][geney] = \
                            genome_v1['blueprint'][cortical_area]["block_boundaries"][1]
                        genome_v2['blueprint'][genez] = \
                            genome_v1['blueprint'][cortical_area]["block_boundaries"][2]
                    if key == "relative_coordinate":
                        genex = "_____10c-" + cortical_area + "-" + "cx-rcordx-i"
                        geney = "_____10c-" + cortical_area + "-" + "cx-rcordy-i"
                        genez = "_____10c-" + cortical_area + "-" + "cx-rcordz-i"

                        genome_v2['blueprint'][genex] = \
                            genome_v1['blueprint'][cortical_area]["relative_coordinate"][0]
                        genome_v2['blueprint'][geney] = \
                            genome_v1['blueprint'][cortical_area]["relative_coordinate"][1]
                        genome_v2['blueprint'][genez] = \
                            genome_v1['blueprint'][cortical_area]["relative_coordinate"][2]
                    if key == "2d_coordinate":
                        genex = "_____10c-" + cortical_area + "-" + "cx-2dcorx-i"
                        geney = "_____10c-" + cortical_area + "-" + "cx-2dcory-i"

                        genome_v2['blueprint'][genex] = \
                            genome_v1['blueprint'][cortical_area]["2d_coordinate"][0]
                        genome_v2['blueprint'][geney] = \
                            genome_v1['blueprint'][cortical_area]["2d_coordinate"][1]

            elif key == "cortical_mapping_dst":
                gene = "_____10c-" + cortical_area + "-cx-dstmap-d"
                destination_map = {}
                for destination in genome_v1['blueprint'][cortical_area]["cortical_mapping_dst"]:
                    destination_map[destination] = list()
                    for entry in genome_v1['blueprint'][cortical_area]["cortical_mapping_dst"][destination]:
                        morphology_id = entry["morphology_id"]
                        morphology_scalar = entry["morphology_scalar"]
                        postSynapticCurrent_multiplier = entry["postSynapticCurrent_multiplier"]
                        plasticity_flag = entry["plasticity_flag"]

                        if "plasticity_constant" in entry:
                            plasticity_constant = entry["plasticity_constant"]
                        else:
                            plasticity_constant = 1

                        if "ltp_multiplier" in entry:
                            ltp_multiplier = entry["ltp_multiplier"]
                        else:
                            ltp_multiplier = 1

                        if "ltp_multiplier" in entry:
                            ltd_multiplier = entry["ltd_multiplier"]
                        else:
                            ltd_multiplier = 1

                        destination_map[destination].append([morphology_id,
                                                             morphology_scalar,
                                                             postSynapticCurrent_multiplier,
                                                             plasticity_flag,
                                                             plasticity_constant,
                                                             ltp_multiplier,
                                                             ltd_multiplier])

                genome_v2['blueprint'][gene] = destination_map
            else:
                print("Warning! ", key, " not found in genome_1_template!")

    return genome_v2


def morphology_convertor(morphology_in):
    morphology_out = dict()
    morphology_out["parameters"] = dict()
    # Upgrade old genome missing type
    if "type" in morphology_in:
        morphology_out = morphology_in
    else:
        if "vectors" in morphology_in:
            morphology_out["type"] = "vectors"
            morphology_out["parameters"]["vectors"] = morphology_in["vectors"]
            print("morphology_out:", morphology_out)
        elif "patterns" in morphology_in:
            morphology_out["type"] = "patterns"
            morphology_out["parameters"]["patterns"] = morphology_in["patterns"]
        elif "composite" in morphology_in:
            morphology_out["type"] = "composite"
            morphology_out["parameters"]["src_seed"] = morphology_in["composite"]["parameters"]["src_seed"]
            morphology_out["parameters"]["src_pattern"] = morphology_in["composite"]["parameters"]["src_pattern"]
            morphology_out["parameters"]["mapper_morphology"] = morphology_in["composite"]["mapper_morphology"]
        elif "functions" in morphology_in:
            morphology_out["type"] = "functions"
        else:
            pass

    print("morphology out 1", morphology_out)
    # Fix pattern nesting
    if "patterns" in morphology_out["parameters"]:
        for pattern in morphology_out["parameters"]["patterns"]:
            if not valid_pattern(pattern):
                print("#### >>>", pattern)
                if len(pattern) == 3:
                    print("> >", morphology_in)
                    morphology_out["parameters"]["patterns"] = [morphology_out["parameters"]["patterns"]]
                    print("> >", morphology_out)

    if "class" not in morphology_out:
        morphology_out["class"] = "custom"

    return morphology_out


def valid_pattern(lst):
    # Check if the input is a list
    if not isinstance(lst, list):
        return False

    for sublist in lst:
        # Check if each element of the list is also a list
        if not isinstance(sublist, list):
            return False

        # Check if the inner list has a length of 3
        if len(sublist) != 3:
            return False

        # Check if each element in the inner list is an integer, '*', or '?'
        for item in sublist:
            if not (isinstance(item, int) or item in ['*', '?']):
                return False

    return True


def genome_morphology_updator(genome):
    try:
        for morphology in genome["neuron_morphologies"]:
            if not morphology:
                genome["neuron_morphologies"].pop(morphology)
            genome["neuron_morphologies"][morphology] = morphology_convertor(genome["neuron_morphologies"][morphology])
        runtime_data.genome_validity = genome_validator(genome)
    except Exception as e:
        print("Error during genome morphology update!", e, traceback.print_exc())

    return genome


def is_memory_cortical_area(cortical_area):
    if "MEMORY" in runtime_data.genome["blueprint"][cortical_area]["sub_group_id"]:
        return True
    else:
        return False


def genome_physiology_updator(genome: dict):
    if "physiology" not in genome:
        genome["physiology"] = {}
    if "max_burst_count" in genome:
        genome.pop("max_burst_count")
    if "burst_delay" in genome:
        genome["physiology"]["burst_delay"] = genome["burst_delay"]
        genome.pop("burst_delay")
    if "max_age" in genome:
        genome["physiology"]["max_age"] = genome["max_age"]
        genome.pop("max_age")
    if "evolution_burst_count" in genome:
        genome["physiology"]["evolution_burst_count"] = genome["evolution_burst_count"]
        genome.pop("evolution_burst_count")
    if "ipu_idle_threshold" in genome:
        genome["physiology"]["ipu_idle_threshold"] = genome["ipu_idle_threshold"]
        genome.pop("ipu_idle_threshold")
    if "plasticity_queue_depth" in genome:
        genome["physiology"]["plasticity_queue_depth"] = genome["plasticity_queue_depth"]
        genome.pop("plasticity_queue_depth")
    if "lifespan_mgmt_interval" in genome:
        genome["physiology"]["lifespan_mgmt_interval"] = genome["lifespan_mgmt_interval"]
        genome.pop("lifespan_mgmt_interval")

    return genome


def genome_stat_updator(genome: dict):
    if "stats" not in genome:
        genome["stats"] = {}
    if "cortical_count" not in genome["stats"]:
        genome["stats"]["innate_cortical_area_count"] = 0
    if "neuron_count" not in genome["stats"]:
        genome["stats"]["innate_neuron_count"] = 0
    if "synapse_count" not in genome["stats"]:
        genome["stats"]["innate_synapse_count"] = 0
    return genome


gene_decoder = {
    "_______c-______-cx-__name-t": "cortical_name",
    "_______c-______-cx-_n_cnt-i": "cortical_neuron_count",
    "_______c-______-cx-gd_vis-b": "godot_visualization",
    "_______c-______-cx-rcordx-i": "relative_coordinate_x",
    "_______c-______-cx-rcordy-i": "relative_coordinate_y",
    "_______c-______-cx-rcordz-i": "relative_coordinate_z",
    "_______c-______-cx-2dcorx-i": "2d_coordinate_x",
    "_______c-______-cx-2dcory-i": "2d_coordinate_y",
    "_______c-______-cx-___bbx-i": "block_boundary_x",
    "_______c-______-cx-___bby-i": "block_boundary_y",
    "_______c-______-cx-___bbz-i": "block_boundary_z",
    "_______c-______-cx-synatt-f": "synapse_attractivity",
    "_______c-______-cx-__rand-b": "location_generation_type",
    "_______c-______-cx-dstmap-d": "cortical_mapping_dst",
    "_______c-______-cx-de_gen-f": "degeneration",
    "_______c-______-nx-pstcr_-f": "postsynaptic_current",
    "_______c-______-nx-pstcrm-f": "postsynaptic_current_max",
    "_______c-______-nx-fire_t-f": 'firing_threshold',
    "_______c-______-nx-ftincx-f": "firing_threshold_increment_x",
    "_______c-______-nx-ftincy-f": "firing_threshold_increment_y",
    "_______c-______-nx-ftincz-f": "firing_threshold_increment_z",
    "_______c-______-nx-fthlim-f": "firing_threshold_limit",
    "_______c-______-nx-mp_acc-b": "mp_charge_accumulation",
    "_______c-______-nx-mp_psp-b": "mp_driven_psp",
    "_______c-______-nx-refrac-i": "refractory_period",
    "_______c-______-nx-leak_c-f": "leak_coefficient",
    "_______c-______-nx-leak_v-f": "leak_variability",
    "_______c-______-nx-c_fr_c-i": "consecutive_fire_cnt_max",
    "_______c-______-nx-snooze-f": "snooze_length",
    "_______c-______-cx-memory-b": "is_mem_type",
    "_______c-______-cx-mem__t-i": "longterm_mem_threshold",
    "_______c-______-cx-mem_gr-i": "lifespan_growth_rate",
    "_______c-______-cx-mem_ls-i": "init_lifespan",
    "_______c-______-nx-excite-f": "neuron_excitability"
}

genome_1_template = {
    "sub_group_id": "",
    "per_voxel_neuron_cnt": 1,
    "synapse_attractivity": 100,
    "degeneration": 0,
    "psp_uniform_distribution": False,
    "postsynaptic_current_max": 99999,
    "cortical_mapping_dst": {},
    "block_boundaries": [
        None,
        None,
        None
    ],
    "relative_coordinate": [
        0,
        0,
        0
    ],
    "2d_coordinate": [
        0,
        0
    ],
    "visualization": True,
    "postsynaptic_current": 1,
    'firing_threshold': 1,
    "refractory_period": 0,
    "leak_coefficient": 0,
    "leak_variability": 0,
    "consecutive_fire_cnt_max": 0,
    "snooze_length": 0,
    "firing_threshold_increment_x": 0,
    "firing_threshold_increment_y": 0,
    "firing_threshold_increment_z": 0,
    "firing_threshold_limit": 0,
    "mp_charge_accumulation": True,
    "mp_driven_psp": False,
    "is_mem_type": False,
    "longterm_mem_threshold": 100,
    "lifespan_growth_rate": 1,
    "init_lifespan": 9,
    "neuron_excitability": 100
    }

genome_2_to_1 = {
    "_n_cnt-i": "per_voxel_neuron_cnt",
    "gd_vis-b": "visualization",
    "__name-t": "cortical_name",
    "rcordx-i": "relative_coordinate",
    "rcordy-i": "relative_coordinate",
    "rcordz-i": "relative_coordinate",
    "2dcorx-i": "2d_coordinate",
    "2dcory-i": "2d_coordinate",
    "___bbx-i": "block_boundaries",
    "___bby-i": "block_boundaries",
    "___bbz-i": "block_boundaries",
    "__rand-b": "location_generation_type",
    "synatt-f": "synapse_attractivity",
    "pstcr_-f": "postsynaptic_current",
    "pstcrm-f": "postsynaptic_current_max",
    "fire_t-f": 'firing_threshold',
    "ftincx-f": "firing_threshold_increment_x",
    "ftincy-f": "firing_threshold_increment_y",
    "ftincz-f": "firing_threshold_increment_z",
    "fthlim-f": "firing_threshold_limit",
    "refrac-i": "refractory_period",
    "leak_c-f": "leak_coefficient",
    "leak_v-f": "leak_variability",
    "c_fr_c-i": "consecutive_fire_cnt_max",
    "snooze-f": "snooze_length",
    "_group-t": "group_id",
    "subgrp-t": "sub_group_id",
    "dstmap-d": "cortical_mapping_dst",
    "de_gen-f": "degeneration",
    "pspuni-b": "psp_uniform_distribution",
    "mp_acc-b": "mp_charge_accumulation",
    "mp_psp-b": "mp_driven_psp",
    "memory-b": "is_mem_type",
    "mem__t-i": "longterm_mem_threshold",
    "mem_gr-i": "lifespan_growth_rate",
    "mem_ls-i": "init_lifespan",
    "excite-f": "neuron_excitability"
}

genome_1_to_2 = {
    "cortical_name": "cx-__name-t",
    "group_id": "cx-_group-t",
    "sub_group_id": "cx-subgrp-t",
    "per_voxel_neuron_cnt": "cx-_n_cnt-i",
    "visualization": "cx-gd_vis-b",
    "location_generation_type": "cx-__rand-b",
    "synapse_attractivity": "cx-synatt-f",
    "postsynaptic_current": "nx-pstcr_-f",
    "postsynaptic_current_max": "nx-pstcrm-f",
    'firing_threshold': "nx-fire_t-f",
    "firing_threshold_increment_x": "nx-ftincx-f",
    "firing_threshold_increment_y": "nx-ftincy-f",
    "firing_threshold_increment_z": "nx-ftincz-f",
    "neuron_excitability": "nx-excite-f",
    "firing_threshold_limit": "nx-fthlim-f",
    "refractory_period": "nx-refrac-i",
    "leak_coefficient": "nx-leak_c-f",
    "leak_variability": "nx-leak_v-f",
    "consecutive_fire_cnt_max": "nx-c_fr_c-i",
    "snooze_length": "nx-snooze-f",
    "degeneration": "cx-de_gen-f",
    "psp_uniform_distribution": "cx-pspuni-b",
    "cortical_mapping_dst": "cx-dstmap-d",
    "mp_charge_accumulation": "nx-mp_acc-b",
    "mp_driven_psp": "nx-mp_psp-b",
    "is_mem_type": "cx-memory-b",
    "longterm_mem_threshold": "cx-mem__t-i",
    "lifespan_growth_rate": "cx-mem_gr-i",
    "init_lifespan": "cx-mem_ls-i"
}
