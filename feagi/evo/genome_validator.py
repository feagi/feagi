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
Provides a series of methods to validate genome syntax

Supporting Genome Versions: 2.0
"""

import logging
import traceback

# Terminal color codes for pretty printing
class Bcolors:
    OKGREEN = '\033[92m'
    RED = '\033[91m'
    ENDC = '\033[0m'

from feagi.evo.genome_properties import *
from feagi.evo.templates import cortical_types


logger = logging.getLogger(__name__)


def cortical_list_gen(blueprint):
    cortical_list_ = set()
    for gene in blueprint:
        cortical_area = gene.split(genome_properties["structure"]["segment_seperator"])[1]
        if cortical_area not in cortical_list_:
            cortical_list_.add(cortical_area)
    return cortical_list_


def morphology_validator(genome):
    neuron_morphologies = genome["neuron_morphologies"]
    genome_validity = True
    for morphology in neuron_morphologies:
        try:
            if "type" not in neuron_morphologies[morphology]:
                genome_validity = False
                logger.warning(f'Morphology "{morphology}" does not have Type definition.')
                return genome_validity

            else:
                if neuron_morphologies[morphology]["type"] not in ["vectors", "patterns", "functions", "composite"]:
                    genome_validity = False
                    logger.warning(f'Morphology "{morphology}" has an unsupported Type.')
                    return genome_validity

                if neuron_morphologies[morphology]["type"] == "composite":
                    if "src_seed" not in neuron_morphologies[morphology]["parameters"] or \
                        "src_pattern" not in neuron_morphologies[morphology]["parameters"] or \
                            "mapper_morphology" not in neuron_morphologies[morphology]["parameters"]:
                        genome_validity = False
                        logger.warning(f'Morphology "{morphology}" has an incorrect set of parameters.')
                        return genome_validity

                if neuron_morphologies[morphology]["type"] == "vectors":
                    if "vectors" not in neuron_morphologies[morphology]["parameters"]:
                        genome_validity = False
                        logger.warning(f'Morphology "{morphology}" has an incorrect set of parameters.')
                        return genome_validity
                    else:
                        vector_definition = neuron_morphologies[morphology]["parameters"]["vectors"]
                        if type(vector_definition) is not list:
                            genome_validity = False
                            logger.warning(f'Morphology "{morphology}" has an incorrect vector parameter definition.')
                            return genome_validity
                        else:
                            for vector in vector_definition:
                                if len(vector) != 3:
                                    genome_validity = False
                                    logger.warning(f'Morphology "{morphology}" has an incorrect vector parameter definition.')
                                    return genome_validity

                if neuron_morphologies[morphology]["type"] == "patterns":
                    if "patterns" not in neuron_morphologies[morphology]["parameters"]:
                        genome_validity = False
                        logger.warning(f'Morphology "{morphology}" has an incorrect set of parameters.')
                        return genome_validity
                    else:
                        pattern_definition = neuron_morphologies[morphology]["parameters"]["patterns"]
                        if type(pattern_definition) is not list:
                            genome_validity = False
                            logger.warning(f'Morphology "{morphology}" has an incorrect pattern parameter definition.')
                            return genome_validity
                        else:
                            for pattern in pattern_definition:
                                if len(pattern) != 2:
                                    genome_validity = False
                                    logger.warning(f'Morphology "{morphology}" has an incorrect pattern parameter definition.')
                                    return genome_validity
                                else:
                                    for pattern_segment in pattern:
                                        if len(pattern_segment) != 3:
                                            genome_validity = False
                                            logger.warning(f'Morphology "{morphology}" has an incorrect pattern parameter definition.')
                                            return genome_validity

            if "parameters" not in neuron_morphologies[morphology]:
                genome_validity = False
                logger.warning(f'Morphology "{morphology}" does not have Parameters definition.')
                return genome_validity

        except Exception as e:
            genome_validity = False
            logger.error(f'Exception during "{morphology}" morphology validation: {e}')

    return genome_validity


def blueprint_validator(genome):
    valid_genome = False
    blueprint = genome["blueprint"]
    try:
        neuron_morphologies = genome["neuron_morphologies"]
        cortical_list = cortical_list_gen(blueprint)
        valid_genome = True
    except KeyError as e:
        logger.error(f'Error during blueprint validation: {e}')

    def gene_segments(gene_):
        guide = \
            genome_properties["structure"]["segment_guide"].split(genome_properties["structure"]["segment_seperator"])
        segments = gene_.split(genome_properties["structure"]["segment_seperator"])
        if len(segments) != genome_properties["structure"]["segment_count"]:
            logger.error(f'{gene_}')
            logger.error(f"\t\t\tError: Gene structure should only have {genome_properties['structure']['segment_count']} segments seperated by a {genome_properties['structure']['segment_seperator']}")
            return False

        for index in range(len(guide)):
            if len(guide[index]) != len(segments[index]):
                logger.error(f'{gene_}')
                logger.error(f"\t\t\tError: Incorrect segment size for {segments[index]}")
                return False

        return True

    def destination_rules(gene_):
        segments = gene_.split(genome_properties["structure"]["segment_seperator"])
        if segments[3] == "dstmap":
            for destination in blueprint[gene_]:
                # Check for a valid destination cortical area reference
                if destination not in cortical_list:
                    logger.error(f'{gene_}')
                    logger.error(f"\t\t\tError: Destination cortical area associated with neuron morphology not found: {destination}")
                    return False

                # Check for a valid destination rule usage
                for rule in blueprint[gene_][destination]:
                    if rule[0] not in neuron_morphologies:
                        logger.error(f'{gene_}')
                        logger.error(f"\t\t\tError: Referenced neuron morphology is not defined!\n\t\t\t Rule: {rule}")
                        return False
        return True

    def special_areas(gene_):
        segments = gene_.split(genome_properties["structure"]["segment_seperator"])
        special_core_types = {"IPU", "OPU", "CORE"}
        cortical_area = segments[1]
        if segments[3] == "_group":
            defined_cortical_type = blueprint[gene_]
            if defined_cortical_type in special_core_types:
                if cortical_area not in cortical_types[defined_cortical_type]["supported_devices"]:
                    return False

        return True

    for gene in blueprint:
        if not gene_segments(gene):
            valid_genome = False
            logger.error(f'{gene}')
        if not destination_rules(gene):
            logger.error(f'{gene}')
            valid_genome = False
        if not special_areas(gene):
            logger.error(f'{gene}')
            valid_genome = False

    return valid_genome


def print_validity(validity_status):
    if validity_status:
        logger.info("* -- * " * 20)
        logger.info("* -- * " * 20)
        logger.info("\t\t\t\t\t\t\tGenome validation completed successfully!!")
        logger.info("* -- * " * 20)
        logger.info("* -- * " * 20)
    else:
        logger.error("! ! " * 30)
        logger.error("! ! " * 30)
        logger.error("\t\t\t\t\t\t\tErrors detected during genome validation!!")
        logger.error("! ! " * 30)
        logger.error("! ! " * 30)


def genome_validator(genome):
    genome_validity = morphology_validator(genome=genome) and \
                      blueprint_validator(genome=genome)
    print_validity(validity_status=genome_validity)
    return genome_validity
