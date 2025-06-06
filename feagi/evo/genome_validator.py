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

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import traceback

# Terminal color codes for pretty printing
class Bcolors:
    OKGREEN = '\033[92m'
    RED = '\033[91m'
    ENDC = '\033[0m'

from feagi.evo.genome_properties import *
from feagi.evo.templates import cortical_types


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
        logger.info("Genome has been validated.", status="[OK]")
    else:
        logger.warning("Genome validation failed.", status="[ERR]")


def genome_validator(genome):
    genome_validity = morphology_validator(genome=genome) and \
                      blueprint_validator(genome=genome)
    print_validity(validity_status=genome_validity)
    return genome_validity


def genome_validator_with_errors(genome):
    """
    Enhanced genome validator that returns detailed validation results.
    
    This function provides the same validation logic as genome_validator()
    but returns detailed error information instead of just a boolean result.
    
    Args:
        genome: The genome data to validate
        
    Returns:
        dict: {
            "valid": bool,
            "errors": List[str],  # Specific error messages
            "error_summary": str  # Summary of all errors
        }
    """
    errors = []
    
    # Validate morphologies
    if not morphology_validator(genome=genome):
        errors.append("Morphology validation failed - check neuron_morphologies section")
    
    # Validate blueprint
    if not blueprint_validator(genome=genome):
        errors.append("Blueprint validation failed - check blueprint section")
    
    is_valid = len(errors) == 0
    
    if not is_valid:
        error_summary = f"Multiple validation errors: {len(errors)} issues found"
    else:
        error_summary = "Valid"
    
    return {
        "valid": is_valid,
        "errors": errors,
        "error_summary": error_summary
    }


def sanitize_invalid_morphologies(genome):
    """
    Detect and remove invalid morphologies from the genome during auto-recovery.
    
    This function identifies morphologies that are missing required fields and removes them,
    also cleaning up any blueprint references to those invalid morphologies.
    
    Args:
        genome: The genome data to sanitize
        
    Returns:
        dict: {
            "genome": dict,  # The sanitized genome
            "removed_morphologies": List[str],  # Names of removed morphologies
            "fixed_references": List[str],  # Blueprint entries that were fixed
            "recovery_summary": str  # Summary of what was fixed
        }
    """
    removed_morphologies = []
    fixed_references = []
    
    # Create a copy of the genome to modify
    import copy
    sanitized_genome = copy.deepcopy(genome)
    
    # Check if neuron_morphologies section exists
    if "neuron_morphologies" not in sanitized_genome:
        return {
            "genome": sanitized_genome,
            "removed_morphologies": [],
            "fixed_references": [],
            "recovery_summary": "No morphologies section found - nothing to sanitize"
        }
    
    morphologies = sanitized_genome["neuron_morphologies"]
    
    # Identify invalid morphologies
    invalid_morphologies = []
    
    for morph_name, morph_data in morphologies.items():
        if not isinstance(morph_data, dict):
            invalid_morphologies.append(morph_name)
            continue
            
        # Check for required fields
        if "type" not in morph_data or not morph_data["type"]:
            invalid_morphologies.append(morph_name)
            continue
            
        # Check for other validation criteria
        morph_type = morph_data.get("type", "")
        if morph_type not in ["vectors", "patterns", "function"]:
            invalid_morphologies.append(morph_name)
            continue
    
    # Remove invalid morphologies
    for morph_name in invalid_morphologies:
        if morph_name in morphologies:
            del morphologies[morph_name]
            removed_morphologies.append(morph_name)
    
    # Clean up blueprint references to removed morphologies
    if "blueprint" in sanitized_genome:
        blueprint = sanitized_genome["blueprint"]
        keys_to_remove = []
        
        for gene_key, gene_value in blueprint.items():
            # Check if this is a mapping that references a removed morphology
            if isinstance(gene_value, dict) and "morphology" in gene_value:
                morphology_ref = gene_value["morphology"]
                if morphology_ref in removed_morphologies:
                    keys_to_remove.append(gene_key)
                    fixed_references.append(f"Removed blueprint entry {gene_key} (referenced invalid morphology '{morphology_ref}')")
            
            # Also check for direct morphology references in cortical mappings
            elif "cortical_mappings" in gene_key and isinstance(gene_value, list):
                for mapping in gene_value:
                    if isinstance(mapping, dict) and "morphology" in mapping:
                        if mapping["morphology"] in removed_morphologies:
                            # For now, we'll note this but not remove entire mappings
                            fixed_references.append(f"Found mapping in {gene_key} with invalid morphology '{mapping['morphology']}'")
        
        # Remove invalid blueprint entries
        for key in keys_to_remove:
            del blueprint[key]
    
    # Also clean up cortical_mappings if they exist
    if "cortical_mappings" in sanitized_genome and isinstance(sanitized_genome["cortical_mappings"], list):
        mappings = sanitized_genome["cortical_mappings"]
        valid_mappings = []
        
        for mapping in mappings:
            if isinstance(mapping, dict) and "morphology" in mapping:
                if mapping["morphology"] not in removed_morphologies:
                    valid_mappings.append(mapping)
                else:
                    fixed_references.append(f"Removed cortical mapping from {mapping.get('source', 'unknown')} to {mapping.get('destination', 'unknown')} (invalid morphology '{mapping['morphology']}')")
            else:
                valid_mappings.append(mapping)
        
        sanitized_genome["cortical_mappings"] = valid_mappings
    
    # Generate recovery summary
    if removed_morphologies:
        recovery_summary = f"Auto-recovery: Removed {len(removed_morphologies)} invalid morphologies: {', '.join(removed_morphologies)}"
        if fixed_references:
            recovery_summary += f"; Fixed {len(fixed_references)} blueprint references"
    else:
        recovery_summary = "Auto-recovery: No invalid morphologies found to remove"
    
    return {
        "genome": sanitized_genome,
        "removed_morphologies": removed_morphologies,
        "fixed_references": fixed_references,
        "recovery_summary": recovery_summary
    }
