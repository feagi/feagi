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


# Terminal color codes for pretty printing
class Bcolors:
    OKGREEN = "\033[92m"
    RED = "\033[91m"
    ENDC = "\033[0m"


from feagi.evo.genome_properties import *
from feagi.evo.templates import cortical_types


def cortical_list_gen(blueprint):
    cortical_list_ = set()
    for gene in blueprint:
        cortical_area = gene.split(genome_properties["structure"]["segment_seperator"])[
            1
        ]
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
                logger.warning(
                    f'Morphology "{morphology}" does not have Type definition.'
                )
                return genome_validity

            else:
                if neuron_morphologies[morphology]["type"] not in [
                    "vectors",
                    "patterns",
                    "functions",
                    "composite",
                ]:
                    genome_validity = False
                    logger.warning(
                        f'Morphology "{morphology}" has an unsupported Type.'
                    )
                    return genome_validity

                # NEW: Check for dimension_sensitive field
                if "dimension_sensitive" not in neuron_morphologies[morphology]:
                    logger.info(
                        f'Morphology "{morphology}" missing dimension_sensitive field - will auto-add during recovery'
                    )

                if neuron_morphologies[morphology]["type"] == "composite":
                    if (
                        "src_seed" not in neuron_morphologies[morphology]["parameters"]
                        or "src_pattern"
                        not in neuron_morphologies[morphology]["parameters"]
                        or "mapper_morphology"
                        not in neuron_morphologies[morphology]["parameters"]
                    ):
                        genome_validity = False
                        logger.warning(
                            f'Morphology "{morphology}" has an incorrect set of parameters.'
                        )
                        return genome_validity

                if neuron_morphologies[morphology]["type"] == "vectors":
                    if "vectors" not in neuron_morphologies[morphology]["parameters"]:
                        genome_validity = False
                        logger.warning(
                            f'Morphology "{morphology}" has an incorrect set of parameters.'
                        )
                        return genome_validity
                    else:
                        vector_definition = neuron_morphologies[morphology][
                            "parameters"
                        ]["vectors"]
                        if type(vector_definition) is not list:
                            genome_validity = False
                            logger.warning(
                                f'Morphology "{morphology}" has an incorrect vector parameter definition.'
                            )
                            return genome_validity
                        else:
                            for vector in vector_definition:
                                if len(vector) != 3:
                                    genome_validity = False
                                    logger.warning(
                                        f'Morphology "{morphology}" has an incorrect vector parameter definition.'
                                    )
                                    return genome_validity

                if neuron_morphologies[morphology]["type"] == "patterns":
                    if "patterns" not in neuron_morphologies[morphology]["parameters"]:
                        genome_validity = False
                        logger.warning(
                            f'Morphology "{morphology}" has an incorrect set of parameters.'
                        )
                        return genome_validity
                    else:
                        pattern_definition = neuron_morphologies[morphology][
                            "parameters"
                        ]["patterns"]
                        if type(pattern_definition) is not list:
                            genome_validity = False
                            logger.warning(
                                f'Morphology "{morphology}" has an incorrect pattern parameter definition.'
                            )
                            return genome_validity
                        else:
                            for pattern in pattern_definition:
                                if len(pattern) != 2:
                                    genome_validity = False
                                    logger.warning(
                                        f'Morphology "{morphology}" has an incorrect pattern parameter definition.'
                                    )
                                    return genome_validity
                                else:
                                    for pattern_segment in pattern:
                                        if len(pattern_segment) != 3:
                                            genome_validity = False
                                            logger.warning(
                                                f'Morphology "{morphology}" has an incorrect pattern parameter definition.'
                                            )
                                            return genome_validity

            if "parameters" not in neuron_morphologies[morphology]:
                genome_validity = False
                logger.warning(
                    f'Morphology "{morphology}" does not have Parameters definition.'
                )
                return genome_validity

        except Exception as e:
            genome_validity = False
            logger.error(f'Exception during "{morphology}" morphology validation: {e}')

    return genome_validity


def _gene_segments_validator(gene_, verbose=True):
    """Validate gene segment structure. Returns True if valid, False otherwise."""
    guide = genome_properties["structure"]["segment_guide"].split(
        genome_properties["structure"]["segment_seperator"]
    )
    segments = gene_.split(genome_properties["structure"]["segment_seperator"])

    if len(segments) != genome_properties["structure"]["segment_count"]:
        if verbose:
            logger.error(f"INVALID GENE STRUCTURE: {gene_}")
            logger.error(
                f"  → PROBLEM: Gene has {len(segments)} segments but requires exactly {genome_properties['structure']['segment_count']} segments"
            )
            logger.error(
                f"  → EXPECTED FORMAT: {genome_properties['structure']['segment_seperator'].join(guide)}"
            )
            logger.error(f"  → CURRENT SEGMENTS: {segments}")
            logger.error(
                f"  → FIX: Ensure gene follows proper segment structure with {genome_properties['structure']['segment_seperator']} separators"
            )
        return False

    for index in range(len(guide)):
        if len(guide[index]) != len(segments[index]):
            if verbose:
                logger.error(f"INVALID SEGMENT LENGTH: {gene_}")
                logger.error(
                    f'  → PROBLEM: Segment "{segments[index]}" has length {len(segments[index])} but requires length {len(guide[index])}'
                )
                logger.error(
                    f'  → EXPECTED: Segment {index + 1} should be "{guide[index]}" (length {len(guide[index])})'
                )
                logger.error(
                    f'  → CURRENT: Segment {index + 1} is "{segments[index]}" (length {len(segments[index])})'
                )
                logger.error("  → FIX: Adjust segment length to match expected format")
            return False

    return True


def _destination_rules_validator(
    gene_, blueprint, cortical_list, neuron_morphologies, verbose=True
):
    """Validate destination rules for a gene. Returns True if valid, False otherwise."""
    segments = gene_.split(genome_properties["structure"]["segment_seperator"])
    if segments[3] == "dstmap":
        for destination in blueprint[gene_]:
            # Check for a valid destination cortical area reference
            if destination not in cortical_list:
                if verbose:
                    logger.error(f"INVALID DESTINATION CORTICAL AREA: {gene_}")
                    logger.error(
                        f'  → PROBLEM: Destination "{destination}" does not exist in the genome'
                    )
                    logger.error(f"  → AVAILABLE AREAS: {sorted(list(cortical_list))}")
                    logger.error(
                        f'  → FIX: Use an existing cortical area ID or define the cortical area "{destination}" first'
                    )
                return False

            # Check for a valid destination rule usage
            for rule in blueprint[gene_][destination]:
                if rule[0] not in neuron_morphologies:
                    if verbose:
                        logger.error(f"INVALID MORPHOLOGY REFERENCE: {gene_}")
                        logger.error(
                            f'  → PROBLEM: Morphology "{rule[0]}" is not defined in neuron_morphologies section'
                        )
                        logger.error(f"  → RULE: {rule}")
                        logger.error(
                            f"  → AVAILABLE MORPHOLOGIES: {sorted(list(neuron_morphologies.keys()))}"
                        )
                        logger.error(
                            f'  → FIX: Define morphology "{rule[0]}" in neuron_morphologies or use an existing morphology'
                        )
                    return False
    return True


def _special_areas_validator(gene_, blueprint, verbose=True):
    """Validate special areas for a gene. Returns True if valid, False otherwise."""
    segments = gene_.split(genome_properties["structure"]["segment_seperator"])
    special_core_types = {"IPU", "OPU", "CORE"}
    cortical_area = segments[1]
    if segments[3] == "_group":
        defined_cortical_type = blueprint[gene_]

        # CUSTOM areas are allowed with any cortical area ID - skip validation
        if defined_cortical_type == "CUSTOM":
            if verbose:
                logger.info(
                    f'CUSTOM AREA ALLOWED: {gene_} → Custom cortical area "{cortical_area}" with type "{defined_cortical_type}" is valid'
                )
            return True

        # For IPU, OPU, CORE types, check if cortical area ID is supported
        if defined_cortical_type in special_core_types:
            if (
                cortical_area
                not in cortical_types[defined_cortical_type]["supported_devices"]
            ):
                if verbose:
                    logger.error(f"UNSUPPORTED SPECIAL AREA: {gene_}")
                    logger.error(
                        f'  → PROBLEM: Cortical area "{cortical_area}" is not supported for type "{defined_cortical_type}"'
                    )
                    logger.error(
                        f"  → SUPPORTED AREAS FOR {defined_cortical_type}: {list(cortical_types[defined_cortical_type]['supported_devices'].keys())}"
                    )
                    logger.error(
                        "  → FIX: Use a supported cortical area ID or change the cortical type to CUSTOM"
                    )
                return False

    return True


def validate_cortical_parameters(blueprint):
    """
    Validate cortical area parameters for correct types and ranges.
    
    This function validates parameters like excitability, thresholds, etc.
    to ensure they are within acceptable ranges and types.
    
    Args:
        blueprint: The blueprint section of the hierarchical genome
        
    Returns:
        dict: {
            "valid": bool,
            "errors": List[str],
            "warnings": List[str]
        }
    """
    errors = []
    warnings = []
    
    # Cortical parameter validation rules
    parameter_rules = {
        "neuron_excitability": {
            "type": (int, float),
            "min": 0.0,
            "max": 1.0,
            "auto_fix_max": True,  # Auto-clamp values > 1.0 to 1.0
            "description": "Neuron firing probability (0.0 = never fire, 1.0 = always fire when threshold met)"
        },
        "firing_threshold": {
            "type": (int, float),
            "min": 0.0,
            "description": "Neuron firing threshold"
        },
        "refractory_period": {
            "type": int,
            "min": 0,
            "description": "Neuron refractory period in timesteps"
        },
        "leak_coefficient": {
            "type": (int, float),
            "min": 0.0,
            "max": 1.0,
            "description": "Membrane potential decay rate"
        }
    }
    
    for cortical_id, area_data in blueprint.items():
        if not isinstance(area_data, dict):
            continue
            
        # In hierarchical format, properties are stored directly under the cortical area
        # NOT under a "parameters" sub-dict
        for param_name, rules in parameter_rules.items():
            if param_name in area_data:
                value = area_data[param_name]
                param_desc = rules["description"]
                
                # Type validation
                if not isinstance(value, rules["type"]):
                    errors.append(
                        f"INVALID PARAMETER TYPE: Cortical area '{cortical_id}' parameter "
                        f"'{param_name}' should be {rules['type']}, got {type(value).__name__}. "
                        f"Description: {param_desc}"
                    )
                    continue
                
                # Range validation
                if "min" in rules and value < rules["min"]:
                    errors.append(
                        f"INVALID PARAMETER RANGE: Cortical area '{cortical_id}' parameter "
                        f"'{param_name}' value {value} is below minimum {rules['min']}. "
                        f"Description: {param_desc}"
                    )
                
                if "max" in rules and value > rules["max"]:
                    if rules.get("auto_fix_max", False):
                        # Auto-fix: clamp to maximum value
                        area_data[param_name] = rules["max"]
                        warnings.append(
                            f"AUTO-CORRECTED: Cortical area '{cortical_id}' parameter "
                            f"'{param_name}' value {value} exceeded maximum {rules['max']}, "
                            f"clamped to {rules['max']}. Description: {param_desc}"
                        )
                    else:
                        errors.append(
                            f"INVALID PARAMETER RANGE: Cortical area '{cortical_id}' parameter "
                            f"'{param_name}' value {value} exceeds maximum {rules['max']}. "
                            f"Description: {param_desc}"
                        )
                
                # Special validation for excitability
                if param_name == "neuron_excitability":
                    if value < 0.0:
                        errors.append(
                            f"INVALID EXCITABILITY: Cortical area '{cortical_id}' excitability "
                            f"{value} cannot be negative. Use 0.0 for neurons that never fire, "
                            f"1.0 for normal firing probability."
                        )
                    elif 0.0 < value < 0.01:
                        warnings.append(
                            f"LOW EXCITABILITY WARNING: Cortical area '{cortical_id}' excitability "
                            f"{value} is very low (< 1%). Neurons will rarely fire."
                        )
    
    is_valid = len(errors) == 0
    
    return {
        "valid": is_valid,
        "errors": errors,
        "warnings": warnings
    }


def blueprint_validator(genome):
    """
    Responsible for validating integrity of genome by checking correctness of
    segments and references.

    This function ensures:
    - Gene structure follows expected format
    - Destination cortical areas exist
    - Morphology references are valid
    - Special area types are supported
    - Cortical parameters are within valid ranges
    """
    try:
        blueprint = genome["blueprint"]
        neuron_morphologies = genome["neuron_morphologies"]
        cortical_list = cortical_list_gen(blueprint)
        valid_genome = True
    except KeyError as e:
        logger.error(f"Error during blueprint validation: {e}")
        return False

    def gene_segments(gene_):
        return _gene_segments_validator(gene_)

    def destination_rules(gene_):
        return _destination_rules_validator(
            gene_, blueprint, cortical_list, neuron_morphologies
        )

    def special_areas(gene_):
        return _special_areas_validator(gene_, blueprint)

    # NEW: Validate cortical parameters including excitability
    def cortical_parameters():
        param_validation = validate_cortical_parameters(blueprint)
        if not param_validation["valid"]:
            for error in param_validation["errors"]:
                logger.error(error)
            valid_genome = False
        
        # Log warnings but don't fail validation
        for warning in param_validation["warnings"]:
            logger.warning(warning)
        
        return param_validation["valid"]

    # Skip invalid cortical area IDs with warnings instead of failing validation
    def check_cortical_area_validity():
        """Check for invalid cortical area IDs and warn about them, but don't fail validation."""
        invalid_areas = []

        for gene in blueprint:
            segments = gene.split(genome_properties["structure"]["segment_seperator"])
            if len(segments) >= 2:
                cortical_area = segments[1]

                # Check if this cortical area has any valid definition
                has_group_definition = False
                for check_gene in blueprint:
                    check_segments = check_gene.split(
                        genome_properties["structure"]["segment_seperator"]
                    )
                    if (
                        len(check_segments) >= 4
                        and check_segments[1] == cortical_area
                        and check_segments[3] == "_group"
                    ):
                        has_group_definition = True
                        break

                # If cortical area has no group definition, it's likely a typo
                if not has_group_definition and cortical_area not in invalid_areas:
                    # Check if it looks like a known typo (e.g., ii_inf should be i_iinf)
                    potential_fix = None
                    all_supported_ids = set()

                    # Collect all supported cortical area IDs
                    for area_type in ["IPU", "OPU", "CORE"]:
                        if (
                            area_type in cortical_types
                            and "supported_devices" in cortical_types[area_type]
                        ):
                            all_supported_ids.update(
                                cortical_types[area_type]["supported_devices"].keys()
                            )

                    # Check for common typos
                    if cortical_area == "ii_inf" and "i_iinf" in all_supported_ids:
                        potential_fix = "i_iinf"

                    if potential_fix:
                        logger.warning(
                            f"INVALID CORTICAL AREA ID (LIKELY TYPO): {cortical_area}"
                        )
                        logger.warning(
                            f'  → PROBLEM: Cortical area "{cortical_area}" is not defined and appears to be a typo'
                        )
                        logger.warning(
                            f'  → SUGGESTED FIX: Change "{cortical_area}" to "{potential_fix}"'
                        )
                        logger.warning(
                            "  → ACTION: Skipping this cortical area and continuing validation"
                        )
                    else:
                        logger.warning(f"INVALID CORTICAL AREA ID: {cortical_area}")
                        logger.warning(
                            f'  → PROBLEM: Cortical area "{cortical_area}" has no group definition'
                        )
                        logger.warning(
                            "  → ACTION: Skipping this cortical area and continuing validation"
                        )

                    invalid_areas.append(cortical_area)

        return invalid_areas

    # Check for invalid cortical areas but don't fail validation
    invalid_areas = check_cortical_area_validity()
    if invalid_areas:
        logger.info(
            f"Skipped {len(invalid_areas)} invalid cortical area(s): {', '.join(invalid_areas)}"
        )

    for gene in blueprint:
        segments = gene.split(genome_properties["structure"]["segment_seperator"])

        # Skip genes for invalid cortical areas
        if len(segments) >= 2 and segments[1] in invalid_areas:
            continue

        if not gene_segments(gene):
            valid_genome = False
        if not destination_rules(gene):
            valid_genome = False
        if not special_areas(gene):
            valid_genome = False

    # Validate cortical area parameters (including excitability)
    if not cortical_parameters():
        valid_genome = False

    return valid_genome


def print_validity(validity_status):
    if validity_status:
        logger.info("Genome has been validated.", status="[OK]")
    else:
        logger.warning("Genome validation failed.", status="[ERR]")


def add_missing_dimension_sensitive_fields(genome):
    """
    Add missing dimension_sensitive fields to morphologies with type-based defaults.
    
    This function ensures backward compatibility by automatically adding the dimension_sensitive
    field to existing morphologies based on their type:
    - patterns/vectors: False (dimension-agnostic)
    - functions: True (dimension-sensitive)
    - composite/other: False (conservative default)
    
    Args:
        genome: The genome data to modify
        
    Returns:
        int: Number of morphologies that had the field added
    """
    added_count = 0
    
    if "neuron_morphologies" not in genome:
        return added_count
        
    morphologies = genome["neuron_morphologies"]
    
    for morph_name, morph_data in morphologies.items():
        if not isinstance(morph_data, dict):
            continue
            
        if "dimension_sensitive" not in morph_data:
            morph_type = morph_data.get("type", "")
            
            # Set defaults based on morphology type
            if morph_type in ["patterns", "vectors"]:
                # Patterns and vectors are typically dimension-agnostic
                morph_data["dimension_sensitive"] = False
                logger.info(f"AUTO-MIGRATION: Added dimension_sensitive=False to {morph_type} morphology '{morph_name}'")
            elif morph_type == "functions":
                # Functions are typically dimension-sensitive (e.g., projectors)
                morph_data["dimension_sensitive"] = True
                logger.info(f"AUTO-MIGRATION: Added dimension_sensitive=True to {morph_type} morphology '{morph_name}'")
            else:
                # Conservative default for composite or unknown types
                morph_data["dimension_sensitive"] = False
                logger.info(f"AUTO-MIGRATION: Added dimension_sensitive=False to {morph_type} morphology '{morph_name}' (conservative default)")
            
            added_count += 1
    
    return added_count


def genome_validator(genome):
    genome_validity = (
        morphology_validator(genome=genome)
        and blueprint_validator(genome=genome)
        and validate_physiology_section(genome)["valid"]
    )
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
        errors.append(
            "Morphology validation failed - check neuron_morphologies section"
        )

    # Validate blueprint
    if not blueprint_validator(genome=genome):
        errors.append("Blueprint validation failed - check blueprint section")

    # Validate physiology
    physiology_result = validate_physiology_section(genome)
    if not physiology_result["valid"]:
        errors.extend(physiology_result["errors"])

    is_valid = len(errors) == 0

    if not is_valid:
        error_summary = f"Multiple validation errors: {len(errors)} issues found"
    else:
        error_summary = "Valid"

    return {"valid": is_valid, "errors": errors, "error_summary": error_summary}


def auto_correct_ipu_assignments(genome):
    """
    Auto-correct IPU assignments that are not supported.

    This function detects areas assigned to IPU type that should be CUSTOM type,
    and automatically corrects them based on the supported IPU list in templates.

    Args:
        genome: The genome data to correct (modified in-place)

    Returns:
        dict: {
            "genome": dict,  # The corrected genome (same as input, modified in-place)
            "corrected_areas": List[str],  # Names of corrected cortical areas
            "correction_summary": str,  # Summary of what was corrected
            "warnings": List[str]  # Validation warnings for the response
        }
    """
    corrected_areas = []
    warnings = []

    # Check if blueprint section exists
    if "blueprint" not in genome:
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "No blueprint section found - nothing to correct",
            "warnings": [],
        }

    blueprint = genome["blueprint"]

    # Get supported IPU list from templates
    try:
        from feagi.evo.templates import supported_ipu_list

        supported_ipus = set(supported_ipu_list)
    except ImportError:
        logger.warning(
            "Could not import supported_ipu_list from templates - skipping IPU validation"
        )
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "IPU validation skipped - template not available",
            "warnings": [],
        }

    # Find cortical areas with IPU assignments that should be CUSTOM
    for gene_key, gene_value in blueprint.items():
        if not isinstance(gene_key, str):
            continue

        parts = gene_key.split("-")
        if len(parts) < 4:
            continue

        # Check if this is a _group gene
        if parts[3] == "_group":
            cortical_area_id = parts[1]
            current_group = gene_value

            # Check if this cortical area is assigned to IPU but not supported
            if current_group == "IPU" and cortical_area_id not in supported_ipus:
                # Auto-correct the group assignment
                blueprint[gene_key] = "CUSTOM"
                corrected_areas.append(f"{cortical_area_id}: IPU → CUSTOM")
                warning_msg = f"AUTO-CORRECTION: '{cortical_area_id}' assigned to IPU but not supported - converted to CUSTOM type"
                warnings.append(warning_msg)
                logger.warning(warning_msg)

    # Generate correction summary
    if corrected_areas:
        correction_summary = f"IPU assignment auto-correction: Fixed {len(corrected_areas)} areas: {', '.join(corrected_areas)}"
    else:
        correction_summary = "No IPU assignment corrections needed"

    return {
        "genome": genome,
        "corrected_areas": corrected_areas,
        "correction_summary": correction_summary,
        "warnings": warnings,
    }


def auto_correct_invalid_ipu_opu_areas(genome):
    """
    Auto-correct cortical areas that are assigned to IPU/OPU but have invalid cortical IDs.

    This function detects cortical areas assigned to IPU or OPU types but with
    cortical IDs that aren't supported for those types, and automatically converts
    them to CUSTOM type instead of failing validation.

    Args:
        genome: The genome data to correct (modified in-place)

    Returns:
        dict: {
            "genome": dict,  # The corrected genome (same as input, modified in-place)
            "corrected_areas": List[str],  # Names of corrected cortical areas
            "correction_summary": str,  # Summary of what was corrected
            "warnings": List[str]  # Validation warnings for the response
        }
    """
    corrected_areas = []
    warnings = []

    # Check if blueprint section exists
    if "blueprint" not in genome:
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "No blueprint section found - nothing to correct",
            "warnings": [],
        }

    blueprint = genome["blueprint"]

    # Import cortical types for validation
    try:
        from feagi.evo.templates import cortical_types
    except ImportError:
        error_msg = "CRITICAL: Cannot import cortical_types from templates.py - invalid IPU/OPU area correction will fail"
        logger.error(error_msg)
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "Invalid IPU/OPU area correction failed: cortical_types not available",
            "warnings": [error_msg],
        }

    # Collect all supported cortical area IDs for IPU and OPU
    supported_ipu_ids = set()
    supported_opu_ids = set()

    if "IPU" in cortical_types and "supported_devices" in cortical_types["IPU"]:
        supported_ipu_ids = set(cortical_types["IPU"]["supported_devices"].keys())

    if "OPU" in cortical_types and "supported_devices" in cortical_types["OPU"]:
        supported_opu_ids = set(cortical_types["OPU"]["supported_devices"].keys())

    # Find cortical areas that need correction
    for gene_key, gene_value in blueprint.items():
        if not isinstance(gene_key, str):
            continue

        parts = gene_key.split("-")
        if len(parts) < 4:
            continue

        # Check if this is a _group gene
        if parts[3] == "_group":
            cortical_area_id = parts[1]
            current_group = gene_value

            # Check if this area is assigned to IPU but not supported
            if current_group == "IPU" and cortical_area_id not in supported_ipu_ids:
                # Auto-correct to CUSTOM
                blueprint[gene_key] = "CUSTOM"
                corrected_areas.append(f"{cortical_area_id}: IPU → CUSTOM")
                warning_msg = f"AUTO-CORRECTION: '{cortical_area_id}' assigned to IPU but not supported - converted to CUSTOM type"
                warnings.append(warning_msg)
                logger.warning(warning_msg)

            # Check if this area is assigned to OPU but not supported
            elif current_group == "OPU" and cortical_area_id not in supported_opu_ids:
                # Auto-correct to CUSTOM
                blueprint[gene_key] = "CUSTOM"
                corrected_areas.append(f"{cortical_area_id}: OPU → CUSTOM")
                warning_msg = f"AUTO-CORRECTION: '{cortical_area_id}' assigned to OPU but not supported - converted to CUSTOM type"
                warnings.append(warning_msg)
                logger.warning(warning_msg)

    # Generate correction summary
    if corrected_areas:
        correction_summary = f"Invalid IPU/OPU area auto-correction: Converted {len(corrected_areas)} areas to CUSTOM type: {', '.join(corrected_areas)}"
    else:
        correction_summary = "No invalid IPU/OPU areas found to correct"

    return {
        "genome": genome,
        "corrected_areas": corrected_areas,
        "correction_summary": correction_summary,
        "warnings": warnings,
    }


def remove_invalid_cortical_areas(genome):
    """
    Remove cortical areas with completely invalid IDs from the genome.

    This function only removes cortical areas that have no group definition at all
    (likely typos with no valid structure). Areas with invalid IPU/OPU assignments
    are handled by auto_correct_invalid_ipu_opu_areas() instead.

    Args:
        genome: The genome data to clean (modified in-place)

    Returns:
        dict: {
            "genome": dict,  # The cleaned genome (same as input, modified in-place)
            "removed_areas": List[str],  # Names of removed cortical areas
            "removal_summary": str  # Summary of what was removed
        }
    """
    removed_areas = []

    # Check if blueprint section exists
    if "blueprint" not in genome:
        return {
            "genome": genome,
            "removed_areas": [],
            "removal_summary": "No blueprint section found - nothing to remove",
        }

    blueprint = genome["blueprint"]

    # Find cortical areas that have NO group definition at all (likely typos)
    invalid_areas = []
    all_areas = set()
    area_groups = {}

    # First, collect all cortical areas and their group definitions
    for gene_key, gene_value in blueprint.items():
        parts = gene_key.split("-")
        if len(parts) >= 2:
            cortical_area_id = parts[1]
            all_areas.add(cortical_area_id)

            # Check for group definitions
            if len(parts) >= 4 and parts[3] == "_group":
                area_groups[cortical_area_id] = gene_value

    # Only remove areas that have NO group definition at all
    for area_id in all_areas:
        if area_id not in area_groups:
            # No group definition - likely a structural typo
            invalid_areas.append(area_id)
            logger.warning(
                f"REMOVING INVALID AREA: '{area_id}' has no group definition (likely structural typo)"
            )

    # Remove all genes for completely invalid cortical areas
    genes_to_remove = []
    for gene_key in blueprint.keys():
        parts = gene_key.split("-")
        if len(parts) >= 2:
            cortical_area_id = parts[1]
            if cortical_area_id in invalid_areas:
                genes_to_remove.append(gene_key)

    # Remove the genes
    for gene_key in genes_to_remove:
        del blueprint[gene_key]

    removed_areas = invalid_areas

    # Generate removal summary
    if removed_areas:
        removal_summary = f"Invalid cortical area removal: Removed {len(removed_areas)} areas with no group definition: {', '.join(removed_areas)}"
    else:
        removal_summary = "No invalid cortical areas found to remove"

    return {
        "genome": genome,
        "removed_areas": removed_areas,
        "removal_summary": removal_summary,
    }


def sanitize_invalid_morphologies(genome):
    """
    Detect and remove invalid morphologies from the genome during auto-recovery.
    Also adds missing required neuron properties with default values.

    This function identifies morphologies that are missing required fields and removes them,
    also cleaning up any blueprint references to those invalid morphologies.
    Additionally, it ensures all cortical areas have the required neuron properties.

    Args:
        genome: The genome data to sanitize (modified in-place)

    Returns:
        dict: {
            "genome": dict,  # The sanitized genome (same as input, modified in-place)
            "removed_morphologies": List[str],  # Names of removed morphologies
            "fixed_references": List[str],  # Blueprint entries that were fixed
            "recovery_summary": str,  # Summary of what was fixed
            "validation_warnings": List[str]  # All validation warnings for API response
        }
    """
    removed_morphologies = []
    fixed_references = []
    validation_warnings = []

    # Work directly on the original genome (no copy needed)
    # This ensures that all changes persist back to the caller

    # ALWAYS auto-recover missing physiology section first (regardless of other issues)
    physiology_recovery_result = sanitize_missing_physiology(genome)
    if (
        physiology_recovery_result["recovery_summary"]
        != "Physiology section is complete"
    ):
        fixed_references.append(physiology_recovery_result["recovery_summary"])

    # Auto-correct cortical area type assignments for CORE areas
    cortical_type_correction_result = auto_correct_cortical_area_types(genome)
    if (
        cortical_type_correction_result["correction_summary"]
        != "No cortical area type corrections needed"
    ):
        fixed_references.append(cortical_type_correction_result["correction_summary"])
    # Collect warnings from cortical type corrections
    validation_warnings.extend(cortical_type_correction_result.get("warnings", []))

    # Auto-correct IPU assignments to CUSTOM
    ipu_correction_result = auto_correct_ipu_assignments(genome)
    if (
        ipu_correction_result["correction_summary"]
        != "No IPU assignment corrections needed"
    ):
        fixed_references.append(ipu_correction_result["correction_summary"])
    # Collect warnings from IPU assignment corrections
    validation_warnings.extend(ipu_correction_result.get("warnings", []))

    # Auto-correct invalid IPU/OPU areas to CUSTOM type (instead of removing them)
    invalid_ipu_opu_correction_result = auto_correct_invalid_ipu_opu_areas(genome)
    if (
        invalid_ipu_opu_correction_result["correction_summary"]
        != "No invalid IPU/OPU areas found to correct"
    ):
        fixed_references.append(invalid_ipu_opu_correction_result["correction_summary"])
    # Collect warnings from invalid IPU/OPU area corrections
    validation_warnings.extend(invalid_ipu_opu_correction_result.get("warnings", []))

    # Remove only completely invalid cortical areas (those with no group definition)
    invalid_area_removal_result = remove_invalid_cortical_areas(genome)
    if (
        invalid_area_removal_result["removal_summary"]
        != "No invalid cortical areas found to remove"
    ):
        fixed_references.append(invalid_area_removal_result["removal_summary"])

    # NEW: Auto-add missing dimension_sensitive fields to morphologies
    added_dimension_fields = add_missing_dimension_sensitive_fields(genome)
    if added_dimension_fields > 0:
        fixed_references.append(f"Added dimension_sensitive field to {added_dimension_fields} morphologies with type-based defaults")

    # Check if neuron_morphologies section exists
    if "neuron_morphologies" not in genome:
        return {
            "genome": genome,
            "removed_morphologies": [],
            "fixed_references": fixed_references,  # Include physiology recovery
            "recovery_summary": (
                f"Auto-recovery: {'; '.join(fixed_references)}"
                if fixed_references
                else "Auto-recovery: No issues found to fix"
            ),
            "validation_warnings": validation_warnings,
        }

    morphologies = genome["neuron_morphologies"]

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
    if "blueprint" in genome:
        blueprint = genome["blueprint"]
        keys_to_remove = []

        for gene_key, gene_value in blueprint.items():
            # Check if this is a mapping that references a removed morphology
            if isinstance(gene_value, dict) and "morphology" in gene_value:
                morphology_ref = gene_value["morphology"]
                if morphology_ref in removed_morphologies:
                    keys_to_remove.append(gene_key)
                    fixed_references.append(
                        f"Removed blueprint entry {gene_key} (referenced invalid morphology '{morphology_ref}')"
                    )

            # Also check for direct morphology references in cortical mappings
            elif "cortical_mappings" in gene_key and isinstance(gene_value, list):
                for mapping in gene_value:
                    if isinstance(mapping, dict) and "morphology" in mapping:
                        if mapping["morphology"] in removed_morphologies:
                            # For now, we'll note this but not remove entire mappings
                            fixed_references.append(
                                f"Found mapping in {gene_key} with invalid morphology '{mapping['morphology']}'"
                            )

        # Remove invalid blueprint entries
        for key in keys_to_remove:
            del blueprint[key]

        # Add missing required neuron properties with default values
        # Import the cortical template for default values - THE SINGLE SOURCE OF TRUTH
        try:
            from feagi.evo.templates import (
                cortical_property_mappings,
                cortical_structural_properties,
                cortical_template,
            )
        except ImportError:
            logger.error(
                "CRITICAL: Cannot import cortical_template and property mappings from templates.py - auto-recovery will fail"
            )
            return {
                "genome": genome,
                "removed_morphologies": removed_morphologies,
                "fixed_references": fixed_references,
                "recovery_summary": (
                    f"Auto-recovery: {'; '.join(fixed_references)}"
                    if fixed_references
                    else "Auto-recovery failed: cortical_template and mappings not available"
                ),
                "validation_warnings": validation_warnings,
            }

        # Extract cortical areas from blueprint and add missing properties
        cortical_areas = set()
        for gene_key in blueprint.keys():
            # Extract cortical area ID from gene key (format: _____10c-AREA_ID-...)
            parts = gene_key.split("-")
            if len(parts) >= 2:
                cortical_area_id = parts[1]
                cortical_areas.add(cortical_area_id)

        missing_properties_count = 0

        # Add missing neuron properties using values from cortical_template
        for cortical_area in cortical_areas:
            for prop_name, prop_pattern in cortical_property_mappings.items():
                # Construct the expected gene key for this property
                gene_key = f"_____10c-{cortical_area}-{prop_pattern}"

                # Check if this property is missing from the blueprint
                if gene_key not in blueprint:
                    # Get the default value from cortical_template (THE SINGLE SOURCE OF TRUTH)
                    default_value = cortical_template.get(
                        prop_name, 0
                    )  # 0 as final fallback
                    blueprint[gene_key] = default_value
                    missing_properties_count += 1
                    fixed_references.append(
                        f"Added missing property {gene_key} = {default_value}"
                    )

        # Add missing structural properties with sensible defaults for position coordinates
        for cortical_area in cortical_areas:
            for prop_name, prop_pattern in cortical_structural_properties.items():
                # Construct the expected gene key for this property
                gene_key = f"_____10c-{cortical_area}-{prop_pattern}"

                # Check if this property is missing from the blueprint
                if gene_key not in blueprint:
                    # For structural properties, provide sensible defaults
                    if prop_name in ["rcordx", "rcordy", "rcordz"]:
                        # Position coordinates default to 0
                        default_value = 0
                    elif prop_name in ["bbx", "bby", "bbz"]:
                        # Dimension defaults to 1 (minimum valid dimension)
                        default_value = 1
                    elif prop_name == "name":
                        # Name defaults to the cortical area ID
                        default_value = cortical_area
                    else:
                        # Fallback for any other structural properties
                        default_value = 0

                    blueprint[gene_key] = default_value
                    missing_properties_count += 1
                    fixed_references.append(
                        f"Added missing structural property {gene_key} = {default_value}"
                    )

        if missing_properties_count > 0:
            fixed_references.append(
                f"Added {missing_properties_count} missing neuron and structural properties with default values from cortical_template"
            )

    # Also clean up cortical_mappings if they exist
    if "cortical_mappings" in genome and isinstance(genome["cortical_mappings"], list):
        mappings = genome["cortical_mappings"]
        valid_mappings = []

        for mapping in mappings:
            if isinstance(mapping, dict) and "morphology" in mapping:
                if mapping["morphology"] not in removed_morphologies:
                    valid_mappings.append(mapping)
                else:
                    fixed_references.append(
                        f"Removed cortical mapping from {mapping.get('source', 'unknown')} to {mapping.get('destination', 'unknown')} (invalid morphology '{mapping['morphology']}')"
                    )
            else:
                valid_mappings.append(mapping)

        genome["cortical_mappings"] = valid_mappings

    # Generate recovery summary
    recovery_actions = []
    if removed_morphologies:
        recovery_actions.append(
            f"Removed {len(removed_morphologies)} invalid morphologies: {', '.join(removed_morphologies)}"
        )
    if missing_properties_count > 0:
        recovery_actions.append(
            f"Added {missing_properties_count} missing neuron and structural properties"
        )
    if len(fixed_references) > len(recovery_actions):
        recovery_actions.append(
            f"Fixed {len(fixed_references) - len(recovery_actions)} blueprint references"
        )

    if recovery_actions:
        recovery_summary = f"Auto-recovery: {'; '.join(recovery_actions)}"
    else:
        recovery_summary = (
            f"Auto-recovery: {'; '.join(fixed_references)}"
            if fixed_references
            else "Auto-recovery: No issues found to fix"
        )

    return {
        "genome": genome,
        "removed_morphologies": removed_morphologies,
        "fixed_references": fixed_references,
        "recovery_summary": recovery_summary,
        "validation_warnings": validation_warnings,
    }


def sanitize_missing_physiology(genome):
    """
    Detect and add missing physiology properties from the genome during auto-recovery.

    This function ensures the physiology section exists and contains all required properties
    with appropriate default values from the physiology_template.

    Args:
        genome: The genome data to sanitize

    Returns:
        dict: {
            "genome": dict,  # The sanitized genome
            "added_properties": List[str],  # Names of added physiology properties
            "recovery_summary": str  # Summary of what was fixed
        }
    """
    added_properties = []

    # Import the physiology template for default values
    try:
        from feagi.evo.templates import physiology_property_types, physiology_template
    except ImportError:
        logger.error(
            "CRITICAL: Cannot import physiology_template from templates.py - physiology auto-recovery will fail"
        )
        return {
            "genome": genome,
            "added_properties": [],
            "recovery_summary": "Physiology auto-recovery failed: physiology_template not available",
        }

    # Ensure physiology section exists
    if "physiology" not in genome:
        logger.info(
            "MISSING PHYSIOLOGY SECTION: Creating physiology section with default values"
        )
        genome["physiology"] = {}
        added_properties.append("physiology section")

    physiology = genome["physiology"]

    # Check and add missing physiology properties
    for prop_name, default_value in physiology_template.items():
        if prop_name not in physiology:
            physiology[prop_name] = default_value
            added_properties.append(f"{prop_name} = {default_value}")
            logger.info(
                f"PHYSIOLOGY RECOVERY: Added missing property '{prop_name}' = {default_value}"
            )
        else:
            # Validate existing property type
            expected_type = physiology_property_types.get(prop_name, "unknown")
            current_value = physiology[prop_name]

            # Type validation and correction
            if expected_type == "float" and not isinstance(current_value, (int, float)):
                try:
                    physiology[prop_name] = float(current_value)
                    added_properties.append(f"{prop_name} type corrected to float")
                except (ValueError, TypeError):
                    physiology[prop_name] = default_value
                    added_properties.append(
                        f"{prop_name} = {default_value} (invalid value corrected)"
                    )
            elif expected_type == "int" and not isinstance(current_value, int):
                try:
                    physiology[prop_name] = int(current_value)
                    added_properties.append(f"{prop_name} type corrected to int")
                except (ValueError, TypeError):
                    physiology[prop_name] = default_value
                    added_properties.append(
                        f"{prop_name} = {default_value} (invalid value corrected)"
                    )

    # Generate recovery summary
    if added_properties:
        recovery_summary = f"Physiology auto-recovery: Added/corrected {len(added_properties)} properties: {', '.join(added_properties)}"
    else:
        recovery_summary = "Physiology section is complete"

    return {
        "genome": genome,
        "added_properties": added_properties,
        "recovery_summary": recovery_summary,
    }


def validate_physiology_section(genome):
    """
    Validate the physiology section of the genome.

    Args:
        genome: The genome data to validate

    Returns:
        dict: {
            "valid": bool,
            "errors": List[str],
            "error_summary": str
        }
    """
    errors = []

    # Check if physiology section exists
    if "physiology" not in genome:
        errors.append(
            "MISSING PHYSIOLOGY SECTION: Physiology section is required for FEAGI to function"
        )
        return {
            "valid": False,
            "errors": errors,
            "error_summary": "Missing physiology section",
        }

    physiology = genome["physiology"]

    # Import required templates
    try:
        from feagi.evo.templates import physiology_property_types, physiology_template
    except ImportError:
        errors.append("CRITICAL: Cannot import physiology_template from templates.py")
        return {
            "valid": False,
            "errors": errors,
            "error_summary": "Template import failed",
        }

    # Validate required properties
    for prop_name, default_value in physiology_template.items():
        if prop_name not in physiology:
            errors.append(
                f"MISSING PHYSIOLOGY PROPERTY: '{prop_name}' is required (default: {default_value})"
            )
        else:
            # Validate property type
            expected_type = physiology_property_types.get(prop_name, "unknown")
            current_value = physiology[prop_name]

            if expected_type == "float" and not isinstance(current_value, (int, float)):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be a number (float), got {type(current_value).__name__}"
                )
            elif expected_type == "int" and not isinstance(current_value, int):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be an integer, got {type(current_value).__name__}"
                )
            elif expected_type == "bool" and not isinstance(current_value, bool):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be a boolean, got {type(current_value).__name__}"
                )

    is_valid = len(errors) == 0

    if not is_valid:
        error_summary = f"Physiology validation failed: {len(errors)} issues found"
    else:
        error_summary = "Physiology section is valid"

    return {"valid": is_valid, "errors": errors, "error_summary": error_summary}


def auto_correct_cortical_area_types(genome):
    """
    Auto-correct cortical area type assignments when cortical IDs belong to CORE type.

    This function detects when cortical areas are assigned to wrong types (e.g., IPU or OPU)
    when they should be CORE type based on their cortical_id, and automatically corrects them.

    Args:
        genome: The genome data to correct (modified in-place)

    Returns:
        dict: {
            "genome": dict,  # The corrected genome (same as input, modified in-place)
            "corrected_areas": List[str],  # Names of corrected cortical areas
            "correction_summary": str,  # Summary of what was corrected
            "warnings": List[str]  # Validation warnings for the response
        }
    """
    corrected_areas = []
    warnings = []

    # Import cortical types for validation
    try:
        from feagi.evo.templates import cortical_types
    except ImportError:
        error_msg = "CRITICAL: Cannot import cortical_types from templates.py - cortical area type correction will fail"
        logger.error(error_msg)
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "Cortical area type correction failed: cortical_types not available",
            "warnings": [error_msg],
        }

    # Check if blueprint section exists
    if "blueprint" not in genome:
        return {
            "genome": genome,
            "corrected_areas": [],
            "correction_summary": "No blueprint section found - nothing to correct",
            "warnings": [],
        }

    blueprint = genome["blueprint"]

    # Get all supported CORE cortical area IDs
    core_cortical_ids = set()
    if "CORE" in cortical_types and "supported_devices" in cortical_types["CORE"]:
        core_cortical_ids = set(cortical_types["CORE"]["supported_devices"].keys())

    # Find cortical areas with _group assignments
    for gene_key, gene_value in blueprint.items():
        if not isinstance(gene_key, str):
            continue

        parts = gene_key.split("-")
        if len(parts) < 4:
            continue

        # Check if this is a _group gene
        if parts[3] == "_group":
            cortical_area_id = parts[1]
            current_group = gene_value

            # Check if this cortical area should be CORE type
            if cortical_area_id in core_cortical_ids and current_group != "CORE":
                # Auto-correct the group assignment
                blueprint[gene_key] = "CORE"
                corrected_areas.append(f"{cortical_area_id}: {current_group} → CORE")
                warning_msg = f"CORTICAL TYPE AUTO-CORRECTION: '{cortical_area_id}' changed from '{current_group}' to 'CORE'"
                warnings.append(warning_msg)
                logger.info(warning_msg)

    # Generate correction summary
    if corrected_areas:
        correction_summary = f"Cortical area type auto-correction: Fixed {len(corrected_areas)} areas: {', '.join(corrected_areas)}"
    else:
        correction_summary = "No cortical area type corrections needed"

    return {
        "genome": genome,
        "corrected_areas": corrected_areas,
        "correction_summary": correction_summary,
        "warnings": warnings,
    }


def blueprint_validator_silent(genome):
    """
    Silent version of blueprint_validator that doesn't log errors.
    Used for initial validation checks before auto-recovery.
    """
    valid_genome = False

    # ✅ CRITICAL FIX: Check if blueprint exists before accessing it
    try:
        blueprint = genome["blueprint"]
    except KeyError:
        return False

    try:
        neuron_morphologies = genome["neuron_morphologies"]
        cortical_list = cortical_list_gen(blueprint)
        valid_genome = True
    except KeyError:
        return False

    def gene_segments(gene_):
        return _gene_segments_validator(gene_)

    def destination_rules(gene_):
        return _destination_rules_validator(
            gene_, blueprint, cortical_list, neuron_morphologies
        )

    def special_areas(gene_):
        return _special_areas_validator(gene_, blueprint)

    # Check for invalid cortical areas but don't fail validation
    invalid_areas = []
    for gene in blueprint:
        segments = gene.split(genome_properties["structure"]["segment_seperator"])
        if len(segments) >= 2:
            cortical_area = segments[1]

            # Check if this cortical area has any valid definition
            has_group_definition = False
            for check_gene in blueprint:
                check_segments = check_gene.split(
                    genome_properties["structure"]["segment_seperator"]
                )
                if (
                    len(check_segments) >= 4
                    and check_segments[1] == cortical_area
                    and check_segments[3] == "_group"
                ):
                    has_group_definition = True
                    break

            # If cortical area has no group definition, it's likely a typo
            if not has_group_definition and cortical_area not in invalid_areas:
                invalid_areas.append(cortical_area)

    for gene in blueprint:
        segments = gene.split(genome_properties["structure"]["segment_seperator"])

        # Skip genes for invalid cortical areas
        if len(segments) >= 2 and segments[1] in invalid_areas:
            continue

        if not gene_segments(gene):
            valid_genome = False
        if not destination_rules(gene):
            valid_genome = False
        if not special_areas(gene):
            valid_genome = False

    return valid_genome


def genome_validator_with_errors_silent(genome):
    """
    Silent version of genome validation that doesn't log errors.
    Used for initial validation checks before auto-recovery.

    Args:
        genome: The genome data to validate

    Returns:
        dict: {
            "valid": bool,
            "errors": List[str],  # Specific error messages (but not logged)
            "error_summary": str  # Summary of all errors
        }
    """
    errors = []

    # Validate morphologies (silent)
    if not morphology_validator_silent(genome):
        errors.append(
            "Morphology validation failed - check neuron_morphologies section"
        )

    # Validate blueprint (silent)
    if not blueprint_validator_silent(genome):
        errors.append("Blueprint validation failed - check blueprint section")

    # Validate physiology (silent)
    physiology_result = validate_physiology_section_silent(genome)
    if not physiology_result["valid"]:
        errors.extend(physiology_result["errors"])

    is_valid = len(errors) == 0

    if not is_valid:
        error_summary = f"Multiple validation errors: {len(errors)} issues found"
    else:
        error_summary = "Valid"

    return {"valid": is_valid, "errors": errors, "error_summary": error_summary}


def morphology_validator_silent(genome):
    """Silent version of morphology_validator that doesn't log errors."""
    if "neuron_morphologies" not in genome:
        return False

    neuron_morphologies = genome["neuron_morphologies"]
    for morphology in neuron_morphologies:
        try:
            if "type" not in neuron_morphologies[morphology]:
                return False
            else:
                if neuron_morphologies[morphology]["type"] not in [
                    "vectors",
                    "patterns",
                    "functions",
                    "composite",
                ]:
                    return False

                if neuron_morphologies[morphology]["type"] == "composite":
                    if (
                        "src_seed" not in neuron_morphologies[morphology]["parameters"]
                        or "src_pattern"
                        not in neuron_morphologies[morphology]["parameters"]
                        or "mapper_morphology"
                        not in neuron_morphologies[morphology]["parameters"]
                    ):
                        return False

                if neuron_morphologies[morphology]["type"] == "vectors":
                    if "vectors" not in neuron_morphologies[morphology]["parameters"]:
                        return False
                    else:
                        vector_definition = neuron_morphologies[morphology][
                            "parameters"
                        ]["vectors"]
                        if type(vector_definition) is not list:
                            return False
                        else:
                            for vector in vector_definition:
                                if len(vector) != 3:
                                    return False

                if neuron_morphologies[morphology]["type"] == "patterns":
                    if "patterns" not in neuron_morphologies[morphology]["parameters"]:
                        return False
                    else:
                        pattern_definition = neuron_morphologies[morphology][
                            "parameters"
                        ]["patterns"]
                        if type(pattern_definition) is not list:
                            return False
                        else:
                            for pattern in pattern_definition:
                                if len(pattern) != 2:
                                    return False
                                else:
                                    for pattern_segment in pattern:
                                        if len(pattern_segment) != 3:
                                            return False

            if "parameters" not in neuron_morphologies[morphology]:
                return False

        except Exception:
            return False

    return True


def validate_physiology_section_silent(genome):
    """Silent version of validate_physiology_section that doesn't log errors."""
    errors = []

    # Check if physiology section exists
    if "physiology" not in genome:
        errors.append(
            "MISSING PHYSIOLOGY SECTION: Physiology section is required for FEAGI to function"
        )
        return {
            "valid": False,
            "errors": errors,
            "error_summary": "Missing physiology section",
        }

    physiology = genome["physiology"]

    # Import required templates
    try:
        from feagi.evo.templates import physiology_property_types, physiology_template
    except ImportError:
        errors.append("CRITICAL: Cannot import physiology_template from templates.py")
        return {
            "valid": False,
            "errors": errors,
            "error_summary": "Template import failed",
        }

    # Validate required properties
    for prop_name, default_value in physiology_template.items():
        if prop_name not in physiology:
            errors.append(
                f"MISSING PHYSIOLOGY PROPERTY: '{prop_name}' is required (default: {default_value})"
            )
        else:
            # Validate property type
            expected_type = physiology_property_types.get(prop_name, "unknown")
            current_value = physiology[prop_name]

            if expected_type == "float" and not isinstance(current_value, (int, float)):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be a number (float), got {type(current_value).__name__}"
                )
            elif expected_type == "int" and not isinstance(current_value, int):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be an integer, got {type(current_value).__name__}"
                )
            elif expected_type == "bool" and not isinstance(current_value, bool):
                errors.append(
                    f"INVALID PHYSIOLOGY TYPE: '{prop_name}' should be a boolean, got {type(current_value).__name__}"
                )

    is_valid = len(errors) == 0

    if not is_valid:
        error_summary = f"Physiology validation failed: {len(errors)} issues found"
    else:
        error_summary = "Physiology section is valid"

    return {"valid": is_valid, "errors": errors, "error_summary": error_summary}
