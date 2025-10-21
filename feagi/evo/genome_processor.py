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
#  ==============================================================================

import copy
import json
import traceback
from abc import ABC, abstractmethod
from pathlib import Path
from time import time
from typing import Any, Dict, List, Tuple, Union

from feagi.core.state_manager import FeagiStateManager, GenomeState
from feagi.evo.genome_editor import save_genome
from feagi.evo.genome_validator import genome_validator
from feagi.evo.templates import core_morphologies, cortical_types
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Helper to get state manager instance
state = FeagiStateManager.instance()

# TODO: Replace with actual global/singleton instance if available
connectome_manager = None  # <-- Set this to the actual instance in your app


def merge_core_morphologies(genome):
    for core_morphology in core_morphologies:
        genome["neuron_morphologies"][core_morphology] = core_morphologies[
            core_morphology
        ].copy()
    return genome


def genome_ver_check(genome):
    try:
        if genome["version"] == "2.0":
            # Genome Version 2.0 detected
            try:
                state.genome_validity = genome_validator(genome)
                logger.info(f"Genome validity={state.genome_validity}")
            except Exception as e:
                logger.error(f"Error during genome validation!! {e}")
            genome = merge_core_morphologies(genome)
            genome = genome_morphology_updator(genome)
            genome = genome_physiology_updator(genome=genome)
            genome = genome_stat_updator(genome=genome)
            save_genome(
                genome=genome, file_name=state.connectome_path + "genome.json"
            )
            genome1 = genome_2_1_convertor(flat_genome=genome["blueprint"])
            genome_2_hierarchifier(flat_genome=genome["blueprint"])
            genome["blueprint"] = genome1["blueprint"]
            update_template()
            return genome
        else:
            logger.error("ERROR! Genome is not compatible with 2.0 standard")
    except KeyError as e:
        logger.error(f"Exception during genome version check {e}")
        pass


def update_template():
    # Helper to map cortical_area (genome id) to area_id (int)
    def get_area_by_genome_id(genome_id):
        # TODO: Implement actual mapping from genome_id to area_id
        # For now, assume area name == genome_id and search all areas
        if connectome_manager is None:
            raise RuntimeError("connectome_manager instance is not set!")
        for area in connectome_manager._areas.values():
            if area.name == genome_id:
                return area
        return None

    for cortical_area in state.genome["blueprint"]:
        area = get_area_by_genome_id(cortical_area)
        if area is None:
            continue  # or raise error
        cortical_type = area.type.upper()  # e.g., 'IPU', 'OPU', etc.
        if cortical_type in ["IPU", "OPU"]:
            cortical_size = state.genome["blueprint"][cortical_area][
                "block_boundaries"
            ]

            if "dev_count" not in state.genome["blueprint"][cortical_area]:
                state.genome["blueprint"][cortical_area]["dev_count"] = (
                    cortical_size[0]
                    / cortical_types[cortical_type]["supported_devices"][
                        cortical_area
                    ]["resolution"][0]
                )

                cortical_types[cortical_type]["supported_devices"][
                    cortical_area
                ]["resolution"][1] = cortical_size[1]
                cortical_types[cortical_type]["supported_devices"][
                    cortical_area
                ]["resolution"][2] = cortical_size[2]
            else:
                dev_count = state.genome["blueprint"][cortical_area][
                    "dev_count"
                ]

                if dev_count != 0:
                    cortical_types[cortical_type]["supported_devices"][
                        cortical_area
                    ]["resolution"][0] = int(cortical_size[0] / dev_count)
                    cortical_types[cortical_type]["supported_devices"][
                        cortical_area
                    ]["resolution"][1] = cortical_size[1]
                    cortical_types[cortical_type]["supported_devices"][
                        cortical_area
                    ]["resolution"][2] = cortical_size[2]
                else:
                    state.genome["blueprint"][cortical_area]["dev_count"] = 1
                    cortical_types[cortical_type]["supported_devices"][
                        cortical_area
                    ]["resolution"] = cortical_size


def genome_2_print(genome):
    for cortical_area in genome:
        logger.info(cortical_area)
        for gene in genome[cortical_area]:
            try:
                logger.info(
                    f"       {genome_2_to_1[gene]} \n\t\t\t "
                    f"{genome[cortical_area][gene]}"
                )
            except Exception:
                pass


def genome_2_validator(genome_2):
    """Conducts various test to ensure the stability of the Genome 2.0."""
    standard_gene_length = 27

    def structure_test_gene_lengths():
        """Check length requirements for each gene."""
        gene_anomalies = 0
        for key in genome_2:
            if len(key) != standard_gene_length:
                logger.warning(
                    f"Warning! Key did not meet length requirement: {key}"
                )
                gene_anomalies += 1
        if gene_anomalies == 0:
            logger.info("\nGene length verification ...... PASSED!")
        else:
            logger.info(
                "\nGene length verification...... Failed!   ",
                gene_anomalies,
                " anomalies detected",
            )
        return gene_anomalies


def genome_2_hierarchifier(flat_genome):
    """Converts Genome 2.0 to a hierarchical data structure."""
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
    """Extract cortical areas list from genome v1 format."""
    cortical_list = list()
    # Handle both blueprint key and direct cortical area access
    if "blueprint" in genome:
        for cortical_area in genome["blueprint"]:
            cortical_list.append(cortical_area)
    else:
        for cortical_area in genome:
            cortical_list.append(cortical_area)
    return cortical_list


def genome_2_cortical_list(flat_genome):
    """Generates a list of cortical areas inside genome."""
    try:
        cortical_list = list()
        # Check if genome has blueprint section (hierarchical format)
        if 'blueprint' in flat_genome:
            keys_to_check = flat_genome['blueprint']
        else:
            keys_to_check = flat_genome
            
        for key in keys_to_check:
            if json_comment_catcher(key):
                # CRITICAL FIX: Extract cortical ID properly by finding the area between dashes
                # Format: _____10c-CORTICAL_ID-cx-property-type
                if key.startswith("_____10c-") and key[7] == "c":
                    parts = key.split("-")
                    if len(parts) >= 3:
                        cortical_id = parts[1]  # The cortical ID is the second part
                        if cortical_id not in cortical_list:
                            cortical_list.append(cortical_id)
        return cortical_list
    except Exception as e:
        logger.error(
            "Exception during genome_2_cortical_list", e, traceback.print_exc()
        )


def json_comment_catcher(key):
    if key[:1] == "/":
        return False
    else:
        return True


def cortical_area_id_update_checker(cortical_id):
    """Responsible for updating deprecated cortical names to new ones."""
    if cortical_id == "i__v0C":
        return "iic400"
    elif cortical_id == "i_v0BL":
        return "iic000"
    elif cortical_id == "i_v0BR":
        return "iic200"
    elif cortical_id == "i_v0BM":
        return "iic100"
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
    genome["blueprint"] = dict()
    cortical_list = genome_2_cortical_list(flat_genome)
    logger.info("%" * 30)
    logger.info(cortical_list)
    # Assign a blank template to each cortical area
    for cortical_area in cortical_list:
        genome["blueprint"][
            cortical_area_id_update_checker(cortical_id=cortical_area)
        ] = copy.deepcopy(genome_1_template)

    # Populate each cortical area with
    for cortical_area in genome["blueprint"]:
        try:
            # Check if genome has blueprint section (hierarchical format)
            if 'blueprint' in flat_genome:
                genes_to_check = flat_genome['blueprint']
            else:
                genes_to_check = flat_genome
                
            for gene in genes_to_check:
                if json_comment_catcher(gene):
                    # CRITICAL FIX: Extract cortical ID properly by splitting on dashes
                    if gene.startswith("_____10c-"):
                        parts = gene.split("-")
                        if len(parts) >= 4:  # Need at least: _____10c, cortical_id, cx/nx, property
                            raw_cortical_id = parts[1]
                            cortical_id = cortical_area_id_update_checker(
                                cortical_id=raw_cortical_id
                            )
                            # Extract the property part (include the cx/nx part)
                            # Format: _____10c-AREA1-cx-dstmap-d -> we want "cx-dstmap-d"
                            exon = "-".join(parts[2:])  # Skip _____10c, cortical_id
                            
                            # Also create a version without the cx/nx prefix for dictionary lookup
                            # Format: "cx-dstmap-d" -> "dstmap-d"
                            exon_without_prefix = "-".join(parts[3:]) if len(parts) > 3 else exon
                        else:
                            continue  # Skip malformed keys
                    else:
                        continue  # Skip non-cortical keys
                    # gene_type = gene[16:18]  # Unused variable removed
                    # Try both full exon and exon without prefix
                    if exon in genome_2_to_1:
                        lookup_exon = exon
                    elif exon_without_prefix in genome_2_to_1:
                        lookup_exon = exon_without_prefix
                    else:
                        lookup_exon = None
                        
                    if lookup_exon:
                        if cortical_id == cortical_area:
                            if genome_2_to_1[lookup_exon] == "cortical_name":
                                genome["blueprint"][cortical_area][
                                    genome_2_to_1[lookup_exon]
                                ] = genes_to_check[gene]
                            elif (
                                genome_2_to_1[lookup_exon]
                                == "location_generation_type"
                            ):
                                if genes_to_check[gene]:
                                    genome["blueprint"][cortical_area][
                                        genome_2_to_1[lookup_exon]
                                    ] = "random"
                                else:
                                    genome["blueprint"][cortical_area][
                                        genome_2_to_1[lookup_exon]
                                    ] = "sequential"
                            elif genome_2_to_1[lookup_exon] == "cortical_mapping_dst":
                                for destination in genes_to_check[gene]:
                                    if json_comment_catcher(
                                        genes_to_check[gene][destination]
                                    ) and json_comment_catcher(destination):
                                        for mapping_recipe in genes_to_check[
                                            gene
                                        ][destination]:
                                            if (
                                                destination
                                                not in genome["blueprint"][
                                                    cortical_area
                                                ][genome_2_to_1[lookup_exon]]
                                            ):
                                                genome["blueprint"][
                                                    cortical_area
                                                ][genome_2_to_1[lookup_exon]][
                                                    destination
                                                ] = list()

                                            # IMPROVED: More robust flat to hierarchical conversion
                                            temp_dict = dict()
                                            
                                            # Validate array has minimum required elements
                                            if not isinstance(mapping_recipe, list) or len(mapping_recipe) < 4:
                                                logger.warning(
                                                    f"Invalid mapping recipe format in {cortical_area} -> {destination}: {mapping_recipe}"
                                                )
                                                continue

                                            temp_dict["morphology_id"] = (
                                                mapping_recipe[0]
                                            )
                                            temp_dict["morphology_scalar"] = (
                                                mapping_recipe[1]
                                            )
                                            temp_dict[
                                                "postSynapticCurrent_multiplier"
                                            ] = mapping_recipe[2]
                                            temp_dict["plasticity_flag"] = (
                                                mapping_recipe[3]
                                            )
                                            
                                            # Handle optional plasticity parameters with defaults
                                            temp_dict["plasticity_constant"] = (
                                                mapping_recipe[4] if len(mapping_recipe) > 4 else 1
                                            )
                                            temp_dict["ltp_multiplier"] = (
                                                mapping_recipe[5] if len(mapping_recipe) > 5 else 1
                                            )
                                            temp_dict["ltd_multiplier"] = (
                                                mapping_recipe[6] if len(mapping_recipe) > 6 else 1
                                            )

                                            genome["blueprint"][cortical_area][
                                                genome_2_to_1[lookup_exon]
                                            ][destination].append(temp_dict)

                            elif genome_2_to_1[lookup_exon] == "block_boundaries":
                                if gene[24] == "x":
                                    genome["blueprint"][cortical_area][
                                        "block_boundaries"
                                    ][0] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create cortical_dimensions dict format
                                    if "cortical_dimensions" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["cortical_dimensions"] = {}
                                    genome["blueprint"][cortical_area]["cortical_dimensions"]["width"] = genes_to_check[gene]
                                elif gene[24] == "y":
                                    genome["blueprint"][cortical_area][
                                        "block_boundaries"
                                    ][1] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create cortical_dimensions dict format
                                    if "cortical_dimensions" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["cortical_dimensions"] = {}
                                    genome["blueprint"][cortical_area]["cortical_dimensions"]["height"] = genes_to_check[gene]
                                elif gene[24] == "z":
                                    genome["blueprint"][cortical_area][
                                        "block_boundaries"
                                    ][2] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create cortical_dimensions dict format
                                    if "cortical_dimensions" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["cortical_dimensions"] = {}
                                    genome["blueprint"][cortical_area]["cortical_dimensions"]["depth"] = genes_to_check[gene]
                                else:
                                    pass

                            elif genome_2_to_1[lookup_exon] == "relative_coordinate":
                                if gene[24] == "x":
                                    genome["blueprint"][cortical_area][
                                        "relative_coordinate"
                                    ][0] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create coordinates_3d dict format
                                    if "coordinates_3d" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["coordinates_3d"] = {}
                                    genome["blueprint"][cortical_area]["coordinates_3d"]["x"] = genes_to_check[gene]
                                elif gene[24] == "y":
                                    genome["blueprint"][cortical_area][
                                        "relative_coordinate"
                                    ][1] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create coordinates_3d dict format
                                    if "coordinates_3d" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["coordinates_3d"] = {}
                                    genome["blueprint"][cortical_area]["coordinates_3d"]["y"] = genes_to_check[gene]
                                elif gene[24] == "z":
                                    genome["blueprint"][cortical_area][
                                        "relative_coordinate"
                                    ][2] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create coordinates_3d dict format
                                    if "coordinates_3d" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["coordinates_3d"] = {}
                                    genome["blueprint"][cortical_area]["coordinates_3d"]["z"] = genes_to_check[gene]
                                else:
                                    pass
                            elif genome_2_to_1[lookup_exon] == "2d_coordinate":
                                if gene[24] == "x":
                                    genome["blueprint"][cortical_area][
                                        "2d_coordinate"
                                    ][0] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create coordinates_2d array format
                                    if "coordinates_2d" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["coordinates_2d"] = [0, 0]
                                    genome["blueprint"][cortical_area]["coordinates_2d"][0] = genes_to_check[gene]
                                elif gene[24] == "y":
                                    genome["blueprint"][cortical_area][
                                        "2d_coordinate"
                                    ][1] = genes_to_check[gene]
                                    # CRITICAL FIX: Also create coordinates_2d array format
                                    if "coordinates_2d" not in genome["blueprint"][cortical_area]:
                                        genome["blueprint"][cortical_area]["coordinates_2d"] = [0, 0]
                                    genome["blueprint"][cortical_area]["coordinates_2d"][1] = genes_to_check[gene]
                                else:
                                    pass

                            else:
                                # Enhanced property handling - explicit mappings for critical properties
                                if "nx-mp_acc-b" in exon:
                                    genome["blueprint"][cortical_area]["mp_charge_accumulation"] = genes_to_check[gene]
                                elif "nx-mp_psp-b" in exon:
                                    genome["blueprint"][cortical_area]["mp_driven_psp"] = genes_to_check[gene]
                                elif "fire_t-f" in exon:
                                    genome["blueprint"][cortical_area]["firing_threshold"] = genes_to_check[gene]
                                elif "refrac-i" in exon:
                                    genome["blueprint"][cortical_area]["refractory_period"] = genes_to_check[gene]
                                elif "leak_c-f" in exon:
                                    genome["blueprint"][cortical_area]["leak_coefficient"] = genes_to_check[gene]
                                elif "leak_v-f" in exon:
                                    genome["blueprint"][cortical_area]["leak_variability"] = genes_to_check[gene]
                                elif "pspuni-b" in exon:
                                    genome["blueprint"][cortical_area]["psp_uniform_distribution"] = genes_to_check[gene]
                                else:
                                    # Fallback to dictionary mapping for other properties
                                    try:
                                        genome["blueprint"][cortical_area][
                                            genome_2_to_1[lookup_exon]
                                        ] = genes_to_check[gene]
                                    except Exception as e:
                                        logger.error(
                                            f"Key not processed: {cortical_area} {e} "
                                            f"{traceback.print_exc()}"
                                        )

        except Exception as e:
            logger.error(
                f"Exception during gene translation of {cortical_area}",
                e,
                traceback.print_exc(),
            )
    return genome


def genome_v1_v2_converter(genome_v1):
    genome_v2 = genome_v1.copy()
    genome_v2.pop("blueprint")
    genome_v2["blueprint"] = {}

    for cortical_area in genome_v1["blueprint"]:
        area_data = genome_v1["blueprint"][cortical_area]
        
        # CRITICAL FIX: Handle 3D coordinates - support multiple naming conventions
        # Check for coordinates_3d (dict format), relative_coordinate (array format), or position
        coordinates_3d = None
        if "coordinates_3d" in area_data:
            # Dict format: {"x": 1, "y": 2, "z": 3}
            coords = area_data["coordinates_3d"]
            if isinstance(coords, dict):
                coordinates_3d = [coords.get("x", 0), coords.get("y", 0), coords.get("z", 0)]
            elif isinstance(coords, (list, tuple)) and len(coords) >= 3:
                coordinates_3d = list(coords[:3])
        elif "relative_coordinate" in area_data:
            # Array format: [x, y, z]
            coords = area_data["relative_coordinate"]
            if isinstance(coords, (list, tuple)) and len(coords) >= 3:
                coordinates_3d = list(coords[:3])
        elif "position" in area_data:
            # Alternative array format
            coords = area_data["position"]
            if isinstance(coords, (list, tuple)) and len(coords) >= 3:
                coordinates_3d = list(coords[:3])
        
        # Export 3D coordinates to flat format if found
        if coordinates_3d:
            genex = "_____10c-" + cortical_area + "-" + "cx-rcordx-i"
            geney = "_____10c-" + cortical_area + "-" + "cx-rcordy-i"
            genez = "_____10c-" + cortical_area + "-" + "cx-rcordz-i"
            genome_v2["blueprint"][genex] = coordinates_3d[0]
            genome_v2["blueprint"][geney] = coordinates_3d[1]
            genome_v2["blueprint"][genez] = coordinates_3d[2]
        
        # CRITICAL FIX: Handle dimensions - support multiple naming conventions
        # Check for cortical_dimensions (dict format) or block_boundaries (array format)
        dimensions = None
        if "cortical_dimensions" in area_data:
            # Dict format: {"width": 10, "height": 10, "depth": 10}
            dims = area_data["cortical_dimensions"]
            if isinstance(dims, dict):
                dimensions = [dims.get("width", 1), dims.get("height", 1), dims.get("depth", 1)]
            elif isinstance(dims, (list, tuple)) and len(dims) >= 3:
                dimensions = list(dims[:3])
        elif "block_boundaries" in area_data:
            # Array format: [width, height, depth]
            dims = area_data["block_boundaries"]
            if isinstance(dims, (list, tuple)) and len(dims) >= 3:
                dimensions = list(dims[:3])
        elif "dimensions" in area_data:
            # Alternative array format
            dims = area_data["dimensions"]
            if isinstance(dims, (list, tuple)) and len(dims) >= 3:
                dimensions = list(dims[:3])
        
        # Export dimensions to flat format if found
        if dimensions:
            genex = "_____10c-" + cortical_area + "-" + "cx-___bbx-i"
            geney = "_____10c-" + cortical_area + "-" + "cx-___bby-i"
            genez = "_____10c-" + cortical_area + "-" + "cx-___bbz-i"
            genome_v2["blueprint"][genex] = dimensions[0]
            genome_v2["blueprint"][geney] = dimensions[1]
            genome_v2["blueprint"][genez] = dimensions[2]
        
        # CRITICAL FIX: Handle 2D coordinates - support multiple naming conventions
        # Check for coordinates_2d or 2d_coordinate
        coordinates_2d = None
        if "coordinates_2d" in area_data:
            coords = area_data["coordinates_2d"]
            if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                coordinates_2d = list(coords[:2])
        elif "2d_coordinate" in area_data:
            coords = area_data["2d_coordinate"]
            if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                coordinates_2d = list(coords[:2])
        
        # Export 2D coordinates to flat format if found
        if coordinates_2d:
            genex = "_____10c-" + cortical_area + "-" + "cx-2dcorx-i"
            geney = "_____10c-" + cortical_area + "-" + "cx-2dcory-i"
            genome_v2["blueprint"][genex] = coordinates_2d[0]
            genome_v2["blueprint"][geney] = coordinates_2d[1]
        
        # Handle all other properties
        for key in area_data:
            if type(key) is not dict and key not in ["cortical_mapping_dst"]:
                # Skip coordinate/dimension properties we already handled
                if key in [
                    "coordinates_3d", "relative_coordinate", "position",
                    "cortical_dimensions", "block_boundaries", "dimensions",
                    "coordinates_2d", "2d_coordinate"
                ]:
                    continue
                    
                if key in genome_1_to_2:
                    gene = (
                        "_____10c-" + cortical_area + "-" + genome_1_to_2[key]
                    )
                    genome_v2["blueprint"][gene] = area_data[key]

            elif key == "cortical_mapping_dst":
                gene = "_____10c-" + cortical_area + "-cx-dstmap-d"
                destination_map = {}
                for destination in genome_v1["blueprint"][cortical_area][
                    "cortical_mapping_dst"
                ]:
                    destination_map[destination] = list()
                    for entry in genome_v1["blueprint"][cortical_area][
                        "cortical_mapping_dst"
                    ][destination]:
                        # CRITICAL FIX: Handle both dictionary and array formats
                        if isinstance(entry, dict):
                            # Dictionary format (expected hierarchical format)
                            morphology_id = entry["morphology_id"]
                            morphology_scalar = entry["morphology_scalar"]
                            postSynapticCurrent_multiplier = entry[
                                "postSynapticCurrent_multiplier"
                            ]
                            plasticity_flag = entry["plasticity_flag"]

                            if "plasticity_constant" in entry:
                                plasticity_constant = entry["plasticity_constant"]
                            else:
                                plasticity_constant = 1

                            if "ltp_multiplier" in entry:
                                ltp_multiplier = entry["ltp_multiplier"]
                            else:
                                ltp_multiplier = 1

                            if "ltd_multiplier" in entry:
                                ltd_multiplier = entry["ltd_multiplier"]
                            else:
                                ltd_multiplier = 1
                                
                        elif isinstance(entry, list) and len(entry) >= 4:
                            # Array format (flat genome format found in hierarchical)
                            morphology_id = entry[0]
                            morphology_scalar = entry[1]
                            postSynapticCurrent_multiplier = entry[2]
                            plasticity_flag = entry[3]
                            
                            # Handle optional plasticity parameters
                            plasticity_constant = entry[4] if len(entry) > 4 else 1
                            ltp_multiplier = entry[5] if len(entry) > 5 else 1
                            ltd_multiplier = entry[6] if len(entry) > 6 else 1
                            
                        else:
                            # Invalid format - skip this entry with warning
                            logger.warning(
                                f"Invalid cortical mapping entry format in {cortical_area} -> {destination}: {entry}"
                            )
                            continue

                        destination_map[destination].append(
                            [
                                morphology_id,
                                morphology_scalar,
                                postSynapticCurrent_multiplier,
                                plasticity_flag,
                                plasticity_constant,
                                ltp_multiplier,
                                ltd_multiplier,
                            ]
                        )

                genome_v2["blueprint"][gene] = destination_map
            else:
                logger.warning(
                    f"Warning! {key} not found in genome_1_template!"
                )

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
            logger.info(f"morphology_out: {morphology_out}")
        elif "patterns" in morphology_in:
            morphology_out["type"] = "patterns"
            morphology_out["parameters"]["patterns"] = morphology_in[
                "patterns"
            ]
        elif "composite" in morphology_in:
            morphology_out["type"] = "composite"
            morphology_out["parameters"]["src_seed"] = morphology_in[
                "composite"
            ]["parameters"]["src_seed"]
            morphology_out["parameters"]["src_pattern"] = morphology_in[
                "composite"
            ]["parameters"]["src_pattern"]
            morphology_out["parameters"]["mapper_morphology"] = morphology_in[
                "composite"
            ]["mapper_morphology"]
        elif "functions" in morphology_in:
            morphology_out["type"] = "functions"
        else:
            pass

    # Fix pattern nesting
    if "patterns" in morphology_out["parameters"]:
        for pattern in morphology_out["parameters"]["patterns"]:
            if not valid_pattern(pattern):
                logger.info(f"#### >>> {pattern}")
                if len(pattern) == 3:
                    logger.info(f"> > {morphology_in}")
                    morphology_out["parameters"]["patterns"] = [
                        morphology_out["parameters"]["patterns"]
                    ]
                    logger.info(f"> > {morphology_out}")

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
            if not (isinstance(item, int) or item in ["*", "?"]):
                return False

    return True


def genome_morphology_updator(genome):
    try:
        for morphology in genome["neuron_morphologies"]:
            if not morphology:
                genome["neuron_morphologies"].pop(morphology)
            genome["neuron_morphologies"][morphology] = morphology_convertor(
                genome["neuron_morphologies"][morphology]
            )
        state.genome_validity = genome_validator(genome)
    except Exception as e:
        logger.error(
            "Error during genome morphology update!", e, traceback.print_exc()
        )

    return genome


def is_memory_cortical_area(cortical_area):
    cortical_obj = state.genome["blueprint"].get(cortical_area)
    if cortical_obj:
        if (
            "MEMORY"
            in state.genome["blueprint"][cortical_area]["sub_group_id"]
        ):
            return True
        else:
            return False
    else:
        return False


def genome_physiology_updator(genome: dict):
    if "physiology" not in genome:
        genome["physiology"] = {}
    if "max_burst_count" in genome:
        genome.pop("max_burst_count")

    #  MIGRATION: Convert burst_delay to simulation_timestep for backward
    #  compatibility
    if "burst_delay" in genome:
        genome["physiology"]["simulation_timestep"] = genome["burst_delay"]
        genome.pop("burst_delay")
        print(
            f"🔄 [GENOME] Migrated burst_delay → simulation_timestep: {genome['physiology']['simulation_timestep']}"
        )

    # Also handle physiology.burst_delay → physiology.simulation_timestep
    if "burst_delay" in genome.get("physiology", {}):
        genome["physiology"]["simulation_timestep"] = genome["physiology"][
            "burst_delay"
        ]
        genome["physiology"].pop("burst_delay")
        print(
            f"🔄 [GENOME] Migrated physiology.burst_delay → physiology.simulation_timestep: {genome['physiology']['simulation_timestep']}"
        )

    if "max_age" in genome:
        genome["physiology"]["max_age"] = genome["max_age"]
        genome.pop("max_age")
    if "evolution_burst_count" in genome:
        genome["physiology"]["evolution_burst_count"] = genome[
            "evolution_burst_count"
        ]
        genome.pop("evolution_burst_count")
    if "ipu_idle_threshold" in genome:
        genome["physiology"]["ipu_idle_threshold"] = genome[
            "ipu_idle_threshold"
        ]
        genome.pop("ipu_idle_threshold")
    if "plasticity_queue_depth" in genome:
        genome["physiology"]["plasticity_queue_depth"] = genome[
            "plasticity_queue_depth"
        ]
        genome.pop("plasticity_queue_depth")
    if "lifespan_mgmt_interval" in genome:
        genome["physiology"]["lifespan_mgmt_interval"] = genome[
            "lifespan_mgmt_interval"
        ]
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
    "_______c-______-cx-gd_vis-b": "visualization",
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
    "_______c-______-nx-fire_t-f": "firing_threshold",
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
    "_______c-______-cx-tmpdpt-i": "temporal_depth",
    "_______c-______-nx-excite-f": "neuron_excitability",
    "_______c-______-cx-devcnt-i": "dev_count",
}

genome_1_template = {
    "sub_group_id": "",
    "per_voxel_neuron_cnt": 1,
    "synapse_attractivity": 100,
    "degeneration": 0,
    "psp_uniform_distribution": True,
    "postsynaptic_current_max": 99999,
    "cortical_mapping_dst": {},
    "block_boundaries": [None, None, None],
    "relative_coordinate": [0, 0, 0],
    "2d_coordinate": [0, 0],
    "visualization": True,
    "postsynaptic_current": 1,
    "firing_threshold": 1,
    "refractory_period": 0,
    "leak_coefficient": 0,
    "leak_variability": 0,
    "consecutive_fire_cnt_max": 0,
    "snooze_length": 0,
    "firing_threshold_increment_x": 0,
    "firing_threshold_increment_y": 0,
    "firing_threshold_increment_z": 0,
    "firing_threshold_limit": 0,
    "mp_charge_accumulation": False,
    "mp_driven_psp": False,
    "is_mem_type": False,
    "longterm_mem_threshold": 100,
    "lifespan_growth_rate": 1,
    "init_lifespan": 9,
    "temporal_depth": 1,
    "neuron_excitability": 1.0,
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
    "fire_t-f": "firing_threshold",
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
    "nx-mp_acc-b": "mp_charge_accumulation",
    "mp_psp-b": "mp_driven_psp",
    "nx-mp_psp-b": "mp_driven_psp",
    "memory-b": "is_mem_type",
    "mem__t-i": "longterm_mem_threshold",
    "mem_gr-i": "lifespan_growth_rate",
    "mem_ls-i": "init_lifespan",
    "tmpdpt-i": "temporal_depth",
    "excite-f": "neuron_excitability",
    "devcnt-i": "dev_count",
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
    "firing_threshold": "nx-fire_t-f",
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
    "init_lifespan": "cx-mem_ls-i",
    "temporal_depth": "cx-tmpdpt-i",
    "dev_count": "cx-devcnt-i",
}


def process_and_load_genome(genome_data, core_api_service):
    """Process and load a genome with comprehensive state management.

    Args:
        genome_data: The genome data to process and load
        core_api_service: CoreAPIService instance for genome loading

    Returns:
        dict: Result containing load time and metadata
    """
    start_time = time()

    # Get state manager
    state_manager = FeagiStateManager.instance()

    # Set loading state
    state_manager.set_genome_state(GenomeState.LOADING)

    try:
        # Process and load the genome
        load_result = core_api_service.load_genome(genome_data)

        # Extract success value from the result
        if isinstance(load_result, dict) and "success" in load_result:
            success = load_result["success"]
        else:
            # Handle case where result might be a boolean or something else
            success = bool(load_result)

        # Update state based on result
        if success:
            state_manager.set_genome_state(GenomeState.LOADED)
            #  Don't increment the counter here - core_api_service already does
            #  it
            # state_manager.increment_genome_counter()
        else:
            state_manager.set_genome_state(GenomeState.ERROR)

        # Calculate load time
        load_time = time() - start_time

        # Return a clean result without redundant success field
        return {
            "load_time": round(load_time, 3),
            "genome_counter": state_manager.get_genome_counter(),
        }

    except Exception as e:
        # Set error state and re-raise
        state_manager.set_genome_state(GenomeState.ERROR)
        # Remove emoji parameter that's causing the error
        logger.error(f"Error during genome processing: {str(e)}")
        raise


#  ==============================================================================
# MODERN OOP GENOME PROCESSOR ARCHITECTURE
#  ==============================================================================


class GenomeVersionError(Exception):
    """Raised when genome version is not supported."""

    pass


class GenomeValidationError(Exception):
    """Raised when genome validation fails."""

    pass


# REMOVED: Duplicate genome processor classes
# The production system uses genome_2_1_convertor function above
# This eliminates duplication and ensures consistency

# Placeholder for future OOP genome processor if needed
class BaseGenomeProcessor(ABC):
    """Abstract base class for genome processors.

    Each genome version should have its own processor that inherits from this
    class. This ensures consistent interface while allowing version-specific
    implementations.
    """

    def __init__(self, genome_data: Dict[str, Any]):
        self.genome_data = genome_data
        self.logger = logger

    @abstractmethod
    def get_version(self) -> str:
        """Get the genome version this processor handles."""
        pass

    @abstractmethod
    def validate_genome(self) -> Tuple[bool, List[str]]:
        """Validate the genome structure and content.

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        pass

    @abstractmethod
    def extract_cortical_areas(self) -> Dict[str, Dict[str, Any]]:
        """Extract cortical area definitions from the genome.

        Returns:
            Dictionary mapping cortical_id to area properties
        """
        pass

    @abstractmethod
    def extract_cortical_mappings(
        self,
    ) -> Dict[str, Dict[str, List[Dict[str, Any]]]]:
        """Extract cortical mappings from the genome.

        Returns:
            Dictionary mapping src_area_id to {dst_area_id: [connection_specs]}
        """
        pass

    @abstractmethod
    def extract_morphologies(self) -> Dict[str, Dict[str, Any]]:
        """Extract morphology definitions from the genome.

        Returns:
            Dictionary mapping morphology_id to morphology definition
        """
        pass

    @abstractmethod
    def extract_physiology(self) -> Dict[str, Any]:
        """Extract physiology parameters from the genome.

        Returns:
            Dictionary of physiology parameters
        """
        pass


def load_genome_from_file(genome_path: Union[str, Path]) -> Dict[str, Any]:
    """Load genome data from a JSON file.

    Args:
        genome_path: Path to the genome file

    Returns:
        Dictionary containing genome data

    Raises:
        FileNotFoundError: If genome file doesn't exist
        json.JSONDecodeError: If genome file is not valid JSON
    """
    genome_path = Path(genome_path)

    if not genome_path.exists():
        raise FileNotFoundError(f"Genome file not found: {genome_path}")

    try:
        with open(genome_path, "r") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        raise json.JSONDecodeError(
            f"Invalid JSON in genome file {genome_path}: {e}"
        ) from e


# REMOVED: create_genome_processor and process_genome_file functions
# These referenced the removed GenomeProcessor class
# Use genome_2_1_convertor directly for flat-to-hierarchical conversion


def get_morphology_registry(
    genome_morphologies: Dict[str, Any],
) -> Dict[str, Dict[str, Any]]:
    """Convert genome morphologies to a registry format.

    Args:
        genome_morphologies: Morphologies section from genome

    Returns:
        Dictionary mapping morphology_id to morphology definition
    """
    registry = {}

    for morphology_id, morphology_def in genome_morphologies.items():
        if isinstance(morphology_def, dict):
            registry[morphology_id] = morphology_def
        else:
            logger.warning(
                f"Skipping invalid morphology definition for {morphology_id}"
            )

    return registry
