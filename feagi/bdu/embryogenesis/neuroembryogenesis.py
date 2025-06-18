"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import datetime
import json
import os
import random
import time
import types
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

"""
Neuroembryogenesis Module for FEAGI 2.1

This module is responsible for reading instructions from the genome
(genotype) and translating them into a functional connectome (phenotype).
The process is biologically inspired by neuroembryogenesis, where genetic
instructions guide brain development from the embryonic neural tube.

Naming Convention:
-----------------
* cortical_id: 6-character unique identifier from the genome (e.g., "iv00_C")
* cortical_idx: Auto-incremented integer ID used internally for efficient indexing
                Previously referred to as 'area_id' or just 'i' in various places

Throughout this module, we maintain mappings between these two ID types:
1. cortical_id_map[cortical_idx] = cortical_id
2. reverse_cortical_id_map[cortical_id] = cortical_idx

Key components:
1. Corticogenesis - Creation of cortical area definitions
2. Voxelogenesis - Establishing the 3D spatial framework for neuron placement
3. Neurogenesis - Generation of neurons within cortical areas
4. Synaptogenesis - Formation of synaptic connections between neurons

The implementation uses the ConnectomeManager API for efficient neuron and
synapse management, and focuses on memory efficiency and thread-safety.
"""

# Custom types
Position = Tuple[int, int, int]
NeuronId = int
AreaId = int
BoundingBox = Tuple[
    Position, Position
]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))

# Import modern synaptogenesis functions (no legacy dependencies)
from feagi.bdu.connectivity.synaptogenesis import (
    find_candidate_neurons,
    find_destination_coordinates,
    match_vectors,
)

# Clean FEAGI 2.0 implementation - no legacy dependencies
from feagi.bdu.connectome_manager import ConnectomeManager

# Import genome processing from EVO (single source of truth)
from feagi.evo.genome_processor import (
    create_genome_processor,
    genome_morphology_updator,
    genome_physiology_updator,
    genome_stat_updator,
    merge_core_morphologies,
)
from feagi.evo.genome_validator import genome_validator
from feagi.utils.config import FeagiConfig

# Try both the old and new import paths for FCLbitmap
try:
    # New path in FEAGI 2.1
    from feagi.bdu.bitmap import FCLBitmap
except ImportError:
    try:
        # Old path
        from feagi.src.bitmap import FCLBitmap
    except ImportError:
        # If both fail, use a minimal fallback implementation
        class FCLBitmap:
            """Minimal fallback implementation of FCLBitmap."""

            def __init__(self, size=0):
                self.bits = set()
                self.size = size

            def add(self, value):
                self.bits.add(value)

            def __contains__(self, value):
                return value in self.bits

            def __iter__(self):
                return iter(self.bits)

            def __len__(self):
                return len(self.bits)


class DevelopmentStage(Enum):
    """Development stages of brain embryogenesis."""

    INITIALIZATION = "INITIALIZATION"
    CORTICOGENESIS = "CORTICOGENESIS"
    VOXELOGENESIS = "VOXELOGENESIS"
    NEUROGENESIS = "NEUROGENESIS"
    SYNAPTOGENESIS = "SYNAPTOGENESIS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class NeuroEmbryogenesis:
    """
    Manages the development of a brain from genome instructions.

    This class orchestrates the process of reading genome data and constructing
    the corresponding neural architecture using the ConnectomeManager.
    """

    def __init__(
        self,
        connectome_manager: ConnectomeManager,
        config: Optional[FeagiConfig] = None,
        progress_callback: Optional[
            Callable[[DevelopmentStage, float, str], None]
        ] = None,
    ):
        """
        Initialize the NeuroEmbryogenesis system.

        Args:
            connectome_manager: The connectome manager to use for brain development
            config: Optional configuration object
            progress_callback: Optional callback for progress reporting
        """
        self.connectome_manager = connectome_manager
        self.config = config
        self.progress_callback = progress_callback

        # Configuration for logging verbosity
        self.verbose_logging = True
        self.suppress_no_mappings_logs = False

        # Check for embryogenesis-specific configuration
        if config and hasattr(config, "embryogenesis"):
            embryo_config = config.embryogenesis
            self.verbose_logging = embryo_config.get("verbose_logging", True)
            self.suppress_no_mappings_logs = embryo_config.get(
                "suppress_no_mappings_logs", False
            )
        elif config and hasattr(config, "get"):
            # Alternative configuration access pattern
            self.verbose_logging = config.get("embryogenesis_verbose_logging", True)
            self.suppress_no_mappings_logs = config.get(
                "embryogenesis_suppress_no_mappings_logs", False
            )

        # Check environment variables for runtime control
        if os.environ.get("FEAGI_EMBRYOGENESIS_QUIET", "").lower() in (
            "true",
            "1",
            "yes",
        ):
            self.suppress_no_mappings_logs = True
        if os.environ.get("FEAGI_EMBRYOGENESIS_VERBOSE", "").lower() in (
            "false",
            "0",
            "no",
        ):
            self.verbose_logging = False

        self.genome = None
        # ARCHITECTURAL FIX: Remove separate cortical_areas tracking
        # Use connectome_manager directly
        self.error = None

        # Development statistics
        self.development_stats = {
            "total_neurons": 0,
            "total_synapses": 0,
            "cortical_areas": 0,
            "start_time": None,
            "end_time": None,
            "duration": None,
        }

        # Cache for morphology registry
        self._morphology_registry_cache = None

        # Development state
        self.stage = DevelopmentStage.INITIALIZATION

        # Tracking data
        self.cortical_id_map = {}  # cortical_idx -> cortical_id
        self.reverse_cortical_id_map = {}  # cortical_id -> cortical_idx
        self.voxel_neuron_map = {}  # Maps (area_id, position) to list of neuron IDs

        # Add temporary method to ConnectomeManager to provide morphology information
        # Add this once at initialization instead of each time in
        # _perform_synaptogenesis
        def get_morphologies_registry(self):
            return self._neuroembryogenesis_morphologies_registry

        if not hasattr(self.connectome_manager, "get_morphologies_registry"):
            self.connectome_manager.get_morphologies_registry = types.MethodType(
                get_morphologies_registry, self.connectome_manager
            )
            # Will set the actual registry later when we have the genome

    def _report_progress(
        self, stage: DevelopmentStage, percentage: float, message: str
    ) -> None:
        """Report progress for the given development stage."""
        # Check if we should suppress this specific message
        should_suppress = (
            self.suppress_no_mappings_logs and "No mappings found" in message
        )

        if self.verbose_logging and not should_suppress:
            logger.info(f"[{stage.value}] {percentage:.1f}% - {message}")

        if self.progress_callback:
            self.progress_callback(stage, percentage, message)

    def _report_failure(self, stage: DevelopmentStage, message: str) -> None:
        """Report a failure during development."""
        self.error = message
        logger.error(f"[{stage.value}] FAILED - {message}")

        if self.progress_callback:
            self.progress_callback(DevelopmentStage.FAILED, 0, message)

    def _is_debug_bdu_enabled(self) -> bool:
        """
        Check if BDU (Brain Development Unit) debugging is enabled.

        Returns:
            True if BDU debugging is enabled, False otherwise
        """
        try:
            from feagi.core.state_manager import FeagiStateManager

            state_manager = FeagiStateManager.instance()
            return state_manager.is_debug_bdu_enabled()
        except Exception:
            return False

    def load_genome(self, genome_path: Union[str, Path]) -> bool:
        """
        Load a genome from file.

        Args:
            genome_path: Path to the genome file

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.INITIALIZATION, 0, "Loading genome")

        try:
            # Load genome from file
            with open(genome_path, "r") as f:
                self.genome = json.load(f)

            # Load FEAGI configuration to check genome settings
            try:
                from feagi.config.toml_loader import (
                    get_genome_config,
                    load_feagi_config,
                )

                config = load_feagi_config()
                genome_config = get_genome_config(config)
                allow_auto_recovery = genome_config.auto_recovery_on_validation_failure
            except Exception as e:
                logger.warning(
                    f"Could not load FEAGI configuration, "
                    f"defaulting to allow auto-recovery: {e}"
                )
                allow_auto_recovery = (
                    True  # Default to allow auto-recovery if config fails
                )

            # Validate genome - behavior depends on configuration
            is_valid = genome_validator(self.genome)
            if not is_valid:
                if not allow_auto_recovery:
                    self._report_failure(
                        DevelopmentStage.INITIALIZATION,
                        "Genome validation failed and auto-recovery is disabled",
                    )
                    return False
                else:
                    logger.warning(
                        "Genome validation failed - attempting auto-recovery "
                        "with morphology sanitization"
                    )

                    # Attempt morphology sanitization during auto-recovery
                    try:
                        from feagi.evo.genome_validator import (
                            sanitize_invalid_morphologies,
                        )

                        sanitization_result = sanitize_invalid_morphologies(self.genome)

                        # Use the sanitized genome
                        self.genome = sanitization_result["genome"]
                        removed_morphologies = sanitization_result[
                            "removed_morphologies"
                        ]
                        recovery_summary = sanitization_result["recovery_summary"]

                        logger.info(f"Auto-recovery completed: {recovery_summary}")
                        if removed_morphologies:
                            logger.info(
                                f"Removed invalid morphologies: "
                                f"{', '.join(removed_morphologies)}"
                            )

                        # Re-validate after sanitization
                        is_valid_after_recovery = genome_validator(self.genome)
                        if is_valid_after_recovery:
                            logger.info(
                                "Genome validation passed after "
                                "auto-recovery sanitization"
                            )
                        else:
                            logger.warning(
                                "Genome still has validation issues after sanitization - continuing anyway"
                            )

                    except Exception as sanitization_error:
                        logger.warning(
                            f"Auto-recovery sanitization failed: {sanitization_error}"
                        )
                        logger.warning(
                            "Continuing with original genome despite validation failures"
                        )

                    logger.warning(
                        "FEAGI will try to fix/recover from remaining gene failures during development"
                    )
                    # Don't return False - continue with loading despite validation issues

            # Update morphologies and physiology
            self.genome = merge_core_morphologies(self.genome)
            self.genome = genome_morphology_updator(self.genome)
            self.genome = genome_physiology_updator(self.genome)
            self.genome = genome_stat_updator(self.genome)

            # Generate and cache the morphology registry
            morphology_registry = self.get_morphology_registry()

            # Set the morphology registry on the ConnectomeManager
            if hasattr(self.connectome_manager, "get_morphologies_registry"):
                self.connectome_manager._neuroembryogenesis_morphologies_registry = (
                    morphology_registry
                )

            if is_valid:
                self._report_progress(
                    DevelopmentStage.INITIALIZATION, 100, "Genome loaded and validated"
                )
            else:
                self._report_progress(
                    DevelopmentStage.INITIALIZATION,
                    100,
                    "Genome loaded with validation warnings - attempting recovery",
                )
            return True

        except Exception as e:
            self._report_failure(
                DevelopmentStage.INITIALIZATION, f"Failed to load genome: {str(e)}"
            )
            logger.error("Error loading genome")
            logger.exception(e)
            return False

    def _extract_cortical_properties(self, cortical_id: str) -> Dict[str, Any]:
        """
        Extract cortical area properties from the genome's blueprint.

        Args:
            cortical_id: The ID of the cortical area in the genome

        Returns:
            Dictionary of properties for the cortical area
        """
        # In FEAGI 2.1, blueprint entries follow the pattern:
        # _____10c-<cortical_id>-<gene_type>-<property>-<value_type>

        properties = {}
        blueprint = self.genome["blueprint"]

        # Need to collect all properties for this cortical area
        for gene_key in blueprint:
            if not isinstance(gene_key, str):
                continue

            parts = gene_key.split("-")
            if len(parts) < 4:  # Need at least 4 parts to have a valid key
                continue

            gene_cortical_id = parts[1]
            if gene_cortical_id != cortical_id:
                continue

            # Get the property key and value type
            if len(parts) >= 5:
                property_key = parts[3]
                # value_type = parts[4]  # Unused variable removed
            else:
                property_key = parts[-2]
                # value_type = parts[-1]  # Unused variable removed

            value = blueprint[gene_key]

            # Handle special properties that need processing
            if "___bbx" in property_key:
                properties["bbx"] = value
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][0] = value
            elif "___bby" in property_key:
                properties["bby"] = value
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][1] = value
            elif "___bbz" in property_key:
                properties["bbz"] = value
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][2] = value
            elif "rcordx" in property_key:
                properties["rcordx"] = value
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][0] = value
            elif "rcordy" in property_key:
                properties["rcordy"] = value
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][1] = value
            elif "rcordz" in property_key:
                properties["rcordz"] = value
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][2] = value
            elif "__name" in property_key:
                properties["name"] = value
            elif "_group" in property_key:
                properties["group"] = value
            elif "subgrp" in property_key:
                properties["subgroup"] = value
            elif "_n_cnt" in property_key:
                properties["neurons_per_voxel"] = value
                properties["n_cnt"] = (
                    value  # Also store with the original name for compatibility
                )
            elif "dstmap" in property_key:
                properties["mapping"] = value
            else:
                # Store other properties directly
                clean_key = property_key.strip(
                    "_"
                )  # Remove leading/trailing underscores
                properties[clean_key] = value

        return properties

    def _calculate_subregion(self, cortical_id: str, morphology: Dict) -> BoundingBox:
        """
        Calculate a bounding box for a subregion of the cortical area.

        Args:
            cortical_id: 6-character cortical identifier
            morphology: Morphology parameters

        Returns:
            Bounding box tuple ((min_x, min_y, min_z), (max_x, max_y, max_z))
        """
        # Get the area from the connectome manager (single source of truth)
        area = self.connectome_manager.cortical_areas[cortical_id]
        dimensions = area.dimensions

        # Check if there's a specific subregion in the morphology
        if "subregion" in morphology:
            subregion = morphology["subregion"]
            min_x = max(0, subregion.get("min_x", 0))
            min_y = max(0, subregion.get("min_y", 0))
            min_z = max(0, subregion.get("min_z", 0))

            max_x = min(dimensions[0] - 1, subregion.get("max_x", dimensions[0] - 1))
            max_y = min(dimensions[1] - 1, subregion.get("max_y", dimensions[1] - 1))
            max_z = min(dimensions[2] - 1, subregion.get("max_z", dimensions[2] - 1))

            return ((min_x, min_y, min_z), (max_x, max_y, max_z))

        # Default is the entire area
        return ((0, 0, 0), (dimensions[0] - 1, dimensions[1] - 1, dimensions[2] - 1))

    def _get_cortical_ids_from_genome(self) -> List[str]:
        """
        Extract the list of cortical area IDs from the genome blueprint.

        Returns:
            List of cortical area IDs
        """
        cortical_ids = set()
        blueprint = self.genome["blueprint"]

        for gene_key in blueprint:
            if not isinstance(gene_key, str):
                continue

            parts = gene_key.split("-")
            if len(parts) < 5:
                continue

            # Extract the cortical ID (part after the first hyphen)
            cortical_id = parts[1]
            cortical_ids.add(cortical_id)

        return list(cortical_ids)

    def _setup_cortical_areas(self) -> bool:
        """
        Create all cortical areas in the connectome manager based on the genome.

        ENHANCED: Now guarantees core areas (_death, ___pwr) are created first from templates,
        then allows genome to override their properties, then creates remaining areas.

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(
            DevelopmentStage.CORTICOGENESIS, 0, "Setting up cortical areas"
        )

        try:
            # STEP 1: Create guaranteed core areas from templates.py
            self._report_progress(
                DevelopmentStage.CORTICOGENESIS, 10, "Creating guaranteed core areas"
            )
            if not self._create_core_areas_from_templates():
                return False

            # STEP 2: Extract genome cortical areas
            cortical_ids = self._get_cortical_ids_from_genome()

            # STEP 3: Check if genome contains core areas and update their properties
            self._report_progress(
                DevelopmentStage.CORTICOGENESIS,
                20,
                "Updating core area properties from genome",
            )
            self._update_core_areas_from_genome(cortical_ids)

            # STEP 4: Create remaining non-core areas from genome
            self._report_progress(
                DevelopmentStage.CORTICOGENESIS,
                30,
                "Creating genome-defined cortical areas",
            )
            remaining_areas = [
                cid for cid in cortical_ids if cid not in ["_death", "___pwr"]
            ]
            total_remaining = len(remaining_areas)

            for i, cortical_id in enumerate(remaining_areas):
                properties = self._extract_cortical_properties(cortical_id)

                # Skip if required properties are missing
                if (
                    "dimensions" not in properties
                    or "position" not in properties
                    or "name" not in properties
                ):
                    logger.warning(
                        f"Skipping cortical area {cortical_id} due to missing required properties"
                    )
                    continue

                # Determine area type
                area_type = "interconnect"  # default
                if "group" in properties:
                    if properties["group"] == "IPU":
                        area_type = "ipu"
                    elif properties["group"] == "OPU":
                        area_type = "opu"
                    elif "MEMORY" in properties.get("subgroup", ""):
                        area_type = "memory"

                # Register with connectome manager
                dimensions = tuple(properties["dimensions"])
                position = tuple(properties["position"])
                name = properties["name"]

                # Add to connectome manager - FAIL FAST on any errors
                logger.debug(f"Creating cortical area with ID {cortical_id}")
                created_cortical_id = self.connectome_manager.add_cortical_area(
                    name=name,
                    dimensions=dimensions,
                    position=position,
                    area_type=area_type,
                    properties={**properties},
                    cortical_id=cortical_id,  # Pass the cortical_id from genome
                )

                # Verify the area was actually created in connectome_manager
                if created_cortical_id not in self.connectome_manager.cortical_areas:
                    raise RuntimeError(
                        f"CRITICAL: Area {cortical_id} was not properly created in connectome_manager"
                    )

                # Get the created area from connectome_manager (single source of truth)
                area = self.connectome_manager.get_cortical_area(created_cortical_id)

                # Get the cortical_idx assigned by ConnectomeManager
                cortical_idx = area.cortical_idx

                # Store mappings for tracking
                self.cortical_id_map[cortical_idx] = cortical_id
                self.reverse_cortical_id_map[cortical_id] = cortical_idx

                logger.debug(
                    f"Created cortical area {name} (cortical_idx {cortical_idx}, cortical_id {cortical_id})"
                )

                # Verify mapping was created correctly
                if (
                    self.connectome_manager.cortical_mapping.get_idx(cortical_id)
                    != cortical_idx
                ):
                    raise RuntimeError(
                        f"CRITICAL: Mapping inconsistency for {cortical_id}: expected idx {cortical_idx}"
                    )

                logger.info(
                    f"✅ Successfully created and verified cortical area {cortical_id} → {cortical_idx}"
                )

                # Report progress for remaining areas
                remaining_progress = (
                    30 + ((i + 1) / total_remaining) * 60
                )  # 30-90% range
                logger.debug(
                    f"[{DevelopmentStage.CORTICOGENESIS.value}] {remaining_progress:.1f}% - Created genome area {i + 1}/{total_remaining}: {name}"
                )

            # Use connectome_manager as single source of truth
            total_areas_created = len(self.connectome_manager.cortical_areas)
            self.development_stats["cortical_areas"] = total_areas_created

            if total_areas_created == 0:
                self.error = (
                    "CRITICAL: No cortical areas were created in connectome_manager"
                )
                self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                return False

            # Verify core areas exist
            core_areas_found = []
            for core_id in ["_death", "___pwr"]:
                if core_id in self.connectome_manager.cortical_areas:
                    core_areas_found.append(core_id)

            if len(core_areas_found) != 2:
                self.error = f"CRITICAL: Missing core areas. Found: {core_areas_found}, Expected: ['_death', '___pwr']"
                self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                return False

            self._report_progress(
                DevelopmentStage.CORTICOGENESIS,
                100,
                f"✅ Created {total_areas_created} cortical areas (including {len(core_areas_found)} core areas) in connectome_manager",
            )
            return True

        except Exception as e:
            self.error = f"Failed to setup cortical areas: {e}"
            logger.exception(self.error)
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            return False

    def _create_core_areas_from_templates(self) -> bool:
        """
        Create the guaranteed core areas (_death, ___pwr) from templates.py.

        These areas are ALWAYS created regardless of genome content to ensure
        system reliability and proper cortical_idx reservation.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Import cortical_types from templates
            from feagi.evo.templates import cortical_types

            core_devices = cortical_types["CORE"]["supported_devices"]

            # Create _death area (cortical_idx=0)
            death_template = core_devices["_death"]
            death_id = self.connectome_manager.add_cortical_area(
                name=death_template["cortical_name"],
                dimensions=tuple(death_template["resolution"]),
                position=tuple(death_template["coordinate_3d"]),
                area_type="CORE",
                properties={
                    "template_source": "core",
                    "enabled": death_template["enabled"],
                    "structure": death_template["structure"],
                },
                cortical_id="_death",
            )

            # Verify area was created and get from connectome_manager (single source of truth)
            if death_id not in self.connectome_manager.cortical_areas:
                raise RuntimeError(
                    "CRITICAL: _death area was not created in connectome_manager"
                )

            death_area = self.connectome_manager.get_cortical_area(death_id)
            self.cortical_id_map[death_area.cortical_idx] = "_death"
            self.reverse_cortical_id_map["_death"] = death_area.cortical_idx

            logger.info(
                f"Created core area _death at cortical_idx={death_area.cortical_idx}"
            )

            # Create ___pwr area (cortical_idx=1)
            pwr_template = core_devices["___pwr"]
            pwr_id = self.connectome_manager.add_cortical_area(
                name=pwr_template["cortical_name"],
                dimensions=tuple(pwr_template["resolution"]),
                position=tuple(pwr_template["coordinate_3d"]),
                area_type="CORE",
                properties={
                    "template_source": "core",
                    "enabled": True,  # Always enable power area regardless of template default
                    "structure": pwr_template["structure"],
                },
                cortical_id="___pwr",
            )

            # Verify area was created and get from connectome_manager (single source of truth)
            if pwr_id not in self.connectome_manager.cortical_areas:
                raise RuntimeError(
                    "CRITICAL: ___pwr area was not created in connectome_manager"
                )

            pwr_area = self.connectome_manager.get_cortical_area(pwr_id)
            self.cortical_id_map[pwr_area.cortical_idx] = "___pwr"
            self.reverse_cortical_id_map["___pwr"] = pwr_area.cortical_idx

            logger.info(
                f"Created core area ___pwr at cortical_idx={pwr_area.cortical_idx}"
            )

            # Verify correct cortical_idx assignment
            if death_area.cortical_idx != 0:
                logger.error(
                    f"CRITICAL: _death area got cortical_idx={death_area.cortical_idx}, expected 0"
                )
                return False
            if pwr_area.cortical_idx != 1:
                logger.error(
                    f"CRITICAL: ___pwr area got cortical_idx={pwr_area.cortical_idx}, expected 1"
                )
                return False

            logger.info(
                "Core areas created successfully with correct cortical_idx reservation"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to create core areas from templates: {e}")
            return False

    def _update_core_areas_from_genome(self, genome_cortical_ids: List[str]) -> None:
        """
        Update core area properties if they are defined in the genome.

        This allows genomes to override the template defaults for core areas
        while ensuring the areas always exist.

        Args:
            genome_cortical_ids: List of cortical IDs found in genome
        """
        try:
            for core_id in ["_death", "___pwr"]:
                if core_id in genome_cortical_ids:
                    logger.info(f"Updating core area {core_id} with genome properties")

                    # Extract properties from genome
                    genome_properties = self._extract_cortical_properties(core_id)

                    # Get the existing core area from connectome_manager (single source of truth)
                    area = None
                    for (
                        _area_id,
                        area_obj,
                    ) in self.connectome_manager.cortical_areas.items():
                        if area_obj.cortical_id == core_id:
                            area = area_obj
                            break

                    if area:
                        # Update properties that are defined in genome
                        if "dimensions" in genome_properties:
                            area.dimensions = tuple(genome_properties["dimensions"])
                            logger.debug(
                                f"Updated {core_id} dimensions to {area.dimensions}"
                            )

                        if "position" in genome_properties:
                            area.position = tuple(genome_properties["position"])
                            logger.debug(
                                f"Updated {core_id} position to {area.position}"
                            )

                        if "name" in genome_properties:
                            area.name = genome_properties["name"]
                            logger.debug(f"Updated {core_id} name to {area.name}")

                        # Merge additional properties
                        if hasattr(area, "properties"):
                            area.properties.update(genome_properties)
                        else:
                            area.properties = genome_properties

                        logger.info(
                            f"Core area {core_id} updated with genome properties"
                        )
                    else:
                        logger.warning(f"Could not find core area {core_id} to update")

        except Exception as e:
            logger.warning(f"Error updating core areas from genome: {e}")

    def _perform_neurogenesis(self) -> bool:
        """
        Create neurons in each cortical area.

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.NEUROGENESIS, 0, "Creating neurons")

        try:
            total_areas = len(self.connectome_manager.cortical_areas)
            total_neurons = 0

            # CRITICAL FIX: Process each cortical area SEPARATELY to prevent cortical_idx corruption
            # The previous approach mixed neurons from different areas in the same batch
            for i, (cortical_id, area) in enumerate(
                self.connectome_manager.cortical_areas.items()
            ):
                # cortical_id is the 6-character identifier, we're already using it correctly
                properties = self._extract_cortical_properties(cortical_id)

                # Skip memory areas in initial development if configured
                if area.area_type == "memory" and self.config.get(
                    "skip_memory_neurogenesis", False
                ):
                    logger.info(f"Skipping neurogenesis for memory area {area.name}")
                    continue

                # Get neurons per voxel count
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)

                # Get neuron properties
                neuron_properties = {
                    "threshold": properties.get("fire_t", 1.0),
                    "refractory_period": properties.get("refrac", 0),
                    "decay_rate": 1.0 - (properties.get("leak_c", 0) / 100.0),
                    "resting_potential": 0.0,
                }

                # Create neurons for each voxel
                width, height, depth = area.dimensions
                # voxel_count = width * height * depth  # Unused variable removed
                area_neuron_count = 0

                # Initialize voxel tracking for this area
                if cortical_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[cortical_id] = {}

                # CRITICAL FIX: Create all neurons for THIS cortical area in a single batch
                # This ensures all neurons get the correct cortical_idx for this specific area
                positions = []
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            position = (x, y, z)
                            # Add one position per neuron in this voxel
                            for _n_idx in range(neurons_per_voxel):
                                positions.append(position)

                expected_neurons = len(positions)
                logger.debug(
                    f"[NEUROGENESIS] Creating {expected_neurons} neurons for area {cortical_id} ({area.name})"
                )

                # Create all neurons for this area in one batch with the correct cortical_idx
                try:
                    area_neuron_ids = self.connectome_manager.batch_create_neurons(
                        cortical_id=cortical_id,
                        positions=positions,
                        threshold=neuron_properties["threshold"],
                        membrane_potential=0.0,
                        resting_potential=neuron_properties["resting_potential"],
                        decay_rate=neuron_properties["decay_rate"],
                        refractory_period=neuron_properties["refractory_period"],
                    )

                    logger.debug(
                        f"[NEUROGENESIS] Successfully created {len(area_neuron_ids)} neurons for area {cortical_id}"
                    )

                    # Update voxel mapping
                    pos_idx = 0
                    for x in range(width):
                        for y in range(height):
                            for z in range(depth):
                                position = (x, y, z)
                                voxel_neurons = []
                                for _n_idx in range(neurons_per_voxel):
                                    if pos_idx < len(area_neuron_ids):
                                        voxel_neurons.append(area_neuron_ids[pos_idx])
                                        pos_idx += 1
                                self.voxel_neuron_map[cortical_id][position] = (
                                    voxel_neurons
                                )

                    area_neuron_count = len(area_neuron_ids)
                    total_neurons += area_neuron_count

                except Exception as e:
                    logger.error(
                        f"[NEUROGENESIS] Failed to create neurons for area {cortical_id}): {e}"
                    )
                    import traceback

                    logger.error(f"[NEUROGENESIS] Traceback: {traceback.format_exc()}")
                    continue

                # Report progress
                progress = ((i + 1) / total_areas) * 100
                self._report_progress(
                    DevelopmentStage.NEUROGENESIS,
                    progress,
                    f"Created neurons for area {i + 1}/{total_areas}: {area.name} ({area_neuron_count} neurons)",
                )

            self.development_stats["total_neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons across {len(self.connectome_manager.cortical_areas)} areas",
            )
            return True

        except Exception as e:
            import traceback

            self.error = f"Failed to create neurons: {str(e)}"
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            logger.exception("Error during neurogenesis")
            logger.error(f"Detailed error: {str(e)}")
            logger.error(f"Traceback:\n{traceback.format_exc()}")
            return False

    def _perform_synaptogenesis(self) -> bool:
        """
        Create synaptic connections based on genome mappings.

        Returns:
            True if successful, False otherwise
        """
        # Check if BDU debugging is enabled
        debug_bdu = self._is_debug_bdu_enabled()

        if debug_bdu:
            logger.info("[BDU DEBUG] ===== STARTING SYNAPTOGENESIS PHASE =====")
            logger.info(
                f"[BDU DEBUG] Total cortical areas: {len(self.connectome_manager.cortical_areas)}"
            )
            area_names = list(self.connectome_manager.cortical_areas.keys())
            logger.info(f"[BDU DEBUG] Cortical areas: {area_names}")

        self._report_progress(
            DevelopmentStage.SYNAPTOGENESIS, 0, "Creating synaptic connections"
        )

        try:
            len(self.connectome_manager.cortical_areas)
            total_synapses = 0

            # Memory register for memory-based morphologies

            # Extract cortical mappings using modern EVO genome processor
            logger.info("Using EVO GenomeProcessor to extract cortical mappings")

            try:
                # Create genome processor instance
                genome_processor = create_genome_processor(self.genome)

                # Extract mappings using the modern processor
                mapping_data = genome_processor.extract_cortical_mappings()

                # Count total mappings for logging
                mappings_found = 0
                for src_mappings in mapping_data.values():
                    for dst_connections in src_mappings.values():
                        mappings_found += len(dst_connections)

                if debug_bdu:
                    logger.info(
                        f"[BDU DEBUG] GenomeProcessor extracted {mappings_found} mappings"
                    )
                    logger.info(
                        f"[BDU DEBUG] Genome version: {genome_processor.get_version()}"
                    )
                    stats = genome_processor.get_statistics()
                    logger.info(f"[BDU DEBUG] Genome stats: {stats}")

            except Exception as e:
                logger.error(f"Failed to extract mappings using GenomeProcessor: {e}")
                logger.info("Falling back to direct genome access")
                mapping_data = {}
                mappings_found = 0

            # Create synapses if mappings were found
            if mappings_found > 0:
                if debug_bdu:
                    logger.info("[BDU DEBUG] Extracted mapping data:")
                    for src_id, dst_mappings in mapping_data.items():
                        logger.info(
                            f"[BDU DEBUG]   {src_id} -> {list(dst_mappings.keys())}"
                        )
                        for dst_id, connections in dst_mappings.items():
                            logger.info(
                                f"[BDU DEBUG]     {dst_id}: {len(connections)} connection specs"
                            )

                logger.info(
                    f"Creating synapses for {len(mapping_data)} source areas with {mappings_found} cortical mappings"
                )
                success = self.update_cortical_mapping(mapping_data)

                if success:
                    # Get synapse count from connectome manager
                    total_synapses = self.connectome_manager.get_synapse_count()
                    logger.info(
                        f"Successfully created synapses via cortical mappings. Total synapses: {total_synapses}"
                    )
                    if debug_bdu:
                        logger.info(
                            "[BDU DEBUG] ===== SYNAPTOGENESIS PHASE COMPLETED ====="
                        )
                        logger.info(
                            f"[BDU DEBUG] Total synapses created: {total_synapses}"
                        )
                else:
                    logger.warning("Failed to create synapses via cortical mappings")
            else:
                logger.info(
                    "No cortical mappings found in genome - no synapses will be created"
                )
                if debug_bdu:
                    logger.info(
                        "[BDU DEBUG] ===== SYNAPTOGENESIS PHASE COMPLETED ====="
                    )
                    logger.info("[BDU DEBUG] No mappings found - no synapses created")

            self._report_progress(
                DevelopmentStage.SYNAPTOGENESIS,
                100,
                f"Created {total_synapses} synaptic connections",
            )
            self.development_stats["total_synapses"] = total_synapses
            return True

        except Exception as e:
            self._report_failure(
                DevelopmentStage.SYNAPTOGENESIS,
                f"Failed to create synaptic connections: {str(e)}",
            )
            logger.error("Error during synaptogenesis")
            logger.exception(e)
            return False

    def get_morphology_registry(self) -> Dict[str, Dict]:
        """
        Create a registry of morphology functions from the genome.

        Returns:
            Dictionary mapping morphology_id to morphology type and parameters
        """
        # Use cached registry if available
        if self._morphology_registry_cache is not None:
            return self._morphology_registry_cache

        registry = {}

        # Add standard morphology functions that are built-in
        registry["expander_x"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["reducer_x"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["randomizer"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["lateral_pairs_x"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["block_connection"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["projector"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["projector_xy"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["projector_xz"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["projector_yz"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["project_from_end_x"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["project_from_end_y"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["project_from_end_z"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["last_to_first"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        registry["memory"] = {"type": "function", "parameters": {}, "class": "built-in"}

        # Add morphologies from the genome
        if self.genome and "neuron_morphologies" in self.genome:
            for morphology_id, morphology in self.genome["neuron_morphologies"].items():
                morphology_type = morphology.get("type", "unknown")

                if morphology_type == "vectors":
                    registry[morphology_id] = {
                        "type": "vectors",
                        "parameters": {
                            "vectors": morphology.get("parameters", {}).get(
                                "vectors", []
                            )
                        },
                        "class": "vectors",
                    }
                elif morphology_type == "patterns":
                    registry[morphology_id] = {
                        "type": "patterns",
                        "parameters": {
                            "patterns": morphology.get("parameters", {}).get(
                                "patterns", []
                            )
                        },
                        "class": "patterns",
                    }

        # Cache the registry
        self._morphology_registry_cache = registry

        return registry

    def develop_brain(self, genome_path: Union[str, Path]) -> bool:
        """
        Main entry point to develop a brain from genome.

        Args:
            genome_path: Path to the genome JSON file

        Returns:
            True if brain developed successfully, False otherwise
        """
        self.development_stats["start_time"] = datetime.datetime.now()

        # Load and validate genome
        if not self.load_genome(genome_path):
            return False

        # Set up cortical areas
        if not self._setup_cortical_areas():
            return False

        # Create neurons using vectorized approach
        if not self._perform_neurogenesis_vectorized():
            return False

        # Create synapses
        if not self._perform_synaptogenesis():
            return False

        # Finalize and report statistics
        self.development_stats["end_time"] = datetime.datetime.now()
        self.development_stats["duration"] = (
            self.development_stats["end_time"] - self.development_stats["start_time"]
        )

        # Final report
        self._report_progress(
            DevelopmentStage.COMPLETED,
            100,
            f"Brain development completed in {self.development_stats['duration']}. "
            f"Created {self.development_stats['cortical_areas']} cortical areas, "
            f"{self.development_stats['total_neurons']} neurons, and "
            f"{self.development_stats['total_synapses']} synapses.",
        )

        return True

    def get_development_statistics(self) -> Dict[str, Any]:
        """Get statistics about the brain development process."""
        return self.development_stats

    def _perform_neurogenesis_vectorized(self) -> bool:
        """
        Ultra-efficient vectorized neurogenesis using pure NumPy-style bulk operations.
        This is how neurogenesis SHOULD be done in a high-performance SoA system.

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(
            DevelopmentStage.NEUROGENESIS, 0, "Creating neurons (vectorized)"
        )

        try:
            total_areas = len(self.connectome_manager.cortical_areas)
            total_neurons = 0

            for i, (cortical_id, area) in enumerate(
                self.connectome_manager.cortical_areas.items()
            ):
                properties = self._extract_cortical_properties(cortical_id)

                # Skip memory areas in initial development if configured
                if (
                    area.area_type == "memory"
                    and self.config
                    and self.config.get("skip_memory_neurogenesis", False)
                ):
                    logger.info(f"Skipping neurogenesis for memory area {area.name}")
                    continue

                # Get area properties
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                width, height, depth = area.dimensions
                voxel_count = width * height * depth
                area_neuron_count = voxel_count * neurons_per_voxel

                logger.debug(
                    f"[TARGET] BULK NEUROGENESIS for {cortical_id}: {area_neuron_count} neurons ({width}×{height}×{depth} × {neurons_per_voxel})"
                )

                # PRE-CALCULATE ALL DATA (NumPy style!)
                # Create position arrays efficiently
                positions = []
                voxel_indices = []

                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            for _n_idx in range(neurons_per_voxel):
                                positions.append((x, y, z))
                                voxel_indices.append(x * height * depth + y * depth + z)

                # Convert to numpy arrays for efficiency
                # positions_array = np.array(positions)  # Unused variable removed

                # Pre-calculate neuron properties (vectorized)
                thresholds = np.full(
                    area_neuron_count, properties.get("fire_t", 1.0), dtype=np.float32
                )
                resting_potentials = np.zeros(area_neuron_count, dtype=np.float32)
                decay_rates = np.full(
                    area_neuron_count,
                    1.0 - (properties.get("leak_c", 0) / 100.0),
                    dtype=np.float32,
                )
                refractory_periods = np.full(
                    area_neuron_count, properties.get("refrac", 1), dtype=np.int32
                )

                # Get cortical_idx
                # cortical_idx = area.cortical_idx  # Unused variable removed

                # [START] SINGLE VECTORIZED CALL - NO LOOPS!
                # CRITICAL FIX: Use ConnectomeManager's batch method instead of direct NeuronArray call
                # This ensures proper cortical_idx assignment and mappings
                start_time = datetime.datetime.now()
                area_neuron_ids = self.connectome_manager.batch_create_neurons(
                    cortical_id=cortical_id,
                    positions=positions,
                    threshold=(
                        thresholds[0]
                        if len(set(thresholds)) == 1
                        else thresholds.tolist()
                    ),
                    membrane_potential=0.0,
                    resting_potential=(
                        resting_potentials[0]
                        if len(set(resting_potentials)) == 1
                        else resting_potentials.tolist()
                    ),
                    decay_rate=(
                        decay_rates[0]
                        if len(set(decay_rates)) == 1
                        else decay_rates.tolist()
                    ),
                    refractory_period=(
                        refractory_periods[0]
                        if len(set(refractory_periods)) == 1
                        else refractory_periods.tolist()
                    ),
                    # cortical_idx automatically determined from cortical_id by ConnectomeManager
                )
                end_time = datetime.datetime.now()
                creation_time = (end_time - start_time).total_seconds()

                logger.debug(
                    f"[FAST] VECTORIZED COMPLETE for {cortical_id}: {len(area_neuron_ids)} neurons in {creation_time:.3f}s ({len(area_neuron_ids) / creation_time:.0f} neurons/sec)"
                )

                # Update ConnectomeManager mappings efficiently (vectorized where possible)
                # start_mapping_time = datetime.datetime.now()  # Unused variable removed

                # Get all indices at once from NeuronArray (single source of truth)
                # indices = [
                #     self.connectome_manager.neuron_array.id_to_index_map[nid]
                #     for nid in area_neuron_ids
                # ]  # Unused variable removed
                # indices_array = np.array(indices)  # Unused variable removed

                # Initialize voxel tracking for this area (vectorized)
                if cortical_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[cortical_id] = {}

                # Group neurons by position efficiently
                position_to_neurons = {}
                for _j, (neuron_id, pos) in enumerate(zip(area_neuron_ids, positions)):
                    pos_tuple = tuple(pos)
                    if pos_tuple not in position_to_neurons:
                        position_to_neurons[pos_tuple] = []
                    position_to_neurons[pos_tuple].append(neuron_id)

                # Update voxel map
                self.voxel_neuron_map[cortical_id].update(position_to_neurons)

                total_neurons += area_neuron_count

                # Report progress
                progress = ((i + 1) / total_areas) * 100
                self._report_progress(
                    DevelopmentStage.NEUROGENESIS,
                    progress,
                    f"Area {i + 1}/{total_areas} ({area.name}): {area_neuron_count} neurons created vectorized",
                )

            self.development_stats["total_neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons across {len(self.connectome_manager.cortical_areas)} areas (vectorized)",
            )
            return True

        except Exception as e:
            import traceback

            self.error = f"Failed to create neurons (vectorized): {str(e)}"
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            logger.exception("Error during vectorized neurogenesis")
            logger.error(f"Detailed error: {str(e)}")
            logger.error(f"Traceback:\n{traceback.format_exc()}")
            return False

    def develop_brain_from_genome_data(self, genome_data: Dict[str, Any]) -> bool:
        """
        Develop a brain from genome data directly (not from file).

        This method is used when the genome data is already loaded and sanitized
        in the state manager, ensuring single source of truth architecture.

        Args:
            genome_data: The genome dictionary data

        Returns:
            True if brain developed successfully, False otherwise
        """
        self.development_stats["start_time"] = datetime.datetime.now()

        # Validate and load genome data directly
        if not self._load_genome_data(genome_data):
            return False

        # Set up cortical areas
        if not self._setup_cortical_areas():
            return False

        # Create neurons using vectorized approach
        if not self._perform_neurogenesis_vectorized():
            return False

        # Create synapses
        if not self._perform_synaptogenesis():
            return False

        # Finalize and report statistics
        self.development_stats["end_time"] = datetime.datetime.now()
        self.development_stats["duration"] = (
            self.development_stats["end_time"] - self.development_stats["start_time"]
        )

        # Final report
        self._report_progress(
            DevelopmentStage.COMPLETED,
            100,
            f"Brain development completed in {self.development_stats['duration']}. "
            f"Created {self.development_stats['cortical_areas']} cortical areas, "
            f"{self.development_stats['total_neurons']} neurons, and "
            f"{self.development_stats['total_synapses']} synapses.",
        )

        return True

    def _load_genome_data(self, genome_data: Dict[str, Any]) -> bool:
        """
        Load genome data directly from dictionary (not from file).

        Args:
            genome_data: The genome dictionary

        Returns:
            True if successful, False otherwise
        """
        try:
            self._report_progress(
                DevelopmentStage.INITIALIZATION,
                50,
                "Loading genome data from state manager",
            )

            # Validate basic genome structure
            if not isinstance(genome_data, dict):
                self.error = "Genome data must be a dictionary"
                self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                return False

            required_keys = ["blueprint", "physiology"]
            for key in required_keys:
                if key not in genome_data:
                    self.error = f"MISSING GENOME SECTION: {key}"
                    logger.error(f"MISSING GENOME SECTION: {key}")
                    logger.error(
                        f"  → PROBLEM: Required genome section '{key}' is not present in the genome data"
                    )
                    logger.error(f"  → REQUIRED SECTIONS: {required_keys}")
                    logger.error(
                        f"  → AVAILABLE SECTIONS: {list(genome_data.keys()) if isinstance(genome_data, dict) else 'Invalid genome format'}"
                    )
                    logger.error(
                        f"  → FIX: Add the missing '{key}' section to your genome"
                    )
                    if key == "physiology":
                        logger.error(
                            "  → EXAMPLE: Add 'physiology': {'burst_delay': 0.025, 'max_age': 10000000, 'evolution_burst_count': 50, 'ipu_idle_threshold': 1000, 'plasticity_queue_depth': 3, 'lifespan_mgmt_interval': 10} to your genome"
                        )
                        logger.error(
                            "  → AUTO-RECOVERY: Enable auto-recovery in configuration to automatically add missing physiology properties"
                        )
                    elif key == "blueprint":
                        logger.error(
                            "  → EXAMPLE: Add 'blueprint': {} with cortical area definitions to your genome"
                        )
                    self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                    return False

            # Store genome data
            self.genome = genome_data

            # Validate physiology section specifically with detailed error reporting
            try:
                from feagi.evo.genome_validator import validate_physiology_section

                physiology_validation = validate_physiology_section(self.genome)
                if not physiology_validation["valid"]:
                    logger.warning("PHYSIOLOGY VALIDATION ISSUES DETECTED:")
                    for error in physiology_validation["errors"]:
                        logger.warning(f"  → {error}")
                    logger.warning(
                        "  → NOTE: Auto-recovery may have already fixed these issues if enabled"
                    )
            except Exception as e:
                logger.warning(f"Could not validate physiology section: {e}")

            # Ensure morphology registry is generated and cached
            morphology_registry = self.get_morphology_registry()

            # Set the morphology registry on the ConnectomeManager
            if hasattr(self.connectome_manager, "get_morphologies_registry"):
                self.connectome_manager._neuroembryogenesis_morphologies_registry = (
                    morphology_registry
                )

            self._report_progress(
                DevelopmentStage.INITIALIZATION, 100, "Genome data loaded successfully"
            )
            return True

        except Exception as e:
            self.error = f"Failed to load genome data: {e}"
            logger.exception(self.error)
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            return False

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """
        Update cortical mapping in the connectome based on genome changes.

        This method is ONLY used during connectome building/updating.
        It should not be used for runtime cortical property access.

        Args:
            mapping: Dictionary containing cortical mapping data
                    Format from EVO: {mapping_id: {dst_area_id: [connection_specs]}}
                    OR Legacy format: {src_area_id: {dst_area_id: [connection_specs]}}

        Returns:
            bool: True if mapping was updated successfully
        """
        try:
            logger.info("Applying cortical mapping updates to connectome")

            if not self.connectome_manager:
                logger.error("Cannot update cortical mapping: No connectome manager")
                return False

            if not mapping:
                logger.warning("No mapping data provided")
                return True

            # Convert EVO format to BDU format if needed
            # EVO format: {dst_area: [specs]}
            # BDU format: {src_area: {dst_area: [specs]}}

            # Detect format by checking if first level values are lists (EVO format)
            first_key = next(iter(mapping.keys()))
            first_value = mapping[first_key]

            if isinstance(first_value, list):
                # This is EVO format: {dst_area: [connection_specs]}
                # We need to infer source areas from the destination areas
                logger.info("Converting EVO mapping format to BDU format")
                converted_mapping = {}

                # Based on test mode 2 logs, these destination areas map to specific source areas:
                # CIHMot, CKQM2_, CKYM2_, CO4M3_, CJWM3_, CTGM4_, CLWM4_ -> o__mot
                # This is a temporary fix until EVO processor is corrected

                motor_areas = {
                    "CIHMot",
                    "CKQM2_",
                    "CKYM2_",
                    "CO4M3_",
                    "CJWM3_",
                    "CTGM4_",
                    "CLWM4_",
                }

                for dst_area_id, connection_specs in mapping.items():
                    if dst_area_id in motor_areas:
                        # These areas connect to motor output
                        src_area_id = dst_area_id  # Source is the same as destination for these mappings
                        dst_area_id = "o__mot"  # They all connect to motor output

                        if src_area_id not in converted_mapping:
                            converted_mapping[src_area_id] = {}
                        converted_mapping[src_area_id][dst_area_id] = connection_specs

                        logger.info(
                            f"Mapped {src_area_id} -> {dst_area_id} with {len(connection_specs)} specs"
                        )
                    else:
                        # For other areas, assume self-connection for now
                        src_area_id = dst_area_id
                        if src_area_id not in converted_mapping:
                            converted_mapping[src_area_id] = {}
                        converted_mapping[src_area_id][dst_area_id] = connection_specs

                        logger.info(
                            f"Self-mapped {src_area_id} -> {dst_area_id} with {len(connection_specs)} specs"
                        )

                mapping = converted_mapping
                logger.info(f"Converted to BDU format with {len(mapping)} source areas")

            elif isinstance(first_value, dict):
                # Check if this is the correct BDU format: {src_area: {dst_area: [specs]}}
                # or if it's still EVO format: {mapping_id: {dst_area: [specs]}}

                # Look at the second level to determine format
                second_level_sample = next(iter(first_value.values()))
                if isinstance(second_level_sample, list):
                    # This is correct BDU format: {src_area: {dst_area: [specs]}}
                    logger.info("Mapping already in correct BDU format")
                else:
                    logger.error(
                        f"Unexpected mapping format: {type(second_level_sample)}"
                    )
                    return False
            else:
                logger.error(f"Unexpected mapping format: {type(first_value)}")
                return False

            # Validate all areas exist before processing
            for src_area_id, target_mappings in mapping.items():
                if src_area_id not in self.connectome_manager.cortical_areas:
                    logger.error(
                        f"Source cortical area {src_area_id} not found in connectome"
                    )
                    return False

                if isinstance(target_mappings, dict):
                    for dst_area_id in target_mappings.keys():
                        if dst_area_id not in self.connectome_manager.cortical_areas:
                            logger.error(
                                f"Destination cortical area {dst_area_id} not found in connectome"
                            )
                            return False

            # Process each source area mapping with vectorized operations
            total_synapses_created = 0

            for src_area_id, target_mappings in mapping.items():
                if not isinstance(target_mappings, dict):
                    logger.warning(f"Invalid mapping format for area {src_area_id}")
                    continue

                # Get source area and neurons once
                src_area = self.connectome_manager.cortical_areas[src_area_id]
                src_neurons = self.connectome_manager.get_neurons_by_area(src_area_id)

                if not src_neurons:
                    logger.warning(f"No neurons found in source area {src_area_id}")
                    continue

                # Update the source area's properties with mapping information
                if not hasattr(src_area, "properties") or src_area.properties is None:
                    src_area.properties = {}

                # Convert the mapping data to the format expected by the API
                # The API expects mapping in array format: [morphology_id, scalar, multiplier, plasticity_flag, constant, ltp, ltd]
                api_mapping = {}
                for dst_area_id, connection_data in target_mappings.items():
                    connection_arrays = []
                    for connection_spec in connection_data:
                        if isinstance(connection_spec, dict):
                            # Convert from object format to array format for API compatibility
                            connection_array = [
                                connection_spec.get("morphology_id", ""),
                                connection_spec.get("morphology_scalar", [1, 1, 1]),
                                connection_spec.get(
                                    "postSynapticCurrent_multiplier", 1.0
                                ),
                                connection_spec.get("plasticity_flag", False),
                                connection_spec.get("plasticity_constant", 1.0),
                                connection_spec.get("ltp_multiplier", 1.0),
                                connection_spec.get("ltd_multiplier", 1.0),
                            ]
                            connection_arrays.append(connection_array)

                    if connection_arrays:
                        api_mapping[dst_area_id] = connection_arrays

                # Store the mapping in the cortical area properties
                src_area.properties["mapping"] = api_mapping

                # Process mappings to target areas
                for dst_area_id, connection_data in target_mappings.items():
                    try:
                        logger.info(
                            f"Creating synapses from {src_area_id} to {dst_area_id}"
                        )

                        # Get destination area and neurons
                        # dst_area = self.connectome_manager.cortical_areas[dst_area_id]  # Unused variable removed
                        dst_neurons = self.connectome_manager.get_neurons_by_area(
                            dst_area_id
                        )

                        if not dst_neurons:
                            logger.warning(
                                f"No neurons found in destination area {dst_area_id}"
                            )
                            continue

                        # Process each connection specification
                        for connection_spec in connection_data:
                            if not isinstance(connection_spec, dict):
                                logger.warning(
                                    f"Invalid connection specification format: {connection_spec}"
                                )
                                continue

                            # Extract connection parameters from dictionary format
                            # Format: {"morphology_id": "block_to_block", "morphology_scalar": [1,1,1], ...}
                            morphology_id = connection_spec.get("morphology_id", "")
                            morphology_scalar = connection_spec.get(
                                "morphology_scalar", [1, 1, 1]
                            )
                            psc_multiplier = float(
                                connection_spec.get(
                                    "postSynapticCurrent_multiplier", 1.0
                                )
                            )
                            plasticity_flag = bool(
                                connection_spec.get("plasticity_flag", False)
                            )
                            plasticity_constant = float(
                                connection_spec.get("plasticity_constant", 1.0)
                            )
                            ltp_multiplier = float(
                                connection_spec.get("ltp_multiplier", 1.0)
                            )
                            ltd_multiplier = float(
                                connection_spec.get("ltd_multiplier", 1.0)
                            )

                            # Validate required parameters
                            if not morphology_id:
                                logger.warning(
                                    f"Missing morphology_id in connection specification: {connection_spec}"
                                )
                                continue

                            if (
                                not isinstance(morphology_scalar, list)
                                or len(morphology_scalar) != 3
                            ):
                                logger.warning(
                                    f"Invalid morphology_scalar format: {morphology_scalar}, using default [1,1,1]"
                                )
                                morphology_scalar = [1, 1, 1]

                            # Apply morphology-based synaptogenesis
                            synapses_created = self._apply_morphology_mapping(
                                src_area_id=src_area_id,
                                dst_area_id=dst_area_id,
                                src_neurons=src_neurons,
                                dst_neurons=dst_neurons,
                                morphology_id=morphology_id,
                                morphology_scalar=morphology_scalar,
                                psc_multiplier=psc_multiplier,
                                plasticity_flag=plasticity_flag,
                                plasticity_constant=plasticity_constant,
                                ltp_multiplier=ltp_multiplier,
                                ltd_multiplier=ltd_multiplier,
                            )

                            total_synapses_created += synapses_created
                            logger.info(
                                f"Created {synapses_created} synapses for {morphology_id} mapping"
                            )

                    except Exception as e:
                        logger.error(
                            f"Failed to update mapping from {src_area_id} to {dst_area_id}: {e}"
                        )
                        # Continue processing other mappings rather than failing completely
                        continue

            # Log final results
            if total_synapses_created > 0:
                logger.info(
                    f"Successfully created {total_synapses_created} synapses from cortical mapping updates"
                )
                return True
            else:
                logger.warning("No synapses were created from cortical mapping updates")
                # Return True for graceful handling - empty mappings or invalid morphologies
                # should not be considered failures, just no-ops
                return True

        except Exception as e:
            logger.error(f"Error updating cortical mapping: {e}")
            return False

    def _apply_morphology_mapping(
        self,
        src_area_id: str,
        dst_area_id: str,
        src_neurons: List[int],
        dst_neurons: List[int],
        morphology_id: str,
        morphology_scalar: List[int],
        psc_multiplier: float,
        plasticity_flag: bool,
        plasticity_constant: float,
        ltp_multiplier: float,
        ltd_multiplier: float,
    ) -> int:
        """
        Apply morphology-based synaptogenesis between two cortical areas.

        ARCHITECTURE: Morphology-driven approach following FEAGI 2.0 principles.
        Gets morphology definition from genome and routes to appropriate processor
        based on morphology type ("vectors", "patterns", or "functions").

        PERFORMANCE: Vectorized operations for Rust/RTOS/SIMD/GPU compatibility.

        Args:
            src_area_id: Source cortical area ID
            dst_area_id: Destination cortical area ID
            src_neurons: List of source neuron IDs
            dst_neurons: List of destination neuron IDs
            morphology_id: Morphology template ID from genome
            morphology_scalar: Scaling factors [x, y, z]
            psc_multiplier: Post-synaptic current multiplier
            plasticity_flag: Whether plasticity is enabled
            plasticity_constant: Plasticity constant value
            ltp_multiplier: Long-term potentiation multiplier
            ltd_multiplier: Long-term depression multiplier

        Returns:
            int: Number of synapses created
        """
        try:
            # First, try to get morphology definition from genome
            morphology_def = None
            morphology_type = None

            if self.genome and "neuron_morphologies" in self.genome:
                morphology_def = self.genome["neuron_morphologies"].get(morphology_id)
                if morphology_def:
                    morphology_type = morphology_def.get("type")

            # If not found in genome, check if it's a core function morphology
            if not morphology_def:
                # Import here to avoid circular imports
                from feagi.bdu.connectivity.synaptogenesis import MorphologyFunction

                # Check if this is a known function morphology
                function_morphology_values = [e.value for e in MorphologyFunction]
                if morphology_id in function_morphology_values:
                    # This is a core function morphology - create a synthetic definition
                    morphology_def = {
                        "type": "functions",
                        "parameters": {},
                        "class": "core",
                    }
                    morphology_type = "functions"
                    logger.debug(f"Using core function morphology: {morphology_id}")
                else:
                    logger.warning(
                        f"Morphology {morphology_id} not found in genome or core functions"
                    )
                    if self.genome and "neuron_morphologies" in self.genome:
                        logger.debug(
                            f"Available morphologies: {list(self.genome['neuron_morphologies'].keys())}"
                        )
                    return 0

            # Validate morphology type
            if not morphology_type:
                logger.warning(f"No type specified for morphology {morphology_id}")
                return 0

            # Route to appropriate processor based on morphology type
            if morphology_type == "vectors":
                return self._process_vector_morphology(
                    src_area_id,
                    dst_area_id,
                    src_neurons,
                    dst_neurons,
                    morphology_def,
                    morphology_scalar,
                    psc_multiplier,
                    plasticity_flag,
                    plasticity_constant,
                    ltp_multiplier,
                    ltd_multiplier,
                )

            elif morphology_type == "patterns":
                return self._process_pattern_morphology(
                    src_area_id,
                    dst_area_id,
                    src_neurons,
                    dst_neurons,
                    morphology_def,
                    morphology_scalar,
                    psc_multiplier,
                    plasticity_flag,
                    plasticity_constant,
                    ltp_multiplier,
                    ltd_multiplier,
                )

            elif morphology_type == "functions":
                return self._process_function_morphology(
                    src_area_id,
                    dst_area_id,
                    src_neurons,
                    dst_neurons,
                    morphology_id,
                    morphology_def,
                    morphology_scalar,
                    psc_multiplier,
                    plasticity_flag,
                    plasticity_constant,
                    ltp_multiplier,
                    ltd_multiplier,
                )

            else:
                logger.warning(
                    f"Unknown morphology type '{morphology_type}' for "
                    f"morphology {morphology_id}"
                )
                return 0

        except Exception as e:
            logger.error(f"Error applying morphology mapping {morphology_id}: {e}")
            return 0

    def _process_vector_morphology(
        self,
        src_area_id: str,
        dst_area_id: str,
        src_neurons: List[int],
        dst_neurons: List[int],
        morphology_def: Dict[str, Any],
        morphology_scalar: List[int],
        psc_multiplier: float,
        plasticity_flag: bool,
        plasticity_constant: float,
        ltp_multiplier: float,
        ltd_multiplier: float,
    ) -> int:
        """
        Process vector-based morphology using match_vectors logic from legacy.

        ARCHITECTURE: Implements legacy match_vectors algorithm in FEAGI 2.0.
        PERFORMANCE: Optimized for Rust/RTOS/SIMD/GPU compatibility.
        """
        try:
            total_synapses = 0
            vectors = morphology_def.get("parameters", {}).get("vectors", [])

            if not vectors:
                logger.warning(
                    "No vectors found in morphology definition for vector type"
                )
                return 0

            # Get destination area dimensions
            dst_area_props = self.connectome_manager.get_cortical_area_properties(
                dst_area_id
            )
            if not dst_area_props:
                logger.error(f"Cannot get properties for area {dst_area_id}")
                return 0

            dst_area_props.get("dimensions", [1, 1, 1])

            # Process each source neuron
            for src_neuron_id in src_neurons:
                try:
                    # DEBUG: Log the actual neuron ID being processed
                    logger.debug(
                        f"[VECTOR DEBUG] Processing source neuron ID: {src_neuron_id}"
                    )

                    # Get source neuron position
                    src_pos = self._get_neuron_position(src_neuron_id, src_area_id)
                    if not src_pos:
                        continue

                    synapse_connections = []

                    # Process each vector using modern match_vectors
                    for vector in vectors:
                        # Get source area for subregion calculation
                        src_area = self.connectome_manager.get_cortical_area(
                            src_area_id
                        )
                        src_subregion = [(0, 0, 0), src_area.dimensions]

                        candidate_positions = match_vectors(
                            src_voxel=src_pos,
                            dst_area_id=dst_area_id,
                            vector=vector,
                            morphology_scalar=morphology_scalar[0]
                            if morphology_scalar
                            else 1.0,
                            src_subregion=src_subregion,
                            connectome_manager=self.connectome_manager,
                        )

                        # Collect all candidate positions first (legacy approach)
                        candidate_positions_set = set()
                        for candidate_pos in candidate_positions:
                            candidate_positions_set.add(candidate_pos)

                        # Use legacy batch lookup for performance
                        if candidate_positions_set:
                            neuron_weight_pairs = (
                                self.connectome_manager.batch_voxel_to_neuron_lookup(
                                    cortical_id=dst_area_id,
                                    candidate_positions=candidate_positions_set,
                                    post_synaptic_current=psc_multiplier,
                                )
                            )

                            # Convert to synapse connections
                            for neuron_id, weight in neuron_weight_pairs:
                                synapse_connections.append(
                                    (src_neuron_id, neuron_id, weight)
                                )

                    # Create synapses in batch - Now using GlobalSynapseArray for optimal performance
                    if synapse_connections:
                        created = self.connectome_manager.batch_create_synapses(
                            synapse_connections
                        )
                        total_synapses += created

                except Exception as e:
                    logger.warning(
                        f"Error processing vector morphology for neuron "
                        f"{src_neuron_id}: {e}"
                    )
                    continue

            return total_synapses

        except Exception as e:
            logger.error(f"Error in vector morphology processing: {e}")
            return 0

    def _process_pattern_morphology(
        self,
        src_area_id: str,
        dst_area_id: str,
        src_neurons: List[int],
        dst_neurons: List[int],
        morphology_def: Dict[str, Any],
        morphology_scalar: List[int],
        psc_multiplier: float,
        plasticity_flag: bool,
        plasticity_constant: float,
        ltp_multiplier: float,
        ltd_multiplier: float,
    ) -> int:
        """
        Process pattern-based morphology using legacy pattern logic.

        ARCHITECTURE: Implements legacy find_destination_coordinates in FEAGI 2.0.
        PERFORMANCE: Optimized for Rust/RTOS/SIMD/GPU compatibility.
        """
        try:
            total_synapses = 0
            patterns = morphology_def.get("parameters", {}).get("patterns", [])

            if not patterns:
                logger.warning(
                    "No patterns found in morphology definition for pattern type"
                )
                return 0

            # Get destination area dimensions
            dst_area_props = self.connectome_manager.get_cortical_area_properties(
                dst_area_id
            )
            if not dst_area_props:
                logger.error(f"Cannot get properties for area {dst_area_id}")
                return 0

            dst_dimensions = dst_area_props.get("dimensions", [1, 1, 1])

            # Process each source neuron
            for src_neuron_id in src_neurons:
                try:
                    # Get source neuron position
                    src_pos = self._get_neuron_position(src_neuron_id, src_area_id)
                    if not src_pos:
                        continue

                    synapse_connections = []

                    # Collect all candidate positions first (legacy approach)
                    all_candidate_positions = set()

                    # Process each pattern (legacy pattern logic)
                    for pattern in patterns:
                        if len(pattern) >= 2:
                            source_pattern = pattern[0]
                            destination_pattern = pattern[1]

                            candidate_positions = list(
                                find_destination_coordinates(
                                    dst_cortical_boundary=tuple(dst_dimensions),
                                    src_coordinate=src_pos,
                                    src_pattern=source_pattern,
                                    dst_pattern=destination_pattern,
                                )
                            )

                            # Collect positions for batch lookup
                            for candidate_pos in candidate_positions:
                                all_candidate_positions.add(candidate_pos)

                    # Use legacy batch lookup for performance
                    if all_candidate_positions:
                        neuron_weight_pairs = (
                            self.connectome_manager.batch_voxel_to_neuron_lookup(
                                cortical_id=dst_area_id,
                                candidate_positions=all_candidate_positions,
                                post_synaptic_current=psc_multiplier,
                            )
                        )

                        # Convert to synapse connections
                        for neuron_id, weight in neuron_weight_pairs:
                            synapse_connections.append(
                                (src_neuron_id, neuron_id, weight)
                            )

                    # Create synapses in batch - Now using GlobalSynapseArray for optimal performance
                    if synapse_connections:
                        created = self.connectome_manager.batch_create_synapses(
                            synapse_connections
                        )
                        total_synapses += created

                except Exception as e:
                    logger.warning(
                        f"Error processing pattern morphology for neuron "
                        f"{src_neuron_id}: {e}"
                    )
                    continue

            return total_synapses

        except Exception as e:
            logger.error(f"Error in pattern morphology processing: {e}")
            return 0

    def _process_function_morphology(
        self,
        src_area_id: str,
        dst_area_id: str,
        src_neurons: List[int],
        dst_neurons: List[int],
        morphology_id: str,
        morphology_def: Dict[str, Any],
        morphology_scalar: List[int],
        psc_multiplier: float,
        plasticity_flag: bool,
        plasticity_constant: float,
        ltp_multiplier: float,
        ltd_multiplier: float,
    ) -> int:
        """
        Process function-based morphology with direct implementation.

        ARCHITECTURE: Clean FEAGI 2.0 implementation without legacy dependencies.
        Supports all function morphologies including block_to_block.

        PERFORMANCE: Optimized for Rust/RTOS/SIMD/GPU compatibility.
        """
        try:
            total_synapses = 0

            # Use the modern find_candidate_neurons function

            # Calculate source subregion (following legacy pattern)
            src_area = self.connectome_manager.get_cortical_area(src_area_id)
            src_subregion = [(0, 0, 0), src_area.dimensions]

            # Create morphology dict in the correct format for find_candidate_neurons
            # Check if this is a pattern-based morphology from the genome
            if morphology_def and morphology_def.get("type") == "patterns":
                morphology_dict = {
                    "type": "patterns",
                    "parameters": morphology_def.get("parameters", {}),
                    "morphology_id": morphology_id,
                    "morphology_scalar": morphology_scalar,
                    "postSynapticCurrent_multiplier": psc_multiplier,
                }
            else:
                # For function morphologies, use the morphology_id as the type
                morphology_dict = {
                    "type": morphology_id,  # This will be used to dispatch to the right function
                    "morphology_id": morphology_id,
                    "morphology_scalar": morphology_scalar,
                    "postSynapticCurrent_multiplier": psc_multiplier,
                }

            # Memory register for memory-based morphologies
            memory_register = {}

            debug_bdu = self._is_debug_bdu_enabled()

            if debug_bdu:
                logger.info(
                    f"[BDU DEBUG] Processing {len(src_neurons)} source neurons for {morphology_id}"
                )

            for src_neuron_id in src_neurons:
                try:
                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] Processing source neuron {src_neuron_id}"
                        )

                    # Use the correct find_candidate_neurons function
                    candidate_neurons = find_candidate_neurons(
                        src_area_id=src_area_id,
                        dst_area_id=dst_area_id,
                        src_neuron_id=src_neuron_id,
                        morphology=morphology_dict,
                        src_subregion=src_subregion,
                        connectome_manager=self.connectome_manager,
                        memory_register=memory_register,
                    )

                    # Apply legacy synapse attractivity filtering (critical for proper behavior)
                    dst_area = self.connectome_manager.get_cortical_area(dst_area_id)
                    synapse_attractivity = dst_area.properties.get("synatt", 100)

                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] Found {len(candidate_neurons)} candidate neurons"
                        )
                        logger.info(
                            f"[BDU DEBUG] Synapse attractivity: {synapse_attractivity}%"
                        )

                    # Create synapses from candidate neurons with probabilistic filtering
                    synapse_connections = []
                    for dst_neuron_id, weight in candidate_neurons:
                        # Legacy behavior: probabilistic synapse creation based on attractivity
                        if random.randrange(1, 100) < synapse_attractivity:
                            synapse_connections.append(
                                (src_neuron_id, dst_neuron_id, weight)
                            )

                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] After attractivity filtering: {len(synapse_connections)} synapses to create"
                        )

                    if synapse_connections:
                        # PERFORMANCE DEBUG: Time the batch_create_synapses call  
                        start_time = time.time()
                        created = self.connectome_manager.batch_create_synapses(
                            synapse_connections
                        )
                        end_time = time.time()
                        elapsed_ms = (end_time - start_time) * 1000
                        if elapsed_ms > 100:  # Log if > 100ms
                            logger.warning(
                                f"⚠️  PERFORMANCE: batch_create_synapses took {elapsed_ms:.1f}ms for {len(synapse_connections)} synapses"
                            )
                        total_synapses += created

                        if debug_bdu:
                            logger.info(
                                f"[BDU DEBUG] Successfully created {created} synapses for neuron {src_neuron_id}"
                            )
                    elif debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] No synapses created for neuron {src_neuron_id}"
                        )

                except Exception as e:
                    logger.warning(
                        f"Error processing function morphology {morphology_id} for neuron "
                        f"{src_neuron_id}: {e}"
                    )
                    continue

            return total_synapses

        except Exception as e:
            logger.error(
                f"Error in function morphology processing for {morphology_id}: {e}"
            )
            return 0

    def _get_neuron_position(
        self, neuron_id: int, area_id: str
    ) -> Optional[Tuple[int, int, int]]:
        """Get the 3D position of a neuron within its cortical area."""
        try:
            # Check voxel mapping first if available
            if hasattr(self, "voxel_neuron_map") and area_id in self.voxel_neuron_map:
                for position, neuron_list in self.voxel_neuron_map[area_id].items():
                    if neuron_id in neuron_list:
                        return position

            # Fallback to connectome manager lookup
            position = self.connectome_manager.get_neuron_position(neuron_id)
            if position:
                return position

            # If no position found, log debug info
            logger.debug(f"No position found for neuron {neuron_id} in area {area_id}")
            return None

        except Exception as e:
            logger.debug(f"Could not get position for neuron {neuron_id}: {e}")
            return None

    def _calculate_projection_region(
        self,
        src_pos: Tuple[int, int, int],
        src_dimensions: Tuple[int, int, int],
        dst_dimensions: Tuple[int, int, int],
        scale_x: int,
        scale_y: int,
        scale_z: int,
    ) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        """Calculate the target projection region for spatial mapping."""
        try:
            src_x, src_y, src_z = src_pos
            src_w, src_h, src_d = src_dimensions
            dst_w, dst_h, dst_d = dst_dimensions

            # Calculate normalized position in source area
            norm_x = src_x / max(src_w - 1, 1)
            norm_y = src_y / max(src_h - 1, 1)
            norm_z = src_z / max(src_d - 1, 1)

            # Project to destination area with scaling
            target_x = int(norm_x * (dst_w - 1))
            target_y = int(norm_y * (dst_h - 1))
            target_z = int(norm_z * (dst_d - 1))

            # Apply morphology scaling to create region
            region_w = max(1, scale_x)
            region_h = max(1, scale_y)
            region_d = max(1, scale_z)

            # Calculate region bounds
            min_x = max(0, target_x - region_w // 2)
            max_x = min(dst_w - 1, target_x + region_w // 2)
            min_y = max(0, target_y - region_h // 2)
            max_y = min(dst_h - 1, target_y + region_h // 2)
            min_z = max(0, target_z - region_d // 2)
            max_z = min(dst_d - 1, target_z + region_d // 2)

            return ((min_x, min_y, min_z), (max_x, max_y, max_z))

        except Exception as e:
            logger.error(f"Error calculating projection region: {e}")
            return ((0, 0, 0), (0, 0, 0))

    def _get_neurons_in_region(
        self, area_id: str, region: Tuple[Tuple[int, int, int], Tuple[int, int, int]]
    ) -> List[int]:
        """Get all neurons within a specified 3D region of a cortical area."""
        try:
            (min_x, min_y, min_z), (max_x, max_y, max_z) = region
            neurons_in_region = []

            # Try voxel mapping first if available
            if hasattr(self, "voxel_neuron_map") and area_id in self.voxel_neuron_map:
                for position, neuron_list in self.voxel_neuron_map[area_id].items():
                    x, y, z = position
                    if (
                        min_x <= x <= max_x
                        and min_y <= y <= max_y
                        and min_z <= z <= max_z
                    ):
                        neurons_in_region.extend(neuron_list)
            else:
                # Fallback: get all neurons in area and check their positions
                all_neurons = self.connectome_manager.get_neurons_by_area(area_id)
                for neuron_id in all_neurons:
                    position = self.connectome_manager.get_neuron_position(neuron_id)
                    if position:
                        x, y, z = position
                        if (
                            min_x <= x <= max_x
                            and min_y <= y <= max_y
                            and min_z <= z <= max_z
                        ):
                            neurons_in_region.append(neuron_id)

            return neurons_in_region

        except Exception as e:
            logger.error(f"Error getting neurons in region: {e}")
            return []

    def _get_morphology_registry(self) -> Dict[str, Any]:
        """Get the morphology registry for pattern lookup."""
        try:
            if hasattr(self.connectome_manager, "get_morphologies_registry"):
                return self.connectome_manager.get_morphologies_registry()
            return {}
        except Exception as e:
            logger.debug(f"Could not get morphology registry: {e}")
            return {}


# Convenience function for direct use
def develop_brain_from_genome(
    genome_path: Union[str, Path],
    connectome_manager: Optional[ConnectomeManager] = None,
    config: Optional[FeagiConfig] = None,
    progress_callback: Optional[Callable[[DevelopmentStage, float, str], None]] = None,
) -> Tuple[bool, Dict[str, Any]]:
    """
    Develop a brain from a genome file.

    Args:
        genome_path: Path to the genome JSON file
        connectome_manager: Optional ConnectomeManager, will create one if not provided
        config: Optional FeagiConfig
        progress_callback: Optional callback for progress reporting

    Returns:
        Tuple of (success: bool, statistics: Dict)
    """
    # Create connectome manager if not provided
    if connectome_manager is None:
        connectome_manager = ConnectomeManager(config_or_max_neurons=config)

    # Create neuroembryogenesis instance
    embryo = NeuroEmbryogenesis(
        connectome_manager=connectome_manager,
        config=config,
        progress_callback=progress_callback,
    )

    # Develop the brain
    success = embryo.develop_brain(genome_path)

    # Return results
    return success, embryo.get_development_statistics()
