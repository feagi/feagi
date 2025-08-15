"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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
* cortical_id: 6-character unique identifier from the genome (e.g., "iic400")
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
)

# Clean FEAGI 2.0 implementation - no legacy dependencies
from feagi.bdu.connectome_manager import ConnectomeManager

# Import genome processing from EVO (single source of truth)
from feagi.evo.genome_processor import (
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
    """Manages the development of a brain from genome instructions.

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
        """Initialize the NeuroEmbryogenesis system.

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
            self.verbose_logging = config.get(
                "embryogenesis_verbose_logging", True
            )
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
        self.voxel_neuron_map = (
            {}
        )  # Maps (area_id, position) to list of neuron IDs

        #  Add temporary method to ConnectomeManager to provide morphology
        #  information
        # Add this once at initialization instead of each time in
        # _perform_synaptogenesis
        def get_morphologies_registry(self):
            return self._neuroembryogenesis_morphologies_registry

        if not hasattr(self.connectome_manager, "get_morphologies_registry"):
            self.connectome_manager.get_morphologies_registry = (
                types.MethodType(
                    get_morphologies_registry, self.connectome_manager
                )
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
        """Check if BDU (Brain Development Unit) debugging is enabled.

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
        """Load a genome from file.

        Args:
            genome_path: Path to the genome file

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(
            DevelopmentStage.INITIALIZATION, 0, "Loading genome"
        )

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
                allow_auto_recovery = (
                    genome_config.auto_recovery_on_validation_failure
                )
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

                        sanitization_result = sanitize_invalid_morphologies(
                            self.genome
                        )

                        # Use the sanitized genome
                        self.genome = sanitization_result["genome"]
                        removed_morphologies = sanitization_result[
                            "removed_morphologies"
                        ]
                        recovery_summary = sanitization_result[
                            "recovery_summary"
                        ]

                        logger.info(
                            f"Auto-recovery completed: {recovery_summary}"
                        )
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
                    #  Don't return False - continue with loading despite
                    #  validation issues

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
                    DevelopmentStage.INITIALIZATION,
                    100,
                    "Genome loaded and validated",
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
                DevelopmentStage.INITIALIZATION,
                f"Failed to load genome: {str(e)}",
            )
            logger.error("Error loading genome")
            logger.exception(e)
            return False

    def _extract_cortical_properties(self, cortical_id: str) -> Dict[str, Any]:
        """Extract cortical area properties from genome or ConnectomeManager.

        For core areas created from templates, properties are already stored in ConnectomeManager.
        For genome areas, properties are extracted from the genome using GenomeProcessor.

        Args:
            cortical_id: The ID of the cortical area

        Returns:
            Dictionary of properties for the cortical area
        """
        # First check if area exists in ConnectomeManager with properties (e.g., core areas from templates)
        if cortical_id in self.connectome_manager.cortical_areas:
            area = self.connectome_manager.cortical_areas[cortical_id]
            if hasattr(area, 'properties') and area.properties:
                logger.debug(
                    f"✅ [CONNECTOME-MGR] Using stored properties for {cortical_id}: {len(area.properties)} properties"
                )
                return area.properties

        # Otherwise, extract from hierarchical genome blueprint
        blueprint = self.genome["blueprint"]

        if cortical_id not in blueprint:
            logger.warning(
                f"❌ Cortical area {cortical_id} not found in hierarchical blueprint"
            )
            return {}

        area_definition = blueprint[cortical_id]
        properties = {}

        # Extract all properties from hierarchical format in one clean pass
        property_mappings = {
            # Required properties
            "cortical_name": "name",
            "relative_coordinate": "position",
            "block_boundaries": "dimensions",
            # Optional properties - direct mapping
            "group_id": "group",
            "sub_group_id": "subgroup",
            "cortical_type": "type",
            "per_voxel_neuron_cnt": "neurons_per_voxel",
            "cortical_mapping_dst": "mapping",
            # Neural properties
            "synapse_attractivity": "synapse_attractivity",
            "refractory_period": "refrac",
            "firing_threshold": "fire_t",
            "leak_coefficient": "leak_c",
            "neuron_excitability": "neuron_excitability",
            "postsynaptic_current": "postsynaptic_current",
            "postsynaptic_current_max": "postsynaptic_current_max",
            "degeneration": "degeneration",
            "psp_uniform_distribution": "psp_uniform_distribution",
            "visualization": "visualization",
            "2d_coordinate": "2d_coordinate",
            # Memory properties
            "is_mem_type": "is_mem_type",
            "longterm_mem_threshold": "longterm_mem_threshold",
            "lifespan_growth_rate": "lifespan_growth_rate",
            "init_lifespan": "init_lifespan",
            "temporal_depth": "temporal_depth",
            "consecutive_fire_cnt_max": "consecutive_fire_cnt_max",
            "snooze_length": "snooze_length",
        }

        # Single pass extraction - no duplicates
        for source_key, target_key in property_mappings.items():
            if source_key in area_definition:
                value = area_definition[source_key]
                properties[target_key] = value

                # Handle special cases
                if source_key == "block_boundaries" and len(value) >= 3:
                    #  Set individual dimension properties for
                    #  ConnectomeManager compatibility
                    properties["bbx"] = value[0]
                    properties["bby"] = value[1]
                    properties["bbz"] = value[2]
                elif source_key == "per_voxel_neuron_cnt":
                    # Set legacy alias for compatibility
                    properties["n_cnt"] = value

        logger.debug(
            f"✅ [HIERARCHICAL] Extracted {len(properties)} properties for {cortical_id}"
        )
        return properties

    def _calculate_subregion(
        self, cortical_id: str, morphology: Dict
    ) -> BoundingBox:
        """Calculate a bounding box for a subregion of the cortical area.

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

            max_x = min(
                dimensions[0] - 1, subregion.get("max_x", dimensions[0] - 1)
            )
            max_y = min(
                dimensions[1] - 1, subregion.get("max_y", dimensions[1] - 1)
            )
            max_z = min(
                dimensions[2] - 1, subregion.get("max_z", dimensions[2] - 1)
            )

            return ((min_x, min_y, min_z), (max_x, max_y, max_z))

        # Default is the entire area
        return (
            (0, 0, 0),
            (dimensions[0] - 1, dimensions[1] - 1, dimensions[2] - 1),
        )

    def _get_cortical_ids_from_genome(self) -> List[str]:
        """Extract the list of cortical area IDs from hierarchical genome
        blueprint.

        ARCHITECTURE: Single source of truth - hierarchical genome format only.
        No fallbacks, no format detection, one clean reliable path.

        Returns:
            List of cortical area IDs
        """
        blueprint = self.genome["blueprint"]
        cortical_ids = list(blueprint.keys())

        logger.info(
            f"✅ [HIERARCHICAL] Found {len(cortical_ids)} cortical areas: {sorted(cortical_ids)}"
        )
        return cortical_ids

    def _setup_cortical_areas(self) -> bool:
        """Create all cortical areas in the connectome manager based on the
        genome.

        ENHANCED: Now guarantees core areas (_death, _power) are created first from templates,
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
                DevelopmentStage.CORTICOGENESIS,
                10,
                "Creating guaranteed core areas",
            )
            if not self._create_core_areas_from_templates():
                return False

            # STEP 2: Extract genome cortical areas
            cortical_ids = self._get_cortical_ids_from_genome()

            #  STEP 3: Check if genome contains core areas and update their
            #  properties
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
                cid for cid in cortical_ids if cid not in ["_death", "_power"]
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
                if (
                    created_cortical_id
                    not in self.connectome_manager.cortical_areas
                ):
                    raise RuntimeError(
                        f"CRITICAL: Area {cortical_id} was not properly created in connectome_manager"
                    )

                #  Get the created area from connectome_manager (single source
                #  of truth)
                area = self.connectome_manager.get_cortical_area(
                    created_cortical_id
                )

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
                    self.connectome_manager.cortical_mapping.get_idx(
                        cortical_id
                    )
                    != cortical_idx
                ):
                    raise RuntimeError(
                        f"CRITICAL: Mapping inconsistency for {cortical_id}: expected idx {cortical_idx}"
                    )

                logger.info(
                    f"✅ Successfully created and verified cortical area {cortical_id} → {cortical_idx}"
                )

                # CRITICAL FIX: Register memory areas with ConnectomeManager
                if area_type == "memory":
                    temporal_depth = properties.get("temporal_depth", 1)
                    logger.info(
                        f"🧠 [MEMORY-REG] Registering memory area {cortical_id} with temporal_depth={temporal_depth}"
                    )
                    success = self.connectome_manager.register_memory_area(
                        cortical_id, temporal_depth
                    )
                    if success:
                        logger.info(
                            f"🧠 [MEMORY-REG] Successfully registered memory area {cortical_id}"
                        )
                    else:
                        logger.error(
                            f"🧠 [MEMORY-REG] Failed to register memory area {cortical_id}"
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
                self.error = "CRITICAL: No cortical areas were created in connectome_manager"
                self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                return False

            # Verify core areas exist
            core_areas_found = []
            for core_id in ["_death", "_power"]:
                if core_id in self.connectome_manager.cortical_areas:
                    core_areas_found.append(core_id)

            if len(core_areas_found) != 2:
                self.error = f"CRITICAL: Missing core areas. Found: {core_areas_found}, Expected: ['_death', '_power']"
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
        """Create the guaranteed core areas (_death, _power) from templates.py.

        These areas are ALWAYS created regardless of genome content to ensure
        system reliability and proper cortical_idx reservation.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Import cortical_types and cortical_template from templates
            from feagi.evo.templates import cortical_types, cortical_template

            core_devices = cortical_types["CORE"]["supported_devices"]

            # Create _death area (cortical_idx=0)
            death_template = core_devices["_death"]
            
            # Merge cortical_template properties with core-specific properties
            death_properties = cortical_template.copy()
            death_properties.update({
                "template_source": "core",
                "enabled": death_template["enabled"],
                "structure": death_template["structure"],
                # Map template properties to expected names
                "neurons_per_voxel": death_properties["per_voxel_neuron_cnt"],
                "fire_t": death_properties["firing_threshold"],
                "leak_c": death_properties["leak_coefficient"],
                "refrac": death_properties["refractory_period"],
            })
            
            death_id = self.connectome_manager.add_cortical_area(
                name=death_template["cortical_name"],
                dimensions=tuple(death_template["resolution"]),
                position=tuple(death_template["coordinate_3d"]),
                area_type="CORE",
                properties=death_properties,
                cortical_id="_death",
            )

            #  Verify area was created and get from connectome_manager (single
            #  source of truth)
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

            # Create _power area (cortical_idx=1)
            pwr_template = core_devices["_power"]
            
            # Merge cortical_template properties with core-specific properties
            pwr_properties = cortical_template.copy()
            pwr_properties.update({
                "template_source": "core",
                "enabled": True,  # Always enable power area regardless of template default
                "structure": pwr_template["structure"],
                # Map template properties to expected names
                "neurons_per_voxel": pwr_properties["per_voxel_neuron_cnt"],
                "fire_t": pwr_properties["firing_threshold"],
                "leak_c": pwr_properties["leak_coefficient"],
                "refrac": pwr_properties["refractory_period"],
            })
            
            pwr_id = self.connectome_manager.add_cortical_area(
                name=pwr_template["cortical_name"],
                dimensions=tuple(pwr_template["resolution"]),
                position=tuple(pwr_template["coordinate_3d"]),
                area_type="CORE",
                properties=pwr_properties,
                cortical_id="_power",
            )

            #  Verify area was created and get from connectome_manager (single
            #  source of truth)
            if pwr_id not in self.connectome_manager.cortical_areas:
                raise RuntimeError(
                    "CRITICAL: _power area was not created in connectome_manager"
                )

            pwr_area = self.connectome_manager.get_cortical_area(pwr_id)
            self.cortical_id_map[pwr_area.cortical_idx] = "_power"
            self.reverse_cortical_id_map["_power"] = pwr_area.cortical_idx

            logger.info(
                f"Created core area _power at cortical_idx={pwr_area.cortical_idx}"
            )

            # Verify correct cortical_idx assignment
            if death_area.cortical_idx != 0:
                logger.error(
                    f"CRITICAL: _death area got cortical_idx={death_area.cortical_idx}, expected 0"
                )
                return False
            if pwr_area.cortical_idx != 1:
                logger.error(
                    f"CRITICAL: _power area got cortical_idx={pwr_area.cortical_idx}, expected 1"
                )
                return False

            logger.info(
                "Core areas created successfully with correct cortical_idx reservation"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to create core areas from templates: {e}")
            return False

    def _update_core_areas_from_genome(
        self, genome_cortical_ids: List[str]
    ) -> None:
        """Update core area properties if they are defined in the genome.

        This allows genomes to override the template defaults for core areas
        while ensuring the areas always exist.

        Args:
            genome_cortical_ids: List of cortical IDs found in genome
        """
        try:
            for core_id in ["_death", "_power"]:
                if core_id in genome_cortical_ids:
                    logger.info(
                        f"Updating core area {core_id} with genome properties"
                    )

                    # Extract properties from genome
                    genome_properties = self._extract_cortical_properties(
                        core_id
                    )

                    #  Get the existing core area from connectome_manager
                    #  (single source of truth)
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
                            area.dimensions = tuple(
                                genome_properties["dimensions"]
                            )
                            logger.debug(
                                f"Updated {core_id} dimensions to {area.dimensions}"
                            )

                        if "position" in genome_properties:
                            area.position = tuple(
                                genome_properties["position"]
                            )
                            logger.debug(
                                f"Updated {core_id} position to {area.position}"
                            )

                        if "name" in genome_properties:
                            area.name = genome_properties["name"]
                            logger.debug(
                                f"Updated {core_id} name to {area.name}"
                            )

                        # Merge additional properties
                        if hasattr(area, "properties"):
                            area.properties.update(genome_properties)
                        else:
                            area.properties = genome_properties

                        logger.info(
                            f"Core area {core_id} updated with genome properties"
                        )
                    else:
                        logger.warning(
                            f"Could not find core area {core_id} to update"
                        )

        except Exception as e:
            logger.warning(f"Error updating core areas from genome: {e}")

    def _perform_neurogenesis(self) -> bool:
        """
        FAST neurogenesis for SoA architecture: Just array assignments, no loops!
        Should complete in milliseconds for any brain size.

        Returns:
            True if successful, False otherwise
        """
        self._report_progress(
            DevelopmentStage.NEUROGENESIS,
            0,
            "Creating neurons (SoA-optimized)",
        )

        try:
            total_areas = len(self.connectome_manager.cortical_areas)
            total_neurons = 0
            start_time = datetime.datetime.now()

            for i, (cortical_id, area) in enumerate(
                self.connectome_manager.cortical_areas.items()
            ):
                properties = self._extract_cortical_properties(cortical_id)

                # Skip memory areas if configured
                if area.area_type == "memory" and self.config.get(
                    "skip_memory_neurogenesis", False
                ):
                    logger.info(
                        f"Skipping neurogenesis for memory area {area.name}"
                    )
                    continue

                # Calculate neuron count for this area
                width, height, depth = area.dimensions
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                area_neuron_count = width * height * depth * neurons_per_voxel

                logger.debug(
                    f"[FAST-SoA] Creating {area_neuron_count} neurons for {cortical_id}"
                )

                # FAST: Reserve array indices in bulk
                neuron_array = self.connectome_manager.neuron_array
                start_idx = neuron_array.next_index
                end_idx = start_idx + area_neuron_count

                if end_idx > neuron_array.max_neurons:
                    raise ValueError(
                        f"Not enough capacity for {area_neuron_count} neurons"
                    )

                # FAST: Generate neuron IDs in bulk
                neuron_ids = list(
                    range(
                        neuron_array._next_neuron_id,
                        neuron_array._next_neuron_id + area_neuron_count,
                    )
                )
                neuron_array._next_neuron_id += area_neuron_count

                #  FAST: Update mappings in bulk via ConnectomeManager (single
                #  source of truth)
                indices = np.arange(start_idx, end_idx, dtype=np.int32)
                for j, neuron_id in enumerate(neuron_ids):
                    self.connectome_manager.set_neuron_mapping(
                        neuron_id, start_idx + j
                    )

                # FAST: Set uniform properties with vectorized array slicing
                base_threshold = properties.get("fire_t", 1.0)
                base_decay_rate = 1.0 - (properties.get("leak_c", 0) / 100.0)
                # ARCHITECTURE COMPLIANCE: No fallbacks for required properties
                if "refrac" not in properties:
                    raise ValueError(f"ARCHITECTURE VIOLATION: Missing required property 'refrac' for area {cortical_id}")
                base_refractory = properties["refrac"]

                #  SoA OPTIMIZATION: Set all properties with single array
                #  operations
                neuron_array.valid_mask[start_idx:end_idx] = True
                neuron_array.membrane_potentials[start_idx:end_idx] = 0.0
                neuron_array.resting_potentials[start_idx:end_idx] = 0.0
                neuron_array.thresholds[start_idx:end_idx] = base_threshold
                neuron_array.decay_rates[start_idx:end_idx] = base_decay_rate
                neuron_array.refractory_periods[start_idx:end_idx] = (
                    base_refractory
                )
                neuron_array.refractory_counters[start_idx:end_idx] = 0
                neuron_array.cortical_idxs[start_idx:end_idx] = (
                    area.cortical_idx
                )
                neuron_array.is_active[start_idx:end_idx] = True

                #  FAST: Generate coordinates and apply position-based
                #  variations
                # Create coordinate arrays efficiently
                positions = []
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            for _ in range(neurons_per_voxel):
                                positions.append((x, y, z))

                # SoA OPTIMIZATION: Set coordinates with vectorized operations
                coords_x = np.array(
                    [pos[0] for pos in positions], dtype=np.uint32
                )
                coords_y = np.array(
                    [pos[1] for pos in positions], dtype=np.uint32
                )
                coords_z = np.array(
                    [pos[2] for pos in positions], dtype=np.uint32
                )

                neuron_array.coordinates_x[start_idx:end_idx] = coords_x
                neuron_array.coordinates_y[start_idx:end_idx] = coords_y
                neuron_array.coordinates_z[start_idx:end_idx] = coords_z

                #  FAST: Apply position-based variations with vectorized
                #  operations
                # 1. Firing threshold increment based on position
                fire_increment = properties.get("fire_increment", 0.0)
                if fire_increment != 0.0:
                    # Apply increment based on Z coordinate
                    z_increments = coords_z.astype(np.float32) * fire_increment
                    neuron_array.thresholds[start_idx:end_idx] += z_increments

                # 2. Leak variability based on position
                leak_variability = properties.get("leak_variability", 0.0)
                base_leak = properties.get("leak_c", 0.0)
                if leak_variability != 0.0 and base_leak != 0.0:
                    # Generate random variations for each neuron
                    np.random.seed(42)  # Deterministic for reproducibility
                    variations = (
                        np.random.uniform(
                            -leak_variability,
                            leak_variability,
                            area_neuron_count,
                        )
                        / 100.0
                    )
                    varied_leak = np.clip(
                        base_leak / 100.0 + variations, 0.0, 1.0
                    )
                    neuron_array.decay_rates[start_idx:end_idx] = (
                        1.0 - varied_leak
                    )

                # 3. Neuron excitability (probabilistic firing)
                excitability_value = properties.get("neuron_excitability", 1.0)
                # Validate and clamp excitability to [0.0, 1.0] range
                if excitability_value > 1.0:
                    logger.warning(
                        f"Cortical area {cortical_id}: excitability {excitability_value} > 1.0, clamping to 1.0"
                    )
                    excitability_value = 1.0
                elif excitability_value < 0.0:
                    logger.warning(
                        f"Cortical area {cortical_id}: excitability {excitability_value} < 0.0, clamping to 0.0"
                    )
                    excitability_value = 0.0

                #  Set excitability for all neurons in this cortical area using
                #  the area-aware method
                neuron_array.set_cortical_area_excitability(
                    cortical_idx=area.cortical_idx,
                    start_idx=start_idx,
                    end_idx=end_idx,
                    excitability=excitability_value,
                )

                #  Track areas that use probabilistic firing for performance
                #  optimization
                if not hasattr(self, "_probabilistic_areas"):
                    self._probabilistic_areas = set()

                if excitability_value < 0.999:
                    self._probabilistic_areas.add(cortical_id)
                    logger.info(
                        f"Cortical area {cortical_id} (idx={area.cortical_idx}): probabilistic firing enabled "
                        f"(excitability={excitability_value:.3f}, {area_neuron_count} neurons)"
                    )
                else:
                    # Ensure deterministic areas are not in the set
                    self._probabilistic_areas.discard(cortical_id)

                # FAST: Update voxel mapping efficiently
                if cortical_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[cortical_id] = {}

                neuron_idx = 0
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            position = (x, y, z)
                            voxel_neurons = []
                            for _ in range(neurons_per_voxel):
                                if neuron_idx < len(neuron_ids):
                                    voxel_neurons.append(
                                        neuron_ids[neuron_idx]
                                    )
                                    neuron_idx += 1
                            self.voxel_neuron_map[cortical_id][
                                position
                            ] = voxel_neurons

                #  CRITICAL FIX: Sync neurons with cortical area objects using
                #  vectorized operations
                #  PERFORMANCE: Use bulk set operations and dict comprehensions
                #  instead of loops
                #  Add all created neurons to cortical area in one bulk
                #  operation
                area._neuron_indices.update(neuron_ids)

                #  PERFORMANCE: Bulk update position mappings using zip and
                #  dict operations
                #  The positions list is already calculated above in vectorized
                #  fashion
                area._position_map.update(zip(neuron_ids, positions))

                #  PERFORMANCE: Bulk update position-to-neurons mapping using
                #  defaultdict-style logic
                for neuron_id, position in zip(neuron_ids, positions):
                    if position not in area._position_to_neurons:
                        area._position_to_neurons[position] = []
                    area._position_to_neurons[position].append(neuron_id)

                # Update array state
                neuron_array.next_index = end_idx
                neuron_array.neuron_count = end_idx
                total_neurons += area_neuron_count

                # Report progress
                progress = ((i + 1) / total_areas) * 100
                self._report_progress(
                    DevelopmentStage.NEUROGENESIS,
                    progress,
                    f"Area {i + 1}/{total_areas}: {area.name} ({area_neuron_count} neurons)",
                )

            end_time = datetime.datetime.now()
            duration = (end_time - start_time).total_seconds()

            self.development_stats["total_neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons in {duration:.3f}s ({total_neurons / duration:.0f} neurons/sec)",
            )

            logger.info(
                f"[FAST-SoA] Neurogenesis completed: {total_neurons} neurons in {duration:.3f}s"
            )
            return True

        except Exception as e:
            import traceback

            self.error = f"Failed to create neurons: {str(e)}"
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            logger.exception("Error during neurogenesis")
            logger.error(f"Traceback:\n{traceback.format_exc()}")
            return False

    def _perform_synaptogenesis(self) -> bool:
        """Create synaptic connections based on genome mappings.

        Returns:
            True if successful, False otherwise
        """
        # Check if BDU debugging is enabled
        debug_bdu = self._is_debug_bdu_enabled()

        if debug_bdu:
            logger.info(
                "[BDU DEBUG] ===== STARTING SYNAPTOGENESIS PHASE ====="
            )
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

            #  Extract cortical mappings directly from hierarchical genome
            #  format
            logger.info(
                "Extracting cortical mappings from hierarchical genome"
            )

            try:
                # Extract mappings directly from hierarchical genome
                mapping_data = {}
                mappings_found = 0

                # Iterate through cortical areas in hierarchical blueprint
                blueprint = self.genome.get("blueprint", {})
                logger.info(
                    f"Scanning {len(blueprint)} cortical areas for mappings"
                )

                for cortical_id, area_data in blueprint.items():
                    if (
                        isinstance(area_data, dict)
                        and "cortical_mapping_dst" in area_data
                    ):
                        cortical_mappings = area_data["cortical_mapping_dst"]

                        if (
                            isinstance(cortical_mappings, dict)
                            and cortical_mappings
                        ):
                            # Initialize source area in mapping_data
                            if cortical_id not in mapping_data:
                                mapping_data[cortical_id] = {}

                            logger.info(
                                f"Found {len(cortical_mappings)} mappings from {cortical_id}"
                            )

                            # Process each destination area
                            for (
                                dst_area_id,
                                connection_specs,
                            ) in cortical_mappings.items():
                                if (
                                    isinstance(connection_specs, list)
                                    and connection_specs
                                ):
                                    #  The format is already correct - just use
                                    #  it directly
                                    mapping_data[cortical_id][
                                        dst_area_id
                                    ] = connection_specs
                                    mappings_found += len(connection_specs)
                                    logger.info(
                                        f"  {cortical_id} -> {dst_area_id}: {len(connection_specs)} connections"
                                    )

                logger.info(f"Total mappings extracted: {mappings_found}")

                if debug_bdu:
                    logger.info(
                        f"[BDU DEBUG] Hierarchical extraction found {mappings_found} mappings"
                    )
                    for src_id, dst_mappings in mapping_data.items():
                        logger.info(
                            f"[BDU DEBUG]   {src_id} -> {list(dst_mappings.keys())}"
                        )

            except Exception as e:
                logger.error(
                    f"Failed to extract mappings from hierarchical genome: {e}"
                )
                logger.info("No cortical mappings could be extracted")
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
                    total_synapses = (
                        self.connectome_manager.get_synapse_count()
                    )
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
                    logger.warning(
                        "Failed to create synapses via cortical mappings"
                    )
            else:
                logger.info(
                    "No cortical mappings found in genome - no synapses will be created"
                )
                if debug_bdu:
                    logger.info(
                        "[BDU DEBUG] ===== SYNAPTOGENESIS PHASE COMPLETED ====="
                    )
                    logger.info(
                        "[BDU DEBUG] No mappings found - no synapses created"
                    )

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
        """Create a registry of morphology functions from the genome.

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

        registry["memory"] = {
            "type": "function",
            "parameters": {},
            "class": "built-in",
        }

        # Add morphologies from the genome
        if self.genome and "neuron_morphologies" in self.genome:
            for morphology_id, morphology in self.genome[
                "neuron_morphologies"
            ].items():
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
        """Main entry point to develop a brain from genome.

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

        # Create neurons
        if not self._perform_neurogenesis():
            return False

        # Create synapses
        if not self._perform_synaptogenesis():
            return False

        # Finalize and report statistics
        self.development_stats["end_time"] = datetime.datetime.now()
        self.development_stats["duration"] = (
            self.development_stats["end_time"]
            - self.development_stats["start_time"]
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

    def develop_brain_from_genome_data(
        self, genome_data: Dict[str, Any]
    ) -> bool:
        """Develop a brain from genome data directly (not from file).

        This method is used when the genome data is already loaded and sanitized
        in the state manager, ensuring single source of truth architecture.

        Args:
            genome_data: The genome dictionary data

        Returns:
            True if brain developed successfully, False otherwise
        """
        self.development_stats["start_time"] = datetime.datetime.now()

        #  CRITICAL: Reset brain state before development to ensure consistent
        #  performance
        # This prevents neuron array accumulation between genome loads
        logger.info(
            "Preparing connectome for new genome (resetting brain state)"
        )
        if hasattr(self.connectome_manager, "prepare_for_new_genome"):
            reset_result = self.connectome_manager.prepare_for_new_genome(
                genome_data, save_current_state=False
            )
            if not reset_result.get("success", False):
                self.error = "Failed to reset brain state for new genome"
                logger.error(self.error)
                return False
            logger.info(
                f"Brain reset completed: {reset_result.get('message', 'Unknown result')}"
            )
        else:
            logger.warning(
                "ConnectomeManager lacks prepare_for_new_genome method - performance may be inconsistent"
            )

        # Validate and load genome data directly
        if not self._load_genome_data(genome_data):
            return False

        # Set up cortical areas
        if not self._setup_cortical_areas():
            return False

        # Create neurons (now with properly reset neuron array)
        if not self._perform_neurogenesis():
            return False

        # Create synapses
        if not self._perform_synaptogenesis():
            return False

        # Finalize and report statistics
        self.development_stats["end_time"] = datetime.datetime.now()
        self.development_stats["duration"] = (
            self.development_stats["end_time"]
            - self.development_stats["start_time"]
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
        """Load genome data directly from dictionary (not from file).

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
                            "  → EXAMPLE: Add 'physiology': {'burst_delay': 0.025, 'max_age': 10000000, "
                            "'evolution_burst_count': 50, 'ipu_idle_threshold': 1000, "
                            "'plasticity_queue_depth': 3, 'lifespan_mgmt_interval': 10} to your genome"
                        )
                        logger.error(
                            "  → AUTO-RECOVERY: Enable auto-recovery in configuration to automatically add missing physiology properties"
                        )
                    elif key == "blueprint":
                        logger.error(
                            "  → EXAMPLE: Add 'blueprint': {} with cortical area definitions to your genome"
                        )
                    self._report_progress(
                        DevelopmentStage.FAILED, 0, self.error
                    )
                    return False

            # Store genome data
            self.genome = genome_data

            #  Validate physiology section specifically with detailed error
            #  reporting
            try:
                from feagi.evo.genome_validator import (
                    validate_physiology_section,
                )

                physiology_validation = validate_physiology_section(
                    self.genome
                )
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
                DevelopmentStage.INITIALIZATION,
                100,
                "Genome data loaded successfully",
            )
            return True

        except Exception as e:
            self.error = f"Failed to load genome data: {e}"
            logger.exception(self.error)
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            return False

    def update_cortical_mapping(self, mapping: Dict[str, Any]) -> bool:
        """Update cortical mapping in the connectome based on genome changes.

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
            logger.info(
                "🧠 [MAPPING-DEBUG] NeuroEmbryogenesis.update_cortical_mapping started"
            )
            logger.info(f"🧠 [MAPPING-DEBUG] Input mapping: {mapping}")
            logger.info(
                "🧠 [MAPPING-DEBUG] Applying cortical mapping updates to connectome"
            )

            if not self.connectome_manager:
                logger.error(
                    "Cannot update cortical mapping: No connectome manager"
                )
                return False

            if not mapping:
                logger.warning("No mapping data provided")
                return True

            # Convert EVO format to BDU format if needed
            # EVO format: {dst_area: [specs]}
            # BDU format: {src_area: {dst_area: [specs]}}

            #  Detect format by checking if first level values are lists (EVO
            #  format)
            first_key = next(iter(mapping.keys()))
            first_value = mapping[first_key]

            if isinstance(first_value, list):
                # This is EVO format: {dst_area: [connection_specs]}
                # We need to infer source areas from the destination areas
                logger.info("Converting EVO mapping format to BDU format")
                converted_mapping = {}

                #  Based on test mode 2 logs, these destination areas map to
                #  specific source areas:
                #  CIHMot, CKQM2_, CKYM2_, CO4M3_, CJWM3_, CTGM4_, CLWM4_ ->
                #  co_mot
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
                        dst_area_id = (
                            "co_mot"  # They all connect to motor output
                        )

                        if src_area_id not in converted_mapping:
                            converted_mapping[src_area_id] = {}
                        converted_mapping[src_area_id][
                            dst_area_id
                        ] = connection_specs

                        logger.info(
                            f"Mapped {src_area_id} -> {dst_area_id} with {len(connection_specs)} specs"
                        )
                    else:
                        # For other areas, assume self-connection for now
                        src_area_id = dst_area_id
                        if src_area_id not in converted_mapping:
                            converted_mapping[src_area_id] = {}
                        converted_mapping[src_area_id][
                            dst_area_id
                        ] = connection_specs

                        logger.info(
                            f"Self-mapped {src_area_id} -> {dst_area_id} with {len(connection_specs)} specs"
                        )

                mapping = converted_mapping
                logger.info(
                    f"Converted to BDU format with {len(mapping)} source areas"
                )

            elif isinstance(first_value, dict):
                #  Check if this is the correct BDU format: {src_area:
                #  {dst_area: [specs]}}
                #  or if it's still EVO format: {mapping_id: {dst_area:
                #  [specs]}}

                # Look at the second level to determine format
                second_level_sample = next(iter(first_value.values()))
                if isinstance(second_level_sample, list):
                    #  This is correct BDU format: {src_area: {dst_area:
                    #  [specs]}}
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
                        if (
                            dst_area_id
                            not in self.connectome_manager.cortical_areas
                        ):
                            logger.error(
                                f"Destination cortical area {dst_area_id} not found in connectome"
                            )
                            return False

            # Process each source area mapping with vectorized operations
            total_synapses_created = 0

            for src_area_id, target_mappings in mapping.items():
                if not isinstance(target_mappings, dict):
                    logger.warning(
                        f"Invalid mapping format for area {src_area_id}"
                    )
                    continue

                # Get source area and neurons once
                src_area = self.connectome_manager.cortical_areas[src_area_id]
                src_neurons = self.connectome_manager.get_neurons_by_area(
                    src_area_id
                )

                if not src_neurons:
                    logger.warning(
                        f"No neurons found in source area {src_area_id}"
                    )
                    continue

                # Update the source area's properties with mapping information
                if (
                    not hasattr(src_area, "properties")
                    or src_area.properties is None
                ):
                    src_area.properties = {}

                # Convert the mapping data to the format expected by the API
                #  The API expects mapping in array format: [morphology_id,
                #  scalar, multiplier, plasticity_flag, constant, ltp, ltd]
                api_mapping = {}
                for dst_area_id, connection_data in target_mappings.items():
                    connection_arrays = []
                    for connection_spec in connection_data:
                        if isinstance(connection_spec, dict):
                            #  Convert from object format to array format for
                            #  API compatibility
                            connection_array = [
                                connection_spec.get("morphology_id", ""),
                                connection_spec.get(
                                    "morphology_scalar", [1, 1, 1]
                                ),
                                connection_spec.get(
                                    "postSynapticCurrent_multiplier", 1.0
                                ),
                                connection_spec.get("plasticity_flag", False),
                                connection_spec.get(
                                    "plasticity_constant", 1.0
                                ),
                                connection_spec.get("ltp_multiplier", 1.0),
                                connection_spec.get("ltd_multiplier", 1.0),
                            ]
                            connection_arrays.append(connection_array)

                    if connection_arrays:
                        api_mapping[dst_area_id] = connection_arrays

                # Store the mapping in the cortical area properties
                logger.info(
                    f"🧠 [MAPPING-DEBUG] CRITICAL: Updating ConnectomeManager cortical area {src_area_id}"
                )
                logger.info(
                    f"🧠 [MAPPING-DEBUG] Current properties: {src_area.properties}"
                )
                logger.info(
                    f"🧠 [MAPPING-DEBUG] Setting mapping to: {api_mapping}"
                )
                src_area.properties["mapping"] = api_mapping

                # Update StateManager cortical areas cache
                try:
                    from feagi.core.state_manager import get_state_manager

                    state_manager = get_state_manager()
                    state_manager.update_cortical_areas_cache(
                        src_area_id, "mapping_update"
                    )
                    logger.info(
                        f"🧠 [MAPPING-DEBUG] Updated cache for area {src_area_id}"
                    )
                except Exception as e:
                    logger.warning(
                        f"🧠 [MAPPING-DEBUG] Failed to update cortical areas cache for {src_area_id}: {e}"
                    )

                logger.info(
                    f"🧠 [MAPPING-DEBUG] SUCCESS: ConnectomeManager area {src_area_id} properties updated"
                )
                logger.info(
                    f"🧠 [MAPPING-DEBUG] New properties: {src_area.properties}"
                )

                # Process mappings to target areas
                memory_mappings_processed = 0
                for dst_area_id, connection_data in target_mappings.items():
                    try:
                        logger.info(
                            f"Creating synapses from {src_area_id} to {dst_area_id}"
                        )
                        # Process each connection specification
                        for connection_spec in connection_data:
                            if not isinstance(connection_spec, dict):
                                logger.warning(
                                    f"Invalid connection specification format: {connection_spec}"
                                )
                                continue

                            #  Extract connection parameters from dictionary
                            #  format
                            #  Format: {"morphology_id": "block_to_block",
                            #  "morphology_scalar": [1,1,1], ...}
                            morphology_id = connection_spec.get(
                                "morphology_id", ""
                            )
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

                            #  MEMORY MORPHOLOGY: register upstream mapping
                            #  only, no synapses by design
                            mapping_type = connection_spec.get(
                                "mapping_type", ""
                            ).lower()
                            if (
                                morphology_id.lower() == "memory"
                                or mapping_type == "memory"
                            ):
                                try:
                                    self.connectome_manager.add_memory_area_mapping(
                                        src_area_id, dst_area_id
                                    )
                                    memory_mappings_processed += 1
                                    logger.info(
                                        f"[MEMORY-MAPPING] Registered upstream mapping {src_area_id} -> {dst_area_id} (no synapses by design)"
                                    )
                                except Exception as e:
                                    logger.error(
                                        f"[MEMORY-MAPPING] Failed to register memory mapping {src_area_id} -> {dst_area_id}: {e}"
                                    )
                                # Skip synaptogenesis for memory morphology
                                continue

                            #  NON-MEMORY MORPHOLOGIES: proceed with
                            #  synaptogenesis
                            #  Get destination neurons lazily here to avoid
                            #  false warnings for memory mappings
                            dst_neurons = (
                                self.connectome_manager.get_neurons_by_area(
                                    dst_area_id
                                )
                            )
                            if not dst_neurons:
                                logger.warning(
                                    f"No neurons found in destination area {dst_area_id}"
                                )
                                continue

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
                        #  Continue processing other mappings rather than
                        #  failing completely
                        continue

                # Log final results
                if total_synapses_created > 0:
                    logger.info(
                        f"Successfully created {total_synapses_created} synapses from cortical mapping updates"
                    )
                    return True
                else:
                    if memory_mappings_processed > 0:
                        logger.info(
                            "Memory morphology mappings processed with zero synapses (by design)"
                        )
                        return True
                    logger.warning(
                        "No synapses were created from cortical mapping updates"
                    )
                    #  Return True for graceful handling - empty mappings or
                    #  invalid morphologies
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
        """Apply morphology-based synaptogenesis between two cortical areas.

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
                morphology_def = self.genome["neuron_morphologies"].get(
                    morphology_id
                )
                if morphology_def:
                    morphology_type = morphology_def.get("type")

            # If not found in genome, check if it's a core function morphology
            if not morphology_def:
                # Import here to avoid circular imports
                from feagi.bdu.connectivity.synaptogenesis import (
                    MorphologyFunction,
                )

                # Check if this is a known function morphology
                function_morphology_values = [
                    e.value for e in MorphologyFunction
                ]
                if morphology_id in function_morphology_values:
                    #  This is a core function morphology - create a synthetic
                    #  definition
                    morphology_def = {
                        "type": "functions",
                        "parameters": {},
                        "class": "core",
                    }
                    morphology_type = "functions"
                    logger.debug(
                        f"Using core function morphology: {morphology_id}"
                    )
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
                logger.warning(
                    f"No type specified for morphology {morphology_id}"
                )
                return 0

            #  CRITICAL: Special handling for memory morphology - NO synapse
            #  creation
            if morphology_id == "memory":
                logger.info(
                    f"[MEMORY-MORPHOLOGY] Detected memory morphology from {src_area_id} to {dst_area_id}"
                )
                logger.info(
                    "[MEMORY-MORPHOLOGY] Registering memory mapping without creating synapses"
                )

                #  Import and call syn_memory directly to populate memory
                #  register
                from feagi.bdu.connectivity.rules.functions import syn_memory

                # Create memory register and populate it
                memory_register = {}
                syn_memory(src_area_id, dst_area_id, memory_register)
                logger.info(
                    f"[MEMORY-MORPHOLOGY] Memory register updated: {memory_register}"
                )

                #  Propagate memory register to ConnectomeManager (same as
                #  _process_function_morphology)
                if memory_register:
                    logger.info(
                        f"[MEMORY-PROPAGATION] Memory register found with {len(memory_register)} entries: {memory_register}"
                    )
                    for (
                        memory_area_id,
                        upstream_area_ids,
                    ) in memory_register.items():
                        logger.info(
                            f"[MEMORY-PROPAGATION] Processing memory area {memory_area_id} with upstream areas: {upstream_area_ids}"
                        )
                        for upstream_area_id in upstream_area_ids:
                            try:
                                logger.info(
                                    f"[MEMORY-PROPAGATION] Calling add_memory_area_mapping({upstream_area_id}, {memory_area_id})"
                                )
                                self.connectome_manager.add_memory_area_mapping(
                                    upstream_area_id, memory_area_id
                                )
                                logger.info(
                                    f"[MEMORY-PROPAGATION] Successfully registered memory mapping: {upstream_area_id} -> {memory_area_id}"
                                )
                            except Exception as e:
                                logger.error(
                                    f"[MEMORY-PROPAGATION] Failed to register memory mapping {upstream_area_id} -> {memory_area_id}: {e}"
                                )
                                logger.exception(
                                    "[MEMORY-PROPAGATION] Full exception trace:"
                                )
                    logger.info(
                        "[MEMORY-PROPAGATION] Completed processing all memory register entries"
                    )

                logger.info(
                    "[MEMORY-MORPHOLOGY] Memory morphology processing completed - 0 synapses created as expected"
                )
                return 0  # No synapses created for memory morphology

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
            logger.error(
                f"Error applying morphology mapping {morphology_id}: {e}"
            )
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
        """Process vector-based morphology using numpy vectorized operations.

        PERFORMANCE: Vectorized approach for massive performance improvement.
        Instead of processing 12,288 neurons one-by-one, processes ALL at once.

        Args:
            src_area_id: Source cortical area ID
            dst_area_id: Destination cortical area ID
            src_neurons: List of source neuron IDs
            dst_neurons: List of destination neuron IDs
            morphology_def: Morphology definition from genome
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
            import numpy as np

            vectors = morphology_def.get("parameters", {}).get("vectors", [])
            if not vectors:
                logger.warning(
                    "No vectors found in morphology definition for vector type"
                )
                return 0

            if not src_neurons:
                logger.debug("No source neurons to process")
                return 0

            logger.info(
                f"[VECTOR-NUMPY] Processing {len(src_neurons)} neurons with vectorized operations"
            )

            # Step 1: Extract ALL source neuron positions at once (vectorized)
            source_positions = []
            valid_source_neurons = []

            for src_neuron_id in src_neurons:
                src_pos = self._get_neuron_position(src_neuron_id, src_area_id)
                if src_pos:
                    source_positions.append(src_pos)
                    valid_source_neurons.append(src_neuron_id)

            if not source_positions:
                logger.warning("No valid source positions found")
                return 0

            # Convert to numpy arrays for vectorized operations
            source_neuron_ids = np.array(valid_source_neurons)  # Shape: (N,)
            source_positions = np.array(source_positions)  # Shape: (N, 3)

            logger.debug(
                f"[VECTOR-NUMPY] Extracted {len(source_positions)} valid positions"
            )

            total_synapses = 0

            # Process each vector in the morphology
            for vector in vectors:
                # Get morphology scalar (default to 1.0 if not provided)
                scalar = morphology_scalar[0] if morphology_scalar else 1.0

                #  Step 2: Apply vector [m,n,t] to ALL positions at once (pure
                #  numpy)
                vector_array = np.array(vector) * scalar  # Shape: (3,)
                candidate_positions = (
                    source_positions + vector_array
                )  # Broadcasting! Shape: (N, 3)

                logger.debug(
                    f"[VECTOR-NUMPY] Applied vector {vector} * {scalar} to {len(candidate_positions)} positions"
                )

                # Step 3: Get destination area dimensions for boundary checking
                dst_area = self.connectome_manager.get_cortical_area(
                    dst_area_id
                )
                if not dst_area:
                    logger.warning(
                        f"Cannot get destination area {dst_area_id}"
                    )
                    continue

                dst_dimensions = dst_area.dimensions

                #  Step 4: Filter candidate positions to be within bounds
                #  (vectorized)
                valid_mask = (
                    (candidate_positions[:, 0] >= 0)
                    & (candidate_positions[:, 0] < dst_dimensions[0])
                    & (candidate_positions[:, 1] >= 0)
                    & (candidate_positions[:, 1] < dst_dimensions[1])
                    & (candidate_positions[:, 2] >= 0)
                    & (candidate_positions[:, 2] < dst_dimensions[2])
                )

                valid_candidate_positions = candidate_positions[valid_mask]
                valid_source_neurons_for_vector = source_neuron_ids[valid_mask]

                if len(valid_candidate_positions) == 0:
                    logger.debug(
                        "[VECTOR-NUMPY] No valid candidate positions after boundary filtering"
                    )
                    continue

                logger.debug(
                    f"[VECTOR-NUMPY] {len(valid_candidate_positions)} positions within bounds"
                )

                # Step 5: Batch lookup ALL candidate positions at once
                candidate_positions_set = set(
                    map(tuple, valid_candidate_positions)
                )

                neuron_weight_pairs = (
                    self.connectome_manager.batch_voxel_to_neuron_lookup(
                        cortical_id=dst_area_id,
                        candidate_positions=candidate_positions_set,
                        post_synaptic_current=psc_multiplier,
                    )
                )

                if not neuron_weight_pairs:
                    logger.debug(
                        "[VECTOR-NUMPY] No neurons found at candidate positions"
                    )
                    continue

                #  Step 6: Create position-to-neurons mapping using global
                #  spatial hash
                #  ULTRA-FAST: Use pre-computed spatial hash system to
                #  eliminate all coordinate lookups
                position_to_neurons = {}

                # Build reverse mapping using global spatial hash system
                if neuron_weight_pairs:
                    # Import global spatial hash system
                    from feagi.bdu.spatial_hash import get_spatial_hash

                    spatial_hash = get_spatial_hash()

                    # Extract neuron IDs from the pairs
                    found_neuron_ids = [
                        pair[0] for pair in neuron_weight_pairs
                    ]

                    # Get all positions at once using vectorized lookup
                    if hasattr(
                        self.connectome_manager.neuron_array,
                        "batch_get_coordinates",
                    ):
                        neuron_positions_batch = self.connectome_manager.neuron_array.batch_get_coordinates(
                            found_neuron_ids
                        )
                    else:
                        # Fallback: vectorized coordinate extraction
                        neuron_indices = [
                            self.connectome_manager.get_neuron_index(nid)
                            for nid in found_neuron_ids
                            if self.connectome_manager.has_neuron(nid)
                        ]

                        # Filter out None values from the mapping lookups
                        neuron_indices = [
                            idx for idx in neuron_indices if idx is not None
                        ]

                        if neuron_indices:
                            indices_array = np.array(
                                neuron_indices, dtype=np.int32
                            )
                            coords_x = self.connectome_manager.neuron_array.coordinates_x[
                                indices_array
                            ]
                            coords_y = self.connectome_manager.neuron_array.coordinates_y[
                                indices_array
                            ]
                            coords_z = self.connectome_manager.neuron_array.coordinates_z[
                                indices_array
                            ]
                            neuron_positions_batch = list(
                                zip(coords_x, coords_y, coords_z)
                            )
                        else:
                            neuron_positions_batch = []

                    # Group neurons by position
                    for i, (neuron_id, weight) in enumerate(
                        neuron_weight_pairs
                    ):
                        if i < len(neuron_positions_batch):
                            neuron_pos = neuron_positions_batch[i]
                            if neuron_pos not in position_to_neurons:
                                position_to_neurons[neuron_pos] = []
                            position_to_neurons[neuron_pos].append(
                                (neuron_id, weight)
                            )

                # Step 7: Create synapses (vectorized where possible)
                synapse_connections = []
                for i, candidate_pos in enumerate(valid_candidate_positions):
                    candidate_pos_tuple = tuple(candidate_pos)
                    if candidate_pos_tuple in position_to_neurons:
                        src_neuron_id = valid_source_neurons_for_vector[i]
                        for dst_neuron_id, weight in position_to_neurons[
                            candidate_pos_tuple
                        ]:
                            synapse_connections.append(
                                (src_neuron_id, dst_neuron_id, weight)
                            )

                # Step 8: Batch create synapses
                if synapse_connections:
                    created = self.connectome_manager.batch_create_synapses(
                        synapse_connections
                    )
                    total_synapses += created
                    logger.debug(
                        f"[VECTOR-NUMPY] Created {created} synapses for vector {vector}"
                    )

            logger.info(
                f"[VECTOR-NUMPY] Created {total_synapses} total synapses using vectorized operations"
            )
            return total_synapses

        except Exception as e:
            logger.error(
                f"Error in vectorized vector morphology processing: {e}"
            )
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")
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
        """Process pattern-based morphology using legacy pattern logic.

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
            dst_area_props = (
                self.connectome_manager.get_cortical_area_properties(
                    dst_area_id
                )
            )
            if not dst_area_props:
                logger.error(f"Cannot get properties for area {dst_area_id}")
                return 0

            dst_dimensions = dst_area_props.get("dimensions", [1, 1, 1])

            # Process each source neuron
            for src_neuron_id in src_neurons:
                try:
                    # Get source neuron position
                    src_pos = self._get_neuron_position(
                        src_neuron_id, src_area_id
                    )
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
                                    dst_cortical_boundary=tuple(
                                        dst_dimensions
                                    ),
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
                        neuron_weight_pairs = self.connectome_manager.batch_voxel_to_neuron_lookup(
                            cortical_id=dst_area_id,
                            candidate_positions=all_candidate_positions,
                            post_synaptic_current=psc_multiplier,
                        )

                        # Convert to synapse connections
                        for neuron_id, weight in neuron_weight_pairs:
                            synapse_connections.append(
                                (src_neuron_id, neuron_id, weight)
                            )

                    #  Create synapses in batch - Now using GlobalSynapseArray
                    #  for optimal performance
                    if synapse_connections:
                        created = (
                            self.connectome_manager.batch_create_synapses(
                                synapse_connections
                            )
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
        """Process function-based morphology with direct implementation.

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

            #  Create morphology dict in the correct format for
            #  find_candidate_neurons
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

                    #  Apply legacy synapse attractivity filtering (critical
                    #  for proper behavior)
                    dst_area = self.connectome_manager.get_cortical_area(
                        dst_area_id
                    )
                    synapse_attractivity = dst_area.properties.get(
                        "synatt", 100
                    )

                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] Found {len(candidate_neurons)} candidate neurons"
                        )
                        logger.info(
                            f"[BDU DEBUG] Synapse attractivity: {synapse_attractivity}%"
                        )

                    #  Create synapses from candidate neurons with
                    #  probabilistic filtering
                    synapse_connections = []
                    for dst_neuron_id, weight in candidate_neurons:
                        #  Legacy behavior: probabilistic synapse creation
                        #  based on attractivity
                        if random.randrange(1, 100) < synapse_attractivity:
                            synapse_connections.append(
                                (src_neuron_id, dst_neuron_id, weight)
                            )

                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] After attractivity filtering: {len(synapse_connections)} synapses to create"
                        )

                    if synapse_connections:
                        #  PERFORMANCE DEBUG: Time the batch_create_synapses
                        #  call
                        start_time = time.time()
                        created = (
                            self.connectome_manager.batch_create_synapses(
                                synapse_connections
                            )
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

            #  CRITICAL: Propagate memory register to ConnectomeManager for
            #  memory area tracking
            if memory_register:
                logger.info(
                    f"[MEMORY-PROPAGATION] Memory register found with {len(memory_register)} entries: {memory_register}"
                )
                for (
                    memory_area_id,
                    upstream_area_ids,
                ) in memory_register.items():
                    logger.info(
                        f"[MEMORY-PROPAGATION] Processing memory area {memory_area_id} with upstream areas: {upstream_area_ids}"
                    )
                    for upstream_area_id in upstream_area_ids:
                        try:
                            logger.info(
                                f"[MEMORY-PROPAGATION] Calling add_memory_area_mapping({upstream_area_id}, {memory_area_id})"
                            )
                            self.connectome_manager.add_memory_area_mapping(
                                upstream_area_id, memory_area_id
                            )
                            logger.info(
                                f"[MEMORY-PROPAGATION] Successfully registered memory mapping: {upstream_area_id} -> {memory_area_id}"
                            )
                        except Exception as e:
                            logger.error(
                                f"[MEMORY-PROPAGATION] Failed to register memory mapping {upstream_area_id} -> {memory_area_id}: {e}"
                            )
                            logger.exception(
                                "[MEMORY-PROPAGATION] Full exception trace:"
                            )
                logger.info(
                    "[MEMORY-PROPAGATION] Completed processing all memory register entries"
                )
            else:
                logger.info(
                    f"[MEMORY-PROPAGATION] No memory register found for morphology {morphology_id}"
                )

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
            if (
                hasattr(self, "voxel_neuron_map")
                and area_id in self.voxel_neuron_map
            ):
                for position, neuron_list in self.voxel_neuron_map[
                    area_id
                ].items():
                    if neuron_id in neuron_list:
                        return position

            # Fallback to connectome manager lookup
            position = self.connectome_manager.get_neuron_position(neuron_id)
            if position:
                return position

            # If no position found, log debug info
            logger.debug(
                f"No position found for neuron {neuron_id} in area {area_id}"
            )
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
        self,
        area_id: str,
        region: Tuple[Tuple[int, int, int], Tuple[int, int, int]],
    ) -> List[int]:
        """Get all neurons within a specified 3D region of a cortical area."""
        try:
            (min_x, min_y, min_z), (max_x, max_y, max_z) = region
            neurons_in_region = []

            # Try voxel mapping first if available
            if (
                hasattr(self, "voxel_neuron_map")
                and area_id in self.voxel_neuron_map
            ):
                for position, neuron_list in self.voxel_neuron_map[
                    area_id
                ].items():
                    x, y, z = position
                    if (
                        min_x <= x <= max_x
                        and min_y <= y <= max_y
                        and min_z <= z <= max_z
                    ):
                        neurons_in_region.extend(neuron_list)
            else:
                # Fallback: get all neurons in area and check their positions
                all_neurons = self.connectome_manager.get_neurons_by_area(
                    area_id
                )
                for neuron_id in all_neurons:
                    position = self.connectome_manager.get_neuron_position(
                        neuron_id
                    )
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
    progress_callback: Optional[
        Callable[[DevelopmentStage, float, str], None]
    ] = None,
) -> Tuple[bool, Dict[str, Any]]:
    """Develop a brain from a genome file.

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
