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

import numpy as np  # noqa: F401

from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.evo.genome_processor import (
    genome_morphology_updator,
    genome_physiology_updator,
    genome_stat_updator,
    merge_core_morphologies,
)
from feagi.evo.genome_validator import genome_validator
from feagi.utils.config import FeagiConfig

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
            "mp_charge_accumulation": "mp_charge_accumulation",
            "mp_driven_psp": "mp_driven_psp",
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
                # In additive mode, check if cortical area already exists
                if hasattr(self, 'additive_mode') and self.additive_mode:
                    existing_area = None
                    for area_id, area in self.connectome_manager.cortical_areas.items():
                        if hasattr(area, 'cortical_id') and area.cortical_id == cortical_id:
                            existing_area = area
                            break
                    
                    if existing_area is not None:
                        logger.info(f"Additive mode: Skipping existing cortical area {cortical_id}")
                        # Update mappings for existing area
                        self.cortical_id_map[existing_area.cortical_idx] = cortical_id
                        self.reverse_cortical_id_map[cortical_id] = existing_area.cortical_idx
                        continue
                
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

                # Immediately sync the new area into NPU registry to keep mappings consistent
                try:
                    if hasattr(self.connectome_manager, "sync_cortical_areas_to_npu"):
                        self.connectome_manager.sync_cortical_areas_to_npu()
                except Exception as sync_e:
                    logger.error(
                        f"[NPU-SYNC] Failed to sync cortical areas after creating {cortical_id}: {sync_e}"
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
                
                # Register cortical area name in Rust NPU (for visualization encoding)
                # Get rust_npu_integration from connectome_manager's NPU interface
                if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                    npu_interface = self.connectome_manager._npu_interface
                    if hasattr(npu_interface, '_rust_npu_integration') and npu_interface._rust_npu_integration:
                        rust_npu = npu_interface._rust_npu_integration._rust_npu
                        if rust_npu:
                            rust_npu.register_cortical_area(cortical_idx, cortical_id)
                            logger.debug(f"Registered cortical area {cortical_id} (idx={cortical_idx}) in Rust NPU for visualization")

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
        
        In additive mode, skips creation if core areas already exist.

        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if core areas already exist (for additive mode)
            existing_death = None
            existing_power = None
            
            for area_id, area in self.connectome_manager.cortical_areas.items():
                if hasattr(area, 'cortical_id'):
                    if area.cortical_id == "_death":
                        existing_death = area
                        logger.info(f"Core area _death already exists at cortical_idx={area.cortical_idx}")
                    elif area.cortical_id == "_power":
                        existing_power = area
                        logger.info(f"Core area _power already exists at cortical_idx={area.cortical_idx}")
            
            # Import cortical_types and cortical_template from templates
            from feagi.evo.templates import cortical_types, cortical_template

            core_devices = cortical_types["CORE"]["supported_devices"]
            
            # Get templates (needed for genome sync later)
            death_template = core_devices["_death"]
            pwr_template = core_devices["_power"]

            # Create _death area (cortical_idx=0) only if it doesn't exist
            if existing_death is None:
                
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
                logger.info(
                    f"Created core area _death at cortical_idx={death_area.cortical_idx}"
                )
            else:
                death_area = existing_death
                logger.info(f"Using existing core area _death at cortical_idx={death_area.cortical_idx}")
            
            # Update mappings for death area (whether new or existing)
            self.cortical_id_map[death_area.cortical_idx] = "_death"
            self.reverse_cortical_id_map["_death"] = death_area.cortical_idx
            
            # Register in Rust NPU for visualization
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                npu_interface = self.connectome_manager._npu_interface
                if hasattr(npu_interface, '_rust_npu_integration') and npu_interface._rust_npu_integration:
                    rust_npu = npu_interface._rust_npu_integration._rust_npu
                    if rust_npu:
                        rust_npu.register_cortical_area(death_area.cortical_idx, "_death")
                        logger.debug(f"Registered core area _death (idx={death_area.cortical_idx}) in Rust NPU")

            # Create _power area (cortical_idx=1) only if it doesn't exist
            if existing_power is None:
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
                logger.info(
                    f"Created core area _power at cortical_idx={pwr_area.cortical_idx}"
                )
            else:
                pwr_area = existing_power
                logger.info(f"Using existing core area _power at cortical_idx={pwr_area.cortical_idx}")
            
            # Update mappings for power area (whether new or existing)
            self.cortical_id_map[pwr_area.cortical_idx] = "_power"
            self.reverse_cortical_id_map["_power"] = pwr_area.cortical_idx
            
            # Register in Rust NPU for visualization
            if hasattr(self.connectome_manager, '_npu_interface') and self.connectome_manager._npu_interface:
                npu_interface = self.connectome_manager._npu_interface
                if hasattr(npu_interface, '_rust_npu_integration') and npu_interface._rust_npu_integration:
                    rust_npu = npu_interface._rust_npu_integration._rust_npu
                    if rust_npu:
                        rust_npu.register_cortical_area(pwr_area.cortical_idx, "_power")
                        logger.debug(f"Registered core area _power (idx={pwr_area.cortical_idx}) in Rust NPU")

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

            # Ensure genome blueprint also contains core areas (strict, no fallbacks)
            if hasattr(self, "genome") and isinstance(self.genome, dict):
                blueprint = self.genome.get("blueprint")
                if not isinstance(blueprint, dict):
                    raise ValueError("Genome blueprint missing or invalid during core area sync")

                def _add_core_to_genome_strict(core_id: str, template: dict) -> None:
                    res = template.get("resolution")
                    coord = template.get("coordinate_3d")
                    if not isinstance(res, (list, tuple)) or len(res) != 3:
                        raise ValueError(f"Invalid resolution for {core_id}")
                    if not isinstance(coord, (list, tuple)) or len(coord) != 3:
                        raise ValueError(f"Invalid coordinate_3d for {core_id}")

                    area_def = {
                        "cortical_name": template.get("cortical_name", core_id),
                        # Prefer normalized hierarchical keys used elsewhere
                        "coordinates_3d": [int(coord[0]), int(coord[1]), int(coord[2])],
                        "cortical_dimensions": [int(res[0]), int(res[1]), int(res[2])],
                        "cortical_type": "CORE",
                        "parameters": {},
                    }
                    blueprint[core_id] = area_def
                    logger.info(f"Synchronized core area '{core_id}' into genome blueprint")

                # Add or overwrite to maintain single source of truth
                _add_core_to_genome_strict("_death", death_template)
                _add_core_to_genome_strict("_power", pwr_template)

                # Update StateManager genome (single source of truth)
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    FeagiStateManager.instance().genome = self.genome
                except Exception as e:
                    logger.warning(f"Failed to update StateManager genome after core sync: {e}")

                # Refresh brain region hierarchy if available (metadata-only)
                try:
                    if hasattr(self.connectome_manager, "brain_region_hierarchy"):
                        self.connectome_manager.brain_region_hierarchy.load_from_genome(self.genome)
                except Exception as e:
                    logger.warning(f"Failed to refresh brain region hierarchy after core sync: {e}")
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

                # Always skip neurogenesis for memory areas (handled by MemoryNeuronArray)
                if area.area_type == "memory":
                    logger.info(f"Skipping neurogenesis for memory area {area.name}")
                    continue

                # ✅ RUST NPU: Create neurons for ALL areas including core areas
                # Core areas (_death, _power) need neurons for power injection to work
                # Power injection uses min(neurons) from cortical_idx=1, so we need at least one neuron
                # OLD behavior: Skip core areas (neurons were created by _ensure_core_neurons)
                # NEW behavior: Normal neurogenesis creates all neurons (simpler, cleaner)
                
                # In additive mode, skip neurogenesis for areas that already have neurons
                if hasattr(self, 'additive_mode') and self.additive_mode:
                    # Check if this area already has neurons (fast check via area object)
                    # If area has _neuron_indices set, it already has neurons
                    if hasattr(area, '_neuron_indices') and len(area._neuron_indices) > 0:
                        logger.info(f"Additive mode: Skipping neurogenesis for area {cortical_id} (already has {len(area._neuron_indices)} neurons)")
                        continue

                # Calculate neuron count for this area
                width, height, depth = area.dimensions
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                area_neuron_count = width * height * depth * neurons_per_voxel

                # CRITICAL DEBUG: Log detailed info for power area (gated)
                if cortical_id == "_power":
                    try:
                        if FeagiStateManager.instance().is_debug_npu_enabled():
                            logger.info("[POWER-DEBUG] _power area details:")
                            logger.info(f"  Dimensions: {width}x{height}x{depth}")
                            logger.info(f"  neurons_per_voxel: {neurons_per_voxel}")
                            logger.info(f"  Calculated area_neuron_count: {area_neuron_count}")
                            logger.info(f"  Properties: {properties}")
                    except Exception:
                        pass

                logger.debug(
                    f"[FAST-SoA] Creating {area_neuron_count} neurons for {cortical_id}"
                )

                # Use NPU Interface CRUD methods with cortical locking
                # This replaces direct array manipulation with proper cortical area locking
                logger.debug(f"[NEUROGENESIS] Creating {area_neuron_count} neurons for area {cortical_id} using NPU Interface")
                
                # Check capacity through NPU Interface
                if not self.connectome_manager._npu_interface:
                    raise RuntimeError("NPU Interface not configured for neurogenesis")
                
                npu_interface = self.connectome_manager._npu_interface
                
                # Ensure cortical area is registered with NPU Interface
                if area.cortical_idx not in npu_interface.cortical_areas:
                    logger.debug(f"[NEUROGENESIS] Registering cortical area {cortical_id} (idx={area.cortical_idx}) with NPU Interface")
                    result = npu_interface.create_cortical_area(
                        cortical_idx=area.cortical_idx,
                        dimensions=(width, height, depth),
                        area_type=("memory" if area.area_type == "memory" else "regular"),
                        cortical_id=cortical_id  # Pass the string ID for API lookups
                    )
                    from feagi.npu.interface import OperationResult
                    if result != OperationResult.SUCCESS:
                        raise RuntimeError(f"Failed to register cortical area {area.cortical_idx} with NPU Interface")
                
                # ✅ Use Rust NPU directly for capacity check (no proxy)
                if npu_interface.get_neuron_count() + area_neuron_count > npu_interface.max_neurons:
                    raise ValueError(
                        f"Not enough capacity for {area_neuron_count} neurons"
                    )
                
                # Extract neuron properties from area configuration
                base_threshold = properties.get("fire_t", 1.0)
                base_leak_coefficient = properties.get("leak_c", properties.get("leak_coefficient", 0)) / 100.0  # 0-100 → 0.0-1.0
                # ARCHITECTURE COMPLIANCE: No fallbacks for required properties
                if "refrac" not in properties:
                    raise ValueError(f"ARCHITECTURE VIOLATION: Missing required property 'refrac' for area {cortical_id}")
                _ = properties["refrac"]

                # Generate all positions for this cortical area
                positions = []
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            for _ in range(neurons_per_voxel):
                                positions.append((x, y, z))

                # Use NPU Interface batch creation with cortical locking
                from feagi.npu.interface import NeuronCreationRequest
                from feagi.core.state_manager import get_state_manager
                
                state_manager = get_state_manager()
                
                # Lock the cortical area for neurogenesis
                lock_acquired = False
                try:
                    # Lock only this specific cortical area
                    if not state_manager.lock_cortical_area(area.cortical_idx, locked_by="BDU", operation="neurogenesis"):
                        raise RuntimeError(f"Failed to acquire lock for cortical area {area.cortical_idx}")
                    lock_acquired = True
                    
                    logger.debug(f"[NEUROGENESIS] Locked cortical area {area.cortical_idx} for batch neuron creation")
                    
                    # Extract ALL neural dynamics parameters from genome - NO HARDCODED VALUES
                    base_excitability = properties.get("neuron_excitability", 1.0)
                    consecutive_fire_limit = properties.get("consecutive_fire_cnt_max", 10)
                    if consecutive_fire_limit == 0:
                        consecutive_fire_limit = 10  # Prevent infinite consecutive firing
                    
                    # Extract snooze period from genome (nx-snooze-f gene)
                    snooze_period = int(max(0, round(properties.get("snooze_length", 0))))
                    
                    # Extract membrane potential accumulation from genome (nx-mp_acc-b gene)
                    # Default to True for backward compatibility (integrator behavior)
                    mp_charge_accumulation = bool(properties.get("mp_charge_accumulation", True))
                    
                    # Create batch neuron creation request with ALL parameters from genome
                    request = NeuronCreationRequest(
                        cortical_idx=area.cortical_idx,
                        positions=positions,
                        thresholds=[base_threshold] * area_neuron_count,
                        initial_potentials=[0.0] * area_neuron_count,
                        leak_coefficients=[base_leak_coefficient] * area_neuron_count,
                        refractory_periods=[properties["refrac"]] * area_neuron_count,
                        excitabilities=[base_excitability] * area_neuron_count,
                        resting_potentials=[0.0] * area_neuron_count,
                        consecutive_fire_limits=[consecutive_fire_limit] * area_neuron_count,
                        snooze_periods=[snooze_period] * area_neuron_count,
                        mp_charge_accumulation=[mp_charge_accumulation] * area_neuron_count,
                    )
                    
                    # Use NPU Interface CRUD method for batch creation (gated debug)
                    if cortical_id == "_power":
                        try:
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                logger.info(f"[POWER-DEBUG] About to call create_neurons_batch for _power with {area_neuron_count} neurons")
                        except Exception:
                            pass
                    
                    result = npu_interface.create_neurons_batch(request)
                    
                    if cortical_id == "_power":
                        try:
                            if FeagiStateManager.instance().is_debug_npu_enabled():
                                logger.info(f"[POWER-DEBUG] create_neurons_batch result: success={result.is_success}, count={result.successful_count}")
                        except Exception:
                            pass
                    
                    if not result.is_success:
                        raise RuntimeError(f"Failed to create neurons via NPU Interface: {result.result}")
                    
                    # ✅ Verify correct count was created (Rust owns all neuron data)
                    if result.successful_count != area_neuron_count:
                        raise RuntimeError(f"Expected {area_neuron_count} neurons created, got {result.successful_count}")
                    
                finally:
                    # Always unlock the cortical area, even on exception
                    if lock_acquired:
                        state_manager.unlock_cortical_area(area.cortical_idx, locked_by="BDU")
                        logger.debug(f"[NEUROGENESIS] Unlocked cortical area {area.cortical_idx} after batch neuron creation")
                
                logger.debug(f"[NEUROGENESIS] Successfully created {area_neuron_count} neurons for area {cortical_id}")
                
                # ✅ ARCHITECTURE FIX: All neuron data (IDs, positions, properties) live in Rust NPU
                # Python no longer maintains voxel_neuron_map, _position_map, _neuron_indices
                # Query Rust directly when needed via npu_interface.get_neuron_position(neuron_id)

                # NPU Interface handles array state updates automatically
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
            
            # Update state manager counts after neurogenesis completes
            try:
                self.connectome_manager._update_brain_statistics()
                logger.info("✅ Updated state manager after neurogenesis")
            except Exception as e:
                logger.warning(f"Failed to update state manager after neurogenesis: {e}")
            
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
            # Precondition: Ensure SynapseArray is initialized once before any synaptogenesis work
            if (
                not hasattr(self.connectome_manager, "synapse_array")
                or self.connectome_manager.synapse_array is None
            ):
                raise RuntimeError(
                    "SynapseArray is not initialized. Initialize NPU interface and set ConnectomeManager.synapse_array before synaptogenesis."
                )
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

            # FAIL-FAST PRECHECK: All mapped areas used by non-memory morphologies must have neurons
            if mappings_found > 0:
                for src_id, dst_mappings in mapping_data.items():
                    src_is_memory = False
                    try:
                        if hasattr(self.connectome_manager, 'is_memory_area'):
                            src_is_memory = self.connectome_manager.is_memory_area(src_id)
                    except Exception:
                        src_is_memory = False

                    for dst_id, connection_specs in dst_mappings.items():
                        # Determine if this edge includes any non-memory morphology
                        has_non_memory = False
                        try:
                            for spec in connection_specs:
                                if isinstance(spec, dict):
                                    morph = str(spec.get('morphology_id', '')).lower()
                                elif isinstance(spec, list) and len(spec) >= 1:
                                    morph = str(spec[0]).lower()
                                else:
                                    morph = ''
                                if morph and morph != 'memory':
                                    has_non_memory = True
                                    break
                        except Exception:
                            # If spec parsing fails, be conservative and require neurons
                            has_non_memory = True

                        if not has_non_memory:
                            # Memory-only mapping: skip neuron checks (handled separately later)
                            continue

                        dst_is_memory = False
                        try:
                            if hasattr(self.connectome_manager, 'is_memory_area'):
                                dst_is_memory = self.connectome_manager.is_memory_area(dst_id)
                        except Exception:
                            dst_is_memory = False

                        # For non-memory edges, require areas to exist
                        # (Areas without neurons shouldn't exist after neurogenesis)
                        if not src_is_memory:
                            if src_id not in self.connectome_manager.cortical_areas:
                                raise RuntimeError(
                                    f"Synaptogenesis preflight failed: source area {src_id} doesn't exist"
                                )
                        if not dst_is_memory:
                            if dst_id not in self.connectome_manager.cortical_areas:
                                raise RuntimeError(
                                    f"Synaptogenesis preflight failed: destination area {dst_id} doesn't exist"
                                )

                # Create synapses if mappings were found
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
            
            # Update state manager counts after synaptogenesis completes
            try:
                self.connectome_manager._update_brain_statistics()
                logger.info("✅ Updated state manager after synaptogenesis")
            except Exception as e:
                logger.warning(f"Failed to update state manager after synaptogenesis: {e}")
            
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

        # Run brain region I/O validation/update for file-based path as well
        try:
            if not self._validate_and_update_brain_region_mappings():
                logger.warning(
                    "Brain region mapping validation failed after file-based development"
                )
        except Exception as e:
            logger.warning(f"Brain region mapping validation error: {e}")

        # Final report
        self._report_progress(
            DevelopmentStage.COMPLETED,
            100,
            f"Brain development completed in {self.development_stats['duration']}. "
            f"Created {self.development_stats['cortical_areas']} cortical areas, "
            f"{self.development_stats['total_neurons']} neurons, and "
            f"{self.development_stats['total_synapses']} synapses.",
        )

        # NOTE: Memory mapping rescan is handled automatically during genome processing
        # The rescan_all_memory_mappings() method is available for manual use if needed
        
        # Final comprehensive brain statistics update after all development is complete
        try:
            self.connectome_manager._update_brain_statistics()
            logger.info(
                f"✅ Final brain stats: {self.connectome_manager.get_neuron_count()} neurons, "
                f"{self.connectome_manager.get_synapse_count()} synapses"
            )
        except Exception as e:
            logger.warning(f"Failed to update final brain statistics: {e}")

        return True

    def get_development_statistics(self) -> Dict[str, Any]:
        """Get statistics about the brain development process."""
        return self.development_stats

    # ------------------------------------------------------------------
    # Region membership normalization (embryogenesis-level)
    # ------------------------------------------------------------------
    def _normalize_region_membership_for_embryogenesis(self, genome: Dict[str, Any], constraints) -> None:
        """Normalize region membership per system constraints for old genomes.

        Mutates the provided genome in-place.
        """
        if not genome or "blueprint" not in genome:
            raise ValueError("Genome missing required section: blueprint")

        blueprint = genome["blueprint"]
        # Initialize brain_regions/root if missing
        if "brain_regions" not in genome:
            genome["brain_regions"] = {}
        regions = genome["brain_regions"]
        if "root" not in regions:
            regions["root"] = {
                "title": "Root Brain Region",
                "description": "Default root region for brain organization",
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": [],
                "regions": [],
                "inputs": [],
                "outputs": [],
                "signature": "",
            }

        def get_region_for_area(area_def: Dict[str, Any]) -> str:
            params = area_def.get("parameters", {}) if isinstance(area_def, dict) else {}
            return (
                area_def.get("brain_region_id")
                or area_def.get("region_id")
                or params.get("brain_region_id")
                or params.get("region_id")
                or "root"
            )

        def classify(area_def: Dict[str, Any], area_id: str) -> str:
            # Treat special maintenance areas as CORE
            if isinstance(area_id, str) and area_id.startswith("_"):
                return "CORE"
            group_id = str(area_def.get("group_id", "")).upper()
            if group_id in {"IPU", "OPU", "CORE", "CUSTOM", "MEMORY"}:
                return group_id
            area_type = str(area_def.get("type", "")).lower()
            if area_type == "memory":
                return "MEMORY"
            if area_type == "custom":
                return "CUSTOM"
            params = area_def.get("parameters", {}) if isinstance(area_def, dict) else {}
            if str(params.get("sub_group_id", "")).upper() == "MEMORY":
                return "MEMORY"
            legacy_group = str(params.get("cortical_group", area_def.get("cortical_group", "")).upper())
            if legacy_group in {"IPU", "OPU", "CORE", "CUSTOM", "MEMORY"}:
                return legacy_group
            return "CUSTOM"

        # Move forbidden categories out of subregions to root
        for aid, adef in blueprint.items():
            current_region = get_region_for_area(adef)
            if current_region == "root":
                continue
            category = classify(adef, aid)
            if category not in constraints.subregion_allowed_area_categories:
                adef["brain_region_id"] = "root"
                adef["region_id"] = "root"
                params = adef.get("parameters")
                if isinstance(params, dict):
                    params["brain_region_id"] = "root"
                    params["region_id"] = "root"
                if current_region in regions:
                    old_list = regions[current_region].get("areas", []) or []
                    if aid in old_list:
                        regions[current_region]["areas"] = [x for x in old_list if x != aid]
                root_list = regions["root"].get("areas", []) or []
                if aid not in root_list:
                    root_list.append(aid)
                    regions["root"]["areas"] = root_list

        # Gather custom/memory under root
        movers = []
        for aid, adef in blueprint.items():
            if get_region_for_area(adef) != "root":
                continue
            if classify(adef, aid) in constraints.subregion_allowed_area_categories:
                movers.append(aid)

        if movers and constraints.auto_create_subregion_for_custom_in_root:
            import hashlib

            movers_sorted = sorted(movers)
            digest = hashlib.sha1("|".join(movers_sorted).encode("utf-8")).hexdigest()[:8]
            new_region_id = f"region_autogen_{digest}"
            if new_region_id not in regions:
                # Compute centroid from 2D/3D info
                xs: list[int] = []
                ys: list[int] = []
                zs: list[int] = []
                for m in movers_sorted:
                    adef = blueprint.get(m, {})
                    coords = adef.get("coordinates")
                    if isinstance(coords, dict) and {"x", "y", "z"}.issubset(coords.keys()):
                        xs.append(int(coords["x"]))
                        ys.append(int(coords["y"]))
                        zs.append(int(coords["z"]))
                    else:
                        params = adef.get("parameters", {})
                        x2 = params.get("2dcorx")
                        y2 = params.get("2dcory")
                        if x2 is None or y2 is None:
                            # Default to origin when 2D coordinates are missing, only for centroid calculation
                            xs.append(0)
                            ys.append(0)
                            zs.append(0)
                        else:
                            xs.append(int(x2))
                            ys.append(int(y2))
                            zs.append(0)
                cx = sum(xs) // len(xs)
                cy = sum(ys) // len(ys)
                cz = sum(zs) // len(zs)

                regions[new_region_id] = {
                    "title": "Autogen Region",
                    "description": "Auto-created to house custom/memory areas",
                    "parent_region_id": "root",
                    "coordinate_2d": [cx, cy],
                    "coordinate_3d": [cx, cy, cz],
                    "areas": [],
                    "regions": [],
                    "inputs": [],
                    "outputs": [],
                    "signature": "",
                }
                # Ensure root lists new subregion
                root_regions = regions["root"].get("regions", []) or []
                if new_region_id not in root_regions:
                    root_regions.append(new_region_id)
                    regions["root"]["regions"] = root_regions

            for m in movers_sorted:
                adef = blueprint.get(m, {})
                adef["brain_region_id"] = new_region_id
                adef["region_id"] = new_region_id
                params = adef.get("parameters")
                if isinstance(params, dict):
                    params["brain_region_id"] = new_region_id
                    params["region_id"] = new_region_id
                root_list = regions["root"].get("areas", []) or []
                if m in root_list:
                    regions["root"]["areas"] = [x for x in root_list if x != m]
                lst = regions[new_region_id].get("areas", []) or []
                if m not in lst:
                    lst.append(m)
                    regions[new_region_id]["areas"] = lst

        # Rebuild root.areas from blueprint assignments to ensure consistency
        root_allowed = set(constraints.root_allowed_area_categories)
        rebuilt_root_areas: list[str] = []
        for aid, adef in blueprint.items():
            reg = get_region_for_area(adef)
            if reg == "root" and classify(adef, aid) in root_allowed:
                rebuilt_root_areas.append(aid)
        regions["root"]["areas"] = rebuilt_root_areas

    def develop_brain_from_genome_data(
        self, genome_data: Dict[str, Any], additive_mode: bool = False
    ) -> bool:
        """Develop a brain from genome data directly (not from file).

        This method is used when the genome data is already loaded and sanitized
        in the state manager, ensuring single source of truth architecture.

        Args:
            genome_data: The genome dictionary data
            additive_mode: If True, skip brain reset and only add new structures (for cloning)

        Returns:
            True if brain developed successfully, False otherwise
        """
        # Store additive mode for use in other methods
        self.additive_mode = additive_mode
        
        self.development_stats["start_time"] = datetime.datetime.now()

        # Enforce brain region membership normalization (handles legacy genomes)
        try:
            # Use the same system constants as the service layer
            from feagi.config.toml_loader import get_region_constraints_config
            constraints = get_region_constraints_config({})
            if isinstance(genome_data, dict):
                # Mutate in-place
                self._normalize_region_membership_for_embryogenesis(genome_data, constraints)
                # Persist normalized genome into StateManager before proceeding
                try:
                    from feagi.core.state_manager import FeagiStateManager
                    sm = FeagiStateManager.instance()
                    if hasattr(sm, 'set_genome'):
                        _ = sm.set_genome(genome_data)
                except Exception:
                    # Do not fail development if state manager persistence is unavailable here
                    pass
        except Exception as norm_err:
            logger.error(f"Region membership normalization failed in embryogenesis: {norm_err}")
            return False

        # Brain reset logic - skip in additive mode (for cloning operations)
        if not additive_mode:
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
        else:
            logger.info(
                "Additive mode: Skipping brain reset, preserving existing structures"
            )

        # Validate and load genome data directly
        if not self._load_genome_data(genome_data):
            return False

        # Ensure NPU/SynapseArray readiness before any neuro/synapto-genesis
        try:
            cm = self.connectome_manager
            # If NPU is not set, initialize via ConnectomeManager (uses configured backend)
            if not hasattr(cm, "_npu_interface") or cm._npu_interface is None:
                # Derive backend from config if available, default handled inside initializer
                backend = None
                if hasattr(self.state_manager, "config"):
                    try:
                        backend = self.state_manager.config.get("connectome.backend", None)
                    except Exception:
                        backend = None
                if hasattr(cm, "_initialize_npu_interface"):
                    cm._initialize_npu_interface(backend)
            # Optional: dynamic capacity sizing from genome/config (ConnectomeManager already supports it)
            # Here we ensure the CM exposes max_synapses consistent with NPU
            if hasattr(cm, "_npu_interface") and cm._npu_interface is not None:
                # Sync references to ensure synapse_array is available
                if getattr(cm, "synapse_array", None) is None and hasattr(cm._npu_interface, "synapse_array"):
                    cm.synapse_array = cm._npu_interface.synapse_array
                if getattr(cm, "neuron_array", None) is None and hasattr(cm._npu_interface, "neuron_array"):
                    cm.neuron_array = cm._npu_interface.neuron_array
        except Exception as npu_init_err:
            logger.error(f"Failed to ensure NPU/SynapseArray readiness: {npu_init_err}")
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

        # Final step: Validate and update brain region mappings
        if not self._validate_and_update_brain_region_mappings():
            logger.warning("Brain region mapping validation failed, but continuing with brain development")

        # Final report
        self._report_progress(
            DevelopmentStage.COMPLETED,
            100,
            f"Brain development completed in {self.development_stats['duration']}. "
            f"Created {self.development_stats['cortical_areas']} cortical areas, "
            f"{self.development_stats['total_neurons']} neurons, and "
            f"{self.development_stats['total_synapses']} synapses.",
        )
        
        # Final comprehensive brain statistics update after all development is complete
        try:
            self.connectome_manager._update_brain_statistics()
            logger.info(
                f"✅ Final brain stats: {self.connectome_manager.get_neuron_count()} neurons, "
                f"{self.connectome_manager.get_synapse_count()} synapses"
            )
        except Exception as e:
            logger.warning(f"Failed to update final brain statistics: {e}")

        # CRITICAL DEBUG: Check actual counts at completion of neuroembryogenesis
        logger.info("🧠 [NEUROEMBRYOGENESIS] Brain development completed - checking actual counts:")
        
        # Get actual counts from connectome manager
        if self.connectome_manager:
            actual_neuron_count = self.connectome_manager.get_neuron_count() if hasattr(self.connectome_manager, 'get_neuron_count') else "N/A"
            actual_synapse_count = self.connectome_manager.synapse_count if hasattr(self.connectome_manager, 'synapse_count') else "N/A"
            actual_cortical_areas = len(getattr(self.connectome_manager, 'cortical_areas', {}))
            
            logger.info("🧠 [NEUROEMBRYOGENESIS] ConnectomeManager actual counts:")
            logger.info(f"  - Neurons: {actual_neuron_count}")
            logger.info(f"  - Synapses: {actual_synapse_count}")
            logger.info(f"  - Cortical areas: {actual_cortical_areas}")
        
        # Check state manager counters if available
        if hasattr(self, 'state_manager') and self.state_manager:
            brain_stats = self.state_manager.get_brain_stats() if hasattr(self.state_manager, 'get_brain_stats') else None
            genome_loaded = self.state_manager.is_genome_loaded() if hasattr(self.state_manager, 'is_genome_loaded') else "N/A"
            
            logger.info("🧠 [NEUROEMBRYOGENESIS] StateManager counters:")
            logger.info(f"  - Brain stats: {brain_stats}")
            logger.info(f"  - Genome loaded: {genome_loaded}")
            
            # Also check direct state access
            if hasattr(self.state_manager, '_state'):
                direct_neuron_count = getattr(self.state_manager._state, 'neuron_count', 'N/A')
                direct_synapse_count = getattr(self.state_manager._state, 'synapse_count', 'N/A')
                logger.info(f"  - Direct state neuron_count: {direct_neuron_count}")
                logger.info(f"  - Direct state synapse_count: {direct_synapse_count}")

        # Populate Rust Morton spatial hash from all neurons for ultra-fast lookups
        if hasattr(self.connectome_manager, 'populate_morton_hash_from_existing_neurons'):
            try:
                self.connectome_manager.populate_morton_hash_from_existing_neurons()
            except Exception as e:
                logger.warning(f"Failed to populate Morton hash: {e}")
        
        logger.info("🧠 [NEUROEMBRYOGENESIS] ✅ Returning True - brain development complete")

        return True

    def _validate_and_update_brain_region_mappings(self) -> bool:
        """Validate and update brain region I/O mappings based on cortical mappings.
        
        This method ensures brain region mappings are sound by:
        1. Analyzing all cortical mappings
        2. Automatically assigning IPU areas as inputs to their regions
        3. Automatically assigning OPU areas as outputs to their regions  
        4. Applying cross-region mapping rules for automatic I/O designation
        5. Updating the genome with corrected brain region mappings
        
        Returns:
            True if validation and updates completed successfully
        """
        try:
            logger.info("🧠 [BRAIN REGIONS] Starting brain region mapping validation...")
            
            # Get current genome from state manager
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            
            if not state_manager or not hasattr(state_manager, 'genome'):
                logger.error("No state manager or genome available for brain region validation")
                return False
                
            genome = state_manager.genome
            logger.info(f"🧠 [BRAIN REGIONS] StateManager genome has brain_regions: {bool(genome.get('brain_regions') if genome else False)}")
            if not genome:
                logger.error("No genome data available for brain region validation")
                return False
                
            brain_regions = genome.get("brain_regions", {})
            blueprint = genome.get("blueprint", {})
            
            if not brain_regions:
                logger.info("No brain regions found - skipping validation")
                return True
                
            logger.info(f"🧠 [BRAIN REGIONS] Validating {len(brain_regions)} brain regions...")
            
            # Log initial state of root region
            if 'root' in brain_regions:
                root_before = brain_regions['root']
                logger.info("🧠 [BRAIN REGIONS] Root region BEFORE validation:")
                logger.info(f"   - Areas: {len(root_before.get('areas', []))} total")
                logger.info(f"   - Inputs: {root_before.get('inputs', [])}")
                logger.info(f"   - Outputs: {root_before.get('outputs', [])}")
            
            # Step 1: Auto-assign IPU/OPU areas to region inputs/outputs
            self._auto_assign_ipu_opu_to_regions(brain_regions, blueprint)
            
            # Step 2: Apply cross-region mapping rules
            self._apply_cross_region_mapping_rules(brain_regions, blueprint)
            
            # Step 3: Ensure parent-child relationships are correct
            self._validate_parent_child_relationships(brain_regions)
            
            # Step 4: Update genome with corrected mappings
            state_manager.genome = genome
            
            # Step 5: Sync to ConnectomeManager brain region hierarchy
            if hasattr(self.connectome_manager, 'brain_region_hierarchy'):
                try:
                    self.connectome_manager.brain_region_hierarchy.load_from_genome(genome)
                    logger.info("🧠 [BRAIN REGIONS] Synced hierarchy system with updated mappings")
                except Exception as e:
                    logger.warning(f"Failed to sync brain region hierarchy: {e}")
            
            # Step 6: Force update ConnectomeManager's brain_regions cache
            if hasattr(self.connectome_manager, 'brain_regions'):
                try:
                    self.connectome_manager.brain_regions.update(brain_regions)
                    logger.info("🧠 [BRAIN REGIONS] Updated ConnectomeManager brain_regions cache")
                except Exception as e:
                    logger.warning(f"Failed to update ConnectomeManager brain_regions cache: {e}")
            
            # Log final state of root region
            if 'root' in brain_regions:
                root_after = brain_regions['root']
                logger.info("🧠 [BRAIN REGIONS] Root region AFTER validation:")
                logger.info(f"   - Areas: {len(root_after.get('areas', []))} total")
                logger.info(f"   - Inputs: {root_after.get('inputs', [])}")
                logger.info(f"   - Outputs: {root_after.get('outputs', [])}")
            
            logger.info("🧠 [BRAIN REGIONS] Brain region mapping validation completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during brain region mapping validation: {e}")
            return False

    def _auto_assign_ipu_opu_to_regions(self, brain_regions: Dict[str, Any], blueprint: Dict[str, Any]) -> None:
        """Automatically assign IPU areas as inputs and OPU areas as outputs to their regions."""
        logger.info(f"🧠 [BRAIN REGIONS] *** AUTO-ASSIGN FUNCTION CALLED *** Starting IPU/OPU auto-assignment for {len(brain_regions)} regions")
        logger.info(f"🧠 [BRAIN REGIONS] Blueprint keys: {list(blueprint.keys())[:10]}")
        
        for region_id, region_data in brain_regions.items():
            areas = region_data.get("areas", region_data.get("cortical_areas", []))
            
            logger.info(f"🧠 [BRAIN REGIONS] Processing region {region_id} with {len(areas)} areas")
            
            if not areas:
                logger.info(f"🧠 [BRAIN REGIONS] Region {region_id} has no areas, skipping")
                continue
                
            # Initialize inputs/outputs if not present
            if "inputs" not in region_data:
                region_data["inputs"] = []
            if "outputs" not in region_data:
                region_data["outputs"] = []
                
            inputs = set(region_data["inputs"])
            outputs = set(region_data["outputs"])
            
            # Auto-assign based on area types
            ipu_count = 0
            opu_count = 0
            
            logger.info(f"🧠 [BRAIN REGIONS] Areas: {areas}")
            for area_id in areas:
                logger.info(f"🧠 [BRAIN REGIONS] Processing area {area_id}")
                area_props = blueprint.get(area_id, {})
                # Robust detection across fields: group, cortical_group, group_id, cortical_type
                raw_group = (
                    area_props.get("group")
                    or area_props.get("cortical_group")
                    or area_props.get("group_id")
                    or ""
                )
                raw_type = area_props.get("cortical_type") or ""
                area_group = str(raw_group).upper()
                area_type = str(raw_type).upper()
                
                logger.info(
                    f"🧠 [BRAIN REGIONS] Area {area_id}: props_keys={list(area_props.keys())}, group='{area_group}', cortical_group='{area_props.get('cortical_group', 'NONE')}', cortical_type='{area_type}'"
                )
                logger.debug(f"🧠 [BRAIN REGIONS] Area {area_id}: group='{area_group}', type='{area_type}'")
                
                if (area_group == "IPU" or area_type == "IPU") and area_id not in inputs:
                    region_data["inputs"].append(area_id)
                    ipu_count += 1
                    logger.info(f"🧠 [BRAIN REGIONS] Auto-assigned {area_id} (IPU) as input to {region_id}")
                elif (area_group == "OPU" or area_type == "OPU") and area_id not in outputs:
                    region_data["outputs"].append(area_id)
                    opu_count += 1
                    logger.info(f"🧠 [BRAIN REGIONS] Auto-assigned {area_id} (OPU) as output to {region_id}")
            
            logger.info(f"🧠 [BRAIN REGIONS] Region {region_id}: assigned {ipu_count} IPU inputs, {opu_count} OPU outputs")

    def _apply_cross_region_mapping_rules(self, brain_regions: Dict[str, Any], blueprint: Dict[str, Any]) -> None:
        """Apply cross-region mapping rules for automatic I/O designation."""
        # Get all cortical mappings from the blueprint
        all_mappings = []
        
        for area_id, area_props in blueprint.items():
            mappings = area_props.get("cortical_mapping_dst", {})
            for target_area, mapping_list in mappings.items():
                if mapping_list:  # Only if there are actual mappings
                    all_mappings.append((area_id, target_area))
        
        logger.info(f"🧠 [BRAIN REGIONS] Processing {len(all_mappings)} cortical mappings for cross-region rules")
        
        # Apply the cross-region rule for each mapping
        for source_area, target_area in all_mappings:
            self._apply_mapping_rule_to_regions(source_area, target_area, brain_regions)

    def _apply_mapping_rule_to_regions(self, source_area: str, target_area: str, brain_regions: Dict[str, Any]) -> None:
        """Apply the cross-region mapping rule for a specific cortical mapping."""
        # Find which regions contain these areas
        source_region_id = None
        target_region_id = None
        
        for region_id, region_data in brain_regions.items():
            areas = region_data.get("areas", region_data.get("cortical_areas", []))
            
            if source_area in areas:
                source_region_id = region_id
            if target_area in areas:
                target_region_id = region_id
                
        if not source_region_id or not target_region_id:
            logger.debug(f"🧠 [BRAIN REGIONS] Areas not found in regions: {source_area} → {target_area}")
            return  # Areas not found in any region
            
        if source_region_id == target_region_id:
            logger.debug(f"🧠 [BRAIN REGIONS] Same region mapping: {source_area} → {target_area} (both in {source_region_id})")
            return  # Same region - no cross-region rule needed
            
        logger.debug(f"🧠 [BRAIN REGIONS] Cross-region mapping detected: {source_area} ({source_region_id}) → {target_area} ({target_region_id})")
        
        # Apply the rule: check if source region is NOT in target region's ancestry
        is_ancestor = self._is_region_ancestor(source_region_id, target_region_id, brain_regions)
        logger.debug(f"🧠 [BRAIN REGIONS] Is {source_region_id} ancestor of {target_region_id}? {is_ancestor}")
        
        if is_ancestor:
            # Outside → inside (source region is ancestor of target): input only on target
            target_region = brain_regions[target_region_id]
            if "inputs" not in target_region:
                target_region["inputs"] = []
            if target_area not in target_region["inputs"]:
                target_region["inputs"].append(target_area)
                logger.info(f"🧠 [BRAIN REGIONS] Cross-region rule: {target_area} → INPUT in {target_region_id} (ancestor → descendant)")
        else:
            # Different branches or child → parent: designate both
            source_region = brain_regions[source_region_id]
            if "outputs" not in source_region:
                source_region["outputs"] = []
            if source_area not in source_region["outputs"]:
                source_region["outputs"].append(source_area)
                logger.info(f"🧠 [BRAIN REGIONS] Cross-region rule: {source_area} → OUTPUT in {source_region_id}")
            
            target_region = brain_regions[target_region_id]
            if "inputs" not in target_region:
                target_region["inputs"] = []
            if target_area not in target_region["inputs"]:
                target_region["inputs"].append(target_area)
                logger.info(f"🧠 [BRAIN REGIONS] Cross-region rule: {target_area} → INPUT in {target_region_id}")

    def _is_region_ancestor(self, ancestor_id: str, descendant_id: str, brain_regions: Dict[str, Any]) -> bool:
        """Check if one region is an ancestor of another in the hierarchy."""
        if ancestor_id == descendant_id:
            return False
            
        current_id = descendant_id
        visited = set()
        
        while current_id and current_id not in visited:
            visited.add(current_id)
            
            if current_id not in brain_regions:
                break
                
            parent_id = brain_regions[current_id].get("parent_region_id")
            if parent_id == ancestor_id:
                return True
                
            current_id = parent_id
            
        return False

    def _validate_parent_child_relationships(self, brain_regions: Dict[str, Any]) -> None:
        """Ensure parent-child relationships are consistent in both directions."""
        # Build parent -> children mapping
        for region_id, region_data in brain_regions.items():
            parent_id = region_data.get("parent_region_id")
            
            if parent_id and parent_id in brain_regions:
                parent_region = brain_regions[parent_id]
                
                # Ensure parent has consistent 'regions' field
                if "regions" not in parent_region:
                    parent_region["regions"] = []
                if region_id not in parent_region["regions"]:
                    parent_region["regions"].append(region_id)
                    logger.debug(f"🧠 [BRAIN REGIONS] Added {region_id} to parent {parent_id} regions list")

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
                # Convert to BDU format: {src_area: {dst_area: [connection_specs]}}
                logger.info("Converting EVO mapping format to BDU format")
                converted_mapping = {}

                # For EVO format, we assume the mapping key represents both source and destination
                # This is the correct interpretation without hardcoded assumptions
                for area_id, connection_specs in mapping.items():
                    # In EVO format, the key is typically the source area
                    src_area_id = area_id
                    
                    # Extract destination areas from the connection specs
                    # The actual destination should be determined from the connection specifications
                    # not hardcoded assumptions
                    if src_area_id not in converted_mapping:
                        converted_mapping[src_area_id] = {}
                    
                    # For now, assume self-connection if no explicit destination is provided
                    # This should be enhanced based on actual connection spec structure
                    dst_area_id = area_id  # Default to self-connection
                    converted_mapping[src_area_id][dst_area_id] = connection_specs

                    logger.info(
                        f"Converted mapping {src_area_id} -> {dst_area_id} with {len(connection_specs)} specs"
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
            memory_mappings_processed = 0

            for src_area_id, target_mappings in mapping.items():
                if not isinstance(target_mappings, dict):
                    logger.warning(
                        f"Invalid mapping format for area {src_area_id}"
                    )
                    continue

                # Get source area (existence check - area should have neurons after neurogenesis)
                if src_area_id not in self.connectome_manager.cortical_areas:
                    logger.warning(f"Source area {src_area_id} doesn't exist")
                    continue
                
                src_area = self.connectome_manager.cortical_areas[src_area_id]

                # CRITICAL: All cortical areas should ALWAYS have properties initialized
                # If not, this indicates a serious bug in area creation
                if not hasattr(src_area, "properties"):
                    logger.error(f"💥 [NEURO-MAPPING] CRITICAL BUG: {src_area_id} missing properties attribute!")
                    raise RuntimeError(f"Cortical area {src_area_id} is missing properties attribute - this indicates a bug in area creation")
                
                if src_area.properties is None:
                    logger.error(f"💥 [NEURO-MAPPING] CRITICAL BUG: {src_area_id} has None properties!")
                    raise RuntimeError(f"Cortical area {src_area_id} has None properties - this indicates a bug in area creation")
                
                # Properties should always exist and be a dict - log current state
                existing_count = len(src_area.properties)
                has_destinations = 'cortical_destinations' in src_area.properties
                logger.info(f"🧠 [NEURO-MAPPING] {src_area_id}: existing_props={existing_count}, has_destinations={has_destinations}")
                
                # Properties are guaranteed to exist and be a dict - just proceed with adding mapping

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
                logger.info(f"🧠 [NEURO-MAPPING] {src_area_id}: Setting mapping with {len(api_mapping)} destinations")
                src_area.properties["mapping"] = api_mapping
                
                # CRITICAL DEBUG: Verify cortical_destinations survived mapping assignment
                final_count = len(src_area.properties)
                final_has_destinations = 'cortical_destinations' in src_area.properties
                logger.info(f"🧠 [NEURO-MAPPING] {src_area_id}: AFTER mapping - total_props={final_count}, has_destinations={final_has_destinations}")
                
                if has_destinations and not final_has_destinations:
                    logger.error(f"💥 [NEURO-MAPPING] CORRUPTION: {src_area_id} lost cortical_destinations during mapping assignment!")
                
                # Trigger automatic I/O designation for each target area
                # Only when crossing region boundaries
                try:
                    if hasattr(self.connectome_manager, 'brain_region_hierarchy'):
                        source_region = self.connectome_manager.brain_region_hierarchy.get_region_for_area(src_area_id)
                        for dst_area_id in api_mapping.keys():
                            target_region = self.connectome_manager.brain_region_hierarchy.get_region_for_area(dst_area_id)
                            # Update only if regions differ
                            if source_region and target_region and source_region != target_region:
                                try:
                                    self.connectome_manager.on_cortical_mapping_created(src_area_id, dst_area_id)
                                    logger.debug(
                                        f"🧠 [MAPPING-DEBUG] Cross-region I/O designation {src_area_id}({source_region}) -> {dst_area_id}({target_region})"
                                    )
                                except Exception as e:
                                    logger.warning(
                                        f"🧠 [MAPPING-DEBUG] Failed cross-region I/O designation for {src_area_id} -> {dst_area_id}: {e}"
                                    )
                except Exception as e:
                    logger.warning(f"🧠 [MAPPING-DEBUG] Skipped automatic I/O designation: {e}")
                
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

                            #  NON-MEMORY MORPHOLOGIES: proceed with synaptogenesis
                            #  Check destination area exists (fast check)
                            if dst_area_id not in self.connectome_manager.cortical_areas:
                                logger.warning(
                                    f"Destination area {dst_area_id} doesn't exist"
                                )
                                continue

                            # DIAGNOSTIC: Log EXACT parameters being passed to _apply_morphology_mapping
                            logger.info(
                                "🧠 [API-DIAGNOSTIC] EXACT PARAMETERS for _apply_morphology_mapping:"
                            )
                            logger.info(f"   src_area_id: {src_area_id}")
                            logger.info(f"   dst_area_id: {dst_area_id}")
                            logger.info(f"   src_neurons count: {len(src_neurons)}")
                            logger.info(f"   dst_neurons count: {len(dst_neurons)}")
                            logger.info(f"   src_neurons sample: {sorted(src_neurons)[:5] if src_neurons else []}")
                            logger.info(f"   dst_neurons sample: {sorted(dst_neurons)[:5] if dst_neurons else []}")
                            logger.info(f"   morphology_id: {morphology_id}")
                            logger.info(f"   morphology_scalar: {morphology_scalar}")
                            logger.info(f"   psc_multiplier: {psc_multiplier}")
                            logger.info(f"   plasticity_flag: {plasticity_flag}")
                            logger.info(f"   plasticity_constant: {plasticity_constant}")
                            logger.info(f"   ltp_multiplier: {ltp_multiplier}")
                            logger.info(f"   ltd_multiplier: {ltd_multiplier}")
                            
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
                
                # CRITICAL: Rebuild Rust NPU synapse index after synapse creation
                # Neuroembryogenesis creates synapses via morphology, so Rust NPU needs to rebuild its synapse index
                try:
                    # 🦀 RUST: Access Rust NPU through ProcessManager
                    from feagi.process_manager import get_process_manager
                    pm = get_process_manager()
                    rust_npu_integration = getattr(pm, 'rust_npu_integration', None)
                    
                    if rust_npu_integration and rust_npu_integration._rust_npu:
                        logger.info(f"🦀 [RUST-NPU] Neuroembryogenesis created {total_synapses_created} synapses - rebuilding synapse index...")
                        try:
                            # Rebuild synapse index so new synapses are active
                            rust_npu_integration.rebuild_synapse_index()
                            logger.info("🦀 [RUST-NPU] ✅ Synapse index rebuilt successfully - new synapses are now active")
                        except Exception as rebuild_error:
                            logger.error(f"🦀 [RUST-NPU] Failed to rebuild synapse index: {rebuild_error}")
                            logger.warning("🦀 [RUST-NPU] ⚠️ New synapses will not be active until FEAGI restart")
                    else:
                        logger.debug("🦀 [RUST-NPU] Not yet initialized - new synapses will be indexed on first burst")
                except Exception as rust_error:
                    logger.error(f"🦀 [RUST-NPU] Error during synapse index reload: {rust_error}")
                    logger.exception("Full stack trace:")
                
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
                    #  invalid morphologies should not be considered failures, just no-ops
                    return True

        except Exception as e:
            logger.error(f"Error updating cortical mapping: {e}")
            return False

    def _batch_get_source_positions(
        self, src_area_id: str, src_neurons: List[int]
    ) -> Tuple[Dict[int, Tuple[int, int, int]], List[int]]:
        """Batch retrieve source neuron positions from Rust NPU.
        
        Args:
            src_area_id: Source cortical area ID
            src_neurons: List of source neuron IDs
            
        Returns:
            Tuple of (position_map, valid_neurons) where position_map is neuron_id -> (x, y, z)
        """
        src_cortical_idx = self.connectome_manager.cortical_mapping.get_idx(src_area_id)
        all_src_positions = self.connectome_manager._npu_interface.rust_npu.get_neuron_positions_in_cortical_area(src_cortical_idx)
        src_pos_map = {int(nid): (int(x), int(y), int(z)) for nid, x, y, z in all_src_positions}
        
        # Filter to requested neurons
        valid_neurons = [nid for nid in src_neurons if nid in src_pos_map]
        return src_pos_map, valid_neurons

    def _batch_get_destination_neurons(
        self, dst_area_id: str, psc_multiplier: float
    ) -> Dict[Tuple[int, int, int], List[Tuple[int, float]]]:
        """Batch retrieve destination neurons and build position-to-neurons mapping.
        
        Args:
            dst_area_id: Destination cortical area ID
            psc_multiplier: Post-synaptic current multiplier
            
        Returns:
            Dictionary mapping (x, y, z) -> [(neuron_id, weight), ...]
        """
        dst_cortical_idx = self.connectome_manager.cortical_mapping.get_idx(dst_area_id)
        all_dst_positions_data = self.connectome_manager._npu_interface.rust_npu.get_neuron_positions_in_cortical_area(dst_cortical_idx)
        dst_pos_to_neurons = {}
        for nid, x, y, z in all_dst_positions_data:
            pos = (int(x), int(y), int(z))
            if pos not in dst_pos_to_neurons:
                dst_pos_to_neurons[pos] = []
            dst_pos_to_neurons[pos].append((int(nid), psc_multiplier))
        return dst_pos_to_neurons

    def _propagate_memory_register(self, memory_register: Dict[str, set]) -> None:
        """Propagate memory register mappings to ConnectomeManager.
        
        Args:
            memory_register: Dictionary mapping memory_area_id -> set of upstream_area_ids
        """
        if not memory_register:
            return
            
        logger.info(
            f"[MEMORY-PROPAGATION] Memory register found with {len(memory_register)} entries: {memory_register}"
        )
        for memory_area_id, upstream_area_ids in memory_register.items():
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
                    logger.exception("[MEMORY-PROPAGATION] Full exception trace:")
        logger.info(
            "[MEMORY-PROPAGATION] Completed processing all memory register entries"
        )

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

                #  Populate memory register (inlined from old syn_memory function)
                memory_register = {}
                if dst_area_id not in memory_register:
                    memory_register[dst_area_id] = set()
                memory_register[dst_area_id].add(src_area_id)
                logger.info(
                    f"[MEMORY-MORPHOLOGY] Memory register updated: {memory_register}"
                )
                
                # Propagate memory register to ConnectomeManager
                self._propagate_memory_register(memory_register)

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
            synapse_connections = []  # Accumulate ALL synapses across ALL vectors for single batch creation
            
            # Get destination area dimensions ONCE (shared by all vectors)
            dst_area = self.connectome_manager.get_cortical_area(dst_area_id)
            if not dst_area:
                logger.warning(f"Cannot get destination area {dst_area_id}")
                return 0
            dst_dimensions = dst_area.dimensions
            
            # PHASE 1: Accumulate all candidate positions from ALL vectors
            all_candidate_positions = set()  # (x, y, z) tuples
            vector_mapping_data = []  # Store (valid_candidate_positions, valid_source_neurons) for each vector
            
            for vector in vectors:
                # Get morphology scalar (default to 1.0 if not provided)
                scalar = morphology_scalar[0] if morphology_scalar else 1.0
                
                # Apply vector to ALL positions at once (numpy broadcasting)
                vector_array = np.array(vector) * scalar
                candidate_positions = source_positions + vector_array
                
                # Filter candidate positions to be within bounds
                valid_mask = (
                    (candidate_positions[:, 0] >= 0)
                    & (candidate_positions[:, 0] < dst_dimensions[0])
                    & (candidate_positions[:, 1] >= 0)
                    & (candidate_positions[:, 1] < dst_dimensions[1])
                    & (candidate_positions[:, 2] >= 0)
                    & (candidate_positions[:, 2] < dst_dimensions[2])
                )
                
                valid_candidate_positions = candidate_positions[valid_mask]
                valid_source_neurons = source_neuron_ids[valid_mask]
                
                if len(valid_candidate_positions) > 0:
                    # Store mapping data for later processing
                    vector_mapping_data.append((valid_candidate_positions, valid_source_neurons))
                    
                    # Accumulate unique positions for batch lookup
                    for pos in valid_candidate_positions:
                        all_candidate_positions.add(tuple(pos))
            
            if not all_candidate_positions:
                logger.debug("[VECTOR-NUMPY] No valid candidate positions after processing all vectors")
                return 0
            
            logger.info(f"[VECTOR-NUMPY] Processed {len(vectors)} vectors → {len(all_candidate_positions)} unique candidate positions")
            
            # PHASE 2: ONE batch lookup for ALL candidate positions from ALL vectors (MASSIVE performance gain!)
            neuron_weight_pairs = self.connectome_manager.batch_voxel_to_neuron_lookup(
                cortical_id=dst_area_id,
                candidate_positions=all_candidate_positions,
                post_synaptic_current=psc_multiplier,
            )
            
            if not neuron_weight_pairs:
                logger.debug("[VECTOR-NUMPY] No neurons found at any candidate positions")
                return 0
            
            # PHASE 3: Create position-to-neurons mapping
            position_to_neurons = {}  # (x, y, z) → list of (neuron_id, weight)
            npu_interface = getattr(self.connectome_manager, '_npu_interface', None)
            
            if npu_interface is not None:
                # Get positions for all found neurons from Rust NPU
                for neuron_id, weight in neuron_weight_pairs:
                    position = npu_interface.get_neuron_position(neuron_id)
                    if position is not None:
                        pos_tuple = tuple(position)
                        if pos_tuple not in position_to_neurons:
                            position_to_neurons[pos_tuple] = []
                        position_to_neurons[pos_tuple].append((neuron_id, weight))
            
            # PHASE 4: Match source neurons to target neurons for ALL vectors
            for valid_candidate_positions, valid_source_neurons in vector_mapping_data:
                for i, candidate_pos in enumerate(valid_candidate_positions):
                    candidate_pos_tuple = tuple(candidate_pos)
                    if candidate_pos_tuple in position_to_neurons:
                        src_neuron_id = valid_source_neurons[i]
                        for dst_neuron_id, weight in position_to_neurons[candidate_pos_tuple]:
                            synapse_connections.append((src_neuron_id, dst_neuron_id, weight))

            # Step 8: Batch create ALL accumulated synapses in ONE call (massive performance improvement!)
            if synapse_connections:
                created = self.connectome_manager.batch_create_synapses(
                    synapse_connections
                )
                total_synapses += created
                logger.info(
                    f"[VECTOR-NUMPY] Created {created} synapses in single batch for all vectors"
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
        """Process pattern-based morphology using Rust batch processing.

        ARCHITECTURE: Uses Rust pattern matching for 100x+ performance.
        PERFORMANCE: Optimized for Rust/RTOS/SIMD/GPU compatibility.
        """
        try:
            from feagi_bdu import py_match_patterns
            
            total_synapses = 0
            patterns = morphology_def.get("parameters", {}).get("patterns", [])

            if not patterns:
                logger.warning(
                    "No patterns found in morphology definition for pattern type"
                )
                return 0

            # Get source and destination dimensions
            src_area = self.connectome_manager.get_cortical_area(src_area_id)
            dst_area = self.connectome_manager.get_cortical_area(dst_area_id)
            if not src_area or not dst_area:
                logger.error(f"Cannot get areas: {src_area_id} or {dst_area_id}")
                return 0

            src_dimensions = src_area.dimensions
            dst_dimensions = dst_area.dimensions
            
            # Convert Python patterns to Rust integer patterns
            # -1 = wildcard "*", -2 = skip "?", -3 = exclude "!", >= 0 = exact value
            def convert_pattern_element(elem):
                if elem == "*":
                    return -1
                elif elem == "?":
                    return -2
                elif elem == "!":
                    return -3
                else:
                    return int(elem)
            
            rust_patterns = []
            for pattern in patterns:
                if len(pattern) >= 2:
                    src_pattern = tuple(convert_pattern_element(e) for e in pattern[0])
                    dst_pattern = tuple(convert_pattern_element(e) for e in pattern[1])
                    rust_patterns.append((src_pattern, dst_pattern))
            
            if not rust_patterns:
                logger.warning("No valid patterns after conversion")
                return 0

            # Batch get all source positions (use helper)
            src_pos_map, valid_neurons = self._batch_get_source_positions(src_area_id, src_neurons)
            
            if not valid_neurons:
                logger.warning("No valid source neurons with positions")
                return 0

            logger.info(f"🦀 RUST PATTERNS: Processing {len(valid_neurons)} neurons with {len(rust_patterns)} patterns")
            
            # Batch get ALL destination neurons ONCE (use helper)
            dst_pos_to_neurons = self._batch_get_destination_neurons(dst_area_id, psc_multiplier)
            
            # Process each source neuron with Rust pattern matching
            all_synapse_connections = []
            import time
            start = time.time()
            
            for src_neuron_id in valid_neurons:
                src_pos = src_pos_map[src_neuron_id]
                
                # Call Rust pattern matcher (FAST!)
                matched_positions = py_match_patterns(
                    src_pos,
                    rust_patterns,
                    src_dimensions,
                    dst_dimensions
                )
                
                # Match to actual neurons
                for dst_pos in matched_positions:
                    if dst_pos in dst_pos_to_neurons:
                        for dst_neuron_id, weight in dst_pos_to_neurons[dst_pos]:
                            all_synapse_connections.append((src_neuron_id, dst_neuron_id, weight))
            
            elapsed = (time.time() - start) * 1000
            logger.info(f"🦀 RUST PATTERNS: {len(valid_neurons)} neurons → {len(all_synapse_connections)} synapses in {elapsed:.1f}ms")

            # Create ALL synapses in ONE batch call
            if all_synapse_connections:
                total_synapses = self.connectome_manager.batch_create_synapses(
                    all_synapse_connections
                )
                logger.info(
                    f"✅ PATTERN BATCH: Created {total_synapses} synapses from {len(src_neurons)} source neurons"
                )

            return total_synapses

        except ImportError:
            logger.error("Rust BDU not available for pattern processing. Run: cd feagi-rust && ./build_bdu.sh")
            return 0
        except Exception as e:
            logger.error(f"Error in pattern morphology processing: {e}")
            import traceback
            logger.error(traceback.format_exc())
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

            # Get synapse attractivity once (same for all neurons in area)
            dst_area = self.connectome_manager.get_cortical_area(dst_area_id)
            synapse_attractivity = dst_area.properties.get("synatt", 100)

            # Accumulate ALL synapses across all source neurons for single batch creation
            all_synapse_connections = []

            # RUST OPTIMIZATION: Batch process morphologies
            logger.info(f"🔍 Checking morphology_id='{morphology_id}' (type={type(morphology_id)})")
            
            # PROJECTOR batch
            if morphology_id.lower() == "projector":
                logger.info(f"🦀 RUST BATCH: PROJECTOR for {len(src_neurons)} neurons")
                try:
                    from feagi_bdu import py_syn_projector_batch
                    import time
                    start = time.time()
                    
                    # Batch get source positions (use helper)
                    src_pos_map, valid_neurons = self._batch_get_source_positions(src_area_id, src_neurons)
                    neuron_positions = [src_pos_map[nid] for nid in valid_neurons]
                    
                    logger.info(f"🦀 Got {len(valid_neurons)} valid neurons with positions")
                    
                    if valid_neurons:
                        # Batch call Rust (processes all neurons in parallel)
                        results = py_syn_projector_batch(
                            src_area_id, dst_area_id,
                            valid_neurons, neuron_positions,
                            src_area.dimensions, dst_area.dimensions,
                            None, None
                        )
                        
                        logger.info(f"🦀 Rust batch projection complete, getting destination neurons")
                        
                        # Batch get destination neurons (use helper)
                        dst_pos_to_neurons = self._batch_get_destination_neurons(dst_area_id, psc_multiplier)
                        
                        logger.info(f"🦀 Built destination map with {len(dst_pos_to_neurons)} positions")
                        
                        # Match source neurons to targets
                        for src_idx, src_neuron_id in enumerate(valid_neurons):
                            for dst_pos in results[src_idx]:
                                if dst_pos in dst_pos_to_neurons:
                                    for dst_neuron_id, weight in dst_pos_to_neurons[dst_pos]:
                                        if random.randrange(1, 100) < synapse_attractivity:
                                            all_synapse_connections.append((src_neuron_id, dst_neuron_id, weight))
                        
                        elapsed = (time.time() - start) * 1000
                        logger.info(f"🦀 RUST BATCH: {len(valid_neurons)} neurons projected in {elapsed:.1f}ms ({len(all_synapse_connections)} synapses)")
                    
                    # Skip the loop - already processed
                    src_neurons = []
                    
                except ImportError as e:
                    logger.warning(f"Rust BDU not available - falling back to Python loop: {e}")
                except Exception as e:
                    logger.error(f"❌ RUST BATCH FAILED: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
            
            # EXPANDER_X batch
            elif morphology_id.lower() == "expander_x":
                logger.info(f"🦀 RUST BATCH: EXPANDER_X for {len(src_neurons)} neurons")
                try:
                    from feagi_bdu import py_syn_expander_batch
                    import time
                    start = time.time()
                    
                    # Batch get source positions (use helper)
                    src_pos_map, valid_neurons = self._batch_get_source_positions(src_area_id, src_neurons)
                    neuron_positions = [src_pos_map[nid] for nid in valid_neurons]
                    
                    if valid_neurons:
                        # Batch expand all neurons at once
                        expanded_positions = py_syn_expander_batch(
                            src_area_id, dst_area_id,
                            neuron_positions,
                            src_area.dimensions, dst_area.dimensions
                        )
                        
                        # Batch get destination neurons (use helper)
                        dst_pos_to_neurons = self._batch_get_destination_neurons(dst_area_id, psc_multiplier)
                        
                        # Match source to targets
                        for src_idx, src_neuron_id in enumerate(valid_neurons):
                            dst_pos = expanded_positions[src_idx]
                            if dst_pos in dst_pos_to_neurons:
                                for dst_neuron_id, weight in dst_pos_to_neurons[dst_pos]:
                                    if random.randrange(1, 100) < synapse_attractivity:
                                        all_synapse_connections.append((src_neuron_id, dst_neuron_id, weight))
                        
                        elapsed = (time.time() - start) * 1000
                        logger.info(f"🦀 RUST BATCH: EXPANDER_X {len(valid_neurons)} neurons in {elapsed:.1f}ms ({len(all_synapse_connections)} synapses)")
                    
                    src_neurons = []  # Skip loop
                except Exception as e:
                    logger.error(f"❌ EXPANDER BATCH FAILED: {e}")
                    import traceback
                    logger.error(traceback.format_exc())
            
            # BLOCK_CONNECTION batch
            elif morphology_id.lower() == "block_connection":
                logger.info(f"🦀 RUST BATCH: BLOCK_CONNECTION for {len(src_neurons)} neurons")
                try:
                    from feagi_bdu import py_syn_block_connection
                    import time
                    start = time.time()
                    
                    # Get scaling factor from morphology
                    scaling_factor = morphology_scalar[0] if morphology_scalar else 10
                    
                    # Batch get source positions (use helper)
                    src_pos_map, valid_neurons = self._batch_get_source_positions(src_area_id, src_neurons)
                    neuron_positions = [src_pos_map[nid] for nid in valid_neurons]
                    
                    if valid_neurons:
                        # Batch process block connections
                        mapped_positions = []
                        for pos in neuron_positions:
                            result = py_syn_block_connection(
                                src_area_id, dst_area_id, pos,
                                src_area.dimensions, dst_area.dimensions,
                                scaling_factor
                            )
                            mapped_positions.append(result)
                        
                        # Batch get destination neurons (use helper)
                        dst_pos_to_neurons = self._batch_get_destination_neurons(dst_area_id, psc_multiplier)
                        
                        # Match source to targets
                        for src_idx, src_neuron_id in enumerate(valid_neurons):
                            dst_pos = mapped_positions[src_idx]
                            if dst_pos in dst_pos_to_neurons:
                                for dst_neuron_id, weight in dst_pos_to_neurons[dst_pos]:
                                    if random.randrange(1, 100) < synapse_attractivity:
                                        all_synapse_connections.append((src_neuron_id, dst_neuron_id, weight))
                        
                        elapsed = (time.time() - start) * 1000
                        logger.info(f"🦀 RUST BATCH: BLOCK_CONNECTION {len(valid_neurons)} neurons in {elapsed:.1f}ms ({len(all_synapse_connections)} synapses)")
                    
                    src_neurons = []  # Skip loop
                except Exception as e:
                    logger.error(f"❌ BLOCK_CONNECTION BATCH FAILED: {e}")
                    import traceback
                    logger.error(traceback.format_exc())

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

                    if debug_bdu:
                        logger.info(
                            f"[BDU DEBUG] Found {len(candidate_neurons)} candidate neurons"
                        )

                    # Apply synapse attractivity filtering and accumulate
                    for dst_neuron_id, weight in candidate_neurons:
                        if random.randrange(1, 100) < synapse_attractivity:
                            all_synapse_connections.append(
                                (src_neuron_id, dst_neuron_id, weight)
                            )

                except Exception as e:
                    logger.warning(
                        f"Error processing function morphology {morphology_id} for neuron "
                        f"{src_neuron_id}: {e}"
                    )
                    continue

            # Create ALL synapses in ONE batch call (massive performance improvement)
            if all_synapse_connections:
                if debug_bdu:
                    logger.info(
                        f"[BDU DEBUG] Creating {len(all_synapse_connections)} total synapses in single batch"
                    )
                
                start_time = time.time()
                created = self.connectome_manager.batch_create_synapses(
                    all_synapse_connections
                )
                end_time = time.time()
                elapsed_ms = (end_time - start_time) * 1000
                
                total_synapses += created
                
                logger.info(
                    f"✅ BATCH SYNAPSE CREATION: {created} synapses created in {elapsed_ms:.1f}ms "
                    f"({len(src_neurons)} source neurons → {dst_area_id})"
                )
            elif debug_bdu:
                logger.info(
                    f"[BDU DEBUG] No synapses created for {len(src_neurons)} source neurons"
                )

            # Propagate memory register to ConnectomeManager (if any)
            self._propagate_memory_register(memory_register)

            return total_synapses

        except Exception as e:
            logger.error(
                f"Error in function morphology processing for {morphology_id}: {e}"
            )
            return 0

    def _get_neuron_position(
        self, neuron_id: int, area_id: str
    ) -> Optional[Tuple[int, int, int]]:
        """Get the 3D position of a neuron within its cortical area (queries Rust NPU)."""
        try:
            # ✅ Query Rust NPU directly (single source of truth)
            position = self.connectome_manager.get_neuron_position(neuron_id)
            if position:
                return position

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

            # If source depth is 1 but destination depth > 1, expand across full depth
            # This ensures 2D→3D projections connect to any depth layer deterministically
            if src_d == 1 and dst_d > 1 and region_d == 1:
                min_z = 0
                max_z = dst_d - 1

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

            # ✅ Query Rust NPU: get all neurons in area and filter by position
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
