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

'''
Neuroembryogenesis Module for FEAGI 2.1

This module is responsible for reading instructions from the genome (genotype) and translating
them into a functional connectome (phenotype). The process is biologically inspired by
neuroembryogenesis, where genetic instructions guide brain development from the embryonic neural tube.

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

The implementation uses the ConnectomeManager API for efficient neuron and synapse management,
and focuses on memory efficiency and thread-safety.
'''

import os
import sys
import json
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import random
import concurrent.futures
import types
import datetime
from enum import Enum
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional, Union, Any, Callable
import numpy as np

# Custom types
Position = Tuple[int, int, int]
NeuronId = int
AreaId = int
BoundingBox = Tuple[Position, Position]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))

from feagi.bdu.connectome_manager import ConnectomeManager, CorticalArea
from feagi.bdu.connectivity.synapse_rule import SynapseRule
from feagi.bdu.connectivity.synaptogenesis_rules import neighbor_finder

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


# Handle import from either new or old structure
try:
    # Try new imports first
    from feagi.evo.genome_editor import save_genome
    from feagi.evo.genome_validator import genome_validator
    from feagi.evo.genome_properties import genome_properties
    
    # Import these functions directly if possible
    try:
        from feagi.evo.genome_processor import (
            merge_core_morphologies, 
            genome_morphology_updator, 
            genome_physiology_updator, 
            genome_stat_updator
        )
    except ImportError:
        # Implement minimal versions if not available
        def merge_core_morphologies(genome):
            """Merges core morphologies into the genome."""
            # We'll implement this directly since the import might be problematic
            # This is simplified and would need to be expanded for full functionality
            return genome
            
        def genome_morphology_updator(genome):
            """Updates morphologies in the genome."""
            return genome
            
        def genome_physiology_updator(genome):
            """Updates physiology in the genome."""
            if "physiology" not in genome:
                genome["physiology"] = {}
            return genome
            
        def genome_stat_updator(genome):
            """Updates stats in the genome."""
            if "stats" not in genome:
                genome["stats"] = {}
            return genome
        
except ImportError:
    # Fall back to old imports if necessary
    try:
        from src.evo.genome_editor import save_genome
        from src.evo.genome_validator import genome_validator
        from src.evo.genome_processor import (
            merge_core_morphologies, 
            genome_morphology_updator, 
            genome_physiology_updator, 
            genome_stat_updator
        )
    except ImportError:
        # Define minimal working implementations if imports fail
        def save_genome(genome, file_name=''):
            """Placeholder for save_genome function."""
            logger.warning("Using placeholder save_genome function")
            try:
                with open(file_name, "w") as f:
                    json.dump(genome, f, indent=2)
                return True
            except Exception as e:
                logger.error(f"Failed to save genome: {e}")
                return False
                
        def genome_validator(genome):
            """Placeholder for genome_validator function."""
            logger.warning("Using placeholder genome_validator function")
            return True  # Always assume valid
            
        def merge_core_morphologies(genome):
            """Placeholder for merge_core_morphologies function."""
            logger.warning("Using placeholder merge_core_morphologies function")
            return genome
            
        def genome_morphology_updator(genome):
            """Placeholder for genome_morphology_updator function."""
            logger.warning("Using placeholder genome_morphology_updator function")
            return genome
            
        def genome_physiology_updator(genome):
            """Placeholder for genome_physiology_updator function."""
            logger.warning("Using placeholder genome_physiology_updator function")
            if "physiology" not in genome:
                genome["physiology"] = {}
            return genome
            
        def genome_stat_updator(genome):
            """Placeholder for genome_stat_updator function."""
            logger.warning("Using placeholder genome_stat_updator function")
            if "stats" not in genome:
                genome["stats"] = {}
            return genome

from feagi.utils.config import FeagiConfig


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
    
    def __init__(self, 
                 connectome_manager: ConnectomeManager,
                 config: Optional[FeagiConfig] = None,
                 progress_callback: Optional[Callable[[DevelopmentStage, float, str], None]] = None):
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
        if config and hasattr(config, 'embryogenesis'):
            embryo_config = config.embryogenesis
            self.verbose_logging = embryo_config.get('verbose_logging', True)
            self.suppress_no_mappings_logs = embryo_config.get('suppress_no_mappings_logs', False)
        elif config and hasattr(config, 'get'):
            # Alternative configuration access pattern
            self.verbose_logging = config.get('embryogenesis_verbose_logging', True)
            self.suppress_no_mappings_logs = config.get('embryogenesis_suppress_no_mappings_logs', False)
        
        # Check environment variables for runtime control
        if os.environ.get('FEAGI_EMBRYOGENESIS_QUIET', '').lower() in ('true', '1', 'yes'):
            self.suppress_no_mappings_logs = True
        if os.environ.get('FEAGI_EMBRYOGENESIS_VERBOSE', '').lower() in ('false', '0', 'no'):
            self.verbose_logging = False
            
        self.genome = None
        self.cortical_areas = {}
        self.error = None
        
        # Development statistics
        self.development_stats = {
            "total_neurons": 0,
            "total_synapses": 0,
            "cortical_areas": 0,
            "start_time": None,
            "end_time": None,
            "duration": None
        }
        
        # Cache for morphology registry
        self._morphology_registry_cache = None
        
        # Development state
        self.stage = DevelopmentStage.INITIALIZATION
        
        # Tracking data
        self.cortical_id_map = {}  # cortical_idx -> cortical_id (6-char genome ID)
        self.reverse_cortical_id_map = {}  # cortical_id -> cortical_idx
        self.voxel_neuron_map = {}  # Maps (area_id, position) to list of neuron IDs
        
        # Add temporary method to ConnectomeManager to provide morphology information
        # Add this once at initialization instead of each time in _perform_synaptogenesis
        def get_morphologies_registry(self):
            return self._neuroembryogenesis_morphologies_registry
            
        if not hasattr(self.connectome_manager, 'get_morphologies_registry'):
            setattr(self.connectome_manager, 'get_morphologies_registry', 
                   types.MethodType(get_morphologies_registry, self.connectome_manager))
            # Will set the actual registry later when we have the genome
        
    def _report_progress(self, stage: DevelopmentStage, percentage: float, message: str) -> None:
        """Report progress for the given development stage."""
        # Check if we should suppress this specific message
        should_suppress = (
            self.suppress_no_mappings_logs and 
            "No mappings found" in message
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
            with open(genome_path, 'r') as f:
                self.genome = json.load(f)
            
            # Load FEAGI configuration to check genome settings
            try:
                from feagi.config.toml_loader import load_feagi_config, get_genome_config
                config = load_feagi_config()
                genome_config = get_genome_config(config)
                allow_auto_recovery = genome_config.auto_recovery_on_validation_failure
            except Exception as e:
                logger.warning(f"Could not load FEAGI configuration, defaulting to allow auto-recovery: {e}")
                allow_auto_recovery = True  # Default to allow auto-recovery if config fails
            
            # Validate genome - behavior depends on configuration
            is_valid = genome_validator(self.genome)
            if not is_valid:
                if not allow_auto_recovery:
                    self._report_failure(DevelopmentStage.INITIALIZATION, "Genome validation failed and auto-recovery is disabled")
                    return False
                else:
                    logger.warning("Genome validation failed - attempting auto-recovery with morphology sanitization")
                    
                    # Attempt morphology sanitization during auto-recovery
                    try:
                        from feagi.evo.genome_validator import sanitize_invalid_morphologies
                        sanitization_result = sanitize_invalid_morphologies(self.genome)
                        
                        # Use the sanitized genome
                        self.genome = sanitization_result["genome"]
                        removed_morphologies = sanitization_result["removed_morphologies"]
                        recovery_summary = sanitization_result["recovery_summary"]
                        
                        logger.info(f"Auto-recovery completed: {recovery_summary}")
                        if removed_morphologies:
                            logger.info(f"Removed invalid morphologies: {', '.join(removed_morphologies)}")
                        
                        # Re-validate after sanitization
                        is_valid_after_recovery = genome_validator(self.genome)
                        if is_valid_after_recovery:
                            logger.info("Genome validation passed after auto-recovery sanitization")
                        else:
                            logger.warning("Genome still has validation issues after sanitization - continuing anyway")
                            
                    except Exception as sanitization_error:
                        logger.warning(f"Auto-recovery sanitization failed: {sanitization_error}")
                        logger.warning("Continuing with original genome despite validation failures")
                    
                    logger.warning("FEAGI will try to fix/recover from remaining gene failures during development")
                    # Don't return False - continue with loading despite validation issues
            
            # Update morphologies and physiology
            self.genome = merge_core_morphologies(self.genome)
            self.genome = genome_morphology_updator(self.genome)
            self.genome = genome_physiology_updator(self.genome)
            self.genome = genome_stat_updator(self.genome)
            
            # Generate and cache the morphology registry
            morphology_registry = self.get_morphology_registry()
            
            # Set the morphology registry on the ConnectomeManager
            if hasattr(self.connectome_manager, 'get_morphologies_registry'):
                setattr(self.connectome_manager, '_neuroembryogenesis_morphologies_registry', morphology_registry)
            
            if is_valid:
                self._report_progress(DevelopmentStage.INITIALIZATION, 100, "Genome loaded and validated")
            else:
                self._report_progress(DevelopmentStage.INITIALIZATION, 100, "Genome loaded with validation warnings - attempting recovery")
            return True
            
        except Exception as e:
            self._report_failure(DevelopmentStage.INITIALIZATION, f"Failed to load genome: {str(e)}")
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
                value_type = parts[4]
            else:
                property_key = parts[-2]
                value_type = parts[-1]
                
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
                properties["n_cnt"] = value  # Also store with the original name for compatibility
            elif "dstmap" in property_key:
                properties["mapping"] = value
            else:
                # Store other properties directly
                clean_key = property_key.strip('_')  # Remove leading/trailing underscores
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
        # Get the area from the connectome manager
        area = self.cortical_areas[cortical_id]
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
        
        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.CORTICOGENESIS, 0, "Setting up cortical areas")
        
        try:
            cortical_ids = self._get_cortical_ids_from_genome()
            total_areas = len(cortical_ids)
            
            for i, cortical_id in enumerate(cortical_ids):
                properties = self._extract_cortical_properties(cortical_id)
                
                # Skip if required properties are missing
                if "dimensions" not in properties or "position" not in properties or "name" not in properties:
                    logger.warning(f"Skipping cortical area {cortical_id} due to missing required properties")
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
                
                # Add to connectome manager
                try:
                    logger.debug(f"Creating cortical area with ID {cortical_id}")
                    # Update to match the new ConnectomeManager API
                    created_cortical_id = self.connectome_manager.add_cortical_area(
                        name=name,
                        dimensions=dimensions,
                        position=position,
                        area_type=area_type,
                        properties={**properties},
                        cortical_id=cortical_id  # Pass the cortical_id from genome
                    )
                    
                    # Get the created area
                    area = self.connectome_manager.get_cortical_area(created_cortical_id)
                    
                    # Store in our tracking maps
                    self.cortical_areas[created_cortical_id] = area
                    
                    # Get the cortical_idx assigned by ConnectomeManager
                    cortical_idx = area.cortical_idx
                    
                    # Store mappings
                    self.cortical_id_map[cortical_idx] = cortical_id
                    self.reverse_cortical_id_map[cortical_id] = cortical_idx
                    
                    logger.debug(f"Created cortical area {name} (cortical_idx {cortical_idx}, cortical_id {cortical_id})")
                except Exception as e:
                    logger.error(f"Failed to create cortical area {cortical_id}: {e}")
                    continue
                
                # Report progress - Log detailed per-area progress at DEBUG level to reduce noise
                progress = ((i + 1) / total_areas) * 100
                logger.debug(f"[{DevelopmentStage.CORTICOGENESIS.value}] {progress:.1f}% - Created cortical area {i+1}/{total_areas}: {name}")
            
            self.development_stats["cortical_areas"] = len(self.cortical_areas)
            
            if not self.cortical_areas:
                self.error = "No valid cortical areas found in genome"
                self._report_progress(DevelopmentStage.FAILED, 0, self.error)
                return False
                
            self._report_progress(
                DevelopmentStage.CORTICOGENESIS, 
                100, 
                f"Created {len(self.cortical_areas)} cortical areas"
            )
            return True
            
        except Exception as e:
            self.error = f"Failed to setup cortical areas: {e}"
            logger.exception(self.error)
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            return False
    
    def _perform_neurogenesis(self) -> bool:
        """
        Create neurons in each cortical area.
        
        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.NEUROGENESIS, 0, "Creating neurons")
        
        try:
            total_areas = len(self.cortical_areas)
            total_neurons = 0
            
            for i, (cortical_id, area) in enumerate(self.cortical_areas.items()):
                # cortical_id is the 6-character identifier, we're already using it correctly
                properties = self._extract_cortical_properties(cortical_id)
                
                # Skip memory areas in initial development if configured
                if area.area_type == "memory" and self.config.get("skip_memory_neurogenesis", False):
                    logger.info(f"Skipping neurogenesis for memory area {area.name}")
                    continue
                
                # Get neurons per voxel count
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                
                # Get neuron properties
                neuron_properties = {
                    "threshold": properties.get("fire_t", 1.0),
                    "refractory_period": properties.get("refrac", 0),
                    "decay_rate": 1.0 - (properties.get("leak_c", 0) / 100.0),
                    "resting_potential": 0.0
                }
                
                # Create neurons for each voxel
                width, height, depth = area.dimensions
                voxel_count = width * height * depth
                area_neuron_count = 0
                
                # Initialize voxel tracking for this area
                if cortical_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[cortical_id] = {}
                
                # Performance optimization: Batch neuron creation
                batch_size = 1000  # Create neurons in batches of 1000
                neuron_specs = []
                positions_map = {}  # Map to track positions for later assignment to voxel_neuron_map
                
                # Progress reporting setup
                report_interval = max(1, voxel_count // 10)  # Report 10 times during processing
                voxel_num = 0
                
                # Create neuron specifications in batches
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            position = (x, y, z)
                            voxel_neurons = []
                            positions_map[voxel_num] = position
                            
                            # Add neuron specifications to the batch
                            for n_idx in range(neurons_per_voxel):
                                neuron_specs.append((
                                    cortical_id,
                                    position,
                                    neuron_properties["threshold"],
                                    neuron_properties["refractory_period"],
                                    neuron_properties["decay_rate"],
                                    neuron_properties["resting_potential"],
                                    {"neuron_index": n_idx, "voxel_id": voxel_num}
                                ))
                            
                            # Process batch if it reaches the batch size
                            if len(neuron_specs) >= batch_size:
                                # Create neurons in batch
                                neuron_ids = self._batch_create_neurons(neuron_specs)
                                
                                # Assign neurons to voxels
                                self._assign_neurons_to_voxels(neuron_ids, positions_map)
                                
                                # Update counts
                                area_neuron_count += len(neuron_ids)
                                
                                # Clear batch data
                                neuron_specs = []
                                positions_map = {}
                            
                            voxel_num += 1
                            
                            # Report progress periodically - Log detailed voxel progress at DEBUG level to reduce noise
                            if voxel_num % report_interval == 0 or voxel_num == voxel_count:
                                voxel_progress = (voxel_num / voxel_count) * 100
                                area_progress = ((i + voxel_progress/100) / total_areas) * 100
                                logger.debug(f"[{DevelopmentStage.NEUROGENESIS.value}] {area_progress:.1f}% - Area {i+1}/{total_areas} ({area.name}): {voxel_progress:.1f}% complete")
                
                # Process any remaining neurons in the batch
                if neuron_specs:
                    neuron_ids = self._batch_create_neurons(neuron_specs)
                    self._assign_neurons_to_voxels(neuron_ids, positions_map)
                    area_neuron_count += len(neuron_ids)
                
                total_neurons += area_neuron_count
                logger.debug(f"Created {area_neuron_count} neurons in area {area.name}")
            
            self.development_stats["total_neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons across {len(self.cortical_areas)} areas"
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
            
    def _batch_create_neurons(self, neuron_specs):
        """
        Create multiple neurons using TRUE vectorized operations (NumPy-style bulk initialization).
        
        Args:
            neuron_specs: List of (cortical_id, position, threshold, refractory_period, decay_rate, resting_potential, properties) tuples
            
        Returns:
            List of created neuron IDs with their associated voxel IDs
        """
        neuron_ids = []
        
        # Group neuron specs by cortical_id for vectorized processing
        by_area = {}
        for spec in neuron_specs:
            cortical_id, position, threshold, refractory_period, decay_rate, resting_potential, properties = spec
            if cortical_id not in by_area:
                by_area[cortical_id] = {
                    "positions": [],
                    "properties": [],
                    "voxel_ids": [],
                    "thresholds": [],
                    "refractory_periods": [],
                    "decay_rates": [],
                    "resting_potentials": []
                }
            
            by_area[cortical_id]["positions"].append(position)
            by_area[cortical_id]["properties"].append(properties)
            by_area[cortical_id]["voxel_ids"].append(properties["voxel_id"])
            by_area[cortical_id]["thresholds"].append(threshold)
            by_area[cortical_id]["refractory_periods"].append(refractory_period)
            by_area[cortical_id]["decay_rates"].append(decay_rate)
            by_area[cortical_id]["resting_potentials"].append(resting_potential)
        
        # Process neurons by area using TRUE VECTORIZED OPERATIONS
        for cortical_id, area_specs in by_area.items():
            positions = area_specs["positions"]
            num_neurons = len(positions)
            
            logger.debug(f"[START] VECTORIZED CREATION for {cortical_id}: {num_neurons} neurons")
            
            try:
                # Get the cortical area object to get cortical_idx
                area = self.connectome_manager.get_cortical_area(cortical_id)
                cortical_idx = area.cortical_idx
                
                # PURE VECTORIZED APPROACH - No loops, no individual calls!
                area_neuron_ids = self.connectome_manager.neuron_array.batch_create_neurons(
                    cortical_idx=cortical_idx,
                    positions=positions,
                    thresholds=area_specs["thresholds"],      # Pass as list for vectorized ops
                    membrane_potentials=0.0,                  # Single value broadcasted
                    resting_potentials=area_specs["resting_potentials"],
                    decay_rates=area_specs["decay_rates"],
                    refractory_periods=area_specs["refractory_periods"]
                )
                
                logger.debug(f"[OK] VECTORIZED SUCCESS for {cortical_id}: created {len(area_neuron_ids)} neurons in one operation")
                
                # Update ConnectomeManager ID mappings for the batch
                for i, neuron_id in enumerate(area_neuron_ids):
                    # Map neuron_id to index in ConnectomeManager
                    index = self.connectome_manager.neuron_array.id_to_index_map[neuron_id]
                    self.connectome_manager.neuron_id_to_index[neuron_id] = index
                    self.connectome_manager.index_to_neuron_id[index] = neuron_id
                    
                    # Update area tracking efficiently
                    if cortical_id not in self.connectome_manager.area_neuron_masks:
                        self.connectome_manager.area_neuron_masks[cortical_id] = np.zeros(
                            self.connectome_manager.max_neurons, dtype=np.bool_
                        )
                    self.connectome_manager.area_neuron_masks[cortical_id][index] = True
                    
                    # Add to area's neuron list
                    area.add_neuron(neuron_id, positions[i])
                    
                    # Update backward compatibility tracking
                    self.connectome_manager._neuron_to_position[neuron_id] = (cortical_id, *positions[i], index)
                    
                    # Construct result with voxel ID
                    neuron_ids.append((neuron_id, area_specs["voxel_ids"][i]))
                
            except Exception as e:
                logger.error(f"[DEBUG] VECTORIZED CREATION FAILED for {cortical_id}: {str(e)}")
                import traceback
                logger.error(f"[DEBUG] Traceback: {traceback.format_exc()}")
                
                # This should NOT happen with proper SoA implementation
                raise RuntimeError(f"Vectorized neuron creation failed - SoA implementation issue: {e}")
        
        return neuron_ids
        
    def _assign_neurons_to_voxels(self, neuron_ids, positions_map):
        """
        Assign the newly created neurons to their corresponding voxels in the voxel_neuron_map.
        
        Args:
            neuron_ids: List of (neuron_id, voxel_id) tuples
            positions_map: Dictionary mapping voxel_id to position tuples
        """
        # Group neurons by voxel ID
        voxel_neurons = {}
        for neuron_id, voxel_id in neuron_ids:
            if voxel_id not in voxel_neurons:
                voxel_neurons[voxel_id] = []
            voxel_neurons[voxel_id].append(neuron_id)
        
        # Add neurons to voxel_neuron_map
        for voxel_id, neurons in voxel_neurons.items():
            position = positions_map[voxel_id]
            cortical_id = self.connectome_manager.get_cortical_area_for_neuron(neurons[0])
            if cortical_id not in self.voxel_neuron_map:
                self.voxel_neuron_map[cortical_id] = {}
            self.voxel_neuron_map[cortical_id][position] = neurons
    
    def _perform_synaptogenesis(self) -> bool:
        """
        Create synaptic connections based on genome mappings.
        
        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.SYNAPTOGENESIS, 0, "Creating synaptic connections")
        
        try:
            total_areas = len(self.cortical_areas)
            total_synapses = 0
            
            # Memory register for memory-based morphologies
            memory_register = {}
            
            # Extract mapping data from the genome
            mapping_data = {}
            if "cortical_mappings" in self.genome:
                for mapping in self.genome["cortical_mappings"]:
                    src_id = mapping["source"]
                    dst_id = mapping["destination"]
                    
                    if src_id not in mapping_data:
                        mapping_data[src_id] = []
                    
                    mapping_data[src_id].append(mapping)
            
            for i, (src_cortical_id, src_area) in enumerate(self.cortical_areas.items()):
                properties = self._extract_cortical_properties(src_cortical_id)
                
                # Get mappings for this area
                if src_cortical_id not in mapping_data:
                    # Log at DEBUG level instead of INFO to reduce noise during normal operation
                    logger.debug(f"[{DevelopmentStage.SYNAPTOGENESIS.value}] {100 * i / total_areas:.1f}% - No mappings found for area {i+1}/{total_areas} ({src_area.name})")
                    continue
                
                mappings = mapping_data[src_cortical_id]
                
                for mapping in mappings:
                    dst_cortical_id = mapping["destination"]
                    
                    # Skip if destination area not created
                    if dst_cortical_id not in self.cortical_areas:
                        continue
                    
                    dst_area = self.cortical_areas[dst_cortical_id]
                    morphology = mapping["morphology"]
                    
                    # Get source area neurons
                    src_neurons = self.connectome_manager.get_neurons_by_area(src_cortical_id)
                    
                    # Calculate source subregion
                    src_subregion = self._calculate_subregion(src_cortical_id, morphology)
                    
                    # Process each source neuron
                    neuron_count = len(src_neurons)
                    for j, src_neuron_id in enumerate(src_neurons):
                        if j % 100 == 0:
                            # Update progress every 100 neurons
                            completion = (i + (j / neuron_count)) / total_areas
                            self._report_progress(
                                DevelopmentStage.SYNAPTOGENESIS, 
                                100 * completion, 
                                f"Area {i+1}/{total_areas} ({src_area.name}): {j}/{neuron_count} neurons"
                            )
                        
                        # Find target neurons based on connectivity rules
                        dst_neurons_with_weights = neighbor_finder(
                            src_cortical_id=src_cortical_id,
                            dst_cortical_id=dst_cortical_id,
                            src_neuron_id=src_neuron_id,
                            morphology=morphology,
                            src_subregion=src_subregion,
                            connectome_manager=self.connectome_manager,
                            memory_register=memory_register
                        )
                        
                        # Create synaptic connections
                        for dst_neuron_id, weight in dst_neurons_with_weights:
                            self.connectome_manager.create_synapse(
                                pre_neuron_id=src_neuron_id,
                                post_neuron_id=dst_neuron_id,
                                weight=weight,
                                is_plastic=morphology.get("plasticity", False)
                            )
                            total_synapses += 1
            
            self._report_progress(DevelopmentStage.SYNAPTOGENESIS, 100, f"Created {total_synapses} synaptic connections")
            self.development_stats["total_synapses"] = total_synapses
            return True
            
        except Exception as e:
            self._report_failure(DevelopmentStage.SYNAPTOGENESIS, f"Failed to create synaptic connections: {str(e)}")
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
            "class": "built-in"
        }
        
        registry["reducer_x"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["randomizer"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["lateral_pairs_x"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["block_connection"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["projector"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["last_to_first"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        registry["memory"] = {
            "type": "function", 
            "parameters": {},
            "class": "built-in"
        }
        
        # Add morphologies from the genome
        if self.genome and "neuron_morphologies" in self.genome:
            for morphology_id, morphology in self.genome["neuron_morphologies"].items():
                morphology_type = morphology.get("type", "unknown")
                
                if morphology_type == "vectors":
                    registry[morphology_id] = {
                        "type": "vectors",
                        "parameters": {
                            "vectors": morphology.get("parameters", {}).get("vectors", [])
                        },
                        "class": "vectors"
                    }
                elif morphology_type == "patterns":
                    registry[morphology_id] = {
                        "type": "patterns",
                        "parameters": {
                            "patterns": morphology.get("parameters", {}).get("patterns", [])
                        },
                        "class": "patterns"
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
            f"{self.development_stats['total_synapses']} synapses."
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
        self._report_progress(DevelopmentStage.NEUROGENESIS, 0, "Creating neurons (vectorized)")
        
        try:
            total_areas = len(self.cortical_areas)
            total_neurons = 0
            
            for i, (cortical_id, area) in enumerate(self.cortical_areas.items()):
                properties = self._extract_cortical_properties(cortical_id)
                
                # Skip memory areas in initial development if configured
                if area.area_type == "memory" and self.config and self.config.get("skip_memory_neurogenesis", False):
                    logger.info(f"Skipping neurogenesis for memory area {area.name}")
                    continue
                
                # Get area properties
                neurons_per_voxel = properties.get("neurons_per_voxel", 1)
                width, height, depth = area.dimensions
                voxel_count = width * height * depth
                area_neuron_count = voxel_count * neurons_per_voxel
                
                logger.debug(f"[TARGET] BULK NEUROGENESIS for {cortical_id}: {area_neuron_count} neurons ({width}×{height}×{depth} × {neurons_per_voxel})")
                
                # PRE-CALCULATE ALL DATA (NumPy style!)
                # Create position arrays efficiently
                positions = []
                voxel_indices = []
                
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            for n_idx in range(neurons_per_voxel):
                                positions.append((x, y, z))
                                voxel_indices.append(x * height * depth + y * depth + z)
                
                # Convert to numpy arrays for efficiency
                positions_array = np.array(positions)
                
                # Pre-calculate neuron properties (vectorized)
                thresholds = np.full(area_neuron_count, properties.get("fire_t", 1.0), dtype=np.float32)
                resting_potentials = np.zeros(area_neuron_count, dtype=np.float32)
                decay_rates = np.full(area_neuron_count, 1.0 - (properties.get("leak_c", 0) / 100.0), dtype=np.float32)
                refractory_periods = np.full(area_neuron_count, properties.get("refrac", 1), dtype=np.int32)
                
                # Get cortical_idx 
                cortical_idx = area.cortical_idx
                
                # [START] SINGLE VECTORIZED CALL - NO LOOPS!
                start_time = datetime.datetime.now()
                area_neuron_ids = self.connectome_manager.neuron_array.batch_create_neurons(
                    cortical_idx=cortical_idx,
                    positions=positions,
                    thresholds=thresholds.tolist(),
                    membrane_potentials=0.0,
                    resting_potentials=resting_potentials.tolist(),
                    decay_rates=decay_rates.tolist(),
                    refractory_periods=refractory_periods.tolist()
                )
                end_time = datetime.datetime.now()
                creation_time = (end_time - start_time).total_seconds()
                
                logger.debug(f"[FAST] VECTORIZED COMPLETE for {cortical_id}: {len(area_neuron_ids)} neurons in {creation_time:.3f}s ({len(area_neuron_ids)/creation_time:.0f} neurons/sec)")
                
                # Update ConnectomeManager mappings efficiently (vectorized where possible)
                start_mapping_time = datetime.datetime.now()
                
                # Bulk update area tracking
                if cortical_id not in self.connectome_manager.area_neuron_masks:
                    self.connectome_manager.area_neuron_masks[cortical_id] = np.zeros(
                        self.connectome_manager.max_neurons, dtype=np.bool_
                    )
                
                # Get all indices at once
                indices = [self.connectome_manager.neuron_array.id_to_index_map[nid] for nid in area_neuron_ids]
                indices_array = np.array(indices)
                
                # Vectorized mask update
                self.connectome_manager.area_neuron_masks[cortical_id][indices_array] = True
                
                # Batch update mappings (unavoidable loop but minimized)
                for j, (neuron_id, index) in enumerate(zip(area_neuron_ids, indices)):
                    self.connectome_manager.neuron_id_to_index[neuron_id] = index
                    self.connectome_manager.index_to_neuron_id[index] = neuron_id
                    area.add_neuron(neuron_id, tuple(positions[j]))
                    self.connectome_manager._neuron_to_position[neuron_id] = (cortical_id, *positions[j], index)
                
                end_mapping_time = datetime.datetime.now()
                mapping_time = (end_mapping_time - start_mapping_time).total_seconds()
                
                logger.debug(f"[LINK] MAPPING COMPLETE for {cortical_id}: {len(area_neuron_ids)} mappings in {mapping_time:.3f}s")
                
                # Initialize voxel tracking for this area (vectorized)
                if cortical_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[cortical_id] = {}
                
                # Group neurons by position efficiently
                position_to_neurons = {}
                for j, (neuron_id, pos) in enumerate(zip(area_neuron_ids, positions)):
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
                    f"Area {i+1}/{total_areas} ({area.name}): {area_neuron_count} neurons created vectorized"
                )
            
            self.development_stats["total_neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons across {len(self.cortical_areas)} areas (vectorized)"
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


# Convenience function for direct use
def develop_brain_from_genome(
    genome_path: Union[str, Path],
    connectome_manager: Optional[ConnectomeManager] = None,
    config: Optional[FeagiConfig] = None,
    progress_callback: Optional[Callable[[DevelopmentStage, float, str], None]] = None
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
        progress_callback=progress_callback
    )
    
    # Develop the brain
    success = embryo.develop_brain(genome_path)
    
    # Return results
    return success, embryo.get_development_statistics()
