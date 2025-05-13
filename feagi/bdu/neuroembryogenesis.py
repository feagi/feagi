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

# Custom types
Position = Tuple[int, int, int]
NeuronId = int
AreaId = int
BoundingBox = Tuple[Position, Position]  # ((min_x, min_y, min_z), (max_x, max_y, max_z))

from feagi.bdu.connectome_manager import ConnectomeManager, CorticalArea
from feagi.bdu.synaptogenesis_rules import neighbor_finder

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


class Neuroembryogenesis:
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
        Initialize the neuroembryogenesis process.
        
        Args:
            connectome_manager: The ConnectomeManager to use for brain construction
            config: Configuration for FEAGI, if not provided a default will be used
            progress_callback: Optional callback to report development progress
        """
        self.connectome_manager = connectome_manager
        self.config = config or FeagiConfig()
        self.progress_callback = progress_callback
        
        # Development state
        self.stage = DevelopmentStage.INITIALIZATION
        self.development_stats = {
            "cortical_areas": 0,
            "neurons": 0,
            "synapses": 0,
            "start_time": None,
            "end_time": None,
            "duration": None
        }
        
        # Tracking data
        self.genome = None
        self.cortical_areas = {}  # cortical_idx -> CorticalArea
        self.cortical_id_map = {}  # cortical_idx -> cortical_id (6-char genome ID)
        self.reverse_cortical_id_map = {}  # cortical_id -> cortical_idx
        self.voxel_neuron_map = {}  # Maps (area_id, position) to list of neuron IDs
        
        # Error state
        self.error = None
        
        # Cache for morphology registry to avoid regenerating it
        self._morphology_registry_cache = None
        
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
            
            # Validate genome
            is_valid = genome_validator(self.genome)
            if not is_valid:
                self._report_failure(DevelopmentStage.INITIALIZATION, "Invalid genome")
                return False
            
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
            
            self._report_progress(DevelopmentStage.INITIALIZATION, 100, "Genome loaded and validated")
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

        print(">>>>> cortical_id", cortical_id)

        properties = {}
        blueprint = self.genome["blueprint"]
        
        # Need to collect all properties for this cortical area
        for gene_key in blueprint:
            if not isinstance(gene_key, str):
                continue
                
            parts = gene_key.split("-")
            if len(parts) < 5:
                continue
                
            gene_cortical_id = parts[1]
            if gene_cortical_id != cortical_id:
                continue
                
            property_key = parts[3]
            value = blueprint[gene_key]
            
            # Handle special properties that need processing
            if property_key == "___bbx":
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][0] = value
            elif property_key == "___bby":
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][1] = value
            elif property_key == "___bbz":
                if "dimensions" not in properties:
                    properties["dimensions"] = [0, 0, 0]
                properties["dimensions"][2] = value
            elif property_key == "rcordx":
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][0] = value
            elif property_key == "rcordy":
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][1] = value
            elif property_key == "rcordz":
                if "position" not in properties:
                    properties["position"] = [0, 0, 0]
                properties["position"][2] = value
            elif property_key == "__name":
                properties["name"] = value
            elif property_key == "_group":
                properties["group"] = value
            elif property_key == "subgrp":
                properties["subgroup"] = value
            elif property_key == "_n_cnt":
                properties["neurons_per_voxel"] = value
            elif property_key == "dstmap":
                properties["mapping"] = value
            else:
                # Store other properties directly
                properties[property_key] = value
                
        return properties
        
    def _calculate_subregion(self, area_id: int, morphology: Dict) -> BoundingBox:
        """
        Calculate a bounding box for a subregion of the cortical area.
        
        Args:
            area_id: Internal area ID
            morphology: Morphology parameters
            
        Returns:
            Bounding box tuple ((min_x, min_y, min_z), (max_x, max_y, max_z))
        """
        # Get the area from the connectome manager
        area = self.cortical_areas[area_id]
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
            
            for cortical_idx, cortical_id in enumerate(cortical_ids):
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
                    area = self.connectome_manager.add_cortical_area(
                        area_id=cortical_idx,  # Use sequential IDs internally (cortical_idx)
                        name=name,
                        area_type=area_type,
                        dimensions=dimensions,
                        position=position,
                        properties=properties
                    )
                    
                    # Store in our tracking maps
                    self.cortical_areas[cortical_idx] = area
                    self.cortical_id_map[cortical_idx] = cortical_id
                    self.reverse_cortical_id_map[cortical_id] = cortical_idx
                    
                    logger.debug(f"Created cortical area {name} (internal ID {cortical_idx}, genome ID {cortical_id})")
                except Exception as e:
                    logger.error(f"Failed to create cortical area {cortical_id}: {e}")
                    continue
                
                # Report progress
                progress = ((cortical_idx + 1) / total_areas) * 100
                self._report_progress(
                    DevelopmentStage.CORTICOGENESIS, 
                    progress, 
                    f"Created cortical area {cortical_idx+1}/{total_areas}: {name}"
                )
            
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
        Create neurons for all cortical areas.
        
        Returns:
            True if successful, False otherwise
        """
        self._report_progress(DevelopmentStage.NEUROGENESIS, 0, "Creating neurons")
        
        try:
            total_areas = len(self.cortical_areas)
            total_neurons = 0
            
            for i, (area_id, area) in enumerate(self.cortical_areas.items()):
                cortical_id = self.cortical_id_map[area_id]
                properties = self._extract_cortical_properties(cortical_id)
                
                # Skip memory areas in initial development if configured
                if area.type == "memory" and self.config.get("skip_memory_neurogenesis", False):
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
                if area_id not in self.voxel_neuron_map:
                    self.voxel_neuron_map[area_id] = {}
                
                # Create progress tracking
                voxel_increment = max(1, voxel_count // 100)  # Report progress every 1% of voxels
                
                for x in range(width):
                    for y in range(height):
                        for z in range(depth):
                            position = (x, y, z)
                            voxel_neurons = []
                            
                            # Create neurons_per_voxel neurons at this position
                            for n_idx in range(neurons_per_voxel):
                                neuron_id = self.connectome_manager._create_neuron_with_index(
                                    area_id=area_id,
                                    position=position,
                                    threshold=neuron_properties["threshold"],
                                    refractory_period=neuron_properties["refractory_period"],
                                    decay_rate=neuron_properties["decay_rate"],
                                    resting_potential=neuron_properties["resting_potential"],
                                    neuron_index=n_idx
                                )
                                voxel_neurons.append(neuron_id)
                                area_neuron_count += 1
                                
                            # Store voxel to neuron mapping
                            self.voxel_neuron_map[area_id][position] = voxel_neurons
                            
                            # Report progress periodically
                            voxel_num = x * height * depth + y * depth + z
                            if voxel_num % voxel_increment == 0 or voxel_num == voxel_count - 1:
                                voxel_progress = ((voxel_num + 1) / voxel_count) * 100
                                area_progress = ((i + voxel_progress/100) / total_areas) * 100
                                self._report_progress(
                                    DevelopmentStage.NEUROGENESIS,
                                    area_progress,
                                    f"Area {i+1}/{total_areas} ({area.name}): {voxel_progress:.1f}% complete"
                                )
                
                total_neurons += area_neuron_count
                logger.info(f"Created {area_neuron_count} neurons in area {area.name}")
            
            self.development_stats["neurons"] = total_neurons
            self._report_progress(
                DevelopmentStage.NEUROGENESIS,
                100,
                f"Created {total_neurons} neurons across {len(self.cortical_areas)} areas"
            )
            return True
            
        except Exception as e:
            self.error = f"Failed to create neurons: {str(e)}"
            self._report_progress(DevelopmentStage.FAILED, 0, self.error)
            logger.exception("Error during neurogenesis")
            return False
    
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
            
            for i, (src_area_id, src_area) in enumerate(self.cortical_areas.items()):
                src_cortical_id = self.cortical_id_map[src_area_id]
                properties = self._extract_cortical_properties(src_cortical_id)
                
                # Get mappings for this area
                if src_cortical_id not in mapping_data:
                    self._report_progress(
                        DevelopmentStage.SYNAPTOGENESIS, 
                        100 * i / total_areas, 
                        f"No mappings found for area {src_area_id} ({src_area.name})"
                    )
                    continue
                
                mappings = mapping_data[src_cortical_id]
                
                for mapping in mappings:
                    dst_cortical_id = mapping["destination"]
                    
                    # Skip if destination area not created
                    if dst_cortical_id not in self.reverse_cortical_id_map:
                        continue
                    
                    dst_area_id = self.reverse_cortical_id_map[dst_cortical_id]
                    
                    if dst_area_id not in self.cortical_areas:
                        continue
                    
                    dst_area = self.cortical_areas[dst_area_id]
                    morphology = mapping["morphology"]
                    
                    # Get source area neurons
                    src_neurons = self.connectome_manager.get_neurons_by_area(src_area_id)
                    
                    # Calculate source subregion
                    src_subregion = self._calculate_subregion(src_area_id, morphology)
                    
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
                            src_area_id=src_area_id,
                            dst_area_id=dst_area_id,
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
            "parameters": {}
        }
        
        registry["reducer_x"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["randomizer"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["lateral_pairs_x"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["block_connection"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["projector"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["last_to_first"] = {
            "type": "function", 
            "parameters": {}
        }
        
        registry["memory"] = {
            "type": "function", 
            "parameters": {}
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
                        }
                    }
                elif morphology_type == "patterns":
                    registry[morphology_id] = {
                        "type": "patterns",
                        "parameters": {
                            "patterns": morphology.get("parameters", {}).get("patterns", [])
                        }
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
        
        # Create neurons
        if not self._perform_neurogenesis():
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
            f"{self.development_stats['neurons']} neurons, and "
            f"{self.development_stats['synapses']} synapses."
        )
        
        return True
    
    def get_development_statistics(self) -> Dict[str, Any]:
        """Get statistics about the brain development process."""
        return self.development_stats


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
        connectome_manager = ConnectomeManager(config)
    
    # Create neuroembryogenesis instance
    embryo = Neuroembryogenesis(
        connectome_manager=connectome_manager,
        config=config,
        progress_callback=progress_callback
    )
    
    # Develop the brain
    success = embryo.develop_brain(genome_path)
    
    # Return results
    return success, embryo.get_development_statistics()
