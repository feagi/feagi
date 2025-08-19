"""
NPU Interface - SIMD-Optimized CRUD Operations

This module provides the main interface for interacting with NPU-owned data structures.
It offers SIMD-optimized batch operations for neuron and synapse management, designed
for high-performance, deterministic behavior suitable for Rust migration and RTOS.

Key Features:
- SIMD-optimized batch operations
- Cortical area-aware operations
- Zero-copy data access where possible
- Rust/RTOS compatible design
- GPU acceleration support
- Deterministic behavior (no fallbacks)

Architecture:
- Single source of truth for all neural data
- BDU uses this interface for neurogenesis/synaptogenesis
- Sleep Manager coordinates cortical area locking
- State Manager tracks locking state
"""

from typing import Dict, List, Tuple, Optional, Set, Any, Union
from enum import Enum
import numpy as np
from dataclasses import dataclass
import logging

from feagi.core.state_manager import FeagiStateManager
from feagi.npu.data_structures import NeuronArray, MemoryNeuronArray, SynapseArray, BackendType
from feagi.utils.logger import setup_logger

logger = setup_logger()


class OperationResult(Enum):
    """Result codes for NPU operations."""
    SUCCESS = "success"
    AREA_LOCKED = "area_locked"
    CAPACITY_EXCEEDED = "capacity_exceeded"
    INVALID_INPUT = "invalid_input"
    BACKEND_ERROR = "backend_error"


@dataclass
class BatchOperationResult:
    """Result of a batch operation."""
    result: OperationResult
    successful_count: int
    failed_indices: List[int]
    error_message: Optional[str] = None
    
    @property
    def is_success(self) -> bool:
        return self.result == OperationResult.SUCCESS
    
    @property
    def has_partial_success(self) -> bool:
        return self.successful_count > 0 and len(self.failed_indices) > 0


@dataclass
class NeuronCreationRequest:
    """Request for creating neurons."""
    cortical_idx: int  # Fast integer index for NPU operations
    positions: List[Tuple[int, int, int]]
    neuron_types: Optional[List[int]] = None
    initial_potentials: Optional[List[float]] = None
    thresholds: Optional[List[float]] = None
    leak_coefficients: Optional[List[float]] = None
    excitabilities: Optional[List[float]] = None


@dataclass
class SynapseCreationRequest:
    """Request for creating synapses."""
    source_neuron_ids: List[int]
    target_neuron_ids: List[int]
    weights: List[float]
    delays: Optional[List[int]] = None
    plasticity_types: Optional[List[int]] = None
    plasticity_coefficients: Optional[List[float]] = None


@dataclass
class NeuronUpdateRequest:
    """Request for updating neuron properties."""
    neuron_ids: List[int]
    property_name: str
    values: List[Union[float, int]]


class NPUInterface:
    """
    High-performance interface to NPU-owned neural data structures.
    
    This class provides SIMD-optimized CRUD operations for neurons and synapses,
    with cortical area locking coordination and deterministic behavior.
    
    Design Principles:
    1. Single source of truth - NPU owns all neural data
    2. SIMD-optimized batch operations
    3. Cortical area-aware locking
    4. Zero fallbacks - deterministic behavior
    5. Rust/RTOS compatible
    """
    
    def __init__(self, backend: BackendType = BackendType.CPU):
        """Initialize NPU interface with specified backend."""
        self.backend = backend
        self.state_manager = FeagiStateManager.instance()
        
        # Initialize data structures
        self.neuron_array = NeuronArray(backend=backend)
        self.memory_neuron_array = MemoryNeuronArray(backend=backend)
        self.synapse_array = SynapseArray(backend=backend)
        
        # Track cortical area mappings using fast integer indices
        self.cortical_areas: Dict[int, Dict[str, Any]] = {}  # cortical_idx -> area_info
        self.neuron_to_area: Dict[int, int] = {}  # neuron_id -> cortical_idx
        self.area_neuron_ranges: Dict[int, Tuple[int, int]] = {}  # cortical_idx -> (start_idx, end_idx)
        
        logger.info(f"🧠 NPU Interface initialized with {backend.value} backend")
        logger.info(f"   Max neurons: {self.neuron_array.max_neurons:,}")
        logger.info(f"   Max memory neurons: {self.memory_neuron_array.max_memory_neurons:,}")
        logger.info(f"   Max synapses: {self.synapse_array.max_synapses:,}")
    
    # ===== CORTICAL AREA MANAGEMENT =====
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                           area_type: str = "regular") -> OperationResult:
        """Create a new cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            dimensions: (width, height, depth) dimensions
            area_type: Type of area ("regular" or "memory")
            
        Returns:
            OperationResult indicating success or failure
        """
        if cortical_idx in self.cortical_areas:
            return OperationResult.INVALID_INPUT
            
        # Check if area is locked
        if self._is_area_locked(cortical_idx):
            return OperationResult.AREA_LOCKED
            
        self.cortical_areas[cortical_idx] = {
            "dimensions": dimensions,
            "type": area_type,
            "neuron_count": 0,
            "created": True
        }
        
        logger.info(f"🧠 Created cortical area idx={cortical_idx} ({area_type}): {dimensions}")
        return OperationResult.SUCCESS
    
    def delete_cortical_area(self, cortical_idx: int) -> OperationResult:
        """Delete a cortical area and all its neurons/synapses.
        
        Args:
            cortical_idx: Fast integer index for the cortical area to delete
            
        Returns:
            OperationResult indicating success or failure
        """
        if cortical_idx not in self.cortical_areas:
            return OperationResult.INVALID_INPUT
            
        # Check if area is locked
        if self._is_area_locked(cortical_idx):
            return OperationResult.AREA_LOCKED
            
        # Delete all neurons in this area
        area_neurons = [nid for nid, cidx in self.neuron_to_area.items() if cidx == cortical_idx]
        if area_neurons:
            self.delete_neurons_batch(area_neurons)
            
        # Remove area
        del self.cortical_areas[cortical_idx]
        if cortical_idx in self.area_neuron_ranges:
            del self.area_neuron_ranges[cortical_idx]
            
        logger.info(f"🗑️  Deleted cortical area idx={cortical_idx} and {len(area_neurons)} neurons")
        return OperationResult.SUCCESS
    
    # ===== NEURON CRUD OPERATIONS =====
    
    def create_neurons_batch(self, request: NeuronCreationRequest) -> BatchOperationResult:
        """Create multiple neurons in batch with SIMD optimization.
        
        Args:
            request: Batch neuron creation request
            
        Returns:
            BatchOperationResult with creation results
        """
        cortical_idx = request.cortical_idx
        positions = request.positions
        count = len(positions)
        
        # Validate inputs
        if not positions:
            return BatchOperationResult(OperationResult.INVALID_INPUT, 0, [])
            
        if cortical_idx not in self.cortical_areas:
            return BatchOperationResult(OperationResult.INVALID_INPUT, 0, list(range(count)))
            
        # Check if area is locked
        if self._is_area_locked(cortical_idx):
            return BatchOperationResult(OperationResult.AREA_LOCKED, 0, list(range(count)))
        
        # Determine target array based on area type
        is_memory_area = self.cortical_areas[cortical_idx]["type"] == "memory"
        target_array = self.memory_neuron_array if is_memory_area else self.neuron_array
        
        # Check capacity
        if target_array.count + count > target_array.max_neurons:
            return BatchOperationResult(OperationResult.CAPACITY_EXCEEDED, 0, list(range(count)))
        
        try:
            # Prepare batch data with defaults
            neuron_types = request.neuron_types or [0] * count
            initial_potentials = request.initial_potentials or [0.0] * count
            thresholds = request.thresholds or [1.0] * count
            leak_coefficients = request.leak_coefficients or [0.1] * count
            excitabilities = request.excitabilities or [1.0] * count
            
            # Get starting indices for new neurons
            start_idx = target_array.count
            neuron_ids = list(range(target_array._next_neuron_id, 
                                  target_array._next_neuron_id + count))
            
            # SIMD-optimized batch creation
            indices = target_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                excitabilities=excitabilities,
                cortical_idx=cortical_idx
            )
            
            # Update mappings
            for i, neuron_id in enumerate(neuron_ids):
                self.neuron_to_area[neuron_id] = cortical_idx
                
            # Update area statistics
            self.cortical_areas[cortical_idx]["neuron_count"] += count
            
            # Update area neuron ranges
            if cortical_idx not in self.area_neuron_ranges:
                self.area_neuron_ranges[cortical_idx] = (start_idx, start_idx + count - 1)
            else:
                current_start, current_end = self.area_neuron_ranges[cortical_idx]
                self.area_neuron_ranges[cortical_idx] = (current_start, current_end + count)
            
            logger.info(f"🧠 Created {count} neurons in area idx={cortical_idx} (IDs: {neuron_ids[0]}-{neuron_ids[-1]})")
            return BatchOperationResult(OperationResult.SUCCESS, count, [])
            
        except Exception as e:
            logger.error(f"❌ Failed to create neurons batch: {e}")
            return BatchOperationResult(OperationResult.BACKEND_ERROR, 0, list(range(count)), str(e))
    
    def delete_neurons_batch(self, neuron_ids: List[int]) -> BatchOperationResult:
        """Delete multiple neurons in batch with SIMD optimization.
        
        Args:
            neuron_ids: List of neuron IDs to delete
            
        Returns:
            BatchOperationResult with deletion results
        """
        if not neuron_ids:
            return BatchOperationResult(OperationResult.SUCCESS, 0, [])
        
        # Group neurons by area and check locks
        area_groups: Dict[int, List[int]] = {}  # cortical_idx -> neuron_ids
        failed_indices = []
        
        for i, neuron_id in enumerate(neuron_ids):
            if neuron_id not in self.neuron_to_area:
                failed_indices.append(i)
                continue
                
            cortical_idx = self.neuron_to_area[neuron_id]
            if self._is_area_locked(cortical_idx):
                failed_indices.append(i)
                continue
                
            if cortical_idx not in area_groups:
                area_groups[cortical_idx] = []
            area_groups[cortical_idx].append(neuron_id)
        
        successful_count = 0
        
        # Process each area group
        for cortical_idx, area_neuron_ids in area_groups.items():
            try:
                # Determine target array
                is_memory_area = self.cortical_areas[cortical_idx]["type"] == "memory"
                target_array = self.memory_neuron_array if is_memory_area else self.neuron_array
                
                # SIMD-optimized batch deletion
                deleted_count = target_array.remove_neurons_batch(area_neuron_ids)
                successful_count += deleted_count
                
                # Update mappings
                for neuron_id in area_neuron_ids:
                    if neuron_id in self.neuron_to_area:
                        del self.neuron_to_area[neuron_id]
                
                # Update area statistics
                self.cortical_areas[cortical_idx]["neuron_count"] -= deleted_count
                
                logger.info(f"🗑️  Deleted {deleted_count} neurons from area idx={cortical_idx}")
                
            except Exception as e:
                logger.error(f"❌ Failed to delete neurons from area idx={cortical_idx}: {e}")
                # Add failed indices for this area
                area_start_idx = neuron_ids.index(area_neuron_ids[0])
                failed_indices.extend(range(area_start_idx, area_start_idx + len(area_neuron_ids)))
        
        result = OperationResult.SUCCESS if not failed_indices else OperationResult.AREA_LOCKED
        return BatchOperationResult(result, successful_count, failed_indices)
    
    def update_neurons_batch(self, request: NeuronUpdateRequest) -> BatchOperationResult:
        """Update neuron properties in batch with SIMD optimization.
        
        Args:
            request: Batch neuron update request
            
        Returns:
            BatchOperationResult with update results
        """
        neuron_ids = request.neuron_ids
        property_name = request.property_name
        values = request.values
        
        if len(neuron_ids) != len(values):
            return BatchOperationResult(OperationResult.INVALID_INPUT, 0, list(range(len(neuron_ids))))
        
        # Group by area and check locks
        area_groups: Dict[int, List[Tuple[int, int, Any]]] = {}  # cortical_idx -> [(neuron_id, index, value)]
        failed_indices = []
        
        for i, (neuron_id, value) in enumerate(zip(neuron_ids, values)):
            if neuron_id not in self.neuron_to_area:
                failed_indices.append(i)
                continue
                
            cortical_idx = self.neuron_to_area[neuron_id]
            if self._is_area_locked(cortical_idx):
                failed_indices.append(i)
                continue
                
            if cortical_idx not in area_groups:
                area_groups[cortical_idx] = []
            area_groups[cortical_idx].append((neuron_id, i, value))
        
        successful_count = 0
        
        # Process each area group
        for cortical_idx, area_updates in area_groups.items():
            try:
                # Determine target array
                is_memory_area = self.cortical_areas[cortical_idx]["type"] == "memory"
                target_array = self.memory_neuron_array if is_memory_area else self.neuron_array
                
                # Extract data for batch update
                batch_neuron_ids = [item[0] for item in area_updates]
                batch_values = [item[2] for item in area_updates]
                
                # SIMD-optimized batch update
                updated_count = target_array.update_property_batch(
                    neuron_ids=batch_neuron_ids,
                    property_name=property_name,
                    values=batch_values
                )
                successful_count += updated_count
                
                logger.debug(f"🔄 Updated {updated_count} neurons '{property_name}' in area idx={cortical_idx}")
                
            except Exception as e:
                logger.error(f"❌ Failed to update neurons in area idx={cortical_idx}: {e}")
                # Add failed indices for this area
                for _, original_idx, _ in area_updates:
                    failed_indices.append(original_idx)
        
        result = OperationResult.SUCCESS if not failed_indices else OperationResult.AREA_LOCKED
        return BatchOperationResult(result, successful_count, failed_indices)
    
    # ===== SYNAPSE CRUD OPERATIONS =====
    
    def create_synapses_batch(self, request: SynapseCreationRequest) -> BatchOperationResult:
        """Create multiple synapses in batch with SIMD optimization.
        
        Args:
            request: Batch synapse creation request
            
        Returns:
            BatchOperationResult with creation results
        """
        source_ids = request.source_neuron_ids
        target_ids = request.target_neuron_ids
        weights = request.weights
        
        count = len(source_ids)
        if count != len(target_ids) or count != len(weights):
            return BatchOperationResult(OperationResult.INVALID_INPUT, 0, list(range(count)))
        
        # Check capacity
        if self.synapse_array.count + count > self.synapse_array.max_synapses:
            return BatchOperationResult(OperationResult.CAPACITY_EXCEEDED, 0, list(range(count)))
        
        # Check area locks for all involved neurons
        locked_indices = []
        for i, (src_id, tgt_id) in enumerate(zip(source_ids, target_ids)):
            src_cortical_idx = self.neuron_to_area.get(src_id)
            tgt_cortical_idx = self.neuron_to_area.get(tgt_id)
            
            if src_cortical_idx is None or tgt_cortical_idx is None:
                locked_indices.append(i)
                continue
                
            if self._is_area_locked(src_cortical_idx) or self._is_area_locked(tgt_cortical_idx):
                locked_indices.append(i)
        
        if locked_indices:
            return BatchOperationResult(OperationResult.AREA_LOCKED, 0, locked_indices)
        
        try:
            # Prepare batch data with defaults
            delays = request.delays or [1] * count
            plasticity_types = request.plasticity_types or [0] * count
            plasticity_coeffs = request.plasticity_coefficients or [0.0] * count
            
            # SIMD-optimized batch creation
            created_count = self.synapse_array.add_synapses_batch(
                source_neuron_ids=source_ids,
                target_neuron_ids=target_ids,
                weights=weights,
                delays=delays,
                plasticity_types=plasticity_types,
                plasticity_coefficients=plasticity_coeffs
            )
            
            logger.info(f"🔗 Created {created_count} synapses")
            return BatchOperationResult(OperationResult.SUCCESS, created_count, [])
            
        except Exception as e:
            logger.error(f"❌ Failed to create synapses batch: {e}")
            return BatchOperationResult(OperationResult.BACKEND_ERROR, 0, list(range(count)), str(e))
    
    def delete_synapses_batch(self, synapse_indices: List[int]) -> BatchOperationResult:
        """Delete multiple synapses in batch with SIMD optimization.
        
        Args:
            synapse_indices: List of synapse indices to delete
            
        Returns:
            BatchOperationResult with deletion results
        """
        if not synapse_indices:
            return BatchOperationResult(OperationResult.SUCCESS, 0, [])
        
        # Check area locks for all involved synapses
        locked_indices = []
        for i, syn_idx in enumerate(synapse_indices):
            if syn_idx >= self.synapse_array.count:
                locked_indices.append(i)
                continue
                
            # Get source and target neurons for this synapse
            src_id = self.synapse_array.source_neuron_ids[syn_idx]
            tgt_id = self.synapse_array.target_neuron_ids[syn_idx]
            
            src_cortical_idx = self.neuron_to_area.get(src_id)
            tgt_cortical_idx = self.neuron_to_area.get(tgt_id)
            
            if src_cortical_idx is not None and self._is_area_locked(src_cortical_idx):
                locked_indices.append(i)
                continue
            if tgt_cortical_idx is not None and self._is_area_locked(tgt_cortical_idx):
                locked_indices.append(i)
        
        if locked_indices:
            return BatchOperationResult(OperationResult.AREA_LOCKED, 0, locked_indices)
        
        try:
            # SIMD-optimized batch deletion
            deleted_count = self.synapse_array.remove_synapses_batch(synapse_indices)
            
            logger.info(f"🗑️  Deleted {deleted_count} synapses")
            return BatchOperationResult(OperationResult.SUCCESS, deleted_count, [])
            
        except Exception as e:
            logger.error(f"❌ Failed to delete synapses batch: {e}")
            return BatchOperationResult(OperationResult.BACKEND_ERROR, 0, list(range(len(synapse_indices))), str(e))
    
    # ===== DATA ACCESS METHODS =====
    
    def get_neuron_property(self, neuron_id: int, property_name: str) -> Optional[Union[float, int]]:
        """Get a single neuron property value.
        
        Args:
            neuron_id: Neuron ID
            property_name: Property name
            
        Returns:
            Property value or None if not found
        """
        if neuron_id not in self.neuron_to_area:
            return None
            
        cortical_idx = self.neuron_to_area[neuron_id]
        is_memory_area = self.cortical_areas[cortical_idx]["type"] == "memory"
        target_array = self.memory_neuron_array if is_memory_area else self.neuron_array
        
        return target_array.get_property(neuron_id, property_name)
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get all neuron IDs in a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            List of neuron IDs in the area
        """
        return [nid for nid, cidx in self.neuron_to_area.items() if cidx == cortical_idx]
    
    def get_area_statistics(self, cortical_idx: int) -> Optional[Dict[str, Any]]:
        """Get statistics for a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            Dictionary with area statistics or None if not found
        """
        if cortical_idx not in self.cortical_areas:
            return None
            
        area_info = self.cortical_areas[cortical_idx].copy()
        area_info["is_locked"] = self._is_area_locked(cortical_idx)
        return area_info
    
    # ===== INTERNAL METHODS =====
    
    def _is_area_locked(self, cortical_idx: int) -> bool:
        """Check if a cortical area is locked for BDU operations.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            True if area is locked
        """
        return self.state_manager.is_cortical_area_locked(cortical_idx)
    
    def get_total_statistics(self) -> Dict[str, Any]:
        """Get total system statistics.
        
        Returns:
            Dictionary with total neuron/synapse counts and capacity info
        """
        return {
            "total_neurons": self.neuron_array.count,
            "total_memory_neurons": self.memory_neuron_array.count,
            "total_synapses": self.synapse_array.count,
            "max_neurons": self.neuron_array.max_neurons,
            "max_memory_neurons": self.memory_neuron_array.max_memory_neurons,
            "max_synapses": self.synapse_array.max_synapses,
            "cortical_areas": len(self.cortical_areas),
            "backend": self.backend.value
        }
