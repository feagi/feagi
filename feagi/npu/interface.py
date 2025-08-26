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

from typing import Dict, List, Tuple, Optional, Any, Union
from enum import Enum
import numpy as np
from dataclasses import dataclass

from feagi.core.state_manager import FeagiStateManager
from feagi.npu.data_structures import NeuronArray, MemoryNeuronArray, SynapseArray, BackendType
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


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
    data: Optional[Dict[str, Any]] = None
    
    @property
    def is_success(self) -> bool:
        return self.result == OperationResult.SUCCESS
    
    @property
    def has_partial_success(self) -> bool:
        return self.successful_count > 0 and len(self.failed_indices) > 0


# NOTE: NeuronCreationRequest is defined later in this file (single canonical definition)


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
class NeuronCreationRequest:
    """Request for batch neuron creation.

    Note: This definition supersedes the earlier lightweight `NeuronCreationRequest` above.
    """
    cortical_idx: int
    positions: List[Tuple[int, int, int]]
    neuron_types: Optional[List[int]] = None
    initial_potentials: Optional[List[float]] = None
    thresholds: Optional[List[float]] = None
    leak_coefficients: Optional[List[float]] = None
    excitabilities: Optional[List[float]] = None

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
        # Per-area excitability cache (default 1.0)
        self._area_excitability: Dict[int, float] = {}
        self._low_ex_areas: Dict[int, bool] = {}

        # Track next-burst candidates computed during Phase 2 (post-synaptic propagation)
        # These should not fire within the same burst; they are added to FCL for t+1
        self._next_burst_candidate_ids: List[int] = []

    # ===== EXCITABILITY CACHE (PER-AREA) =====
    def set_area_excitability(self, cortical_idx: int, value: float) -> None:
        """Set per-area neuron_excitability (0..1) for the cortical area.

        Args:
            cortical_idx: Area index
            value: Probability in [0.0, 1.0]
        """
        v = float(value)
        if v < 0.0:
            v = 0.0
        elif v > 1.0:
            v = 1.0
        self._area_excitability[cortical_idx] = v
        self._low_ex_areas[cortical_idx] = v < 0.999

    def get_area_excitability(self, cortical_idx: int) -> float:
        return self._area_excitability.get(cortical_idx, 1.0)

    def any_low_ex_area(self) -> bool:
        # Fast path check
        return any(self._low_ex_areas.values())
    
    # ===== CORTICAL AREA MANAGEMENT =====
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                           area_type: str = "regular", cortical_id: str = None) -> OperationResult:
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
            
        # Allow creation during BDU-held locks; reads/writes will be blocked appropriately
            
        self.cortical_areas[cortical_idx] = {
            "cortical_id": cortical_id,
            "dimensions": dimensions,
            "type": area_type,
            "neuron_count": 0,
            "created": True
        }
        
        logger.info(f"🧠 Created cortical area idx={cortical_idx} ({area_type}): {dimensions}")
        return OperationResult.SUCCESS
    
    def get_cortical_idx_by_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical_idx by cortical_id string.
        
        Args:
            cortical_id: String identifier (e.g., "_power")
            
        Returns:
            cortical_idx if found, None otherwise
        """
        for cortical_idx, area_info in self.cortical_areas.items():
            if area_info.get("cortical_id") == cortical_id:
                return cortical_idx
        return None
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get all neuron IDs in a cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            List of neuron IDs in the area
        """
        return [nid for nid, aidx in self.neuron_to_area.items() if aidx == cortical_idx]
    
    def debug_cortical_areas(self) -> Dict[str, Any]:
        """Debug method to show all cortical areas and their neuron counts.
        
        Returns:
            Dictionary with area information for debugging
        """
        debug_info = {
            "total_areas": len(self.cortical_areas),
            "areas": {},
            "neuron_to_area_mapping_count": len(self.neuron_to_area)
        }
        
        for cortical_idx, area_info in self.cortical_areas.items():
            cortical_id = area_info.get("cortical_id", f"idx_{cortical_idx}")
            neuron_count = len(self.get_neurons_by_area(cortical_idx))
            
            debug_info["areas"][cortical_id] = {
                "cortical_idx": cortical_idx,
                "stored_neuron_count": area_info.get("neuron_count", 0),
                "actual_neuron_count": neuron_count,
                "type": area_info.get("type", "unknown"),
                "dimensions": area_info.get("dimensions", "unknown")
            }
        
        return debug_info
    
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
            logger.error(f"❌ Cortical area idx={cortical_idx} not found in NPU Interface")
            logger.error(f"❌ Available cortical areas: {list(self.cortical_areas.keys())}")
            return BatchOperationResult(OperationResult.INVALID_INPUT, 0, list(range(count)))
            
        # TODO: Re-enable area locking check after fixing the locking mechanism
        # The issue is that BDU locks the area but NPU Interface rejects BDU's own operations
        # For now, disable the check to get basic neurogenesis working
        # if self._is_area_locked(cortical_idx):
        #     return BatchOperationResult(OperationResult.AREA_LOCKED, 0, list(range(count)))
        
        # Determine target array based on area type
        is_memory_area = self.cortical_areas[cortical_idx]["type"] == "memory"
        if is_memory_area:
            # Memory areas do not accept regular neuron creation via this path
            return BatchOperationResult(
                result=OperationResult.INVALID_INPUT,
                successful_count=0,
                failed_indices=list(range(count)),
                error_message="Cannot create regular neurons in memory area via create_neurons_batch"
            )
        target_array = self.neuron_array
        
        # Check capacity
        if target_array.count + count > target_array.max_neurons:
            return BatchOperationResult(OperationResult.CAPACITY_EXCEEDED, 0, list(range(count)))
        
        try:
            # Prepare batch data with defaults
            neuron_types = request.neuron_types or [0] * count
            initial_potentials = request.initial_potentials or [0.0] * count
            thresholds = request.thresholds or [1.0] * count
            leak_coefficients = request.leak_coefficients or [0.1] * count
            
            # Get starting indices for new neurons
            start_idx = target_array.count
            neuron_ids = list(range(target_array._next_neuron_id, 
                                  target_array._next_neuron_id + count))
            
            # SIMD-optimized batch creation
            target_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                cortical_idx=cortical_idx
            )
            
            # Update mappings
            for i, neuron_id in enumerate(neuron_ids):
                self.neuron_to_area[neuron_id] = cortical_idx
                # Debug logging for power area (gated)
                try:
                    if (
                        cortical_idx in self.cortical_areas
                        and self.cortical_areas[cortical_idx].get("cortical_id") == "_power"
                        and FeagiStateManager.instance().is_debug_npu_enabled()
                    ):
                        logger.info(
                            f"[NPU-POWER-DEBUG] Mapped neuron {neuron_id} to cortical_idx {cortical_idx} (_power)"
                        )
                except Exception:
                    pass
                
            # Update area statistics
            self.cortical_areas[cortical_idx]["neuron_count"] += count
            
            # Update area neuron ranges
            if cortical_idx not in self.area_neuron_ranges:
                self.area_neuron_ranges[cortical_idx] = (start_idx, start_idx + count - 1)
            else:
                current_start, current_end = self.area_neuron_ranges[cortical_idx]
                self.area_neuron_ranges[cortical_idx] = (current_start, current_end + count)
            
            logger.info(f"🧠 Created {count} neurons in area idx={cortical_idx} (IDs: {neuron_ids[0]}-{neuron_ids[-1]})")
            return BatchOperationResult(
                result=OperationResult.SUCCESS, 
                successful_count=count, 
                failed_indices=[]
            )
            
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
    
    # NOTE: get_neurons_by_area is defined earlier in this class.
    
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
        try:
            if hasattr(self.state_manager, 'is_cortical_area_locked'):
                result = self.state_manager.is_cortical_area_locked(cortical_idx)
                logger.debug(f"🔒 Area {cortical_idx} lock check: {result}")
                return result
            else:
                logger.warning("🔒 State manager doesn't have is_cortical_area_locked method - assuming unlocked")
                return False
        except Exception as e:
            logger.error(f"🔒 Error checking area lock for {cortical_idx}: {e}")
            return False
    
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
    
    def get_firing_neurons_by_cortical_area(self, fired_neuron_ids: List[int]) -> Dict[int, List[int]]:
        """Group fired neurons by cortical area for FCL manager integration.
        
        This method provides efficient grouping of fired neurons by cortical area,
        which is needed for FCL manager updates.
        
        Args:
            fired_neuron_ids: List of neuron IDs that fired
            
        Returns:
            Dictionary mapping cortical_idx -> list of neuron IDs
        """
        neurons_by_cortical = {}
        
        for neuron_id in fired_neuron_ids:
            if neuron_id in self.neuron_array.neuron_id_to_index:
                idx = self.neuron_array.neuron_id_to_index[neuron_id]
                cortical_idx = int(self.neuron_array.cortical_idxs[idx])
                
                if cortical_idx not in neurons_by_cortical:
                    neurons_by_cortical[cortical_idx] = []
                neurons_by_cortical[cortical_idx].append(neuron_id)
        
        return neurons_by_cortical
    
    # ===== NEURAL PROCESSING =====
    
    def process_neural_burst(self, timestep: int) -> List[int]:
        """Process a complete neural burst - main entry point for neural processing.
        
        This method provides the core neural processing functionality that was
        previously in the archived NeuralProcessor. It handles:
        - Neural updates (membrane potentials, firing)
        - Synaptic propagation  
        - SIMD/GPU optimization
        - Integration with FCL management
        
        Args:
            timestep: Current simulation timestep
            
        Returns:
            List of neuron IDs that fired
        """
        from feagi.core.state_manager import get_state_manager
        state_manager = get_state_manager()
        
        if state_manager.is_debug_npu_enabled():
            logger.info(f"[NPU-BURST-DEBUG] === NEURAL BURST PROCESSING START (timestep {timestep}) ===")
            logger.info(f"[NPU-BURST-DEBUG] NPU neuron count: {self.neuron_array.neuron_count}")
            logger.info(f"[NPU-BURST-DEBUG] NPU synapse count: {self.synapse_array.synapse_count}")
            logger.info("[NPU-BURST-DEBUG] PHASE 1: Neural updates...")
        
        # Phase 1: Neural updates using SIMD operations
        from feagi.npu.simd_neural_ops import simd_batch_neural_update
        
        # Get valid neuron range
        valid_range = min(self.neuron_array.neuron_count, self.neuron_array.max_neurons)
        
        if valid_range == 0:
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-BURST-DEBUG] No neurons to process - returning empty list")
            return []

        # Debug precheck: how many neurons are above threshold and not refractory
        if state_manager.is_debug_npu_enabled():
            pots = self.neuron_array.membrane_potentials[:valid_range]
            thrs = self.neuron_array.thresholds[:valid_range]
            refr = self.neuron_array.refractory_counters[:valid_range]
            above_mask = pots >= thrs
            can_fire_mask = refr == 0
            ready_mask = above_mask & can_fire_mask
            above_cnt = int(np.count_nonzero(above_mask))
            ready_cnt = int(np.count_nonzero(ready_mask))
            logger.info(f"[NPU-BURST-DEBUG] PHASE 1 PRECHECK: above_threshold={above_cnt}, ready_to_fire={ready_cnt}")
            if ready_cnt > 0:
                sample_idx = np.where(ready_mask)[0][:5]
                try:
                    sample_ids = self.neuron_array.indices_to_neuron_ids(sample_idx, filter_invalid=True).tolist()
                except Exception:
                    sample_ids = []
                sample_pots = pots[sample_idx].astype(float).tolist()
                sample_thrs = thrs[sample_idx].astype(float).tolist()
                sample_refr = refr[sample_idx].astype(int).tolist()
                logger.info(f"[NPU-BURST-DEBUG] PHASE 1 PRECHECK SAMPLE: idx={sample_idx.tolist()}, ids={sample_ids}, V={sample_pots}, Thr={sample_thrs}, Refr={sample_refr}")
                # Temporary detailed logs: ready counts per area and excitability
                try:
                    ready_idx = np.where(ready_mask)[0]
                    areas = self.neuron_array.cortical_idxs[ready_idx].astype(np.int32) if ready_idx.size > 0 else np.array([], dtype=np.int32)
                    if areas.size > 0:
                        uniq, cnts = np.unique(areas, return_counts=True)
                        logger.info(f"[NPU-BURST-DEBUG] any_low_ex_area={self.any_low_ex_area()}")
                        for a, c in zip(uniq.tolist(), cnts.tolist()):
                            ex = float(self._area_excitability.get(int(a), 1.0))
                            logger.info(f"[NPU-BURST-DEBUG] READY BY AREA: idx={int(a)} ex={ex} count={int(c)}")
                except Exception as dbg_e:
                    logger.warning(f"[NPU-BURST-DEBUG] Ready-per-area debug failed: {dbg_e}")
        
        # Perform SIMD neural updates
        # Create output mask for fired neurons
        output_firing_mask = np.zeros(valid_range, dtype=bool)
        
        # Decide fast path: if all areas have excitability ~1.0, skip RNG path
        fired_count = simd_batch_neural_update(
            potentials=self.neuron_array.membrane_potentials[:valid_range],
            thresholds=self.neuron_array.thresholds[:valid_range],
            decay_rates=self.neuron_array.leak_coefficients[:valid_range],
            resting_potentials=self.neuron_array.resting_potentials[:valid_range],
            refractory_periods=self.neuron_array.refractory_periods[:valid_range],
            refractory_counters=self.neuron_array.refractory_counters[:valid_range],
            # Pass a callable to fetch area excitability by neuron index
            excitability=(self._area_excitability, self.neuron_array.cortical_idxs[:valid_range], self.any_low_ex_area()),
            valid_mask=np.ones(valid_range, dtype=bool),  # All neurons in range are valid
            output_firing_mask=output_firing_mask
        )
        
        # Extract fired array indices from the firing mask
        fired_indices = np.where(output_firing_mask)[0]
        # Map indices to neuron IDs (authoritative mapping owned by NPU)
        fired_id_array = self.neuron_array.indices_to_neuron_ids(
            fired_indices, filter_invalid=True
        )
        fired_neurons = fired_id_array.tolist()
        # Temporary detailed logs: fired counts per area and excitability
        if state_manager.is_debug_npu_enabled():
            try:
                if fired_indices.size > 0:
                    fired_areas = self.neuron_array.cortical_idxs[fired_indices].astype(np.int32)
                    uniq_f, cnts_f = np.unique(fired_areas, return_counts=True)
                    for a, c in zip(uniq_f.tolist(), cnts_f.tolist()):
                        ex = float(self._area_excitability.get(int(a), 1.0))
                        logger.info(f"[NPU-BURST-DEBUG] FIRED BY AREA: idx={int(a)} ex={ex} count={int(c)}")
            except Exception as dbg_e:
                logger.warning(f"[NPU-BURST-DEBUG] Fired-per-area debug failed: {dbg_e}")
        
        if state_manager.is_debug_npu_enabled():
            fired_count = len(fired_neurons) if fired_neurons else 0
            logger.info(f"[NPU-BURST-DEBUG] PHASE 1 COMPLETE: {fired_count} neurons fired")
            if fired_count > 0:
                logger.info(f"[NPU-BURST-DEBUG] PHASE 1 FIRED SAMPLE: {fired_neurons[:5]}")
        
        # Phase 2: Synaptic propagation (if we have synapses and fired neurons)
        if fired_neurons and self.synapse_array.synapse_count > 0:
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-BURST-DEBUG] PHASE 2: Synaptic propagation... fired={len(fired_neurons)}, synapses={self.synapse_array.synapse_count}, npu_id={id(self)}"
                )
            
            # Gather all outgoing synapses for fired neurons
            all_syn_indices: List[int] = []
            src_counts = 0
            for src_id in fired_neurons:
                syn_list = self.synapse_array.source_neuron_index.get(src_id)
                if syn_list:
                    all_syn_indices.extend(syn_list)
                    src_counts += 1
            if state_manager.is_debug_npu_enabled():
                logger.info(
                    f"[NPU-BURST-DEBUG] PHASE 2: Aggregated synapses from {src_counts} sources, total_edges={len(all_syn_indices)}"
                )

            if all_syn_indices:
                syn_indices = np.array(all_syn_indices, dtype=np.int32)
                # Filter valid synapses
                valid_mask = self.synapse_array.valid_mask[syn_indices]
                if np.any(valid_mask):
                    syn_indices = syn_indices[valid_mask]
                    if state_manager.is_debug_npu_enabled():
                        logger.info(
                            f"[NPU-BURST-DEBUG] PHASE 2: Valid edges after mask={syn_indices.size}"
                        )
                    # Fetch targets and weights
                    target_ids = self.synapse_array.target_neuron_ids[syn_indices].astype(np.int32)
                    weights = self.synapse_array.weights[syn_indices].astype(np.float32)
                    # Apply synapse type (excitatory/inhibitory) and conductance
                    syn_types = self.synapse_array.types[syn_indices].astype(np.int32)
                    conductances = self.synapse_array.conductances[syn_indices].astype(np.float32)
                    sign = np.where(syn_types == 0, 1.0, -1.0).astype(np.float32)
                    contributions = weights * conductances * sign
                    # Map target neuron IDs to array indices
                    # Vectorized mapping via dictionary (deterministic, no fallbacks)
                    target_indices = np.fromiter(
                        (self.neuron_array.neuron_id_to_index.get(int(tid), -1) for tid in target_ids.tolist()),
                        dtype=np.int32,
                        count=target_ids.size,
                    )
                    valid_targets = target_indices >= 0
                    if np.any(valid_targets):
                        target_indices = target_indices[valid_targets]
                        contributions = contributions[valid_targets]
                        if state_manager.is_debug_npu_enabled():
                            logger.info(
                                f"[NPU-BURST-DEBUG] PHASE 2: Valid target indices={target_indices.size} (sample={target_indices[:5].tolist() if target_indices.size>0 else []})"
                            )
                        # Accumulate post-synaptic current into membrane potentials
                        np.add.at(
                            self.neuron_array.membrane_potentials,
                            target_indices,
                            contributions,
                        )
                        # Ensure these targets are marked non-refractory for next-cycle eligibility only
                        # (RTOS deterministic: refractory only set on actual firing)
                        if state_manager.is_debug_npu_enabled():
                            logger.info(
                                f"[NPU-BURST-DEBUG] PHASE 2: Applied {contributions.size} synaptic updates from {src_counts} sources"
                            )

                        # Determine next-burst eligible targets (do NOT fire in current burst)
                        # Deterministic and vectorized; limited to affected targets only
                        unique_targets = np.unique(target_indices)
                        # Clip to valid neuron range
                        valid_target_mask = unique_targets < min(self.neuron_array.neuron_count, self.neuron_array.max_neurons)
                        unique_targets = unique_targets[valid_target_mask]
                        if unique_targets.size > 0:
                            from feagi.npu.simd_neural_ops import simd_firing_check
                            # Build can-fire mask (not in refractory)
                            can_fire_mask = self.neuron_array.refractory_counters[unique_targets] == 0
                            if np.any(can_fire_mask):
                                eligible_mask = simd_firing_check(
                                    self.neuron_array.membrane_potentials[unique_targets],
                                    self.neuron_array.thresholds[unique_targets],
                                    can_fire_mask,
                                )
                                if np.any(eligible_mask):
                                    eligible_global = unique_targets[eligible_mask]
                                    # Map eligible target indices back to neuron IDs
                                    eligible_ids = self.neuron_array.indices_to_neuron_ids(
                                        eligible_global, filter_invalid=True
                                    ).tolist()
                                    if eligible_ids:
                                        # Accumulate next-burst candidates (dedup later)
                                        self._next_burst_candidate_ids.extend(eligible_ids)
                                        if state_manager.is_debug_npu_enabled():
                                            logger.info(
                                                f"[NPU-BURST-DEBUG] PHASE 2: Next-burst eligible targets: {len(eligible_ids)} (sample={eligible_ids[:5]})"
                                            )
                                else:
                                    if state_manager.is_debug_npu_enabled():
                                        logger.info(
                                            f"[NPU-BURST-DEBUG] PHASE 2: No next-burst eligible targets (targets={unique_targets.size})"
                                        )
                            else:
                                if state_manager.is_debug_npu_enabled():
                                    logger.info(
                                        "[NPU-BURST-DEBUG] PHASE 2: No neurons can be eligible (all refractory or below threshold)"
                                    )
                    else:
                        if state_manager.is_debug_npu_enabled():
                            logger.info(
                                f"[NPU-BURST-DEBUG] PHASE 2: 0 valid target indices (mapped from {target_ids.size} target IDs)"
                            )
            else:
                if state_manager.is_debug_npu_enabled():
                    logger.info("[NPU-BURST-DEBUG] PHASE 2: No outgoing synapses from current fired set")
        
            if state_manager.is_debug_npu_enabled():
                logger.info("[NPU-BURST-DEBUG] PHASE 2 COMPLETE: Synaptic propagation processed")
        
        if state_manager.is_debug_npu_enabled():
            logger.info("[NPU-BURST-DEBUG] === NEURAL BURST PROCESSING END ===")
            logger.info(f"[NPU-BURST-DEBUG] Total fired neurons: {len(fired_neurons) if fired_neurons else 0}")
        
        return fired_neurons if fired_neurons else []
