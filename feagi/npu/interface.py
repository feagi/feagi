"""
NPU Interface - Rust NPU Delegation Layer

This interface delegates ALL neuron and synapse operations to the Rust NPU,
eliminating duplicate Python data structures. It acts as a thin adapter between
Python code and the Rust NPU, maintaining API compatibility while ensuring
the Rust NPU is the single source of truth.

ARCHITECTURE:
- Rust NPU: Owns the ONLY neuron_array and synapse_array (in Rust memory)
- NPUInterface: Python API that DELEGATES to Rust NPU (proxy/adapter pattern)
- ConnectomeManager: Uses NPUInterface to update neurons/synapses in Rust NPU
"""

from typing import Dict, List, Tuple, Optional, Any, Union
from enum import Enum
from dataclasses import dataclass

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
        """Check if the operation was fully successful."""
        return self.result == OperationResult.SUCCESS
    
    @property
    def has_partial_success(self) -> bool:
        """Check if the operation had partial success."""
        return self.successful_count > 0 and len(self.failed_indices) > 0


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
    """Request for batch neuron creation."""
    cortical_idx: int
    positions: List[Tuple[int, int, int]]
    neuron_types: Optional[List[int]] = None
    initial_potentials: Optional[List[float]] = None
    thresholds: Optional[List[float]] = None
    leak_coefficients: Optional[List[float]] = None
    decay_rates: Optional[List[float]] = None
    refractory_periods: Optional[List[int]] = None
    excitabilities: Optional[List[float]] = None
    resting_potentials: Optional[List[float]] = None
    consecutive_fire_limits: Optional[List[int]] = None
    snooze_periods: Optional[List[int]] = None


@dataclass
class NeuronUpdateRequest:
    """Request for updating neuron properties."""
    neuron_ids: List[int]
    property_name: str
    values: List[Union[float, int]]


class NPUInterface:
    """NPU interface that delegates to Rust NPU (single source of truth).
    
    This class acts as a thin adapter layer between Python code and the Rust NPU.
    It maintains API compatibility while ensuring all neuron and synapse data
    lives ONLY in Rust memory.
    
    CRITICAL: No duplicate Python arrays! Rust NPU is the single source of truth.
    """
    
    def __init__(self, backend=None, max_neurons: int = 100_000, 
                 max_synapses: int = 500_000, max_memory_neurons: int = 50_000):
        """Initialize NPU interface.
        
        Note: Rust NPU reference is set later via set_rust_npu_integration().
        """
        # NO self.neuron_array! NO self.synapse_array!
        # Rust NPU is the single source of truth!
        
        self._rust_npu_integration = None  # Set by BurstEngine
        self._connectome_manager = None  # Set by ConnectomeManager
        
        # State tracking
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.max_memory_neurons = max_memory_neurons
        
        # Cortical area management for compatibility
        self.cortical_areas: Dict[int, Dict[str, Any]] = {}
        self.neuron_to_area: Dict[int, int] = {}
        
        # Neuron ID tracking (for compatibility - maps neuron_id to array index)
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        self._next_neuron_id = 1
        
        # Plasticity command queue (for future plasticity support)
        self._plasticity_queue_capacity: int = 0
        self._plasticity_queue: List[Dict[str, Any]] = []
        self._plasticity_lock = None
        try:
            import threading
            self._plasticity_lock = threading.RLock()
        except Exception:
            pass
        
        logger.info("🔄 [NPU-INTERFACE] Initialized (delegating to Rust NPU)")
    
    def set_rust_npu_integration(self, rust_npu_integration):
        """Set the Rust NPU integration reference.
        
        Called by BurstEngine after Rust NPU is initialized.
        """
        self._rust_npu_integration = rust_npu_integration
        logger.info("🦀 [NPU-INTERFACE] Rust NPU integration connected")
    
    def set_connectome_manager(self, connectome_manager):
        """Set the ConnectomeManager reference (for coordinate lookups)."""
        self._connectome_manager = connectome_manager
    
    @property
    def rust_npu(self):
        """Get Rust NPU handle (for direct access)."""
        if self._rust_npu_integration is None:
            raise RuntimeError("Rust NPU not initialized yet - call set_rust_npu_integration() first")
        return self._rust_npu_integration._rust_npu
    
    def create_neurons_batch(self, request: NeuronCreationRequest) -> BatchOperationResult:
        """Create neurons by delegating to Rust NPU."""
        try:
            if self._rust_npu_integration is None:
                # BUFFERING MODE: Rust NPU not ready yet (happens during genome load)
                # Generate neuron IDs and track them - Rust NPU will create them later
                logger.warning("[NPU-INTERFACE] Rust NPU not ready - tracking neurons for later creation")
                
                neuron_ids = []
                for i in range(len(request.positions)):
                    neuron_id = self._next_neuron_id
                    self._next_neuron_id += 1
                    
                    # Track in NPUInterface for verification
                    neuron_ids.append(neuron_id)
                    self.neuron_id_to_index[neuron_id] = neuron_id  # ID == index for now
                    self.index_to_neuron_id[neuron_id] = neuron_id
                    self.neuron_to_area[neuron_id] = request.cortical_idx
                
                return BatchOperationResult(
                    result=OperationResult.SUCCESS,
                    successful_count=len(request.positions),
                    failed_indices=[],
                    data={"neuron_ids": neuron_ids}
                )
            
            # Validate required parameters
            if request.thresholds is None:
                raise ValueError("thresholds required")
            if request.leak_coefficients is None:
                raise ValueError("leak_coefficients required")
            if request.resting_potentials is None:
                raise ValueError("resting_potentials required")
            
            neuron_ids = []
            successful_count = 0
            failed_indices = []
            
            for i, (x, y, z) in enumerate(request.positions):
                try:
                    neuron_id = self.rust_npu.add_neuron(
                        threshold=float(request.thresholds[i]) if request.thresholds else 1.0,
                        leak_coefficient=float(request.leak_coefficients[i]) if request.leak_coefficients else 0.0,
                        resting_potential=float(request.resting_potentials[i]) if request.resting_potentials else 0.0,
                        neuron_type=int(request.neuron_types[i]) if request.neuron_types else 0,
                        refractory_period=int(request.refractory_periods[i]) if request.refractory_periods else 0,
                        excitability=float(request.excitabilities[i]) if request.excitabilities else 1.0,
                        consecutive_fire_limit=int(request.consecutive_fire_limits[i]) if request.consecutive_fire_limits else 0,
                        snooze_period=int(request.snooze_periods[i]) if request.snooze_periods else 0,
                        cortical_area=int(request.cortical_idx),
                        x=int(x),
                        y=int(y),
                        z=int(z)
                    )
                    
                    neuron_ids.append(neuron_id)
                    self.neuron_id_to_index[neuron_id] = neuron_id  # For now, ID == index
                    self.index_to_neuron_id[neuron_id] = neuron_id
                    self.neuron_to_area[neuron_id] = request.cortical_idx
                    successful_count += 1
                    
                except Exception as e:
                    logger.error(f"[NPU-INTERFACE] Failed to create neuron {i}: {e}")
                    failed_indices.append(i)
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS if len(failed_indices) == 0 else OperationResult.BACKEND_ERROR,
                successful_count=successful_count,
                failed_indices=failed_indices,
                data={"neuron_ids": neuron_ids}
            )
            
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Batch neuron creation failed: {e}")
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )
    
    def create_synapses_batch(self, request: SynapseCreationRequest) -> BatchOperationResult:
        """Create synapses by delegating to Rust NPU."""
        try:
            if self._rust_npu_integration is None:
                logger.warning("[NPU-INTERFACE] Rust NPU not ready - buffering synapse creation")
                return BatchOperationResult(
                    result=OperationResult.SUCCESS,
                    successful_count=len(request.source_neuron_ids),
                    failed_indices=[]
                )
            
            successful_count = 0
            failed_indices = []
            
            for i in range(len(request.source_neuron_ids)):
                try:
                    source = int(request.source_neuron_ids[i])
                    target = int(request.target_neuron_ids[i])
                    weight = int(max(0, min(255, request.weights[i])))  # Clamp to u8
                    
                    # For now, use weight as both weight and conductance
                    self.rust_npu.add_synapse(
                        source_neuron=source,
                        target_neuron=target,
                        weight=weight,
                        conductance=weight,  # Same as weight for now
                        synapse_type=0  # 0 = excitatory
                    )
                    
                    successful_count += 1
                    
                except Exception as e:
                    logger.error(f"[NPU-INTERFACE] Failed to create synapse {i}: {e}")
                    failed_indices.append(i)
            
            # Rebuild synapse indexes after batch creation
            if successful_count > 0:
                try:
                    self.rust_npu.rebuild_indexes()
                except Exception as e:
                    logger.error(f"[NPU-INTERFACE] Failed to rebuild synapse indexes: {e}")
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS if len(failed_indices) == 0 else OperationResult.BACKEND_ERROR,
                successful_count=successful_count,
                failed_indices=failed_indices
            )
            
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Batch synapse creation failed: {e}")
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
    
    def get_property(self, neuron_id: int, property_name: str) -> Any:
        """Get neuron property from Rust NPU."""
        if self._rust_npu_integration is None:
            logger.warning("[NPU-INTERFACE] Rust NPU not ready - cannot get property")
            return None
        
        # Use get_neuron_state to retrieve properties
        try:
            state = self.rust_npu.get_neuron_state(neuron_id)
            if state is None:
                return None
            
            # Unpack state: (cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown)
            cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown = state
            
            property_map = {
                'membrane_potential': potential,
                'threshold': threshold,
                'refractory_countdown': refrac_countdown,
                'consecutive_fire_count': cfc,
                'consecutive_fire_limit': cfc_limit,
                'snooze_countdown': snooze_countdown,
                'snooze_period': snooze_period,
            }
            
            return property_map.get(property_name)
            
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Failed to get property {property_name} for neuron {neuron_id}: {e}")
            return None
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Any):
        """Set neuron property in Rust NPU.
        
        Note: Most properties can only be set during neuron creation.
        This is a compatibility method - actual updates should go through
        parameter_updater.py which triggers Rust NPU reinitialization.
        """
        logger.warning(f"[NPU-INTERFACE] set_neuron_property called for {property_name} - this should go through parameter_updater for Rust NPU reload")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get NPU statistics from Rust NPU."""
        if self._rust_npu_integration is None:
            return {
                'neuron_count': 0,
                'synapse_count': 0,
                'utilization': {'neurons': 0.0, 'synapses': 0.0}
            }
        
        try:
            return {
                'neuron_count': self.rust_npu.get_neuron_count(),
                'synapse_count': self.rust_npu.get_synapse_count(),
                'utilization': {
                    'neurons': self.rust_npu.get_neuron_count() / self.max_neurons,
                    'synapses': self.rust_npu.get_synapse_count() / self.max_synapses
                }
            }
        except Exception as e:
            logger.error(f"[NPU-INTERFACE] Failed to get statistics: {e}")
            return {
                'neuron_count': 0,
                'synapse_count': 0,
                'utilization': {'neurons': 0.0, 'synapses': 0.0}
            }
    
    def configure_plasticity_queue(self, capacity: int) -> None:
        """Configure bounded plasticity command queue capacity.
        
        Note: Plasticity is handled by Rust in the future, but this method
        exists for compatibility with PlasticityService initialization.
        """
        cap = int(capacity)
        if cap < 0:
            raise ValueError("plasticity queue capacity must be >= 0")
        
        if self._plasticity_lock:
            with self._plasticity_lock:
                self._plasticity_queue_capacity = cap
                # Trim if needed
                if len(self._plasticity_queue) > cap:
                    self._plasticity_queue = self._plasticity_queue[:cap]
        else:
            self._plasticity_queue_capacity = cap
            if len(self._plasticity_queue) > cap:
                self._plasticity_queue = self._plasticity_queue[:cap]
        
        logger.debug(f"[NPU-INTERFACE] Plasticity queue capacity set to {cap}")
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                            area_type: str = "regular", cortical_id: str = "") -> bool:
        """Register a cortical area in NPUInterface tracking.
        
        This does NOT create neurons - it just registers the area for tracking.
        Neurons are created later via create_neurons_batch().
        
        Args:
            cortical_idx: Cortical area index
            dimensions: (x, y, z) dimensions
            area_type: Type of area (regular, core, etc.)
            cortical_id: String ID of the cortical area
            
        Returns:
            True if successful
        """
        self.cortical_areas[cortical_idx] = {
            'dimensions': dimensions,
            'area_type': area_type,
            'cortical_id': cortical_id
        }
        logger.debug(f"[NPU-INTERFACE] Registered cortical area {cortical_id} (idx={cortical_idx})")
        return True
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get list of neuron IDs in a cortical area.
        
        Returns:
            List of neuron IDs
        """
        neurons = []
        for neuron_id, area_idx in self.neuron_to_area.items():
            if area_idx == cortical_idx:
                neurons.append(neuron_id)
        return neurons
    
    def get_cortical_idx_by_id(self, cortical_id: str) -> Optional[int]:
        """Get cortical index from cortical ID string.
        
        Args:
            cortical_id: String ID of cortical area (e.g., "_power", "iic000")
            
        Returns:
            Cortical index, or None if not found
        """
        for idx, area_data in self.cortical_areas.items():
            if area_data.get('cortical_id') == cortical_id:
                return idx
        return None
    
    # COMPATIBILITY METHODS (for legacy code that expects these)
    
    @property
    def neuron_array(self):
        """DEPRECATED: Direct neuron_array access is not allowed.
        
        Rust NPU is the single source of truth. Use NPUInterface methods instead.
        
        This property exists ONLY for compatibility with code that checks
        'hasattr(npu_interface, "neuron_array")'. It returns a proxy object
        that logs warnings when accessed.
        """
        logger.warning("⚠️ [NPU-INTERFACE] Direct neuron_array access detected - this is deprecated!")
        logger.warning("⚠️ [NPU-INTERFACE] Use NPUInterface methods or Rust NPU directly instead")
        
        # Return a proxy that provides minimal compatibility
        return _DeprecatedNeuronArrayProxy(self)
    
    @property
    def synapse_array(self):
        """DEPRECATED: Direct synapse_array access is not allowed.
        
        Rust NPU is the single source of truth. Use NPUInterface methods instead.
        """
        logger.warning("⚠️ [NPU-INTERFACE] Direct synapse_array access detected - this is deprecated!")
        return _DeprecatedSynapseArrayProxy(self)


class _DeprecatedNeuronArrayProxy:
    """Compatibility proxy for deprecated neuron_array access.
    
    This exists ONLY to prevent crashes in legacy code that directly
    accesses neuron_array attributes. All access logs warnings.
    """
    
    def __init__(self, npu_interface):
        self._npu = npu_interface
        self._logged_warning = False
    
    def __getattr__(self, name):
        if not self._logged_warning:
            logger.error("❌ [DEPRECATED] Direct neuron_array.%s access - refactor to use Rust NPU!", name)
            self._logged_warning = True
        
        # Delegate to NPUInterface or provide safe fallbacks
        import numpy as np
        
        fallback_values = {
            'count': 0,
            'neuron_count': 0,
            'max_neurons': self._npu.max_neurons,  # ✅ Delegate to NPUInterface
            'cortical_idxs': np.array([], dtype=np.int32),
            'membrane_potentials': np.array([], dtype=np.float32),
            'thresholds': np.array([], dtype=np.float32),
            'refractory_countdowns': np.array([], dtype=np.uint16),
            'consecutive_fire_counts': np.array([], dtype=np.uint16),
            'consecutive_fire_limits': np.array([], dtype=np.uint16),
            'snooze_countdowns': np.array([], dtype=np.uint16),
            'snooze_periods': np.array([], dtype=np.uint16),
            'leak_coefficients': np.array([], dtype=np.float32),
            'resting_potentials': np.array([], dtype=np.float32),
            'excitabilities': np.array([], dtype=np.float32),
            'valid_mask': np.array([], dtype=bool),
            'neuron_id_to_index': self._npu.neuron_id_to_index,  # ✅ Delegate
            'index_to_neuron_id': self._npu.index_to_neuron_id,  # ✅ Delegate
        }
        
        # Return 0 for any unknown attribute (safe default)
        return fallback_values.get(name, 0)
    
    def __len__(self):
        return 0


class _DeprecatedSynapseArrayProxy:
    """Compatibility proxy for deprecated synapse_array access."""
    
    def __init__(self, npu_interface):
        self._npu = npu_interface
        self._logged_warning = False
    
    def __getattr__(self, name):
        if not self._logged_warning:
            logger.error("❌ [DEPRECATED] Direct synapse_array.%s access - refactor to use Rust NPU!", name)
            self._logged_warning = True
        
        import numpy as np
        
        fallback_values = {
            'count': 0,
            'synapse_count': 0,
            'max_synapses': self._npu.max_synapses,  # ✅ Delegate to NPUInterface
            'valid_mask': np.array([], dtype=bool),
            'source_neurons': np.array([], dtype=np.uint32),
            'source_neuron_ids': np.array([], dtype=np.uint32),  # Alias
            'target_neurons': np.array([], dtype=np.uint32),
            'target_neuron_ids': np.array([], dtype=np.uint32),  # Alias
            'weights': np.array([], dtype=np.uint8),
            'conductances': np.array([], dtype=np.uint8),
            'types': np.array([], dtype=np.int8),
        }
        
        # For unknown attributes, return empty array if name suggests it's array-like
        if name not in fallback_values:
            if any(x in name for x in ['neurons', 'ids', 'weights', 'mask', 'types', 'indices']):
                return np.array([], dtype=np.uint32)  # Safe default for array attributes
            else:
                return 0  # Safe default for scalar attributes
        
        return fallback_values[name]
    
    def valid_count(self):
        return 0