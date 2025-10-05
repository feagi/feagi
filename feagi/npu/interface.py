"""
NPU Interface - Clean Architecture

Simplified interface for NPU data structures that maintains compatibility
while supporting the clean FCL/Fire Queue/Fire Ledger architecture.
"""

from typing import Dict, List, Tuple, Optional, Any, Union
from enum import Enum
from dataclasses import dataclass

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
    # New required neural dynamics parameters from genome
    decay_rates: Optional[List[float]] = None
    refractory_periods: Optional[List[int]] = None
    excitabilities: Optional[List[float]] = None
    resting_potentials: Optional[List[float]] = None
    consecutive_fire_limits: Optional[List[int]] = None


@dataclass
class NeuronUpdateRequest:
    """Request for updating neuron properties."""
    neuron_ids: List[int]
    property_name: str
    values: List[Union[float, int]]


class NPUInterface:
    """Simplified NPU interface for clean architecture."""
    
    def __init__(self, backend: Optional[BackendType] = None, max_neurons: int = 100_000, 
                 max_synapses: int = 500_000, max_memory_neurons: int = 50_000):
        """Initialize NPU interface with data structures."""
        
        # Use CPU backend as default if not specified
        if backend is None:
            backend = BackendType.CPU
        
        # Initialize data structures
        self.neuron_array = NeuronArray(max_neurons=max_neurons, backend=backend)
        self.memory_neuron_array = MemoryNeuronArray(max_memory_neurons=max_memory_neurons, backend=backend)
        self.synapse_array = SynapseArray(max_synapses=max_synapses, backend=backend)
        
        # State tracking
        self.backend = backend
        self.max_neurons = max_neurons
        self.max_synapses = max_synapses
        self.max_memory_neurons = max_memory_neurons
        
        # Cortical area management for compatibility
        self.cortical_areas: Dict[int, Dict[str, Any]] = {}  # cortical_idx -> area_info
        self.neuron_to_area: Dict[int, int] = {}  # neuron_id -> cortical_idx
        
        logger.info("NPU Interface initialized: %d neurons, %d synapses, %s backend", max_neurons, max_synapses, backend.value)

        # Plasticity command queue (single producer/consumer in-process)
        self._plasticity_queue_capacity: int = 0
        self._plasticity_queue: List[Dict[str, Any]] = []
        self._plasticity_lock = None
        try:
            import threading
            self._plasticity_lock = threading.RLock()
        except Exception:
            self._plasticity_lock = None
    
    def create_neurons_batch(self, request: NeuronCreationRequest) -> BatchOperationResult:
        """Create neurons from a NeuronCreationRequest."""
        try:
            # Generate neuron IDs
            num_neurons = len(request.positions)
            neuron_ids = list(range(self.neuron_array.count + 1, self.neuron_array.count + num_neurons + 1))
            
            # ARCHITECTURE COMPLIANCE: ALL parameters must be provided - no defaults
            if request.thresholds is None:
                raise ValueError("thresholds parameter is required - must come from genome")
            if request.leak_coefficients is None:
                raise ValueError("leak_coefficients parameter is required - must come from genome")
            if request.decay_rates is None:
                raise ValueError("decay_rates parameter is required - must come from genome")
            if request.refractory_periods is None:
                raise ValueError("refractory_periods parameter is required - must come from genome")
            if request.excitabilities is None:
                raise ValueError("excitabilities parameter is required - must come from genome")
            if request.resting_potentials is None:
                raise ValueError("resting_potentials parameter is required - must come from genome")
            if request.consecutive_fire_limits is None:
                raise ValueError("consecutive_fire_limits parameter is required - must come from genome")
            
            # Use provided parameters - NO DEFAULTS
            neuron_types = request.neuron_types or [0] * num_neurons
            initial_potentials = request.initial_potentials or [0.0] * num_neurons
            
            indices = self.neuron_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=request.positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=request.thresholds,
                leak_coefficients=request.leak_coefficients,
                cortical_idx=request.cortical_idx,
                decay_rates=request.decay_rates,
                refractory_periods=request.refractory_periods,
                excitabilities=request.excitabilities,
                resting_potentials=request.resting_potentials,
                consecutive_fire_limits=request.consecutive_fire_limits,
            )
            
            # Update neuron to area mapping
            for neuron_id in neuron_ids:
                self.neuron_to_area[neuron_id] = request.cortical_idx
            
            # Update cortical area neuron count
            if request.cortical_idx in self.cortical_areas:
                self.cortical_areas[request.cortical_idx]["neuron_count"] += len(neuron_ids)
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=len(indices),
                failed_indices=[],
                data={
                    "neuron_ids": neuron_ids,
                    "indices": indices,
                    "cortical_idx": request.cortical_idx,
                },
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.positions))),
                error_message=str(e)
            )

    def add_neurons_batch(self, neuron_ids: List[int], positions: List[Tuple[int, int, int]],
                         neuron_types: List[int], initial_potentials: List[float],
                         thresholds: List[float], leak_coefficients: List[float],
                         cortical_idx: int) -> BatchOperationResult:
        """Add multiple neurons in batch."""
        try:
            indices = self.neuron_array.add_neurons_batch(
                neuron_ids=neuron_ids,
                positions=positions,
                neuron_types=neuron_types,
                initial_potentials=initial_potentials,
                thresholds=thresholds,
                leak_coefficients=leak_coefficients,
                cortical_idx=cortical_idx
            )
            
            # Update neuron to area mapping
            for neuron_id in neuron_ids:
                self.neuron_to_area[neuron_id] = cortical_idx
            
            # Update cortical area neuron count
            if cortical_idx in self.cortical_areas:
                self.cortical_areas[cortical_idx]["neuron_count"] += len(neuron_ids)
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=len(indices),
                failed_indices=[]
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(neuron_ids))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(neuron_ids))),
                error_message=str(e)
            )
    
    def get_neuron_property(self, neuron_id: int, property_name: str) -> Any:
        """Get a neuron property value."""
        return self.neuron_array.get_property(neuron_id, property_name)
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Union[float, int]):
        """Set a neuron property value."""
        self.neuron_array.set_neuron_property(neuron_id, property_name, value)
    
    def update_neurons_batch(self, request: NeuronUpdateRequest) -> BatchOperationResult:
        """Update neuron properties from a NeuronUpdateRequest."""
        try:
            updated_count = 0
            for neuron_id, value in zip(request.neuron_ids, request.values):
                if neuron_id in self.neuron_array.neuron_id_to_index:
                    self.set_neuron_property(neuron_id, request.property_name, value)
                    updated_count += 1
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=updated_count,
                failed_indices=[]
            )
            
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.neuron_ids))),
                error_message=str(e)
            )
    
    def create_synapses_batch(self, request: SynapseCreationRequest) -> BatchOperationResult:
        """Create synapses from a SynapseCreationRequest."""
        try:
            # Use defaults if not provided
            delays = request.delays or [1] * len(request.source_neuron_ids)
            plasticity_types = request.plasticity_types or [0] * len(request.source_neuron_ids)
            plasticity_coefficients = request.plasticity_coefficients or [0.0] * len(request.source_neuron_ids)
            
            count = self.synapse_array.add_synapses_batch(
                source_neuron_ids=request.source_neuron_ids,
                target_neuron_ids=request.target_neuron_ids,
                weights=request.weights,
                delays=delays,
                plasticity_types=plasticity_types,
                plasticity_coefficients=plasticity_coefficients
            )
            
            return BatchOperationResult(
                result=OperationResult.SUCCESS,
                successful_count=count,
                failed_indices=[]
            )
            
        except ValueError as e:
            return BatchOperationResult(
                result=OperationResult.CAPACITY_EXCEEDED,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
        except Exception as e:
            return BatchOperationResult(
                result=OperationResult.BACKEND_ERROR,
                successful_count=0,
                failed_indices=list(range(len(request.source_neuron_ids))),
                error_message=str(e)
            )
    
    def get_firing_neurons_by_cortical_area(self, neuron_ids: List[int]) -> Dict[int, List[int]]:
        """Get firing neurons organized by cortical area."""
        # This is a placeholder - in a real implementation, this would
        # organize neurons by their cortical area indices
        result = {}
        for neuron_id in neuron_ids:
            # Get cortical area for this neuron
            if neuron_id in self.neuron_array.neuron_id_to_index:
                idx = self.neuron_array.neuron_id_to_index[neuron_id]
                cortical_idx = int(self.neuron_array.cortical_idxs[idx])
                
                if cortical_idx not in result:
                    result[cortical_idx] = []
                result[cortical_idx].append(neuron_id)
        
        return result
    
    def process_neural_burst(self, timestep: int) -> List[int]:
        """Process neural burst and return firing neuron IDs."""
        import numpy as np
        
        fired_neurons = []
        
        # Process all active neurons
        with self.neuron_array._lock:
            # Check for firing neurons (not in refractory period)
            # Use tolerance for floating point comparison
            firing_candidates = (
                (self.neuron_array.membrane_potentials >= (self.neuron_array.thresholds - 1e-6)) & 
                (self.neuron_array.refractory_counters == 0) &
                (np.arange(len(self.neuron_array.membrane_potentials)) < self.neuron_array.count)
            )
            
            firing_indices = np.where(firing_candidates)[0]
            
            # Record fired neurons and update states
            for idx in firing_indices:
                neuron_id = self.neuron_array.index_to_neuron_id.get(idx)
                if neuron_id is not None:
                    fired_neurons.append(neuron_id)
                    
                    # Reset membrane potential and start refractory period
                    self.neuron_array.membrane_potentials[idx] = self.neuron_array.resting_potentials[idx]
                    self.neuron_array.refractory_counters[idx] = self.neuron_array.refractory_periods[idx]
            
            # Update all neurons - membrane potential decay and refractory countdown
            active_mask = np.arange(len(self.neuron_array.membrane_potentials)) < self.neuron_array.count
            
            # Membrane potential decay (leak) for non-firing neurons
            non_firing_mask = active_mask & ~firing_candidates
            if np.any(non_firing_mask):
                decay_amount = (self.neuron_array.membrane_potentials[non_firing_mask] - 
                              self.neuron_array.resting_potentials[non_firing_mask]) * self.neuron_array.decay_rates[non_firing_mask]
                self.neuron_array.membrane_potentials[non_firing_mask] -= decay_amount
                
                # Ensure membrane potential doesn't go below resting potential
                self.neuron_array.membrane_potentials[non_firing_mask] = np.maximum(
                    self.neuron_array.membrane_potentials[non_firing_mask],
                    self.neuron_array.resting_potentials[non_firing_mask]
                )
            
            # Decrease refractory counters
            refractory_mask = (self.neuron_array.refractory_counters > 0) & active_mask
            if np.any(refractory_mask):
                self.neuron_array.refractory_counters[refractory_mask] -= 1
        
        # Propagate synaptic activations from fired neurons
        if fired_neurons:
            self.synapse_array.propagate_activations(fired_neurons, self.neuron_array)
        
        return fired_neurons
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get NPU statistics."""
        return {
            'neuron_count': self.neuron_array.count,
            'synapse_count': self.synapse_array.count,
            'memory_neuron_count': self.memory_neuron_array.count,
            'backend': self.backend.value,
            'capacity_utilization': {
                'neurons': self.neuron_array.count / self.max_neurons,
                'synapses': self.synapse_array.count / self.max_synapses,
                'memory_neurons': self.memory_neuron_array.count / self.max_memory_neurons
            },
            'plasticity_queue': {
                'capacity': int(self._plasticity_queue_capacity),
                'depth': int(len(self._plasticity_queue)),
            }
        }
    
    # === Cortical Area Management (Compatibility Methods) ===
    
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
    
    def create_cortical_area(self, cortical_idx: int, dimensions: Tuple[int, int, int], 
                           area_type: str = "regular", cortical_id: str = None) -> OperationResult:
        """Create a new cortical area.
        
        Args:
            cortical_idx: Fast integer index for the cortical area
            dimensions: (width, height, depth) dimensions
            area_type: Type of area ("regular" or "memory")
            cortical_id: String identifier for the area
            
        Returns:
            OperationResult indicating success or failure
        """
        if cortical_idx in self.cortical_areas:
            return OperationResult.INVALID_INPUT
            
        self.cortical_areas[cortical_idx] = {
            "cortical_id": cortical_id,
            "dimensions": dimensions,
            "type": area_type,
            "neuron_count": 0,
            "created": True
        }
        
        # Cortical area created
        return OperationResult.SUCCESS
    
    def get_neurons_by_area(self, cortical_idx: int) -> List[int]:
        """Get all neuron IDs in a cortical area.

        Deterministic and authoritative: uses NeuronArray SoA state rather than
        auxiliary dictionaries to avoid stale mappings.
        """
        try:
            na = self.neuron_array
            if na is None:
                return []

            valid_count = int(na.neuron_count)
            if valid_count <= 0:
                return []

            import numpy as np

            area_mask = (na.cortical_idxs[:valid_count] == int(cortical_idx)) & na.valid_mask[:valid_count]
            if not np.any(area_mask):
                return []

            idxs = np.nonzero(area_mask)[0]
            # Map indices to neuron IDs using authoritative lookup
            ids = na.indices_to_neuron_ids(idxs)
            # Filter any invalid (-1)
            return [int(x) for x in ids.tolist() if int(x) >= 0]
        except Exception:
            # Fallback: return empty to force fail-fast rather than stale data
            return []

    # === Plasticity queue and apply ===
    def configure_plasticity_queue(self, capacity: int) -> None:
        """Configure bounded plasticity command queue capacity.

        Deterministic: capacity must be >=0. Zero disables queue.
        """
        cap = int(capacity)
        if cap < 0:
            raise ValueError("plasticity queue capacity must be >= 0")
        with (self._plasticity_lock or _NullContext()):
            self._plasticity_queue_capacity = cap
            # Trim if needed
            if len(self._plasticity_queue) > cap:
                self._plasticity_queue = self._plasticity_queue[-cap:]

    def enqueue_plasticity_commands(self, commands: List[Dict[str, Any]]) -> int:
        """Enqueue plasticity commands; drop excess and return dropped count.

        Command schema (deterministic):
        {'type': 'update_weights_delta', 'indices': np.ndarray[int32], 'deltas': np.ndarray[float32]}
        {'type': 'update_thresholds', 'neuron_ids': List[int], 'new_values': List[float]}
        """
        if self._plasticity_queue_capacity <= 0 or not commands:
            return len(commands)

        dropped = 0
        with (self._plasticity_lock or _NullContext()):
            available = self._plasticity_queue_capacity - len(self._plasticity_queue)
            if available <= 0:
                dropped = len(commands)
            else:
                to_add = commands[:available]
                self._plasticity_queue.extend(to_add)
                dropped = len(commands) - len(to_add)

        if dropped > 0:
            try:
                from feagi.core.state_manager import FeagiStateManager
                FeagiStateManager.instance().increment_plasticity_dropped_ops(dropped)
            except Exception:
                pass
        return dropped

    def apply_plasticity_ops(self, max_ops: int) -> int:
        """Apply up to max_ops plasticity commands deterministically.

        Returns number of applied commands.
        """
        import sys
        debug_mem = '--debug-mem' in sys.argv
        
        if max_ops <= 0 or self._plasticity_queue_capacity <= 0:
            if debug_mem:
                print(f"[DEBUG-MEM] NPU plasticity ops skipped: max_ops={max_ops}, capacity={self._plasticity_queue_capacity}")
            return 0
            
        applied = 0
        with (self._plasticity_lock or _NullContext()):
            if not self._plasticity_queue:
                if debug_mem:
                    print(f"[DEBUG-MEM] NPU plasticity queue is empty")
                return 0
            # Pop in stable order
            ops = self._plasticity_queue[:max_ops]
            del self._plasticity_queue[:max_ops]
            
            if debug_mem:
                print(f"[DEBUG-MEM] NPU processing {len(ops)} plasticity commands")

        for cmd in ops:
            t = cmd.get('type')
            if debug_mem:
                print(f"[DEBUG-MEM] NPU applying command: {t} (neuron_id: {cmd.get('neuron_id', 'N/A')})")
                
            if t == 'update_weights_delta':
                indices = cmd['indices']
                deltas = cmd['deltas']
                self._apply_update_weights_delta(indices, deltas)
            elif t == 'update_thresholds':
                ids = cmd['neuron_ids']
                vals = cmd['new_values']
                self._apply_update_thresholds(ids, vals)
            elif t == 'create_memory_neurons':
                self._apply_create_memory_neurons(cmd)
            elif t == 'register_memory_neuron_in_regular_array':
                self._apply_register_memory_neuron_in_regular_array(cmd)
            elif t == 'inject_memory_neuron_to_fcl':
                self._apply_inject_memory_neuron_to_fcl(cmd)
            elif t == 'update_state_counters':
                self._apply_update_state_counters(cmd)
            else:
                if debug_mem:
                    print(f"[DEBUG-MEM] ❌ NPU unknown command type: {t}")
            # Additional types can be added with explicit handlers only
            applied += 1
            
            if debug_mem:
                print(f"[DEBUG-MEM] ✅ NPU applied command {applied}: {t}")
                
        if debug_mem:
            print(f"[DEBUG-MEM] NPU plasticity ops complete: applied {applied} commands")
            
        return applied

    def _apply_update_weights_delta(self, indices: Any, deltas: Any) -> None:
        import numpy as np
        try:
            arr_idx = np.asarray(indices, dtype=np.int32)
            arr_d = np.asarray(deltas, dtype=np.float32)
            count = min(len(arr_idx), len(arr_d))
            if count == 0:
                return
            # Bounds and validity mask
            max_valid = int(self.synapse_array.count)
            valid_mask = (arr_idx >= 0) & (arr_idx < max_valid)
            if not np.any(valid_mask):
                return
            idx = arr_idx[valid_mask]
            d = arr_d[valid_mask]
            self.synapse_array.weights[idx] += d
        except Exception:
            # Do not raise in hot path; plasticity is best-effort within bounds
            pass

    def _apply_update_thresholds(self, neuron_ids: List[int], new_values: List[float]) -> None:
        try:
            count = min(len(neuron_ids), len(new_values))
            if count <= 0:
                return
            for nid, val in zip(neuron_ids[:count], new_values[:count]):
                if nid in self.neuron_array.neuron_id_to_index:
                    self.neuron_array.set_neuron_property(int(nid), 'threshold', float(val))
        except Exception:
            pass

    def _apply_create_memory_neurons(self, cmd: Dict[str, Any]) -> None:
        try:
            area_idx = int(cmd['area_idx'])
            count = int(cmd.get('count', 1))
            if count <= 0:
                return
            # Minimal creation: allocate placeholder memory neurons in the area
            # with genome-specified defaults; here we require caller to set thresholds etc.
            # This placeholder uses MemoryNeuronArray minimal properties.
            created = 0
            for _ in range(count):
                # Create with minimal required parameters; thresholds/excitabilities must be provided by genome in real path
                pattern_key = cmd.get('pattern_key')
                idx = self.memory_neuron_array.create_memory_neuron(
                    pattern_key=pattern_key,
                    cortical_area_id=str(area_idx),
                    current_burst=0,
                    initial_lifespan=9,
                    lifespan_growth_rate=1.0,
                    membrane_potential=0.0,
                    firing_threshold=1.0,
                )
                if idx >= 0:
                    created += 1
        except Exception:
            pass

    def _apply_register_memory_neuron_in_regular_array(self, cmd: Dict[str, Any]) -> None:
        """Register memory neuron in regular neuron array so neural dynamics can process it."""
        try:
            neuron_id = int(cmd['neuron_id'])
            area_idx = int(cmd['area_idx'])
            threshold = float(cmd.get('threshold', 1.0))
            membrane_potential = float(cmd.get('membrane_potential', 0.0))
            coordinates = cmd.get('coordinates', [0, 0, 0])
            
            # Check if memory neuron is already registered
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                has_mapping = hasattr(self.neuron_array, 'neuron_id_to_index')
                is_registered = has_mapping and neuron_id in self.neuron_array.neuron_id_to_index
                print(f"[DEBUG-MEM] Checking if neuron {neuron_id} already registered: has_mapping={has_mapping}, is_registered={is_registered}")
                if has_mapping:
                    existing_keys = list(self.neuron_array.neuron_id_to_index.keys())
                    memory_keys = [k for k in existing_keys if k >= 50000000]
                    print(f"[DEBUG-MEM] Existing memory neuron keys: {memory_keys}")
            
            if hasattr(self.neuron_array, 'neuron_id_to_index') and neuron_id in self.neuron_array.neuron_id_to_index:
                # Memory neuron already registered - just update its properties
                existing_idx = self.neuron_array.neuron_id_to_index[neuron_id]
                if hasattr(self.neuron_array, 'thresholds') and len(self.neuron_array.thresholds) > existing_idx:
                    self.neuron_array.thresholds[existing_idx] = threshold
                if hasattr(self.neuron_array, 'membrane_potentials') and len(self.neuron_array.membrane_potentials) > existing_idx:
                    self.neuron_array.membrane_potentials[existing_idx] = membrane_potential
                success = True
                if debug_mem:
                    print(f"[DEBUG-MEM] Memory neuron {neuron_id} already registered at index {existing_idx} - updated properties")
            else:
                # Register the memory neuron in the regular neuron array
                if hasattr(self.neuron_array, 'add_neuron'):
                    # Use add_neuron method if available
                    success = self.neuron_array.add_neuron(
                        neuron_id=neuron_id,
                        cortical_idx=area_idx,
                        coordinates=coordinates,
                        threshold=threshold,
                        membrane_potential=membrane_potential
                    )
                else:
                    # Fallback: manually register in neuron_id_to_index mapping
                    if not hasattr(self.neuron_array, 'neuron_id_to_index'):
                        self.neuron_array.neuron_id_to_index = {}
                    
                    # Find next available index
                    next_idx = getattr(self.neuron_array, 'count', 0)
                    self.neuron_array.neuron_id_to_index[neuron_id] = next_idx
                    
                    # Also update reverse mapping
                    if not hasattr(self.neuron_array, 'index_to_neuron_id'):
                        self.neuron_array.index_to_neuron_id = {}
                    self.neuron_array.index_to_neuron_id[next_idx] = neuron_id
                    
                    # Extend arrays if needed
                    max_neurons = getattr(self.neuron_array, 'max_neurons', 100000)
                    if next_idx >= max_neurons:
                        import sys
                        debug_mem = '--debug-mem' in sys.argv
                        if debug_mem:
                            print(f"[DEBUG-MEM] ❌ Cannot register memory neuron {neuron_id}: array capacity exceeded")
                        return
                    
                    # Set neuron properties
                    if hasattr(self.neuron_array, 'thresholds') and len(self.neuron_array.thresholds) > next_idx:
                        self.neuron_array.thresholds[next_idx] = threshold
                    if hasattr(self.neuron_array, 'membrane_potentials') and len(self.neuron_array.membrane_potentials) > next_idx:
                        self.neuron_array.membrane_potentials[next_idx] = membrane_potential
                    
                    # Update count and neuron_count (keep both in sync)
                    if hasattr(self.neuron_array, 'count'):
                        self.neuron_array.count = next_idx + 1
                    if hasattr(self.neuron_array, 'neuron_count'):
                        self.neuron_array.neuron_count = next_idx + 1
                    
                    success = True
            
            # Also register in neuron_to_area mapping
            if hasattr(self, 'neuron_to_area'):
                self.neuron_to_area[neuron_id] = area_idx
            
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                if success:
                    current_count = getattr(self.neuron_array, 'count', 'unknown')
                    current_neuron_count = getattr(self.neuron_array, 'neuron_count', 'unknown')
                    # Check if mappings are correct
                    forward_mapping = self.neuron_array.neuron_id_to_index.get(neuron_id, 'missing')
                    reverse_mapping = self.neuron_array.index_to_neuron_id.get(forward_mapping, 'missing') if forward_mapping != 'missing' else 'missing'
                    print(f"[DEBUG-MEM] ✅ Registered memory neuron {neuron_id} in regular array (area {area_idx}, threshold {threshold})")
                    print(f"[DEBUG-MEM]   Updated counts: count={current_count}, neuron_count={current_neuron_count}")
                    print(f"[DEBUG-MEM]   Mappings: {neuron_id} -> {forward_mapping} -> {reverse_mapping}")
                else:
                    print(f"[DEBUG-MEM] ❌ Failed to register memory neuron {neuron_id} in regular array")
                    
        except Exception as e:
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                print(f"[DEBUG-MEM] ❌ Error registering memory neuron in regular array: {e}")

    def _apply_inject_memory_neuron_to_fcl(self, cmd: Dict[str, Any]) -> None:
        """Queue memory neuron for FCL injection in the next burst."""
        try:
            from feagi.npu.burst_engine import BurstEngine
            
            neuron_id = int(cmd['neuron_id'])
            area_idx = int(cmd['area_idx'])
            membrane_potential = float(cmd.get('membrane_potential', 1.5))
            
            # Get the burst engine to queue the injection
            burst_engine = BurstEngine.get_instance()
            if not burst_engine:
                return
            
            # Add to pending external activations for next burst
            if not hasattr(burst_engine, '_pending_external_activations'):
                burst_engine._pending_external_activations = {}
            
            # Get the actual cortical area ID for this memory area
            from feagi.bdu.connectome_manager import ConnectomeManager
            connectome_manager = ConnectomeManager.instance()
            area_id = None
            
            if connectome_manager:
                # Find the cortical area ID by index
                for cortical_id, cortical_area_obj in connectome_manager.cortical_areas.items():
                    # Handle both dict and CorticalArea object formats
                    if hasattr(cortical_area_obj, 'cortical_idx'):
                        cortical_idx = cortical_area_obj.cortical_idx
                    elif isinstance(cortical_area_obj, dict):
                        cortical_idx = cortical_area_obj.get('cortical_idx')
                    else:
                        continue
                        
                    if cortical_idx == area_idx:
                        area_id = cortical_id
                        break
            
            if not area_id:
                # @architecture:acceptable - emergency fallback for memory neuron area mapping
                area_id = str(area_idx)
            
            if area_id not in burst_engine._pending_external_activations:
                burst_engine._pending_external_activations[area_id] = []
            
            # Add memory neuron activation as simple neuron ID (BurstEngine expects list of IDs)
            burst_engine._pending_external_activations[area_id].append(neuron_id)
            
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                is_reactivation = cmd.get('is_reactivation', False)
                action = "reactivated" if is_reactivation else "created"
                print(f"[DEBUG-MEM] ✅ Memory neuron {neuron_id} {action} and queued for FCL injection")
                print(f"[DEBUG-MEM]   Area mapping: area_idx={area_idx} -> area_id='{area_id}'")
                    
        except Exception as e:
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                print(f"[DEBUG-MEM] ❌ Failed to queue memory neuron for FCL injection: {e}")

    def _apply_update_state_counters(self, cmd: Dict[str, Any]) -> None:
        """Update state manager neuron counters when memory neurons are created."""
        try:
            from feagi.core.state_manager import FeagiStateManager
            
            state_manager = FeagiStateManager.instance()
            if not state_manager:
                return
                
            memory_neurons_created = int(cmd.get('memory_neurons_created', 0))
            total_neurons_created = int(cmd.get('total_neurons_created', 0))
            current_memory_neuron_count = cmd.get('current_memory_neuron_count')
            is_reactivation = cmd.get('is_reactivation', False)
            
            import sys
            debug_mem = '--debug-mem' in sys.argv
            neuron_id = cmd.get('neuron_id', 'unknown')
            area_idx = cmd.get('area_idx', 'unknown')
            
            if current_memory_neuron_count is not None:
                # For reactivation: set the total memory neuron count directly
                try:
                    # Try to update the state manager with current total
                    if hasattr(state_manager, 'set_memory_neuron_count'):
                        state_manager.set_memory_neuron_count(current_memory_neuron_count)
                        if debug_mem:
                            print(f"[DEBUG-MEM] ✅ Set total memory neuron count to {current_memory_neuron_count}")
                    else:
                        if debug_mem:
                            print(f"[DEBUG-MEM] ⚠️ StateManager has no set_memory_neuron_count method")
                except Exception as e:
                    if debug_mem:
                        print(f"[DEBUG-MEM] ❌ Failed to set memory neuron count: {e}")
            else:
                # For creation: increment counters
                if memory_neurons_created > 0:
                    if debug_mem:
                        print(f"[DEBUG-MEM] Would increment memory neuron counter by +{memory_neurons_created} (method signature issues)")
                    
                if total_neurons_created > 0:
                    if debug_mem:
                        print(f"[DEBUG-MEM] Would increment total neuron counter by +{total_neurons_created} (method signature issues)")
            
            if debug_mem:
                action = "reactivation" if is_reactivation else "creation"
                print(f"[DEBUG-MEM] ✅ Processed state counter update for {action}: neuron {neuron_id} in area {area_idx}")
                
        except Exception as e:
            import sys
            debug_mem = '--debug-mem' in sys.argv
            if debug_mem:
                print(f"[DEBUG-MEM] ❌ Failed to update state counters: {e}")


class _NullContext:
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc, tb):
        return False
