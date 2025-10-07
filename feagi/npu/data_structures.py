"""
NPU Data Structures - Clean Architecture

Core data structures for FEAGI's Neural Processing Unit.
Designed for clean separation of concerns and Rust migration readiness.

Key Features:
- Single source of truth for neural data
- Structure of Arrays (SoA) for SIMD/GPU optimization  
- Rust FFI-compatible data types and layouts
- Clean interfaces for two-crate architecture
- High-performance batch operations
"""

from typing import Dict, List, Optional, Tuple, Any, Union
from dataclasses import dataclass
from enum import Enum
import numpy as np
import threading

from feagi.utils.logger import setup_logger
from feagi.config.toml_loader import load_feagi_config

logger = setup_logger(__name__)


@dataclass
class MemoryPatternKey:
    """Key for memory pattern identification and caching."""
    pattern_data: Tuple[bytes, ...]  # Serialized bitmap sequence
    temporal_depth: int  # Number of timesteps
    source_cortical_areas: Tuple[str, ...]  # Upstream areas
    
    def __hash__(self) -> int:
        return hash((self.pattern_data, self.temporal_depth, self.source_cortical_areas))


class BackendType(Enum):
    """Computation backend types."""
    CPU = "cpu"
    CUDA = "cuda"
    METAL = "metal"
    WGPU = "wgpu"
    RUST = "rust"


@dataclass
class SIMDConfig:
    """SIMD configuration for backend-optimized operations."""
    backend: BackendType
    vector_width: int
    alignment_bytes: int
    optimal_batch_size: int
    max_batch_size: int


class SIMDDetector:
    """Detects optimal SIMD configuration for the current backend."""
    
    @staticmethod
    def detect_simd_config(backend: BackendType) -> SIMDConfig:
        """Detect optimal SIMD configuration for the given backend."""
        if backend == BackendType.CPU:
            try:
                import platform
                arch = platform.machine().lower()
                
                if 'arm' in arch or 'aarch64' in arch:
                    # ARM NEON: 128-bit vectors
                    return SIMDConfig(
                        backend=backend,
                        vector_width=4,
                        alignment_bytes=16,
                        optimal_batch_size=256,
                        max_batch_size=2048
                    )
                else:
                    # x86 AVX2: 256-bit vectors
                    return SIMDConfig(
                        backend=backend,
                        vector_width=8,
                        alignment_bytes=32,
                        optimal_batch_size=512,
                        max_batch_size=4096
                    )
            except Exception:
                # Fallback
                return SIMDConfig(
                    backend=backend,
                    vector_width=4,
                    alignment_bytes=16,
                    optimal_batch_size=256,
                    max_batch_size=1024
                )
                
        elif backend == BackendType.CUDA:
            return SIMDConfig(
                backend=backend,
                vector_width=32,
                alignment_bytes=128,
                optimal_batch_size=1024,
                max_batch_size=8192
            )
            
        elif backend == BackendType.RUST:
            return SIMDConfig(
                backend=backend,
                vector_width=8,
                alignment_bytes=32,
                optimal_batch_size=512,
                max_batch_size=2048
            )
            
        else:
            return SIMDConfig(
                backend=backend,
                vector_width=8,
                alignment_bytes=32,
                optimal_batch_size=512,
                max_batch_size=2048
            )


def _load_npu_config() -> Dict[str, Any]:
    """Load NPU configuration from FEAGI configuration file."""
    try:
        config = load_feagi_config()
        
        connectome_config = config.get("connectome", {})
        neural_config = config.get("neural", {})
        resources_config = config.get("resources", {})
        
        min_neuron_space = connectome_config.get("min_neuron_space", 100_000)
        min_synapse_space = connectome_config.get("min_synapse_space", 500_000)
        min_memory_neuron_space = connectome_config.get("min_memory_neuron_space", 50_000)
        
        return {
            "max_neurons": min_neuron_space,
            "max_synapses": min_synapse_space, 
            "max_memory_neurons": min_memory_neuron_space,
            "batch_size": neural_config.get("batch_size", 1000),
            "buffer_multiplier": connectome_config.get("buffer_multiplier", 1.5),
            "use_gpu": resources_config.get("use_gpu", True),
            "gpu_memory_fraction": resources_config.get("gpu_memory_fraction", 0.8)
        }
    except Exception as e:
        logger.warning(f"Failed to load NPU config from TOML: {e}, using defaults")
        return {
            "max_neurons": 100_000,
            "max_synapses": 500_000,
            "max_memory_neurons": 50_000,
            "batch_size": 1000,
            "buffer_multiplier": 1.5,
            "use_gpu": True,
            "gpu_memory_fraction": 0.8
        }


class NeuronArray:
    """Single source of truth for regular neurons - Clean Architecture.
    
    Simplified version optimized for clean NPU architecture and Rust migration.
    Maintains compatibility with existing FEAGI interfaces.
    """
    
    def __init__(self, max_neurons: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize neuron array."""
        if max_neurons is None:
            npu_config = _load_npu_config()
            max_neurons = npu_config["max_neurons"]
            
        self.max_neurons = max_neurons
        self.backend = backend
        self.neuron_count = 0
        self.count = 0  # Alias for compatibility
        
        # Detect SIMD configuration
        self.simd_config = SIMDDetector.detect_simd_config(backend)
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Core neuron properties (SoA format) - NO HARDCODED VALUES, ALL FROM GENOME
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.zeros(max_neurons, dtype=np.float32)  # MUST be set from genome
        self.decay_rates = np.zeros(max_neurons, dtype=np.float32)  # MUST be set from genome
        self.leak_coefficients = np.zeros(max_neurons, dtype=np.float32)  # MUST be set from genome
        self.resting_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.neuron_types = np.zeros(max_neurons, dtype=np.int32)
        
        # Spatial properties
        self.positions_x = np.zeros(max_neurons, dtype=np.int32)
        self.positions_y = np.zeros(max_neurons, dtype=np.int32)
        self.positions_z = np.zeros(max_neurons, dtype=np.int32)
        self.coordinates_x = np.zeros(max_neurons, dtype=np.uint32)
        self.coordinates_y = np.zeros(max_neurons, dtype=np.uint32)
        self.coordinates_z = np.zeros(max_neurons, dtype=np.uint32)
        
        # Firing properties - NO HARDCODED VALUES, ALL FROM GENOME
        self.refractory_periods = np.zeros(max_neurons, dtype=np.uint8)  # MUST be set from genome
        self.refractory_counters = np.zeros(max_neurons, dtype=np.uint8)
        
        # Consecutive fire tracking (RUST-COMPATIBLE: primitive arrays) - FROM GENOME ONLY
        self.consecutive_fire_counts = np.zeros(max_neurons, dtype=np.uint16)
        self.consecutive_fire_limits = np.zeros(max_neurons, dtype=np.uint16)  # MUST be set from genome
        
        # Snooze period tracking (RUST-COMPATIBLE: primitive arrays) - FROM GENOME ONLY
        # Snooze = rest period after consecutive fires, measured in bursts
        self.snooze_periods = np.zeros(max_neurons, dtype=np.uint16)  # MUST be set from genome
        self.snooze_countdowns = np.zeros(max_neurons, dtype=np.uint16)
        
        # Cortical area mapping
        self.cortical_idxs = np.zeros(max_neurons, dtype=np.uint16)
        
        # Neuron ID mapping (NPU owns this)
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        self._next_neuron_id = 1
        
        # Valid neuron tracking
        self.valid_mask = np.zeros(max_neurons, dtype=bool)
        
        # Compatibility fields
        self.next_index = 0
        self.excitabilities = np.zeros(max_neurons, dtype=np.float32)  # MUST be set from genome
        
        logger.info("NeuronArray initialized: %d max neurons, %s backend", max_neurons, backend.value)
    
    def add_neurons_batch(self, neuron_ids: List[int], positions: List[Tuple[int, int, int]],
                         neuron_types: List[int], initial_potentials: List[float],
                         thresholds: List[float], leak_coefficients: List[float],
                         cortical_idx: int, 
                         decay_rates: Optional[List[float]] = None,
                         refractory_periods: Optional[List[int]] = None,
                         excitabilities: Optional[List[float]] = None,
                         resting_potentials: Optional[List[float]] = None,
                         consecutive_fire_limits: Optional[List[int]] = None,
                         snooze_periods: Optional[List[int]] = None) -> List[int]:
        """Add multiple neurons in batch."""
        count = len(neuron_ids)
        if self.count + count > self.max_neurons:
            raise ValueError(f"Cannot add {count} neurons: would exceed capacity")
        
        start_idx = self.count
        indices = list(range(start_idx, start_idx + count))
        end_idx = start_idx + count
        
        # Update arrays - ALL VALUES MUST COME FROM GENOME
        self.membrane_potentials[start_idx:end_idx] = np.array(initial_potentials, dtype=np.float32)
        self.thresholds[start_idx:end_idx] = np.array(thresholds, dtype=np.float32)
        self.leak_coefficients[start_idx:end_idx] = np.array(leak_coefficients, dtype=np.float32)
        self.neuron_types[start_idx:end_idx] = np.array(neuron_types, dtype=np.int32)
        self.cortical_idxs[start_idx:end_idx] = cortical_idx
        
        # Set additional neural dynamics parameters from genome
        if decay_rates is not None:
            self.decay_rates[start_idx:end_idx] = np.array(decay_rates, dtype=np.float32)
        else:
            # Set default decay rates for backward compatibility
            self.decay_rates[start_idx:end_idx] = 0.1
            
        if refractory_periods is not None:
            self.refractory_periods[start_idx:end_idx] = np.array(refractory_periods, dtype=np.uint8)
        else:
            # Set default refractory periods for backward compatibility
            self.refractory_periods[start_idx:end_idx] = 1
            
        if excitabilities is not None:
            self.excitabilities[start_idx:end_idx] = np.array(excitabilities, dtype=np.float32)
        else:
            # Set default excitabilities for backward compatibility
            self.excitabilities[start_idx:end_idx] = 1.0
            
        if resting_potentials is not None:
            self.resting_potentials[start_idx:end_idx] = np.array(resting_potentials, dtype=np.float32)
        else:
            # Set default resting potentials for backward compatibility
            self.resting_potentials[start_idx:end_idx] = 0.0
        
        # Set positions
        positions_array = np.array(positions, dtype=np.int32)
        self.positions_x[start_idx:end_idx] = positions_array[:, 0]
        self.positions_y[start_idx:end_idx] = positions_array[:, 1] 
        self.positions_z[start_idx:end_idx] = positions_array[:, 2]
        self.coordinates_x[start_idx:end_idx] = positions_array[:, 0]
        self.coordinates_y[start_idx:end_idx] = positions_array[:, 1]
        self.coordinates_z[start_idx:end_idx] = positions_array[:, 2]
        
        # Mark as valid
        self.valid_mask[start_idx:end_idx] = True
        
        # Set consecutive fire limits - MUST come from genome
        if consecutive_fire_limits is not None:
            self.consecutive_fire_limits[start_idx:end_idx] = np.array(consecutive_fire_limits, dtype=np.uint16)
        else:
            # Set default consecutive fire limits for backward compatibility
            self.consecutive_fire_limits[start_idx:end_idx] = 5
        # Note: consecutive_fire_counts remain 0 (initialized by default)
        
        # Set snooze periods - MUST come from genome (nx-snooze-f gene)
        if snooze_periods is not None:
            # Convert float to uint16, ensuring only positive integers including 0
            snooze_array = np.array(snooze_periods, dtype=np.float32)
            snooze_array = np.maximum(0, np.round(snooze_array)).astype(np.uint16)
            self.snooze_periods[start_idx:end_idx] = snooze_array
        else:
            # Set default snooze periods for backward compatibility
            self.snooze_periods[start_idx:end_idx] = 0
        # Note: snooze_countdowns remain 0 (initialized by default)
        
        # Update ID mappings
        for i, neuron_id in enumerate(neuron_ids):
            idx = start_idx + i
            self.neuron_id_to_index[neuron_id] = idx
            self.index_to_neuron_id[idx] = neuron_id
        
        # Update counters
        self.count += count
        self.neuron_count += count
        self.next_index = end_idx
        self._next_neuron_id = max(self._next_neuron_id, max(neuron_ids) + 1)
        
        return indices
    
    def get_property(self, neuron_id: int, property_name: str) -> Optional[Union[float, int]]:
        """Get a neuron property value."""
        if neuron_id not in self.neuron_id_to_index:
            return None
            
        idx = self.neuron_id_to_index[neuron_id]
        
        if property_name == "membrane_potential":
            return float(self.membrane_potentials[idx])
        elif property_name == "threshold":
            return float(self.thresholds[idx])
        elif property_name == "leak_coefficient":
            return float(self.leak_coefficients[idx])
        elif property_name == "position":
            return (int(self.positions_x[idx]), int(self.positions_y[idx]), int(self.positions_z[idx]))
        else:
            return None
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Union[float, int]) -> None:
        """Set a property value for a specific neuron."""
        if neuron_id not in self.neuron_id_to_index:
            return
            
        idx = self.neuron_id_to_index[neuron_id]
        
        with self._lock:
            if property_name == "membrane_potential":
                self.membrane_potentials[idx] = float(value)
            elif property_name == "threshold":
                self.thresholds[idx] = float(value)
            elif property_name == "consecutive_fire_limit":
                self.consecutive_fire_limits[idx] = int(value)
            elif property_name == "leak_coefficient":
                self.leak_coefficients[idx] = float(value)
    
    def indices_to_neuron_ids(self, indices: np.ndarray, *, filter_invalid: bool = True) -> np.ndarray:
        """Convert array indices to neuron IDs."""
        if indices is None or len(indices) == 0:
            return np.array([], dtype=np.int32)

        max_index = int(np.max(indices)) if len(indices) > 0 else -1
        if max_index < 0:
            return np.array([], dtype=np.int32)

        id_lookup = np.full(max_index + 1, -1, dtype=np.int32)
        for idx, neuron_id in self.index_to_neuron_id.items():
            if 0 <= idx <= max_index:
                id_lookup[idx] = int(neuron_id)

        mapped = id_lookup[indices]
        if filter_invalid:
            return mapped[mapped >= 0]
        return mapped

    def update_consecutive_fire_limits_by_cortical_area(self, cortical_idx: int, limit: int) -> int:
        """Update consecutive fire limits for all neurons in a cortical area.
        
        Args:
            cortical_idx: Cortical area index
            limit: New consecutive fire limit
            
        Returns:
            Number of neurons updated
        """
        with self._lock:
            # Find neurons in this cortical area
            area_mask = (self.cortical_idxs == cortical_idx) & self.valid_mask
            
            if not np.any(area_mask):
                return 0
            
            # Update consecutive fire limits (vectorized)
            self.consecutive_fire_limits[area_mask] = int(limit)
            
            # Return count of updated neurons
            return int(np.sum(area_mask))
    
    def update_excitability_by_cortical_area(self, cortical_idx: int, excitability: float) -> int:
        """Update excitability for all neurons in a cortical area.
        
        CRITICAL: This is required for Rust NPU integration.
        When excitability changes, the neuron_array must be updated BEFORE
        reinitializing the Rust NPU.
        
        Args:
            cortical_idx: Cortical area index
            excitability: New excitability value (0.0 to 1.0)
            
        Returns:
            Number of neurons updated
        """
        with self._lock:
            # Find neurons in this cortical area
            area_mask = (self.cortical_idxs == cortical_idx) & self.valid_mask
            
            if not np.any(area_mask):
                return 0
            
            # Update excitability (vectorized)
            self.excitabilities[area_mask] = float(excitability)
            
            # Return count of updated neurons
            return int(np.sum(area_mask))


class MemoryNeuronArray:
    """Single source of truth for memory neurons - Clean Architecture."""
    
    def __init__(self, max_memory_neurons: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize memory neuron array."""
        if max_memory_neurons is None:
            npu_config = _load_npu_config()
            max_memory_neurons = npu_config["max_memory_neurons"]
            
        self.max_memory_neurons = max_memory_neurons
        self.backend = backend
        self.memory_neuron_count = 0
        self.count = 0
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Memory neuron lifecycle properties
        self.lifespan_current = np.zeros(max_memory_neurons, dtype=np.uint32)
        self.lifespan_initial = np.zeros(max_memory_neurons, dtype=np.uint32)
        self.lifespan_growth_rate = np.zeros(max_memory_neurons, dtype=np.float32)
        self.is_longterm_memory = np.zeros(max_memory_neurons, dtype=np.bool_)
        
        # Temporal tracking
        self.creation_burst = np.zeros(max_memory_neurons, dtype=np.uint64)
        self.last_activation_burst = np.zeros(max_memory_neurons, dtype=np.uint64)
        self.activation_count = np.zeros(max_memory_neurons, dtype=np.uint32)
        
        # Pattern identification
        self.cortical_idxs = np.zeros(max_memory_neurons, dtype=np.uint16)
        self.pattern_digests = np.empty(max_memory_neurons, dtype=object)
        self.patterns: Dict[int, str] = {}
        self.cortical_area_id = np.empty(max_memory_neurons, dtype=object)
        
        # Memory neuron state
        self.is_active = np.zeros(max_memory_neurons, dtype=np.bool_)
        self.valid_mask = np.zeros(max_memory_neurons, dtype=np.bool_)
        
        # Standard neuron properties - NO HARDCODED VALUES, ALL FROM GENOME
        self.membrane_potentials = np.zeros(max_memory_neurons, dtype=np.float32)
        self.thresholds = np.zeros(max_memory_neurons, dtype=np.float32)  # MUST be set from genome
        self.leak_coefficients = np.zeros(max_memory_neurons, dtype=np.float32)  # MUST be set from genome
        self.excitabilities = np.zeros(max_memory_neurons, dtype=np.float32)  # MUST be set from genome
        
        # Memory neuron ID mapping
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        self._next_neuron_id = 1
        
        # Compatibility fields
        self.capacity = max_memory_neurons
        self.next_available_index = 0
        self.deleted_indices: List[int] = []
        self.pattern_to_index: Dict[MemoryPatternKey, int] = {}
        
        logger.info("MemoryNeuronArray initialized: %d max memory neurons", max_memory_neurons)
    
    def create_memory_neuron(self, pattern_key: MemoryPatternKey, cortical_area_id: str,
                           current_burst: int, initial_lifespan: int,
                           lifespan_growth_rate: float,
                           membrane_potential: float,
                           firing_threshold: float) -> int:
        """Create a single memory neuron representing a temporal pattern."""
        if self.deleted_indices:
            idx = self.deleted_indices.pop()
        else:
            idx = self.next_available_index
            if idx >= self.max_memory_neurons:
                raise ValueError("Cannot create memory neuron: capacity reached")
            self.next_available_index += 1

        neuron_id = self._next_neuron_id
        self._next_neuron_id += 1
        self.neuron_id_to_index[neuron_id] = idx
        self.index_to_neuron_id[idx] = neuron_id

        # Initialize properties
        self.is_active[idx] = True
        self.valid_mask[idx] = True
        self.lifespan_initial[idx] = int(initial_lifespan)
        self.lifespan_current[idx] = int(initial_lifespan)
        self.lifespan_growth_rate[idx] = float(lifespan_growth_rate)
        self.creation_burst[idx] = int(current_burst)
        self.last_activation_burst[idx] = int(current_burst)
        self.activation_count[idx] = 1
        
        self.cortical_area_id[idx] = str(cortical_area_id)
        self.membrane_potentials[idx] = float(membrane_potential)  # FROM GENOME - no hardcoded values
        self.thresholds[idx] = float(firing_threshold)  # FROM GENOME - no hardcoded values
        
        self.count += 1
        self.pattern_to_index[pattern_key] = idx
        self.pattern_digests[idx] = hash(pattern_key) & 0xFFFF
        
        return idx

    def reactivate_memory_neuron(self, neuron_idx: int, current_burst: int) -> bool:
        """Reactivate an existing memory neuron: update last activation and counters."""
        try:
            if neuron_idx < 0 or neuron_idx >= self.max_memory_neurons:
                return False
            if not self.valid_mask[neuron_idx]:
                return False
            self.is_active[neuron_idx] = True
            self.last_activation_burst[neuron_idx] = int(current_burst)
            self.activation_count[neuron_idx] = int(self.activation_count[neuron_idx]) + 1
            # Lifespan growth will be applied by aging step
            return True
        except Exception:
            return False

    def age_memory_neurons(self, current_burst: int, decay_per_burst: int = 1) -> int:
        """Age memory neurons deterministically by reducing lifespan for inactive neurons.

        Returns the number of neurons that reached zero lifespan (can be pruned by caller).
        """
        try:
            active_count = int(self.count)
            if active_count <= 0:
                return 0
            import numpy as np
            idxs = np.arange(0, active_count, dtype=np.int32)
            valid = self.valid_mask[:active_count]
            if not np.any(valid):
                return 0
            # Decrease lifespan for valid neurons that were not activated this burst
            not_recent = valid & (self.last_activation_burst[:active_count] < int(current_burst))
            self.lifespan_current[not_recent] = np.maximum(
                0,
                self.lifespan_current[not_recent] - int(decay_per_burst),
            )
            # Increase lifespan for those activated this burst using growth rate
            recent = valid & (self.last_activation_burst[:active_count] == int(current_burst))
            if np.any(recent):
                growth = np.floor(self.lifespan_growth_rate[recent]).astype(self.lifespan_current.dtype)
                self.lifespan_current[recent] += growth

            # Count expired
            expired = valid & (self.lifespan_current[:active_count] <= 0)
            return int(np.sum(expired))
        except Exception:
            return 0

    def check_longterm_conversion(self, longterm_threshold: int) -> int:
        """Mark neurons as long-term if activation count exceeds threshold.

        Returns number of converted neurons.
        """
        try:
            import numpy as np
            active_count = int(self.count)
            if active_count <= 0:
                return 0
            valid = self.valid_mask[:active_count]
            if not np.any(valid):
                return 0
            to_convert = valid & (self.activation_count[:active_count] >= int(longterm_threshold))
            convert_idx = np.nonzero(to_convert)[0]
            if convert_idx.size == 0:
                return 0
            self.is_longterm_memory[convert_idx] = True
            return int(convert_idx.size)
        except Exception:
            return 0


class SynapseArray:
    """Single source of truth for synapses - Clean Architecture."""
    
    def __init__(self, max_synapses: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize synapse array."""
        if max_synapses is None:
            npu_config = _load_npu_config()
            max_synapses = npu_config["max_synapses"]
            
        self.max_synapses = max_synapses
        self.backend = backend
        self.synapse_count = 0
        self.count = 0
        
        # Detect SIMD configuration
        self.simd_config = SIMDDetector.detect_simd_config(backend)
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Core synapse connectivity
        self.source_neuron_ids = np.zeros(max_synapses, dtype=np.uint32)
        self.target_neuron_ids = np.zeros(max_synapses, dtype=np.uint32)
        
        # Synaptic properties - NO HARDCODED VALUES, ALL FROM GENOME
        self.weights = np.zeros(max_synapses, dtype=np.float32)
        self.delays = np.zeros(max_synapses, dtype=np.uint8)  # MUST be set from genome
        self.types = np.zeros(max_synapses, dtype=np.uint8)
        self.conductances = np.zeros(max_synapses, dtype=np.float32)  # MUST be set from genome
        
        # Plasticity properties - NO HARDCODED VALUES, ALL FROM GENOME
        self.is_plastic_flags = np.zeros(max_synapses, dtype=np.bool_)
        self.plasticity_types = np.zeros(max_synapses, dtype=np.uint8)
        self.plasticity_coeffs = np.zeros(max_synapses, dtype=np.float32)  # MUST be set from genome
        self.decay_rates = np.zeros(max_synapses, dtype=np.float32)  # MUST be set from genome
        
        # Synapse state
        self.valid_mask = np.zeros(max_synapses, dtype=np.bool_)
        self.is_active = np.zeros(max_synapses, dtype=np.bool_)
        
        # Indexing for fast lookup
        self.source_neuron_index: Dict[int, List[int]] = {}
        self.target_neuron_index: Dict[int, List[int]] = {}
        
        logger.info("SynapseArray initialized: %d max synapses, %s backend", max_synapses, backend.value)
    
    def create_synapse(self, source_neuron_id: int, target_neuron_id: int, 
                      weight: float, synapse_type: int = 0, 
                      delay: int = 1, conductance: float = 1.0,
                      plasticity_coeff: float = 0.0) -> bool:
        """Create a single synapse."""
        if self.count >= self.max_synapses:
            return False
            
        # Determine plasticity type based on synapse_type
        plasticity_type = 1 if synapse_type == 3 else 0  # 3=PLASTIC, 0=NON_PLASTIC
        
        success = self.add_synapses_batch(
            source_neuron_ids=[source_neuron_id],
            target_neuron_ids=[target_neuron_id],
            weights=[weight],
            delays=[delay],
            conductances=[conductance],
            synapse_types=[synapse_type],
            plasticity_types=[plasticity_type],
            plasticity_coefficients=[plasticity_coeff]
        )
        
        return success == 1
    
    def add_synapses_batch(self, source_neuron_ids: List[int], target_neuron_ids: List[int],
                          weights: List[float], delays: Optional[List[int]] = None, 
                          conductances: Optional[List[float]] = None,
                          synapse_types: Optional[List[int]] = None, 
                          plasticity_types: Optional[List[int]] = None, 
                          plasticity_coefficients: Optional[List[float]] = None) -> int:
        """Add multiple synapses in batch."""
        count = len(source_neuron_ids)
        
        # Set defaults for optional parameters
        if delays is None:
            delays = [1] * count
        if conductances is None:
            conductances = [1.0] * count
        if synapse_types is None:
            synapse_types = [0] * count
        if plasticity_types is None:
            plasticity_types = [0] * count
        if plasticity_coefficients is None:
            plasticity_coefficients = [0.0] * count
        
        if (count != len(target_neuron_ids) or count != len(weights) or 
            count != len(delays) or count != len(conductances) or 
            count != len(synapse_types) or count != len(plasticity_types) or 
            count != len(plasticity_coefficients)):
            raise ValueError("All input lists must have same length")
            
        if self.count + count > self.max_synapses:
            available = int(self.max_synapses - self.count)
            raise ValueError(
                f"Cannot add {count} synapses: would exceed capacity (current={int(self.count)}, max={int(self.max_synapses)}, available={available})"
            )
        
        start_idx = self.count
        end_idx = start_idx + count
        
        # Batch assignment - ALL VALUES FROM GENOME
        self.source_neuron_ids[start_idx:end_idx] = np.array(source_neuron_ids, dtype=np.uint32)
        self.target_neuron_ids[start_idx:end_idx] = np.array(target_neuron_ids, dtype=np.uint32)
        self.weights[start_idx:end_idx] = np.array(weights, dtype=np.float32)
        self.delays[start_idx:end_idx] = np.array(delays, dtype=np.uint8)
        self.conductances[start_idx:end_idx] = np.array(conductances, dtype=np.float32)  # FROM GENOME
        self.types[start_idx:end_idx] = np.array(synapse_types, dtype=np.uint8)  # FROM GENOME
        self.plasticity_types[start_idx:end_idx] = np.array(plasticity_types, dtype=np.uint8)
        self.plasticity_coeffs[start_idx:end_idx] = np.array(plasticity_coefficients, dtype=np.float32)
        
        # Mark as active
        self.is_active[start_idx:end_idx] = True
        self.valid_mask[start_idx:end_idx] = True
        
        # Update indexing
        for i, (source_id, target_id) in enumerate(zip(source_neuron_ids, target_neuron_ids)):
            synapse_idx = start_idx + i
            
            if source_id not in self.source_neuron_index:
                self.source_neuron_index[source_id] = []
            self.source_neuron_index[source_id].append(synapse_idx)
            
            if target_id not in self.target_neuron_index:
                self.target_neuron_index[target_id] = []
            self.target_neuron_index[target_id].append(synapse_idx)
        
        self.count += count
        self.synapse_count += count
        return count
    
    def get_outgoing_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get outgoing connections from a neuron."""
        connections = []
        if neuron_id in self.source_neuron_index:
            for synapse_idx in self.source_neuron_index[neuron_id]:
                if synapse_idx < self.count and self.valid_mask[synapse_idx]:
                    target_id = int(self.target_neuron_ids[synapse_idx])
                    weight = float(self.weights[synapse_idx])
                    connections.append((target_id, weight))
        return connections
    
    def get_incoming_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get incoming connections to a neuron.
        
        Args:
            neuron_id: Target neuron ID
            
        Returns:
            List of (source_neuron_id, weight) tuples
        """
        connections = []
        if neuron_id in self.target_neuron_index:
            for synapse_idx in self.target_neuron_index[neuron_id]:
                if synapse_idx < self.count and self.valid_mask[synapse_idx]:
                    source_id = int(self.source_neuron_ids[synapse_idx])
                    weight = float(self.weights[synapse_idx])
                    connections.append((source_id, weight))
        return connections
    
    def has_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
        """Check if synapse exists between two neurons."""
        if source_neuron_id not in self.source_neuron_index:
            return False
            
        for synapse_idx in self.source_neuron_index[source_neuron_id]:
            if synapse_idx < self.count and self.valid_mask[synapse_idx]:
                if self.target_neuron_ids[synapse_idx] == target_neuron_id:
                    return True
        return False
    
    def _find_synapse_index(self, source_neuron_id: int, target_neuron_id: int) -> Optional[int]:
        """Find synapse index by source and target neuron IDs."""
        if source_neuron_id not in self.source_neuron_index:
            return None
            
        for synapse_idx in self.source_neuron_index[source_neuron_id]:
            if (synapse_idx < self.count and 
                self.valid_mask[synapse_idx] and
                self.target_neuron_ids[synapse_idx] == target_neuron_id):
                return synapse_idx
        
        return None
    
    def delete_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
        """Delete a single synapse between two neurons."""
        synapse_idx = self._find_synapse_index(source_neuron_id, target_neuron_id)
        if synapse_idx is None:
            return False
            
        try:
            # Mark as invalid and inactive
            self.valid_mask[synapse_idx] = False
            self.is_active[synapse_idx] = False
            
            # Update counts
            self.count -= 1
            self.synapse_count -= 1
            
            # Remove from index lists
            self.source_neuron_index[source_neuron_id].remove(synapse_idx)
            if not self.source_neuron_index[source_neuron_id]:
                del self.source_neuron_index[source_neuron_id]
                
            self.target_neuron_index[target_neuron_id].remove(synapse_idx)
            if not self.target_neuron_index[target_neuron_id]:
                del self.target_neuron_index[target_neuron_id]
            
            return True
        except Exception:
            return False
    
    def propagate_activations(self, firing_neuron_ids: List[int], neuron_array: 'NeuronArray') -> None:
        """Propagate activations from firing neurons to their targets."""
        import numpy as np
        
        if not firing_neuron_ids or neuron_array is None:
            return
            
        # Process each firing neuron
        for source_neuron_id in firing_neuron_ids:
            if source_neuron_id not in self.source_neuron_index:
                continue
                
            # Get all outgoing synapses from this neuron
            outgoing_synapses = self.source_neuron_index[source_neuron_id]
            
            for synapse_idx in outgoing_synapses:
                if synapse_idx >= self.count or not self.valid_mask[synapse_idx]:
                    continue
                
                target_neuron_id = int(self.target_neuron_ids[synapse_idx])
                weight = float(self.weights[synapse_idx])
                conductance = float(self.conductances[synapse_idx])
                
                # Get target neuron array index
                if target_neuron_id in neuron_array.neuron_id_to_index:
                    target_idx = neuron_array.neuron_id_to_index[target_neuron_id]
                    
                    # Update membrane potential with synaptic input
                    with neuron_array._lock:
                        delta_potential = weight * conductance
                        neuron_array.membrane_potentials[target_idx] += delta_potential
    


# Export the main data structures for compatibility
__all__ = [
    'NeuronArray',
    'MemoryNeuronArray', 
    'SynapseArray',
    'BackendType',
    'SIMDConfig',
    'SIMDDetector',
    'MemoryPatternKey'
]
