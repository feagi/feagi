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
        
        # Core neuron properties (SoA format)
        self.membrane_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_neurons, dtype=np.float32)
        self.decay_rates = np.full(max_neurons, 0.9, dtype=np.float32)
        self.leak_coefficients = np.full(max_neurons, 0.1, dtype=np.float32)
        self.resting_potentials = np.zeros(max_neurons, dtype=np.float32)
        self.neuron_types = np.zeros(max_neurons, dtype=np.int32)
        
        # Spatial properties
        self.positions_x = np.zeros(max_neurons, dtype=np.int32)
        self.positions_y = np.zeros(max_neurons, dtype=np.int32)
        self.positions_z = np.zeros(max_neurons, dtype=np.int32)
        self.coordinates_x = np.zeros(max_neurons, dtype=np.uint32)
        self.coordinates_y = np.zeros(max_neurons, dtype=np.uint32)
        self.coordinates_z = np.zeros(max_neurons, dtype=np.uint32)
        
        # Firing properties
        self.refractory_periods = np.ones(max_neurons, dtype=np.uint8)
        self.refractory_counters = np.zeros(max_neurons, dtype=np.uint8)
        
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
        self.excitabilities = np.ones(max_neurons, dtype=np.float32)
        
        logger.info(f"NeuronArray initialized: {max_neurons:,} max neurons, {backend.value} backend")
    
    def add_neurons_batch(self, neuron_ids: List[int], positions: List[Tuple[int, int, int]],
                         neuron_types: List[int], initial_potentials: List[float],
                         thresholds: List[float], leak_coefficients: List[float],
                         cortical_idx: int) -> List[int]:
        """Add multiple neurons in batch."""
        count = len(neuron_ids)
        if self.count + count > self.max_neurons:
            raise ValueError(f"Cannot add {count} neurons: would exceed capacity")
        
        start_idx = self.count
        indices = list(range(start_idx, start_idx + count))
        end_idx = start_idx + count
        
        # Update arrays
        self.membrane_potentials[start_idx:end_idx] = np.array(initial_potentials, dtype=np.float32)
        self.thresholds[start_idx:end_idx] = np.array(thresholds, dtype=np.float32)
        self.leak_coefficients[start_idx:end_idx] = np.array(leak_coefficients, dtype=np.float32)
        self.neuron_types[start_idx:end_idx] = np.array(neuron_types, dtype=np.int32)
        self.cortical_idxs[start_idx:end_idx] = cortical_idx
        
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
        
        # Standard neuron properties
        self.membrane_potentials = np.zeros(max_memory_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_memory_neurons, dtype=np.float32)
        self.leak_coefficients = np.full(max_memory_neurons, 0.1, dtype=np.float32)
        self.excitabilities = np.ones(max_memory_neurons, dtype=np.float32)
        
        # Memory neuron ID mapping
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        self._next_neuron_id = 1
        
        # Compatibility fields
        self.capacity = max_memory_neurons
        self.next_available_index = 0
        self.deleted_indices: List[int] = []
        self.pattern_to_index: Dict[MemoryPatternKey, int] = {}
        
        logger.info(f"MemoryNeuronArray initialized: {max_memory_neurons:,} max memory neurons")
    
    def create_memory_neuron(self, pattern_key: MemoryPatternKey, cortical_area_id: str,
                           current_burst: int, initial_lifespan: int = 9,
                           lifespan_growth_rate: float = 1.0) -> int:
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
        self.membrane_potentials[idx] = 1.5  # Above threshold to fire
        self.thresholds[idx] = 1.0
        
        self.count += 1
        self.pattern_to_index[pattern_key] = idx
        self.pattern_digests[idx] = hash(pattern_key) & 0xFFFF
        
        return idx


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
        
        # Synaptic properties
        self.weights = np.zeros(max_synapses, dtype=np.float32)
        self.delays = np.ones(max_synapses, dtype=np.uint8)
        self.types = np.zeros(max_synapses, dtype=np.uint8)
        self.conductances = np.ones(max_synapses, dtype=np.float32)
        
        # Plasticity properties
        self.is_plastic_flags = np.zeros(max_synapses, dtype=np.bool_)
        self.plasticity_types = np.zeros(max_synapses, dtype=np.uint8)
        self.plasticity_coeffs = np.ones(max_synapses, dtype=np.float32)
        self.decay_rates = np.full(max_synapses, 0.95, dtype=np.float32)
        
        # Synapse state
        self.valid_mask = np.zeros(max_synapses, dtype=np.bool_)
        self.is_active = np.zeros(max_synapses, dtype=np.bool_)
        
        # Indexing for fast lookup
        self.source_neuron_index: Dict[int, List[int]] = {}
        self.target_neuron_index: Dict[int, List[int]] = {}
        
        logger.info(f"SynapseArray initialized: {max_synapses:,} max synapses, {backend.value} backend")
    
    def add_synapses_batch(self, source_neuron_ids: List[int], target_neuron_ids: List[int],
                          weights: List[float], delays: List[int], 
                          plasticity_types: List[int], plasticity_coefficients: List[float]) -> int:
        """Add multiple synapses in batch."""
        count = len(source_neuron_ids)
        if count != len(target_neuron_ids) or count != len(weights):
            raise ValueError("All input lists must have same length")
            
        if self.count + count > self.max_synapses:
            available = int(self.max_synapses - self.count)
            raise ValueError(
                f"Cannot add {count} synapses: would exceed capacity (current={int(self.count)}, max={int(self.max_synapses)}, available={available})"
            )
        
        start_idx = self.count
        end_idx = start_idx + count
        
        # Batch assignment
        self.source_neuron_ids[start_idx:end_idx] = np.array(source_neuron_ids, dtype=np.uint32)
        self.target_neuron_ids[start_idx:end_idx] = np.array(target_neuron_ids, dtype=np.uint32)
        self.weights[start_idx:end_idx] = np.array(weights, dtype=np.float32)
        self.delays[start_idx:end_idx] = np.array(delays, dtype=np.uint8)
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
