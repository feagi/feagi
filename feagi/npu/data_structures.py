"""
NPU Data Structures - Single Source of Truth
============================================

This module provides the core data structures for FEAGI's Neural Processing Unit.
All brain data is owned by NPU with controlled access for other modules.

Key Design Principles:
- Single source of truth for all neural data
- Structure of Arrays (SoA) for SIMD/GPU optimization
- Rust FFI-compatible data types and layouts
- RTOS-friendly deterministic operations
- Zero-allocation hot paths
- Cache-aligned memory for optimal performance

Copyright 2025 Neuraville Inc.
Licensed under the Apache License, Version 2.0
"""

from typing import Dict, List, Optional, Tuple, Any, Union
from dataclasses import dataclass
from enum import Enum
import numpy as np
import threading

from feagi.utils.logger import setup_logger
from feagi.config.toml_loader import load_feagi_config

logger = setup_logger()


@dataclass
class MemoryPatternKey:
    """Key for memory pattern identification and caching.
    
    Used by MemoryProcessor for temporal pattern detection and storage.
    """
    pattern_data: Tuple[bytes, ...]  # Serialized bitmap sequence
    temporal_depth: int  # Number of timesteps
    source_cortical_areas: Tuple[str, ...]  # Upstream areas
    
    def __hash__(self) -> int:
        return hash((self.pattern_data, self.temporal_depth, self.source_cortical_areas))


def _load_npu_config() -> Dict[str, Any]:
    """Load NPU configuration from FEAGI configuration file.
    
    Uses the same logic as ConnectomeManager for consistency.
    
    Returns:
        Dictionary with NPU configuration values
    """
    try:
        config = load_feagi_config()
        
        # Extract connectome configuration (same as ConnectomeManager)
        connectome_config = config.get("connectome", {})
        neural_config = config.get("neural", {})
        resources_config = config.get("resources", {})
        
        # Use same defaults as ConnectomeManager
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
    vector_width: int  # Elements per SIMD instruction
    alignment_bytes: int  # Memory alignment requirement
    optimal_batch_size: int  # Optimal batch size for operations
    max_batch_size: int  # Maximum batch size before performance degrades


class SIMDDetector:
    """Detects optimal SIMD configuration for the current backend."""
    
    @staticmethod
    def detect_simd_config(backend: BackendType) -> SIMDConfig:
        """Detect optimal SIMD configuration for the given backend.
        
        Args:
            backend: Target computation backend
            
        Returns:
            SIMDConfig with optimal parameters for the backend
        """
        if backend == BackendType.CPU:
            # Detect CPU SIMD capabilities
            try:
                import platform
                arch = platform.machine().lower()
                
                if 'arm' in arch or 'aarch64' in arch:
                    # ARM NEON: 128-bit vectors
                    return SIMDConfig(
                        backend=backend,
                        vector_width=4,  # 4 x float32
                        alignment_bytes=16,
                        optimal_batch_size=256,  # 64 SIMD instructions
                        max_batch_size=2048
                    )
                else:
                    # x86 AVX2: 256-bit vectors
                    return SIMDConfig(
                        backend=backend,
                        vector_width=8,  # 8 x float32
                        alignment_bytes=32,
                        optimal_batch_size=512,  # 64 SIMD instructions
                        max_batch_size=4096
                    )
            except Exception:
                # Fallback to conservative settings
                return SIMDConfig(
                    backend=backend,
                    vector_width=4,
                    alignment_bytes=16,
                    optimal_batch_size=256,
                    max_batch_size=1024
                )
                
        elif backend == BackendType.CUDA:
            # CUDA: Warp size 32, optimize for coalesced access
            return SIMDConfig(
                backend=backend,
                vector_width=32,  # Warp size
                alignment_bytes=128,  # Cache line
                optimal_batch_size=1024,  # 32 warps
                max_batch_size=8192
            )
            
        elif backend == BackendType.METAL:
            # Metal: SIMD group size 32 (Apple Silicon)
            return SIMDConfig(
                backend=backend,
                vector_width=32,
                alignment_bytes=64,
                optimal_batch_size=1024,
                max_batch_size=4096
            )
            
        elif backend == BackendType.WGPU:
            # WebGPU: Workgroup size typically 64-256
            return SIMDConfig(
                backend=backend,
                vector_width=64,
                alignment_bytes=64,
                optimal_batch_size=512,
                max_batch_size=2048
            )
            
        else:  # RUST or unknown
            # Conservative Rust-compatible settings
            return SIMDConfig(
                backend=backend,
                vector_width=8,
                alignment_bytes=32,
                optimal_batch_size=512,
                max_batch_size=2048
            )


class NeuronArray:
    """Single source of truth for regular neurons - NPU owned.
    
    Structure of Arrays (SoA) design optimized for:
    - SIMD vectorization (batch operations on aligned arrays)
    - GPU coalesced memory access
    - Cache-friendly memory layout
    - Rust FFI compatibility (primitive types only)
    - RTOS deterministic operations
    """
    
    def __init__(self, max_neurons: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize neuron array as single source of truth.
        
        Args:
            max_neurons: Maximum number of neurons to support (None = load from config)
            backend: Computation backend for optimization
        """
        # Load configuration if max_neurons not provided
        if max_neurons is None:
            npu_config = _load_npu_config()
            max_neurons = npu_config["max_neurons"]
            
        self.max_neurons = max_neurons
        self.backend = backend
        self.neuron_count = 0
        
        # Alias for consistency with other arrays
        self.count = 0
        
        # Detect optimal SIMD configuration
        self.simd_config = SIMDDetector.detect_simd_config(backend)
        
        # Align capacity to SIMD boundaries for optimal performance
        alignment = self.simd_config.vector_width
        self.aligned_capacity = ((max_neurons + alignment - 1) // alignment) * alignment
        
        # Thread safety for concurrent access
        self._lock = threading.RLock()
        
        # Structure of Arrays - all arrays cache-aligned
        alignment_bytes = self.simd_config.alignment_bytes
        
        # Core neuron properties
        self.membrane_potentials = self._create_aligned_array(np.float32, alignment_bytes)
        self.thresholds = self._create_aligned_array(np.float32, alignment_bytes, default_value=1.0)
        self.decay_rates = self._create_aligned_array(np.float32, alignment_bytes, default_value=0.9)
        self.leak_coefficients = self._create_aligned_array(np.float32, alignment_bytes, default_value=0.1)  # Alias for decay_rates
        self.resting_potentials = self._create_aligned_array(np.float32, alignment_bytes)
        self.neuron_types = self._create_aligned_array(np.int32, alignment_bytes)
        # Position arrays (separate for better SIMD performance)
        self.positions_x = self._create_aligned_array(np.int32, alignment_bytes)
        self.positions_y = self._create_aligned_array(np.int32, alignment_bytes)  
        self.positions_z = self._create_aligned_array(np.int32, alignment_bytes)
        
        # Firing and refractory properties
        self.refractory_periods = self._create_aligned_array(np.uint8, alignment_bytes, default_value=1)
        self.refractory_counters = self._create_aligned_array(np.uint8, alignment_bytes)
        self.excitabilities = self._create_aligned_array(np.float32, alignment_bytes, default_value=1.0)
        
        # Spatial and organizational properties
        self.cortical_idxs = self._create_aligned_array(np.uint16, alignment_bytes)
        self.coordinates_x = self._create_aligned_array(np.uint32, alignment_bytes)
        self.coordinates_y = self._create_aligned_array(np.uint32, alignment_bytes)
        self.coordinates_z = self._create_aligned_array(np.uint32, alignment_bytes)
        
        # Neuron ID mapping (NPU owns this mapping)
        self.neuron_id_to_index: Dict[int, int] = {}
        self.index_to_neuron_id: Dict[int, int] = {}
        self._next_neuron_id = 1  # Start from 1, 0 reserved for invalid
        
        # Valid neuron tracking
        self.valid_mask = self._create_aligned_array(np.bool_, alignment_bytes)
        
        # Free index management for O(1) allocation
        self.free_indices: List[int] = []
        self.next_free_index = 0
        
        logger.info(f"NeuronArray initialized: {max_neurons:,} max neurons, {backend.value} backend")
        logger.info(f"SIMD config: {self.simd_config.vector_width} vector width, {self.simd_config.optimal_batch_size} optimal batch size")
    
    def _create_aligned_array(self, dtype: type, alignment_bytes: int, default_value: Any = None) -> np.ndarray:
        """Create cache-aligned numpy array for optimal SIMD performance.
        
        Args:
            dtype: NumPy data type
            alignment_bytes: Memory alignment requirement
            default_value: Default value to fill array with
            
        Returns:
            Cache-aligned numpy array
        """
        # Create array with extra space for alignment
        extra_bytes = alignment_bytes - 1
        
        # Allocate raw memory
        raw_array = np.empty(self.aligned_capacity + extra_bytes // np.dtype(dtype).itemsize, dtype=dtype)
        
        # Calculate aligned offset
        raw_ptr = raw_array.ctypes.data
        aligned_ptr = (raw_ptr + alignment_bytes - 1) & ~(alignment_bytes - 1)
        offset = (aligned_ptr - raw_ptr) // np.dtype(dtype).itemsize
        
        # Create aligned view
        aligned_array = raw_array[offset:offset + self.aligned_capacity]
        
        # Initialize with default value if provided
        if default_value is not None:
            aligned_array.fill(default_value)
        else:
            aligned_array.fill(0)
            
        return aligned_array

    def indices_to_neuron_ids(self, indices: np.ndarray, *, filter_invalid: bool = True) -> np.ndarray:
        """Convert array indices to neuron IDs using the NPU-owned mapping.

        Args:
            indices: NumPy array of integer indices
            filter_invalid: Whether to filter out indices that have no neuron ID

        Returns:
            NumPy array of neuron IDs (dtype=int32). If filter_invalid is False,
            invalid indices are mapped to -1.
        """
        if indices is None or len(indices) == 0:
            return np.array([], dtype=np.int32)

        # Fast path: build a vectorized map via an array if possible
        # Build a temporary array filled with -1, then set known indices
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
    
    def get_optimal_batch_size(self) -> int:
        """Get optimal batch size for SIMD operations on this backend."""
        return self.simd_config.optimal_batch_size
    
    def get_max_batch_size(self) -> int:
        """Get maximum recommended batch size for this backend."""
        return self.simd_config.max_batch_size
    
    def add_neurons_batch(self, neuron_ids: List[int], positions: List[Tuple[int, int, int]],
                         neuron_types: List[int], initial_potentials: List[float],
                         thresholds: List[float], leak_coefficients: List[float],
                         excitabilities: List[float], cortical_idx: int) -> List[int]:
        """Add multiple neurons in batch with SIMD optimization.
        
        Args:
            neuron_ids: List of neuron IDs
            positions: List of (x, y, z) positions
            neuron_types: List of neuron types
            initial_potentials: List of initial membrane potentials
            thresholds: List of firing thresholds
            leak_coefficients: List of leak coefficients
            excitabilities: List of excitability values
            cortical_idx: Fast integer index for the cortical area
            
        Returns:
            List of array indices where neurons were added
        """
        count = len(neuron_ids)
        if self.count + count > self.max_neurons:
            raise ValueError(f"Cannot add {count} neurons: would exceed capacity")
        
        # Get batch of indices
        start_idx = self.count
        indices = list(range(start_idx, start_idx + count))
        
        # SIMD-optimized batch assignment
        end_idx = start_idx + count
        
        # Update arrays in batch
        self.membrane_potentials[start_idx:end_idx] = np.array(initial_potentials, dtype=np.float32)
        self.thresholds[start_idx:end_idx] = np.array(thresholds, dtype=np.float32)
        self.leak_coefficients[start_idx:end_idx] = np.array(leak_coefficients, dtype=np.float32)
        self.excitabilities[start_idx:end_idx] = np.array(excitabilities, dtype=np.float32)
        self.neuron_types[start_idx:end_idx] = np.array(neuron_types, dtype=np.int32)
        
        # Set cortical_idx for fast area lookups
        self.cortical_idxs[start_idx:end_idx] = cortical_idx
        
        # Set spatial coordinates from positions
        positions_array = np.array(positions, dtype=np.int32)
        self.positions_x[start_idx:end_idx] = positions_array[:, 0]
        self.positions_y[start_idx:end_idx] = positions_array[:, 1]
        self.positions_z[start_idx:end_idx] = positions_array[:, 2]
        self.coordinates_x[start_idx:end_idx] = positions_array[:, 0]
        self.coordinates_y[start_idx:end_idx] = positions_array[:, 1]
        self.coordinates_z[start_idx:end_idx] = positions_array[:, 2]
        
        # Update valid mask
        self.valid_mask[start_idx:end_idx] = True
        
        # Update ID mappings
        for i, neuron_id in enumerate(neuron_ids):
            idx = start_idx + i
            self.neuron_id_to_index[neuron_id] = idx
            self.index_to_neuron_id[idx] = neuron_id
        
        # Update counters
        self.count += count
        self.neuron_count += count  # Keep both in sync
        self._next_neuron_id = max(self._next_neuron_id, max(neuron_ids) + 1)
        
        return indices
    
    def remove_neurons_batch(self, neuron_ids: List[int]) -> int:
        """Remove multiple neurons in batch with SIMD optimization.
        
        Args:
            neuron_ids: List of neuron IDs to remove
            
        Returns:
            Number of neurons actually removed
        """
        removed_count = 0
        
        for neuron_id in neuron_ids:
            if neuron_id in self.neuron_id_to_index:
                idx = self.neuron_id_to_index[neuron_id]
                
                # Mark as invalid
                self.valid_mask[idx] = False
                
                # Clear mappings
                del self.neuron_id_to_index[neuron_id]
                if idx in self.index_to_neuron_id:
                    del self.index_to_neuron_id[idx]
                
                removed_count += 1
        
        # Update count
        self.count -= removed_count
        self.neuron_count -= removed_count  # Keep both in sync
        
        # TODO: Implement compaction for better memory efficiency
        # For now, just mark as invalid - compaction can be done periodically
        
        return removed_count
    
    def update_property_batch(self, neuron_ids: List[int], property_name: str, 
                            values: List[Union[float, int]]) -> int:
        """Update a property for multiple neurons in batch with SIMD optimization.
        
        Args:
            neuron_ids: List of neuron IDs
            property_name: Property name to update
            values: List of new values
            
        Returns:
            Number of neurons actually updated
        """
        if len(neuron_ids) != len(values):
            raise ValueError("neuron_ids and values must have same length")
        
        # Get valid indices
        valid_indices = []
        valid_values = []
        
        for neuron_id, value in zip(neuron_ids, values):
            if neuron_id in self.neuron_id_to_index:
                idx = self.neuron_id_to_index[neuron_id]
                valid_indices.append(idx)
                valid_values.append(value)
        
        if not valid_indices:
            return 0
        
        # SIMD-optimized batch update
        indices_array = np.array(valid_indices, dtype=np.int32)
        values_array = np.array(valid_values)
        
        if property_name == "membrane_potential":
            self.membrane_potentials[indices_array] = values_array.astype(np.float32)
        elif property_name == "threshold":
            self.thresholds[indices_array] = values_array.astype(np.float32)
        elif property_name == "leak_coefficient":
            self.leak_coefficients[indices_array] = values_array.astype(np.float32)
        elif property_name == "excitability":
            self.excitabilities[indices_array] = values_array.astype(np.float32)
        elif property_name == "neuron_type":
            self.neuron_types[indices_array] = values_array.astype(np.int32)
        else:
            return 0
        
        return len(valid_indices)
    
    def get_property(self, neuron_id: int, property_name: str) -> Optional[Union[float, int]]:
        """Get a neuron property value.
        
        Args:
            neuron_id: Neuron ID
            property_name: Property name to get
            
        Returns:
            Property value or None if neuron not found
        """
        if neuron_id not in self.neuron_id_to_index:
            return None
            
        idx = self.neuron_id_to_index[neuron_id]
        
        if property_name == "membrane_potential":
            return float(self.membrane_potentials[idx])
        elif property_name == "threshold":
            return float(self.thresholds[idx])
        elif property_name == "leak_coefficient":
            return float(self.leak_coefficients[idx])
        elif property_name == "excitability":
            return float(self.excitabilities[idx])
        elif property_name == "neuron_type":
            return int(self.neuron_types[idx])
        elif property_name == "position":
            return (int(self.positions_x[idx]), int(self.positions_y[idx]), int(self.positions_z[idx]))
        else:
            return None
    
    def set_cortical_area_excitability(self, cortical_idx: int, start_idx: int, end_idx: int, excitability: float) -> None:
        """Set excitability for all neurons in a cortical area range.
        
        Args:
            cortical_idx: Cortical area index (for validation)
            start_idx: Starting neuron index
            end_idx: Ending neuron index (exclusive)
            excitability: Excitability value to set
        """
        with self._lock:
            # Validate range against array capacity, not current count
            # (during neurogenesis, arrays are populated directly)
            if start_idx < 0 or end_idx > self.max_neurons or start_idx >= end_idx:
                raise ValueError(f"Invalid range: start_idx={start_idx}, end_idx={end_idx}, max_neurons={self.max_neurons}")
            
            # Set excitability for the range
            self.excitabilities[start_idx:end_idx] = excitability
            
            # Update count if this extends beyond current count
            if end_idx > self.count:
                self.count = end_idx
                self.neuron_count = end_idx
    
    def set_neuron_property(self, neuron_id: int, property_name: str, value: Union[float, int]) -> None:
        """Set a property value for a specific neuron.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to set
            value: Value to set
        """
        if neuron_id not in self.neuron_id_to_index:
            # Deterministic: ignore writes to unknown IDs (stimulation before registration)
            return
            
        idx = self.neuron_id_to_index[neuron_id]
        
        with self._lock:
            if property_name == "membrane_potential":
                self.membrane_potentials[idx] = float(value)
            elif property_name == "threshold":
                self.thresholds[idx] = float(value)
            elif property_name == "leak_coefficient":
                self.leak_coefficients[idx] = float(value)
            elif property_name == "excitability":
                self.excitabilities[idx] = float(value)
            elif property_name == "neuron_type":
                self.neuron_types[idx] = int(value)
            else:
                raise ValueError(f"Property {property_name} cannot be set")
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary for the neuron array.
        
        Returns:
            Dictionary with performance metrics
        """
        return {
            "total_neurons": self.neuron_count,
            "max_capacity": self.max_neurons,
            "utilization": self.neuron_count / self.max_neurons if self.max_neurons > 0 else 0.0,
            "backend": self.backend.value,
            "simd_config": {
                "vector_width": self.simd_config.vector_width,
                "alignment_bytes": self.simd_config.alignment_bytes,
                "optimal_batch_size": self.simd_config.optimal_batch_size
            },
            "memory_aligned": True,
            "thread_safe": True
        }


class MemoryNeuronArray:
    """Single source of truth for memory neurons - NPU owned.
    
    Memory neurons are completely separate from regular neurons with different:
    - Lifecycle properties (aging, lifespan, long-term conversion)
    - Pattern-based identification (no spatial coordinates)
    - Temporal processing requirements
    """
    
    def __init__(self, max_memory_neurons: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize memory neuron array.
        
        Args:
            max_memory_neurons: Maximum number of memory neurons (None = load from config)
            backend: Computation backend (memory neurons are CPU-optimized)
        """
        # Load configuration if max_memory_neurons not provided
        if max_memory_neurons is None:
            npu_config = _load_npu_config()
            max_memory_neurons = npu_config["max_memory_neurons"]
            
        self.max_memory_neurons = max_memory_neurons
        self.backend = backend  # Memory neurons typically use CPU backend
        self.memory_neuron_count = 0
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Memory neuron lifecycle properties (SoA)
        self.lifespan_current = np.zeros(max_memory_neurons, dtype=np.uint32)
        self.lifespan_initial = np.zeros(max_memory_neurons, dtype=np.uint32)
        self.lifespan_growth_rate = np.zeros(max_memory_neurons, dtype=np.float32)
        self.is_longterm_memory = np.zeros(max_memory_neurons, dtype=np.bool_)
        
        # Temporal tracking
        self.creation_burst = np.zeros(max_memory_neurons, dtype=np.uint64)
        self.last_activation_burst = np.zeros(max_memory_neurons, dtype=np.uint64)
        self.activation_count = np.zeros(max_memory_neurons, dtype=np.uint32)
        
        # Pattern identification (separate from spatial neurons)
        self.cortical_idxs = np.zeros(max_memory_neurons, dtype=np.uint16)  # Fast integer cortical area indices
        self.pattern_digests = np.empty(max_memory_neurons, dtype=object)  # Pattern hash digests
        self.patterns: Dict[int, str] = {}  # neuron_id -> pattern_string
        # Store cortical area string IDs for diagnostics/compat (not on hot path)
        self.cortical_area_id = np.empty(max_memory_neurons, dtype=object)
        
        # Memory neuron state
        self.is_active = np.zeros(max_memory_neurons, dtype=np.bool_)
        self.valid_mask = np.zeros(max_memory_neurons, dtype=np.bool_)
        
        # Standard neuron properties (inherited from regular neurons)
        self.membrane_potentials = np.zeros(max_memory_neurons, dtype=np.float32)
        self.thresholds = np.ones(max_memory_neurons, dtype=np.float32)
        self.leak_coefficients = np.full(max_memory_neurons, 0.1, dtype=np.float32)
        self.excitabilities = np.ones(max_memory_neurons, dtype=np.float32)
        
        # Memory neuron ID mapping (separate from regular neurons)
        self.neuron_id_to_index: Dict[int, int] = {}  # Use same name as NeuronArray for consistency
        self.index_to_neuron_id: Dict[int, int] = {}
        self.count = 0
        self._next_neuron_id = 1
        
        # Free index management
        self.free_indices: List[int] = []
        self.next_free_index = 0

        # Compatibility/diagnostics fields expected by some tests/tools
        self.capacity = max_memory_neurons
        self.next_available_index = 0
        self.deleted_indices: List[int] = []
        # Cache from pattern key to index for O(1) lookup
        self.pattern_to_index: Dict[MemoryPatternKey, int] = {}
        
        logger.info(f"MemoryNeuronArray initialized: {max_memory_neurons:,} max memory neurons")
    
    def add_neurons_batch(self, neuron_ids: List[int], patterns: List[str],
                         initial_potentials: List[float], thresholds: List[float],
                         leak_coefficients: List[float], excitabilities: List[float],
                         cortical_idx: int) -> List[int]:
        """Add multiple memory neurons in batch with SIMD optimization."""
        count = len(neuron_ids)
        if self.count + count > self.max_memory_neurons:
            raise ValueError(f"Cannot add {count} memory neurons: would exceed capacity")
        
        start_idx = self.count
        indices = list(range(start_idx, start_idx + count))
        end_idx = start_idx + count
        
        # Update arrays in batch
        self.membrane_potentials[start_idx:end_idx] = np.array(initial_potentials, dtype=np.float32)
        self.thresholds[start_idx:end_idx] = np.array(thresholds, dtype=np.float32)
        self.leak_coefficients[start_idx:end_idx] = np.array(leak_coefficients, dtype=np.float32)
        self.excitability[start_idx:end_idx] = np.array(excitabilities, dtype=np.float32)
        self.is_active[start_idx:end_idx] = True
        
        # Set cortical_idx for fast area lookups
        self.cortical_idxs[start_idx:end_idx] = cortical_idx
        
        # Update ID mappings and patterns
        for i, (neuron_id, pattern) in enumerate(zip(neuron_ids, patterns)):
            idx = start_idx + i
            self.neuron_id_to_index[neuron_id] = idx
            self.index_to_neuron_id[idx] = neuron_id
            self.patterns[neuron_id] = pattern
        
        self.count += count
        self._next_neuron_id = max(self._next_neuron_id, max(neuron_ids) + 1)
        
        return indices
    
    def remove_neurons_batch(self, neuron_ids: List[int]) -> int:
        """Remove multiple memory neurons in batch."""
        removed_count = 0
        
        for neuron_id in neuron_ids:
            if neuron_id in self.neuron_id_to_index:
                idx = self.neuron_id_to_index[neuron_id]
                
                # Mark as inactive
                self.is_active[idx] = False
                
                # Clear mappings
                del self.neuron_id_to_index[neuron_id]
                if idx in self.index_to_neuron_id:
                    del self.index_to_neuron_id[idx]
                if neuron_id in self.patterns:
                    del self.patterns[neuron_id]
                
                removed_count += 1
        
        self.count -= removed_count
        return removed_count
    
    def update_property_batch(self, neuron_ids: List[int], property_name: str, 
                            values: List[Union[float, int]]) -> int:
        """Update a property for multiple memory neurons in batch."""
        if len(neuron_ids) != len(values):
            raise ValueError("neuron_ids and values must have same length")
        
        valid_indices = []
        valid_values = []
        
        for neuron_id, value in zip(neuron_ids, values):
            if neuron_id in self.neuron_id_to_index:
                idx = self.neuron_id_to_index[neuron_id]
                valid_indices.append(idx)
                valid_values.append(value)
        
        if not valid_indices:
            return 0
        
        indices_array = np.array(valid_indices, dtype=np.int32)
        values_array = np.array(valid_values)
        
        if property_name == "membrane_potential":
            self.membrane_potentials[indices_array] = values_array.astype(np.float32)
        elif property_name == "threshold":
            self.thresholds[indices_array] = values_array.astype(np.float32)
        elif property_name == "leak_coefficient":
            self.leak_coefficients[indices_array] = values_array.astype(np.float32)
        elif property_name == "excitability":
            self.excitability[indices_array] = values_array.astype(np.float32)
        else:
            return 0
        
        return len(valid_indices)
    
    def get_property(self, neuron_id: int, property_name: str) -> Optional[Union[float, int]]:
        """Get a memory neuron property value."""
        if neuron_id not in self.neuron_id_to_index:
            return None
            
        idx = self.neuron_id_to_index[neuron_id]
        
        if property_name == "membrane_potential":
            return float(self.membrane_potentials[idx])
        elif property_name == "threshold":
            return float(self.thresholds[idx])
        elif property_name == "leak_coefficient":
            return float(self.leak_coefficients[idx])
        elif property_name == "excitability":
            return float(self.excitability[idx])
        elif property_name == "pattern":
            return self.patterns.get(neuron_id)
        else:
            return None

    # --- Memory-specific high-level operations used by MemoryProcessor ---

    def create_memory_neuron(
        self,
        pattern_key: MemoryPatternKey,
        cortical_area_id: str,
        current_burst: int,
        initial_lifespan: int = 9,
        lifespan_growth_rate: float = 1.0,
    ) -> int:
        """Create a single memory neuron representing a temporal pattern.

        Args:
            pattern_key: Unique key representing the temporal pattern
            cortical_area_id: String ID of the memory cortical area
            current_burst: Current burst index at creation
            initial_lifespan: Initial lifespan value
            lifespan_growth_rate: Growth multiplier applied on reactivation

        Returns:
            Index of the created memory neuron
        """
        # Reuse deleted slot if available to keep layout dense
        if self.deleted_indices:
            idx = self.deleted_indices.pop()
        else:
            idx = self.next_available_index
            if idx >= self.max_memory_neurons:
                raise ValueError("Cannot create memory neuron: capacity reached")
            self.next_available_index += 1

        # Assign a new neuron_id for bookkeeping (separate from index)
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
        self.is_longterm_memory[idx] = False
        self.creation_burst[idx] = int(current_burst)
        self.last_activation_burst[idx] = int(current_burst)
        self.activation_count[idx] = 1

        # Store cortical area identifiers
        self.cortical_area_id[idx] = str(cortical_area_id)
        # Note: caller should also set cortical_idxs[idx] if available; keep 0 as default otherwise

        # Store pattern mapping for fast lookup
        self.pattern_to_index[pattern_key] = idx
        # Optional digest for quick diagnostics (last 4 bytes of hash)
        self.pattern_digests[idx] = hash(pattern_key) & 0xFFFF

        # Maintain counts
        self.count = max(self.count, idx + 1)

        return idx

    def reactivate_memory_neuron(self, neuron_idx: int, current_burst: int) -> bool:
        """Reactivate an existing memory neuron and apply lifespan growth.

        Returns True if reactivation succeeded, False otherwise.
        """
        if neuron_idx < 0 or neuron_idx >= self.count:
            return False
        if not self.valid_mask[neuron_idx]:
            return False

        # Mark active and increase lifespan deterministically
        self.is_active[neuron_idx] = True
        self.last_activation_burst[neuron_idx] = int(current_burst)
        # Grow lifespan multiplicatively from initial
        base = int(self.lifespan_initial[neuron_idx])
        growth = float(self.lifespan_growth_rate[neuron_idx])
        grown = int(round(base * growth))
        # If current is already higher due to prior growth, keep the max
        self.lifespan_current[neuron_idx] = max(int(self.lifespan_current[neuron_idx]), grown)
        self.activation_count[neuron_idx] = int(self.activation_count[neuron_idx]) + 1
        return True

    def age_memory_neurons(self, current_burst: int) -> List[int]:
        """Age memory neurons by one burst; return indices that died this cycle."""
        died: List[int] = []
        # Vectorized where possible, but keep clarity
        for idx in range(self.count):
            if not self.valid_mask[idx] or not self.is_active[idx]:
                continue
            if self.is_longterm_memory[idx]:
                continue  # Long-term memories do not decay
            if self.lifespan_current[idx] > 0:
                self.lifespan_current[idx] = int(self.lifespan_current[idx]) - 1
            if self.lifespan_current[idx] <= 0:
                # Logical deletion
                self.is_active[idx] = False
                self.valid_mask[idx] = False
                died.append(idx)
                self.deleted_indices.append(idx)
        return died

    def check_longterm_conversion(self, longterm_threshold: int) -> List[int]:
        """Convert qualifying neurons to long-term memory; return converted indices."""
        converted: List[int] = []
        threshold = int(longterm_threshold)
        for idx in range(self.count):
            if not self.valid_mask[idx]:
                continue
            if self.is_longterm_memory[idx]:
                continue
            if int(self.lifespan_initial[idx]) >= threshold:
                self.is_longterm_memory[idx] = True
                converted.append(idx)
        return converted

    def find_memory_neuron_by_pattern(self, pattern_key: MemoryPatternKey) -> Optional[int]:
        """Find an existing memory neuron by pattern key; returns index or None."""
        return self.pattern_to_index.get(pattern_key)

    def get_statistics(self) -> Dict[str, Any]:
        """Return basic statistics for diagnostics and health checks."""
        total_active = int(np.count_nonzero(self.is_active[: self.count]))
        return {
            "total_active_neurons": total_active,
            "total_neurons": int(self.count),
            "capacity": int(self.max_memory_neurons),
            "longterm_count": int(np.count_nonzero(self.is_longterm_memory[: self.count])),
        }


class SynapseArray:
    """Single source of truth for synapses - NPU owned.
    
    Structure of Arrays (SoA) design optimized for:
    - Scatter-gather operations (synaptic propagation)
    - SIMD vectorization of synaptic updates
    - GPU coalesced memory access
    - Rust FFI compatibility
    """
    
    def __init__(self, max_synapses: Optional[int] = None, backend: BackendType = BackendType.CPU):
        """Initialize synapse array as single source of truth.
        
        Args:
            max_synapses: Maximum number of synapses to support (None = load from config)
            backend: Computation backend for optimization
        """
        # Load configuration if max_synapses not provided
        if max_synapses is None:
            npu_config = _load_npu_config()
            max_synapses = npu_config["max_synapses"]
            
        self.max_synapses = max_synapses
        self.backend = backend
        self.synapse_count = 0
        
        # Alias for consistency with other arrays
        self.count = 0
        
        # Detect optimal SIMD configuration
        self.simd_config = SIMDDetector.detect_simd_config(backend)
        
        # Align capacity to SIMD boundaries
        alignment = self.simd_config.vector_width
        self.aligned_capacity = ((max_synapses + alignment - 1) // alignment) * alignment
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Structure of Arrays - cache-aligned for optimal performance
        alignment_bytes = self.simd_config.alignment_bytes
        
        # Core synapse connectivity
        self.source_neuron_ids = self._create_aligned_array(np.uint32, alignment_bytes)
        self.target_neuron_ids = self._create_aligned_array(np.uint32, alignment_bytes)
        
        # Synaptic properties
        self.weights = self._create_aligned_array(np.float32, alignment_bytes)
        self.delays = self._create_aligned_array(np.uint8, alignment_bytes, default_value=1)
        self.types = self._create_aligned_array(np.uint8, alignment_bytes)  # Excitatory/Inhibitory
        self.conductances = self._create_aligned_array(np.float32, alignment_bytes, default_value=1.0)
        
        # Plasticity properties
        self.is_plastic_flags = self._create_aligned_array(np.bool_, alignment_bytes)
        self.plasticity_types = self._create_aligned_array(np.uint8, alignment_bytes)
        self.plasticity_coeffs = self._create_aligned_array(np.float32, alignment_bytes, default_value=1.0)
        self.decay_rates = self._create_aligned_array(np.float32, alignment_bytes, default_value=0.95)
        self.scaling_exponents = self._create_aligned_array(np.float32, alignment_bytes, default_value=1.0)
        
        # Synapse state tracking
        self.valid_mask = self._create_aligned_array(np.bool_, alignment_bytes)
        self.is_active = self._create_aligned_array(np.bool_, alignment_bytes)
        
        # Spatial indexing for fast lookup (NPU owns this)
        self.source_neuron_index: Dict[int, List[int]] = {}  # neuron_id -> list of synapse indices
        self.target_neuron_index: Dict[int, List[int]] = {}  # neuron_id -> list of synapse indices
        
        # Synapse ID mapping
        self.synapse_id_to_index: Dict[int, int] = {}
        self.index_to_synapse_id: Dict[int, int] = {}
        
        # Free index management
        self.free_indices: List[int] = []
        self.next_free_index = 0
        
        logger.info(f"SynapseArray initialized: {max_synapses:,} max synapses, {backend.value} backend")
        logger.info(f"SIMD config: {self.simd_config.vector_width} vector width, {self.simd_config.optimal_batch_size} optimal batch size")
    
    def _create_aligned_array(self, dtype: type, alignment_bytes: int, default_value: Any = None) -> np.ndarray:
        """Create cache-aligned numpy array for optimal SIMD performance."""
        # Same implementation as NeuronArray._create_aligned_array
        extra_bytes = alignment_bytes - 1
        
        raw_array = np.empty(self.aligned_capacity + extra_bytes // np.dtype(dtype).itemsize, dtype=dtype)
        
        raw_ptr = raw_array.ctypes.data
        aligned_ptr = (raw_ptr + alignment_bytes - 1) & ~(alignment_bytes - 1)
        offset = (aligned_ptr - raw_ptr) // np.dtype(dtype).itemsize
        
        aligned_array = raw_array[offset:offset + self.aligned_capacity]
        
        if default_value is not None:
            aligned_array.fill(default_value)
        else:
            aligned_array.fill(0)
            
        return aligned_array
    
    def get_optimal_batch_size(self) -> int:
        """Get optimal batch size for SIMD operations on this backend."""
        return self.simd_config.optimal_batch_size
    
    def get_max_batch_size(self) -> int:
        """Get maximum recommended batch size for this backend."""
        return self.simd_config.max_batch_size
    
    def add_synapses_batch(self, source_neuron_ids: List[int], target_neuron_ids: List[int],
                          weights: List[float], delays: List[int], 
                          plasticity_types: List[int], plasticity_coefficients: List[float]) -> int:
        """Add multiple synapses in batch with SIMD optimization.
        
        Args:
            source_neuron_ids: List of source neuron IDs
            target_neuron_ids: List of target neuron IDs
            weights: List of synaptic weights
            delays: List of synaptic delays
            plasticity_types: List of plasticity types
            plasticity_coefficients: List of plasticity coefficients
            
        Returns:
            Number of synapses actually added
        """
        count = len(source_neuron_ids)
        if count != len(target_neuron_ids) or count != len(weights):
            raise ValueError("All input lists must have same length")
            
        if self.count + count > self.max_synapses:
            raise ValueError(f"Cannot add {count} synapses: would exceed capacity")
        
        start_idx = self.count
        end_idx = start_idx + count
        
        # SIMD-optimized batch assignment
        self.source_neuron_ids[start_idx:end_idx] = np.array(source_neuron_ids, dtype=np.int32)
        self.target_neuron_ids[start_idx:end_idx] = np.array(target_neuron_ids, dtype=np.int32)
        self.weights[start_idx:end_idx] = np.array(weights, dtype=np.float32)
        self.delays[start_idx:end_idx] = np.array(delays, dtype=np.uint8)
        self.plasticity_types[start_idx:end_idx] = np.array(plasticity_types, dtype=np.uint8)
        self.plasticity_coeffs[start_idx:end_idx] = np.array(plasticity_coefficients, dtype=np.float32)
        
        # Mark as active
        self.is_active[start_idx:end_idx] = True
        self.valid_mask[start_idx:end_idx] = True
        
        # Update indexing structures
        for i, (source_id, target_id) in enumerate(zip(source_neuron_ids, target_neuron_ids)):
            synapse_idx = start_idx + i
            
            # Update source neuron index
            if source_id not in self.source_neuron_index:
                self.source_neuron_index[source_id] = []
            self.source_neuron_index[source_id].append(synapse_idx)
            
            # Update target neuron index
            if target_id not in self.target_neuron_index:
                self.target_neuron_index[target_id] = []
            self.target_neuron_index[target_id].append(synapse_idx)
        
        self.count += count
        self.synapse_count += count  # Keep both in sync
        return count
    
    def remove_synapses_batch(self, synapse_indices: List[int]) -> int:
        """Remove multiple synapses in batch with SIMD optimization.
        
        Args:
            synapse_indices: List of synapse indices to remove
            
        Returns:
            Number of synapses actually removed
        """
        removed_count = 0
        
        for idx in synapse_indices:
            if 0 <= idx < self.count and self.is_active[idx]:
                # Mark as inactive
                self.is_active[idx] = False
                removed_count += 1
        
        # Update count (simple approach - could be optimized with compaction)
        self.count -= removed_count
        self.synapse_count -= removed_count  # Keep both in sync
        
        return removed_count
    
    def update_weights_batch(self, synapse_indices: List[int], new_weights: List[float]) -> int:
        """Update synaptic weights in batch with SIMD optimization.
        
        Args:
            synapse_indices: List of synapse indices
            new_weights: List of new weight values
            
        Returns:
            Number of synapses actually updated
        """
        if len(synapse_indices) != len(new_weights):
            raise ValueError("synapse_indices and new_weights must have same length")
        
        valid_indices = []
        valid_weights = []
        
        for idx, weight in zip(synapse_indices, new_weights):
            if 0 <= idx < self.count and self.is_active[idx]:
                valid_indices.append(idx)
                valid_weights.append(weight)
        
        if not valid_indices:
            return 0
        
        # SIMD-optimized batch update
        indices_array = np.array(valid_indices, dtype=np.int32)
        weights_array = np.array(valid_weights, dtype=np.float32)
        
        self.weights[indices_array] = weights_array
        
        return len(valid_indices)
    
    # Individual synapse methods for ConnectomeManager compatibility
    def create_synapse(self, source_neuron_id: int, target_neuron_id: int, weight: float,
                      delay: int = 1, synapse_type: int = 0, plasticity_coeff: float = 0.0) -> bool:
        """Create a single synapse (compatibility method).
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID  
            weight: Synaptic weight
            delay: Synaptic delay (default 1)
            synapse_type: Synapse type (default 0 = excitatory)
            plasticity_coeff: Plasticity coefficient (default 0.0)
            
        Returns:
            True if synapse was created successfully
        """
        try:
            result = self.add_synapses_batch(
                source_neuron_ids=[source_neuron_id],
                target_neuron_ids=[target_neuron_id],
                weights=[weight],
                delays=[delay],
                plasticity_types=[synapse_type],
                plasticity_coefficients=[plasticity_coeff]
            )
            return result > 0
        except Exception:
            return False
    
    def delete_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
        """Delete a single synapse (compatibility method).
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID
            
        Returns:
            True if synapse was deleted successfully
        """
        # Find synapse index
        synapse_idx = self._find_synapse_index(source_neuron_id, target_neuron_id)
        if synapse_idx is None:
            return False
            
        try:
            result = self.remove_synapses_batch([synapse_idx])
            return result > 0
        except Exception:
            return False
    
    def has_synapse(self, source_neuron_id: int, target_neuron_id: int) -> bool:
        """Check if synapse exists (compatibility method).
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID
            
        Returns:
            True if synapse exists
        """
        return self._find_synapse_index(source_neuron_id, target_neuron_id) is not None
    
    def get_synapse_weight(self, source_neuron_id: int, target_neuron_id: int) -> Optional[float]:
        """Get synapse weight (compatibility method).
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID
            
        Returns:
            Synapse weight or None if not found
        """
        synapse_idx = self._find_synapse_index(source_neuron_id, target_neuron_id)
        if synapse_idx is None:
            return None
        return float(self.weights[synapse_idx])
    
    def update_synapse_weight(self, source_neuron_id: int, target_neuron_id: int, new_weight: float) -> bool:
        """Update synapse weight (compatibility method).
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID
            new_weight: New weight value
            
        Returns:
            True if weight was updated successfully
        """
        synapse_idx = self._find_synapse_index(source_neuron_id, target_neuron_id)
        if synapse_idx is None:
            return False
            
        try:
            result = self.update_weights_batch([synapse_idx], [new_weight])
            return result > 0
        except Exception:
            return False
    
    def get_outgoing_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get outgoing connections from a neuron (compatibility method).
        
        Args:
            neuron_id: Source neuron ID
            
        Returns:
            List of (target_neuron_id, weight) tuples
        """
        connections = []
        if neuron_id in self.source_neuron_index:
            for synapse_idx in self.source_neuron_index[neuron_id]:
                if synapse_idx < self.count and self.valid_mask[synapse_idx]:
                    target_id = int(self.target_neuron_ids[synapse_idx])
                    weight = float(self.weights[synapse_idx])
                    connections.append((target_id, weight))
        return connections
    
    def get_incoming_connections(self, neuron_id: int) -> List[Tuple[int, float]]:
        """Get incoming connections to a neuron (compatibility method).
        
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
    
    def batch_create_synapses(self, synapse_specs: List[Tuple[int, int, float]]) -> int:
        """Create multiple synapses from specs (compatibility method).
        
        Args:
            synapse_specs: List of (source_neuron_id, target_neuron_id, weight) tuples
            
        Returns:
            Number of synapses created
        """
        if not synapse_specs:
            return 0
            
        source_ids = []
        target_ids = []
        weights = []
        delays = []
        plasticity_types = []
        plasticity_coeffs = []
        
        for source_id, target_id, weight in synapse_specs:
            source_ids.append(source_id)
            target_ids.append(target_id)
            weights.append(weight)
            delays.append(1)  # Default delay
            plasticity_types.append(0)  # Default excitatory
            plasticity_coeffs.append(0.0)  # Default no plasticity
        
        return self.add_synapses_batch(
            source_neuron_ids=source_ids,
            target_neuron_ids=target_ids,
            weights=weights,
            delays=delays,
            plasticity_types=plasticity_types,
            plasticity_coefficients=plasticity_coeffs
        )
    
    def _find_synapse_index(self, source_neuron_id: int, target_neuron_id: int) -> Optional[int]:
        """Find synapse index by source and target neuron IDs.
        
        Args:
            source_neuron_id: Source neuron ID
            target_neuron_id: Target neuron ID
            
        Returns:
            Synapse index or None if not found
        """
        if source_neuron_id not in self.source_neuron_index:
            return None
            
        for synapse_idx in self.source_neuron_index[source_neuron_id]:
            if (synapse_idx < self.count and 
                self.valid_mask[synapse_idx] and
                self.target_neuron_ids[synapse_idx] == target_neuron_id):
                return synapse_idx
        
        return None


# Export the main data structures
__all__ = [
    'NeuronArray',
    'MemoryNeuronArray', 
    'SynapseArray',
    'BackendType',
    'SIMDConfig',
    'SIMDDetector',
    'MemoryPatternKey'
]
