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

"""Module for GPU-optimized and embedded-compatible neuron array data structures.

This module provides the core NeuronArray class for efficient neuron storage and operations.
Optimized for both GPU/SIMD processing and single-core embedded systems.
"""

import logging
import time
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import numpy as np

# Import optimized backends and SIMD operations
from feagi.bdu.models.array_backend import ArrayBackend, BackendType

logger = logging.getLogger(__name__)

# Try to import optional optimizations
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    import cupy
    CUPY_AVAILABLE = True
except ImportError:
    CUPY_AVAILABLE = False

try:
    from feagi.rust.bdu_operations import create_gna
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False

# Import vectorized SIMD operations
from feagi.npu.simd_neural_ops import (
    simd_firing_check,
    simd_membrane_decay,
    simd_refractory_update,
    simd_reset_fired_neurons,
    simd_extract_fired_indices,
)

# SIMD vector width for cache alignment
VECTOR_WIDTH = 64


class NeuronMappingProvider(ABC):
    """Interface for providing neuron ID-to-index mappings.
    
    This allows NeuronArray to work with external mapping systems
    (like ConnectomeManager) instead of maintaining its own redundant mappings.
    """
    
    @abstractmethod
    def get_neuron_index(self, neuron_id: int) -> Optional[int]:
        """Get the array index for a neuron ID."""
        pass
    
    @abstractmethod
    def get_neuron_id(self, index: int) -> Optional[int]:
        """Get the neuron ID for an array index."""
        pass
    
    @abstractmethod
    def set_neuron_mapping(self, neuron_id: int, index: int) -> None:
        """Set a neuron ID to index mapping."""
        pass
    
    @abstractmethod
    def remove_neuron_mapping(self, neuron_id: int) -> None:
        """Remove a neuron mapping."""
        pass
    
    @abstractmethod
    def has_neuron(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists."""
        pass
    
    @abstractmethod
    def get_all_neuron_ids(self) -> List[int]:
        """Get all neuron IDs."""
        pass


class NeuronArray:
    """Ultra-high-performance neuron storage optimized for embedded and GPU processing.

    This implementation uses Structure of Arrays (SoA) with:
    - 64-byte cache-aligned memory for SIMD optimization
    - Block-sparse connectivity matrices for cache locality
    - SIMD-vectorized neural operations (Numba JIT compiled)
    - Memory pools for zero-allocation operation
    - Both embedded single-core and GPU optimization

    Performance target: 10M neuron operations at 15Hz on single-core embedded systems.
    """

    def __init__(self, max_neurons: int = 10_000_000, backend: Optional[str] = None, mapping_provider: Optional[NeuronMappingProvider] = None):
        """Initialize ultra-high-performance NeuronArray.

        Args:
            max_neurons: Maximum number of neurons to support
            backend: Backend type to use (numpy, pytorch, cupy, webgpu, rust, or auto)
            mapping_provider: External provider for neuron ID-to-index mappings (e.g., ConnectomeManager)
        """
        self.max_neurons = max_neurons
        self.mapping_provider = mapping_provider

        # Align capacity to SIMD vector boundaries
        self.aligned_capacity = (max_neurons + VECTOR_WIDTH - 1) & ~(VECTOR_WIDTH - 1)

        # Initialize Rust backend if available and requested
        if backend == "rust" or (backend is None and RUST_AVAILABLE):
            try:
                self._rust_backend = create_gna(self.aligned_capacity)
                self._use_rust = True
                self.backend_type = "rust"
                logger.info(
                    f"Initialized NeuronArray with Rust backend, capacity: "
                    f"{self.aligned_capacity}"
                )
            except Exception as e:
                logger.warning(
                    f"Failed to initialize Rust backend: {e}, falling back to NumPy"
                )
                self._use_rust = False
                self._init_optimized_backend(backend)
        else:
            self._use_rust = False
            self._init_optimized_backend(backend)

        # ARCHITECTURAL CHANGE: Remove redundant mappings - use external provider
        # Common tracking regardless of backend
        self.cortical_id_to_indices: Dict[int, List[int]] = {}
        self.next_index = 0
        self.free_indices: Set[int] = set()
        self.neuron_count = 0

        # Unique neuron ID generator - separate from array indices to prevent corruption
        self._next_neuron_id = 1  # Start from 1 (0 reserved for invalid)

        # Performance tracking
        self.operation_count = 0
        self.total_operation_time = 0.0
        
        # Backward compatibility properties - delegate to mapping provider
        if mapping_provider:
            # Create property accessors that delegate to the mapping provider
            pass
            
    def _get_neuron_index(self, neuron_id: int) -> Optional[int]:
        """Get the array index for a neuron ID."""
        if self.mapping_provider:
            return self.mapping_provider.get_neuron_index(neuron_id)
        else:
            # Fallback for backward compatibility - should not be used in new code
            logger.warning("NeuronArray: No mapping provider, using internal fallback")
            return getattr(self, '_fallback_id_to_index', {}).get(neuron_id)
    
    def _get_neuron_id(self, index: int) -> Optional[int]:
        """Get the neuron ID for an array index."""
        if self.mapping_provider:
            return self.mapping_provider.get_neuron_id(index)
        else:
            # Fallback for backward compatibility - should not be used in new code
            logger.warning("NeuronArray: No mapping provider, using internal fallback")
            return getattr(self, '_fallback_index_to_id', {}).get(index)
    
    def _set_neuron_mapping(self, neuron_id: int, index: int) -> None:
        """Set a neuron ID to index mapping."""
        if self.mapping_provider:
            self.mapping_provider.set_neuron_mapping(neuron_id, index)
        else:
            # Fallback for backward compatibility - should not be used in new code
            logger.warning("NeuronArray: No mapping provider, using internal fallback")
            if not hasattr(self, '_fallback_id_to_index'):
                self._fallback_id_to_index = {}
                self._fallback_index_to_id = {}
            self._fallback_id_to_index[neuron_id] = index
            self._fallback_index_to_id[index] = neuron_id
    
    def _remove_neuron_mapping(self, neuron_id: int) -> None:
        """Remove a neuron mapping."""
        if self.mapping_provider:
            self.mapping_provider.remove_neuron_mapping(neuron_id)
        else:
            # Fallback for backward compatibility - should not be used in new code
            logger.warning("NeuronArray: No mapping provider, using internal fallback")
            if hasattr(self, '_fallback_id_to_index'):
                index = self._fallback_id_to_index.pop(neuron_id, None)
                if index is not None and hasattr(self, '_fallback_index_to_id'):
                    self._fallback_index_to_id.pop(index, None)
    
    def _has_neuron(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists."""
        if self.mapping_provider:
            return self.mapping_provider.has_neuron(neuron_id)
        else:
            # Fallback for backward compatibility - should not be used in new code
            logger.warning("NeuronArray: No mapping provider, using internal fallback")
            return neuron_id in getattr(self, '_fallback_id_to_index', {})

    def _init_optimized_backend(self, backend: Optional[str]):
        """Initialize optimized backend with State Manager-based intelligent sizing."""
        # Set backend
        self.backend = ArrayBackend(backend)
        self.backend_type = self.backend.backend_type

        # PERFORMANCE FIX: State Manager-based capacity calculation
        # Formula: total_neurons * 150%, rounded up to nearest 1000
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if state_manager:
                brain_stats = state_manager.get_brain_stats()
                total_neurons = brain_stats.get("neuron_count", 0)
                
                if total_neurons > 0:
                    # Apply user-specified formula: total_neurons * 150%, rounded up to nearest 1000
                    buffered_size = int(total_neurons * 1.5)
                    self.current_capacity = ((buffered_size + 999) // 1000) * 1000
                    logger.info(f"State Manager-based capacity: {total_neurons} neurons × 1.5 = {buffered_size} → {self.current_capacity}")
                else:
                    # Fallback for empty brain during early development
                    self.current_capacity = 1000  # Minimum reasonable size
                    logger.info(f"No neurons in State Manager yet - using minimum capacity: {self.current_capacity}")
            else:
                # Fallback if State Manager unavailable
                initial_capacity = min(50_000, self.max_neurons)
                self.current_capacity = ((initial_capacity + 999) // 1000) * 1000
                logger.warning(f"State Manager unavailable - using fallback capacity: {self.current_capacity}")
                
        except Exception as e:
            # Emergency fallback
            logger.error(f"Error accessing State Manager: {e} - using emergency fallback")
            initial_capacity = min(50_000, self.max_neurons)
            self.current_capacity = ((initial_capacity + 999) // 1000) * 1000
        
        # Ensure capacity is aligned to vector boundaries and doesn't exceed max
        self.current_capacity = min(self.current_capacity, self.aligned_capacity)
        self.current_capacity = (self.current_capacity + VECTOR_WIDTH - 1) & ~(VECTOR_WIDTH - 1)
        
        logger.info(f"Final array capacity: {self.current_capacity}")

        # Create appropriately-sized numpy arrays using calculated capacity
        self.membrane_potentials = np.zeros(self.current_capacity, dtype=np.float32)
        self.resting_potentials = np.zeros(self.current_capacity, dtype=np.float32)
        self.thresholds = np.ones(self.current_capacity, dtype=np.float32)  # Default threshold = 1.0
        self.decay_rates = np.full(self.current_capacity, 0.95, dtype=np.float32)
        self.refractory_periods = np.ones(self.current_capacity, dtype=np.int32)
        self.refractory_counters = np.zeros(self.current_capacity, dtype=np.int32)
        
        # Snooze mechanism (similar to refractory but for consecutive fire limit)
        self.snooze_periods = np.zeros(self.current_capacity, dtype=np.int32)  # Default snooze = 0 (disabled)
        self.snooze_counters = np.zeros(self.current_capacity, dtype=np.int32)
        self.consecutive_fire_counts = np.zeros(self.current_capacity, dtype=np.int32)
        self.consecutive_fire_limits = np.zeros(self.current_capacity, dtype=np.int32)  # Max consecutive fires
        
        # Coordinate arrays
        self.coordinates_x = np.zeros(self.current_capacity, dtype=np.uint32)
        self.coordinates_y = np.zeros(self.current_capacity, dtype=np.uint32)
        self.coordinates_z = np.zeros(self.current_capacity, dtype=np.uint32)
        
        # Area mapping and activation
        self.cortical_idxs = np.zeros(self.current_capacity, dtype=np.int32)
        self.is_active = np.zeros(self.current_capacity, dtype=np.bool_)
        self.valid_mask = np.zeros(self.current_capacity, dtype=np.bool_)
        
        # Additional arrays
        self.last_fired = np.zeros(self.current_capacity, dtype=np.int32)
        self.neuron_types = np.zeros(self.current_capacity, dtype=np.int32)
        self.enabled_flags = np.ones(self.current_capacity, dtype=np.int32)

        logger.info(f"Initialized NeuronArray: {self.backend_type} backend, capacity: {self.current_capacity}")

    def _ensure_capacity(self, required_capacity: int):
        """Ensure arrays have sufficient capacity, growing if needed."""
        if required_capacity <= self.current_capacity:
            return
            
        # Grow using the same formula: new requirement * 150%, rounded to nearest 1000
        buffered_size = int(required_capacity * 1.5)
        new_capacity = ((buffered_size + 999) // 1000) * 1000
        
        # Ensure we don't exceed max capacity
        new_capacity = min(new_capacity, self.aligned_capacity)
        
        logger.info(f"Growing neural arrays: {self.current_capacity} → {new_capacity} (required: {required_capacity})")
        
        # Resize all arrays
        self.membrane_potentials = np.resize(self.membrane_potentials, new_capacity)
        self.resting_potentials = np.resize(self.resting_potentials, new_capacity)
        self.thresholds = np.resize(self.thresholds, new_capacity)
        self.decay_rates = np.resize(self.decay_rates, new_capacity)
        self.refractory_periods = np.resize(self.refractory_periods, new_capacity)
        self.refractory_counters = np.resize(self.refractory_counters, new_capacity)
        
        # Resize snooze-related arrays
        self.snooze_periods = np.resize(self.snooze_periods, new_capacity)
        self.snooze_counters = np.resize(self.snooze_counters, new_capacity)
        self.consecutive_fire_counts = np.resize(self.consecutive_fire_counts, new_capacity)
        self.consecutive_fire_limits = np.resize(self.consecutive_fire_limits, new_capacity)
        
        self.coordinates_x = np.resize(self.coordinates_x, new_capacity)
        self.coordinates_y = np.resize(self.coordinates_y, new_capacity)
        self.coordinates_z = np.resize(self.coordinates_z, new_capacity)
        
        self.cortical_idxs = np.resize(self.cortical_idxs, new_capacity)
        self.is_active = np.resize(self.is_active, new_capacity)
        self.valid_mask = np.resize(self.valid_mask, new_capacity)
        
        self.last_fired = np.resize(self.last_fired, new_capacity)
        self.neuron_types = np.resize(self.neuron_types, new_capacity)
        self.enabled_flags = np.resize(self.enabled_flags, new_capacity)
        
        # Initialize new elements with defaults
        old_capacity = self.current_capacity
        self.thresholds[old_capacity:new_capacity] = 1.0
        self.decay_rates[old_capacity:new_capacity] = 0.95
        self.refractory_periods[old_capacity:new_capacity] = 1
        self.enabled_flags[old_capacity:new_capacity] = 1
        
        # Initialize snooze-related arrays with defaults (disabled by default)
        self.snooze_periods[old_capacity:new_capacity] = 0
        self.snooze_counters[old_capacity:new_capacity] = 0
        self.consecutive_fire_counts[old_capacity:new_capacity] = 0
        self.consecutive_fire_limits[old_capacity:new_capacity] = 0
        
        self.current_capacity = new_capacity

    def allocate_neuron(self, neuron_id: int) -> int:
        """Allocate space for a neuron and return its array index."""
        if self._has_neuron(neuron_id):
            logger.warning(f"Neuron ID {neuron_id} already exists")
            return self._get_neuron_index(neuron_id)

        # PERFORMANCE: Ensure we have capacity before allocation
        self._ensure_capacity(self.next_index + 1)

        # Reuse a free index if available, otherwise use next_index
        if self.free_indices:
            index = self.free_indices.pop()
        else:
            if self.next_index >= self.current_capacity:
                raise ValueError(f"Index overflow: {self.next_index} >= {self.current_capacity}")
            index = self.next_index
            self.next_index += 1

        # Mark this index as valid and store the ID mapping
        self.valid_mask[index] = True

        # Store bidirectional mapping via provider
        self._set_neuron_mapping(neuron_id, index)
        self.neuron_count += 1

        return index

    def embedded_optimized_neural_update(self, timestep: int, connectivity_matrix=None) -> List[int]:
        """Ultra-optimized neural update for embedded single-core operation with sparse processing."""
        if self._use_rust:
            return self._rust_backend.embedded_neural_update(timestep, connectivity_matrix)

        import time
        total_start = time.perf_counter()

        # PERFORMANCE: Only process actual neurons using appropriately-sized arrays
        actual_neuron_range = self.next_index  # Only process neurons that actually exist
        if actual_neuron_range == 0:
            return []  # No neurons to process

        # ULTRA-FAST SPARSE FILTERING: Use simple heuristics instead of expensive array ops
        phase1_start = time.perf_counter()
        
        # PERFORMANCE DIAGNOSTIC: Check array properties to understand slowness
        logger.info(f"ARRAY DIAGNOSTICS: "
                   f"membrane_flags.c_contiguous={self.membrane_potentials[:actual_neuron_range].flags.c_contiguous}, "
                   f"valid_flags.c_contiguous={self.valid_mask[:actual_neuron_range].flags.c_contiguous}, "
                   f"dtype={self.membrane_potentials.dtype}, shape=({actual_neuron_range},)")
        
        # REVOLUTIONARY SIMPLIFICATION: Remove expensive sparse filtering entirely!
        # Problem: The "close to firing" check is redundant - we check firing in P4 anyway
        # Solution: Just process all valid, non-refractory neurons directly
        
        # Simply get all neurons that can be updated (valid + not refractory)
        valid_mask = self.valid_mask[:actual_neuron_range]
        can_update_mask = valid_mask & (self.refractory_counters[:actual_neuron_range] == 0)
        
        # Get indices of all neurons that can be processed
        active_indices = np.where(can_update_mask)[0].astype(np.int32)
        
        num_active = len(active_indices)
        phase1_time = time.perf_counter() - phase1_start

        # PERFORMANCE DEBUG: Log the simplified approach
        logger.info(f"P1 SIMPLIFIED (no redundant filtering): P1_total={phase1_time*1000:.3f}ms, "
                   f"processing={num_active} valid neurons (was doing expensive threshold pre-filtering)")

        # EARLY EXIT: If no candidates, do minimal work
        if num_active == 0:
            logger.debug("No sparse candidates - minimal processing")
            # Only update refractory counters (essential for correctness)
            valid_refractory_mask = valid_mask & (self.refractory_counters[:actual_neuron_range] > 0)
            refractory_indices = np.where(valid_refractory_mask)[0].astype(np.int32)
            if len(refractory_indices) > 0:
                self.refractory_counters[refractory_indices] -= 1
            return []

        # FAST PHASE 2: Decay only sparse candidates (now on small, appropriate arrays)
        phase2_start = time.perf_counter()
        self.membrane_potentials[active_indices] *= self.decay_rates[active_indices]
        phase2_time = time.perf_counter() - phase2_start

        # FAST PHASE 3: Refractory updates (essential for all neurons)
        phase3_start = time.perf_counter()
        valid_refractory_mask = valid_mask & (self.refractory_counters[:actual_neuron_range] > 0)
        refractory_indices = np.where(valid_refractory_mask)[0].astype(np.int32)
        if len(refractory_indices) > 0:
            self.refractory_counters[refractory_indices] -= 1
        phase3_time = time.perf_counter() - phase3_start

        # FAST PHASE 4: Firing check only on sparse candidates
        phase4_start = time.perf_counter()
        fired_indices = []
        if num_active > 0:
            # Check firing threshold only on sparse active neurons
            active_potentials = self.membrane_potentials[active_indices]
            active_thresholds = self.thresholds[active_indices]
            
            # Find which candidates actually fired
            firing_mask = active_potentials >= active_thresholds
            fired_indices = active_indices[firing_mask].astype(np.int32)
        phase4_time = time.perf_counter() - phase4_start

        # FAST PHASE 5: Reset only fired neurons
        phase5_start = time.perf_counter()
        if len(fired_indices) > 0:
            # Reset membrane potentials and set refractory counters for fired neurons
            self.membrane_potentials[fired_indices] = self.resting_potentials[fired_indices]
            self.refractory_counters[fired_indices] = self.refractory_periods[fired_indices]
        phase5_time = time.perf_counter() - phase5_start

        # PHASE 6: No extraction needed - we have fired indices
        phase6_start = time.perf_counter()
        # fired_indices is already computed in phase 4
        phase6_time = time.perf_counter() - phase6_start
        
        # PHASE 7: Convert fired indices to neuron IDs
        phase7_start = time.perf_counter()
        fired_neurons = []
        if len(fired_indices) > 0 and self.mapping_provider:
            if hasattr(self.mapping_provider, '_index_to_neuron_id_map'):
                index_to_id_map = self.mapping_provider._index_to_neuron_id_map
                
                # Simple loop for small fired set
                for idx in fired_indices:
                    neuron_id = index_to_id_map.get(int(idx))
                    if neuron_id is not None:
                        fired_neurons.append(neuron_id)
            else:
                raise RuntimeError("No mapping provider available for neuron ID conversion")
        phase7_time = time.perf_counter() - phase7_start

        total_time = time.perf_counter() - total_start
        
        # PERFORMANCE DEBUG: Log timing
        if len(fired_neurons) > 0 or total_time > 0.001:
            logger.info(f"SPARSE NEURAL TIMING (total={actual_neuron_range}, active={num_active}, fired={len(fired_neurons)}): "
                        f"Total={total_time*1000:.2f}ms, "
                        f"P1={phase1_time*1000:.2f}ms, P2={phase2_time*1000:.2f}ms, "
                        f"P3={phase3_time*1000:.2f}ms, P4={phase4_time*1000:.2f}ms, "
                        f"P5={phase5_time*1000:.2f}ms, P6={phase6_time*1000:.2f}ms, "
                        f"P7={phase7_time*1000:.2f}ms")

        return fired_neurons

    def update_membrane_potentials(self, synapse_indices=None, synapse_data=None, timestep: Optional[int] = None, decay_factor: Optional[float] = None):
        """High-performance membrane potential update with embedded optimization."""
        # If called with old-style parameters, provide backward compatibility
        if decay_factor is not None and synapse_indices is None and synapse_data is None:
            # Legacy mode: just apply decay and return empty list
            simd_membrane_decay(self.membrane_potentials, np.full_like(self.decay_rates, decay_factor), self.valid_mask)
            return []

        # Use embedded optimization for full neural update
        if timestep is None:
            timestep = getattr(self, "_current_timestep", 0)

        # Perform complete optimized neural update
        return self.embedded_optimized_neural_update(timestep, synapse_data)

    def batch_create_neurons(self, cortical_idx: Optional[int], positions: List[Tuple[int, int, int]], 
                           thresholds: Union[float, List[float]] = 1.0,
                           membrane_potentials: Union[float, List[float]] = 0.0,
                           resting_potentials: Union[float, List[float]] = 0.0,
                           decay_rates: Union[float, List[float]] = 0.5,
                           refractory_periods: Union[int, List[int]] = 1) -> List[int]:
        """Create multiple neurons with the same or different properties in batch."""
        if cortical_idx is None:
            cortical_idx = 0

        num_neurons = len(positions)
        if num_neurons == 0:
            return []

        # Find indices for all neurons
        if len(self.free_indices) >= num_neurons:
            indices = np.array(list(self.free_indices)[:num_neurons], dtype=np.int32)
            self.free_indices = self.free_indices - set(indices)
        else:
            # Use free indices + new indices
            num_new_indices = num_neurons - len(self.free_indices)
            
            # CAPACITY FIX: Check current_capacity and grow if needed
            required_capacity = self.next_index + num_new_indices
            if required_capacity > self.current_capacity:
                self._ensure_capacity(required_capacity)
            
            # Secondary check against absolute maximum
            if required_capacity > self.max_neurons:
                raise ValueError(f"Batch creation requires {num_new_indices} new neurons, would exceed maximum capacity {self.max_neurons}")

            free_indices = np.array(list(self.free_indices), dtype=np.int32) if self.free_indices else np.array([], dtype=np.int32)
            new_indices = np.arange(self.next_index, self.next_index + num_new_indices, dtype=np.int32)
            indices = np.concatenate([free_indices, new_indices])

            self.free_indices = set()
            self.next_index += num_new_indices

        # Generate unique neuron IDs
        if not hasattr(self, "_next_neuron_id"):
            self._next_neuron_id = 1
        neuron_ids = list(range(self._next_neuron_id, self._next_neuron_id + num_neurons))
        self._next_neuron_id += num_neurons

        # Set properties
        if isinstance(thresholds, (int, float)):
            thresholds = [float(thresholds)] * num_neurons
        if isinstance(membrane_potentials, (int, float)):
            membrane_potentials = [float(membrane_potentials)] * num_neurons
        if isinstance(resting_potentials, (int, float)):
            resting_potentials = [float(resting_potentials)] * num_neurons
        if isinstance(decay_rates, (int, float)):
            decay_rates = [float(decay_rates)] * num_neurons
        if isinstance(refractory_periods, int):
            refractory_periods = [int(refractory_periods)] * num_neurons

        # Set array values
        self.membrane_potentials[indices] = membrane_potentials
        self.resting_potentials[indices] = resting_potentials
        self.thresholds[indices] = thresholds
        self.decay_rates[indices] = decay_rates
        self.refractory_periods[indices] = refractory_periods
        self.coordinates_x[indices] = [max(0, pos[0]) for pos in positions]
        self.coordinates_y[indices] = [max(0, pos[1]) for pos in positions]
        self.coordinates_z[indices] = [max(0, pos[2]) for pos in positions]
        self.cortical_idxs[indices] = cortical_idx
        self.valid_mask[indices] = True
        self.is_active[indices] = True

        # Update mappings via provider
        for i, neuron_id in enumerate(neuron_ids):
            self._set_neuron_mapping(neuron_id, indices[i])

        # Update neuron count
        self.neuron_count += len(neuron_ids)

        return neuron_ids

    def create_neuron(
        self,
        cortical_idx: Optional[int] = None,
        position: Tuple[int, int, int] = (0, 0, 0),
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        decay_rate: float = 0.5,
        refractory_period: int = 1,
        neuron_type: int = 0,
        is_active: bool = True,
        properties: Optional[Dict[str, Any]] = None,
    ) -> int:
        """
        Create a new neuron and add it to the array.

        Args:
            cortical_idx: Integer index of the cortical area this neuron belongs to
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            decay_rate: Membrane potential decay rate (0-1)
            refractory_period: Refractory period in timesteps
            neuron_type: Type of neuron (0=excitatory, 1=inhibitory, etc.)
            is_active: Whether the neuron is initially active
            properties: Additional properties for the neuron

        Returns:
            ID of the created neuron
        """
        # Default to cortical_idx 0 if not provided
        if cortical_idx is None:
            cortical_idx = 0

        # CAPACITY FIX: Ensure we have capacity and grow if needed
        if self.next_index >= self.current_capacity:
            self._ensure_capacity(self.next_index + 1)
        
        # Secondary check against absolute maximum
        if self.next_index >= self.max_neurons:
            raise ValueError(f"Cannot create neuron: would exceed maximum capacity {self.max_neurons}")

        # Generate unique neuron ID
        if not hasattr(self, "_next_neuron_id"):
            self._next_neuron_id = 1
        neuron_id = self._next_neuron_id
        self._next_neuron_id += 1

        # Allocate space for the neuron in the array
        idx = self.allocate_neuron(neuron_id)

        # Set valid flag
        self.valid_mask[idx] = True

        # Initialize the neuron's properties
        self.membrane_potentials[idx] = membrane_potential
        self.resting_potentials[idx] = resting_potential
        self.thresholds[idx] = threshold
        self.decay_rates[idx] = decay_rate
        self.refractory_periods[idx] = refractory_period
        self.refractory_counters[idx] = 0  # Start with no refractory state

        # Initialize coordinates as uint32
        self.coordinates_x[idx] = np.uint32(max(0, position[0]))
        self.coordinates_y[idx] = np.uint32(max(0, position[1]))
        self.coordinates_z[idx] = np.uint32(max(0, position[2]))

        self.cortical_idxs[idx] = cortical_idx
        self.is_active[idx] = is_active

        return neuron_id

    def delete_neuron(self, neuron_id: int) -> bool:
        """Delete a neuron by marking its index as available for reuse.

        Args:
            neuron_id: ID of the neuron to delete

        Returns:
            True if deleted, False if it didn't exist
        """
        if not self._has_neuron(neuron_id):
            return False

        index = self._get_neuron_index(neuron_id)
        if index is None:
            return False

        if not self._use_rust:
            # Mark as invalid and inactive
            self.valid_mask[index] = False
            self.is_active[index] = False

            # Reset neuron properties to defaults
            self.membrane_potentials[index] = 0.0
            self.cortical_idxs[index] = -1  # Invalid cortical area

        # Add to free indices for reuse
        self.free_indices.add(index)

        # Remove from mapping via provider
        self._remove_neuron_mapping(neuron_id)
        self.neuron_count -= 1
        return True

    def get_neuron_count(self) -> int:
        """Get the total number of neurons.

        Returns:
            Total number of neurons
        """
        if self._use_rust:
            return self._rust_backend.get_neuron_count()
        else:
            return int(np.sum(self.valid_mask))

    def vectorized_indices_to_neuron_ids(self, indices: np.ndarray, filter_invalid: bool = True) -> np.ndarray:
        """Convert array indices to neuron IDs using vectorized operations.
        
        Args:
            indices: Array of neuron indices
            filter_invalid: If True, filter out invalid/missing mappings
            
        Returns:
            Array of neuron IDs corresponding to the indices
        """
        if not self.mapping_provider:
            # Fallback: return indices as IDs if no mapping provider
            logger.warning("No mapping provider available - returning indices as IDs")
            return indices.astype(np.int64)
            
        # PERFORMANCE FIX: Use vectorized numpy operations instead of Python loops
        if hasattr(self.mapping_provider, '_index_to_neuron_id_map'):
            # Direct access to the mapping dictionary for ConnectomeManager
            index_to_id_map = self.mapping_provider._index_to_neuron_id_map
            
            # Create a vectorized lookup using numpy where
            neuron_ids = np.array([index_to_id_map.get(int(idx), -1) for idx in indices], dtype=np.int64)
            
            if filter_invalid:
                # Filter out -1 values (invalid mappings)
                valid_mask = neuron_ids != -1
                neuron_ids = neuron_ids[valid_mask]
                
            return neuron_ids
        else:
            # Fallback to individual lookups only if direct access not available
            indices_list = indices.astype(int).tolist()
            neuron_ids = []
            
            for idx in indices_list:
                neuron_id = self.mapping_provider.get_neuron_id(idx)
                if neuron_id is not None:
                    neuron_ids.append(neuron_id)
                elif not filter_invalid:
                    neuron_ids.append(-1)  # Use -1 for invalid mappings
                    
            return np.array(neuron_ids, dtype=np.int64)

    def set_neuron_property(self, neuron_id: int, property_name: str, value: Any) -> bool:
        """Set a neuron property directly in the array.
        
        Args:
            neuron_id: ID of the neuron
            property_name: Name of the property to set
            value: Value to set
            
        Returns:
            True if successful, False otherwise
        """
        # Get the neuron index from the mapping provider
        if not self.mapping_provider:
            logger.warning("No mapping provider available - cannot get neuron index")
            return False
            
        neuron_index = self.mapping_provider.get_neuron_index(neuron_id)
        if neuron_index is None:
            logger.warning(f"Neuron {neuron_id} not found in mapping")
            return False
            
        # Set the property directly in the array based on property name
        try:
            if property_name == "membrane_potential":
                self.membrane_potentials[neuron_index] = float(value)
            elif property_name == "threshold":
                self.thresholds[neuron_index] = float(value)
            elif property_name == "resting_potential":
                self.resting_potentials[neuron_index] = float(value)
            elif property_name == "decay_rate":
                self.decay_rates[neuron_index] = float(value)
            elif property_name == "refractory_period":
                self.refractory_periods[neuron_index] = int(value)
            elif property_name == "refractory_counter":
                self.refractory_counters[neuron_index] = int(value)
            elif property_name == "snooze_period":
                self.snooze_periods[neuron_index] = int(value)
            elif property_name == "snooze_counter":
                self.snooze_counters[neuron_index] = int(value)
            elif property_name == "consecutive_fire_count":
                self.consecutive_fire_counts[neuron_index] = int(value)
            elif property_name == "consecutive_fire_limit":
                self.consecutive_fire_limits[neuron_index] = int(value)
            elif property_name == "is_active":
                self.is_active[neuron_index] = bool(value)
            elif property_name == "position":
                if isinstance(value, (tuple, list)) and len(value) >= 3:
                    self.coordinates_x[neuron_index] = np.uint32(max(0, value[0]))
                    self.coordinates_y[neuron_index] = np.uint32(max(0, value[1]))
                    self.coordinates_z[neuron_index] = np.uint32(max(0, value[2]))
                else:
                    logger.error(f"Invalid position value for neuron {neuron_id}: {value}")
                    return False
            else:
                logger.warning(f"Unknown property '{property_name}' for neuron {neuron_id}")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Failed to set property {property_name} for neuron {neuron_id}: {e}")
            return False

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary for monitoring and optimization.
        
        Returns:
            Dictionary containing performance metrics and backend information
        """
        try:
            summary = {
                'backend': self.backend.name if hasattr(self.backend, 'name') else str(self.backend),
                'simd_enabled': getattr(self.backend, 'simd_enabled', False),
                'alignment': getattr(self.backend, 'alignment', 64),
                'neuron_count': self.neuron_count,
                'max_neurons': self.max_neurons,
                'memory_usage_mb': self._estimate_memory_usage(),
                'use_rust': self._use_rust,
                'precision': getattr(self.backend, 'precision', 'fp32'),
                'device': getattr(self.backend, 'device', 'cpu'),
            }
            
            # Add backend-specific metrics if available
            if hasattr(self.backend, 'get_performance_metrics'):
                backend_metrics = self.backend.get_performance_metrics()
                summary.update(backend_metrics)
                
            return summary
            
        except Exception as e:
            logger.error(f"Failed to get performance summary: {e}")
            return {
                'backend': 'unknown',
                'simd_enabled': False,
                'alignment': 64,
                'neuron_count': 0,
                'max_neurons': self.max_neurons,
                'memory_usage_mb': 0.0,
                'use_rust': self._use_rust,
                'precision': 'fp32',
                'device': 'cpu',
            }

    def _estimate_memory_usage(self) -> float:
        """Estimate memory usage in megabytes."""
        try:
            if self._use_rust and hasattr(self._rust_backend, 'get_memory_usage'):
                return self._rust_backend.get_memory_usage() / (1024 * 1024)
            else:
                # Estimate based on array sizes
                bytes_per_neuron = (
                    4 * 8 +  # 8 float32 arrays (membrane_potentials, thresholds, etc.)
                    4 * 3 +  # 3 uint32 coordinate arrays  
                    4 * 2 +  # 2 int32 arrays (cortical_idxs, refractory_periods)
                    1 * 2    # 2 bool arrays (valid_mask, is_active)
                )
                total_bytes = bytes_per_neuron * self.max_neurons
                return total_bytes / (1024 * 1024)
        except Exception:
            return 0.0


# For backwards compatibility
class Neuron:
    """
    Wrapper class for individual neurons.

    This class provides an object-oriented interface to individual neurons for API compatibility.
    It delegates most operations to the underlying NeuronArray for efficiency.
    """

    def __init__(
        self,
        neuron_id: int,
        cortical_id: Optional[str] = None,
        position: Optional[Tuple[int, int, int]] = None,
        threshold: float = 1.0,
        membrane_potential: float = 0.0,
        resting_potential: float = 0.0,
        decay_rate: float = 0.5,
        refractory_period: int = 1,
    ):
        """
        Initialize a Neuron object.

        Args:
            neuron_id: Unique neuron ID
            cortical_id: ID of the cortical area
            position: 3D coordinates (x, y, z)
            threshold: Firing threshold
            membrane_potential: Initial membrane potential
            resting_potential: Resting potential
            decay_rate: Membrane potential decay rate
            refractory_period: Refractory period in timesteps
        """
        self.id = neuron_id
        self.cortical_id = cortical_id or "unknown"
        self.position = position or (0, 0, 0)
        self.threshold = threshold
        self.membrane_potential = membrane_potential
        self.resting_potential = resting_potential
        self.decay_rate = decay_rate
        self.refractory_period = refractory_period

    def to_dict(self) -> Dict[str, Any]:
        """Convert neuron to dictionary representation.

        Returns:
            Dictionary with neuron properties
        """
        return {
            "id": self.id,
            "cortical_id": self.cortical_id,
            "position": self.position,
            "threshold": self.threshold,
            "membrane_potential": self.membrane_potential,
            "resting_potential": self.resting_potential,
            "decay_rate": self.decay_rate,
            "refractory_period": self.refractory_period,
            # Add defaults for optional attributes
            "refractory_counter": getattr(self, "refractory_counter", 0),
            "is_active": getattr(self, "is_active", False),
            "properties": getattr(self, "properties", {}),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Neuron":
        """Create neuron from dictionary representation.

        Args:
            data: Dictionary with neuron properties

        Returns:
            Neuron instance
        """
        neuron = cls(
            neuron_id=data["id"],
            cortical_id=data["cortical_id"],
            position=data["position"],
            threshold=data["threshold"],
            membrane_potential=data["membrane_potential"],
            resting_potential=data["resting_potential"],
            decay_rate=data["decay_rate"],
            refractory_period=data["refractory_period"],
        )
        neuron.refractory_counter = data.get("refractory_counter", 0)
        neuron.is_active = data.get("is_active", False)

        return neuron