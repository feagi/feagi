"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import numpy as np

# Import optimized backends and SIMD operations
from feagi.bdu.models.array_backend import ArrayBackend

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

# Enable efficient SIMD vectorization (16 values per 512-bit vector)
VECTOR_WIDTH = 16

# MEMORY OPTIMIZATION: Invalid cortical area index for uint16 optimization
INVALID_CORTICAL_IDX = 65535  # Max value for uint16, used instead of -1

# Rust extension availability check
try:
    from feagi.rust.neuron_array import create_gna

    RUST_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    RUST_AVAILABLE = False

# Import vectorized SIMD operations
try:
    from feagi.npu.simd_neural_ops import (
        simd_firing_check,
        simd_membrane_decay,
        simd_refractory_update,
    )

    SIMD_AVAILABLE = True
except ImportError:
    # Fallback implementations
    SIMD_AVAILABLE = False

    def simd_membrane_decay(potentials, decay_rates, mask):
        potentials[mask] *= decay_rates[mask]

    def simd_refractory_update(counters, mask):
        active_mask = mask & (counters > 0)
        counters[active_mask] -= 1

    def simd_firing_check(potentials, thresholds, can_fire_mask):
        return (potentials >= thresholds) & can_fire_mask


# Enhanced SIMD firing function with excitability support
def simd_firing_check_with_excitability(
    potentials, thresholds, can_fire_mask, excitability, rng=None
):
    """SIMD-optimized firing check with probabilistic excitability.

    Optimizations:
    1. Fast path when all excitability >= 0.999 (bypasses random generation entirely)
    2. Vectorized random number generation for excitability < 0.999
    3. Cache-aligned operations for SIMD efficiency

    Args:
        potentials: Membrane potentials array
        thresholds: Firing thresholds array
        can_fire_mask: Mask for neurons that can fire (not in refractory)
        excitability: Excitability values (0.0 to 1.0)

    Returns:
        Boolean mask of neurons that should fire
    """
    #  PERFORMANCE: Fast path - if ALL excitability values are >= 0.999, use
    #  original function
    if np.all(excitability >= 0.999):
        return simd_firing_check(potentials, thresholds, can_fire_mask)

    # PHASE 1: Standard threshold check
    threshold_met = (potentials >= thresholds) & can_fire_mask

    # PHASE 2: Early exit if no neurons met threshold
    if not np.any(threshold_met):
        return threshold_met

    # PHASE 3: Find neurons that need probabilistic check
    threshold_indices = np.where(threshold_met)[0]
    relevant_excitability = excitability[threshold_indices]

    # PHASE 4: Fast path for neurons with excitability >= 0.999
    deterministic_fire = relevant_excitability >= 0.999
    probabilistic_indices = threshold_indices[~deterministic_fire]

    # PHASE 5: Initialize result with deterministic fires
    final_fired_mask = np.zeros_like(threshold_met, dtype=bool)
    final_fired_mask[threshold_indices[deterministic_fire]] = True

    # PHASE 6: Handle probabilistic firing only for neurons that need it
    if len(probabilistic_indices) > 0:
        prob_excitability = excitability[probabilistic_indices]
        if rng is None:
            rng = np.random.default_rng(
                0
            )  # @architecture:acceptable - test isolation fallback
        random_values = rng.random(len(probabilistic_indices))
        probabilistic_fire = random_values < prob_excitability
        final_fired_mask[probabilistic_indices] = probabilistic_fire

    return final_fired_mask


# Fallback SIMD functions for when numba/specialized SIMD isn't available
try:
    from numba import jit

    NUMBA_AVAILABLE = True
    # Deterministic RNG for numba fallback code paths (module-level)
    _NUMBA_FALLBACK_RNG = np.random.default_rng(0)

    @jit(nopython=True)
    def simd_firing_check_with_excitability_numba(
        potentials, thresholds, can_fire_mask, excitability
    ):
        """Numba-optimized version of excitability firing check."""
        # PERFORMANCE: Fast path - if ALL excitability values are >= 0.999
        all_deterministic = True
        for i in range(len(excitability)):
            if excitability[i] < 0.999:
                all_deterministic = False
                break

        if all_deterministic:
            return simd_firing_check(potentials, thresholds, can_fire_mask)

        threshold_met = (potentials >= thresholds) & can_fire_mask

        # Fast path for all excitability >= 0.999
        needs_probability = np.any((excitability < 0.999) & threshold_met)
        if not needs_probability:
            return threshold_met

        # Probabilistic firing
        fired_mask = np.zeros_like(threshold_met, dtype=np.bool_)
        for i in range(len(threshold_met)):
            if threshold_met[i]:
                if excitability[i] >= 0.999:
                    fired_mask[i] = True
                else:
                    #  Use a simple LCG for determinism inside numba-compiled
                    #  function
                    #  Note: Numba cannot use numpy Generator; this is a
                    #  deterministic stub.
                    #  Parameters from Numerical Recipes: a=1664525,
                    #  c=1013904223, m=2**32
                    # Seed derived from index for reproducibility per call
                    state = (1664525 * (i + 1) + 1013904223) % 4294967296
                    rand = state / 4294967296.0
                    fired_mask[i] = rand < excitability[i]

        return fired_mask

except ImportError:
    NUMBA_AVAILABLE = False


class NeuronMappingProvider(ABC):
    """Interface for providing neuron ID-to-index mappings.

    This allows NeuronArray to work with external mapping systems (like
    ConnectomeManager) instead of maintaining its own redundant mappings.
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
    """Ultra-high-performance neuron storage optimized for embedded and GPU
    processing.

    This implementation uses Structure of Arrays (SoA) with:
    - 64-byte cache-aligned memory for SIMD optimization
    - Block-sparse connectivity matrices for cache locality
    - SIMD-vectorized neural operations (Numba JIT compiled)
    - Memory pools for zero-allocation operation
    - Both embedded single-core and GPU optimization

    Performance target: 10M neuron operations at 15Hz on single-core embedded systems.
    """

    def __init__(
        self,
        max_neurons: int = 10_000_000,
        backend: Optional[str] = None,
        mapping_provider: Optional[NeuronMappingProvider] = None,
    ):
        """Initialize ultra-high-performance NeuronArray.

        Args:
            max_neurons: Maximum number of neurons to support
            backend: Backend type to use (numpy, pytorch, cupy, webgpu, rust, or auto)
            mapping_provider: External provider for neuron ID-to-index mappings (e.g., ConnectomeManager)
        """
        self.max_neurons = max_neurons
        self.mapping_provider = mapping_provider

        # Debug flags for diagnostics
        self._debug_refractory = (
            False  # Controls refractory period debug logging
        )

        # Deterministic PRNG for probabilistic firing (@cursor:critical-path)
        try:
            from feagi.config.toml_loader import load_feagi_config

            cfg = load_feagi_config()
            seed = int(cfg.get("npu", {}).get("rng_seed", 0))
        except Exception:
            seed = 0  # @architecture:acceptable - test isolation / default
        self._rng = np.random.default_rng(seed)

        # Align capacity to SIMD vector boundaries
        self.aligned_capacity = (max_neurons + VECTOR_WIDTH - 1) & ~(
            VECTOR_WIDTH - 1
        )

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

        #  ARCHITECTURAL CHANGE: Remove redundant mappings - use external
        #  provider
        # Common tracking regardless of backend
        self.cortical_id_to_indices: Dict[int, List[int]] = {}
        self.next_index = 0
        self.free_indices: Set[int] = set()
        self.neuron_count = 0

        #  Unique neuron ID generator - separate from array indices to prevent
        #  corruption
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
            #  Fallback for backward compatibility - should not be used in new
            #  code
            logger.warning(
                "NeuronArray: No mapping provider, using internal fallback"
            )
            return getattr(self, "_fallback_id_to_index", {}).get(neuron_id)

    def _get_neuron_id(self, index: int) -> Optional[int]:
        """Get the neuron ID for an array index."""
        if self.mapping_provider:
            return self.mapping_provider.get_neuron_id(index)
        else:
            #  Fallback for backward compatibility - should not be used in new
            #  code
            logger.warning(
                "NeuronArray: No mapping provider, using internal fallback"
            )
            return getattr(self, "_fallback_index_to_id", {}).get(index)

    def _set_neuron_mapping(self, neuron_id: int, index: int) -> None:
        """Set a neuron ID to index mapping."""
        if self.mapping_provider:
            self.mapping_provider.set_neuron_mapping(neuron_id, index)
        else:
            #  Fallback for backward compatibility - should not be used in new
            #  code
            logger.warning(
                "NeuronArray: No mapping provider, using internal fallback"
            )
            if not hasattr(self, "_fallback_id_to_index"):
                self._fallback_id_to_index = {}
                self._fallback_index_to_id = {}
            self._fallback_id_to_index[neuron_id] = index
            self._fallback_index_to_id[index] = neuron_id

    def _remove_neuron_mapping(self, neuron_id: int) -> None:
        """Remove a neuron mapping."""
        if self.mapping_provider:
            self.mapping_provider.remove_neuron_mapping(neuron_id)
        else:
            #  Fallback for backward compatibility - should not be used in new
            #  code
            logger.warning(
                "NeuronArray: No mapping provider, using internal fallback"
            )
            if hasattr(self, "_fallback_id_to_index"):
                index = self._fallback_id_to_index.pop(neuron_id, None)
                if index is not None and hasattr(
                    self, "_fallback_index_to_id"
                ):
                    self._fallback_index_to_id.pop(index, None)

    def _has_neuron(self, neuron_id: int) -> bool:
        """Check if a neuron ID exists."""
        if self.mapping_provider:
            return self.mapping_provider.has_neuron(neuron_id)
        else:
            #  Fallback for backward compatibility - should not be used in new
            #  code
            logger.warning(
                "NeuronArray: No mapping provider, using internal fallback"
            )
            return neuron_id in getattr(self, "_fallback_id_to_index", {})

    def _init_optimized_backend(self, backend: Optional[str]):
        """Initialize optimized backend with cache-aligned arrays."""
        # Set backend
        self.backend = ArrayBackend(backend)
        self.backend_type = self.backend.backend_type

        # Create basic numpy arrays for now - simplified implementation
        self.membrane_potentials = np.zeros(
            self.aligned_capacity, dtype=np.float32
        )
        self.resting_potentials = np.zeros(
            self.aligned_capacity, dtype=np.float32
        )

        # Firing-related arrays (grouped for cache locality)
        self.thresholds = np.ones(
            self.aligned_capacity, dtype=np.float32
        )  # Default threshold = 1.0
        self.excitability = np.ones(
            self.aligned_capacity, dtype=np.float32
        )  # Default excitability = 1.0 (100%)
        self.decay_rates = np.full(
            self.aligned_capacity, 0.95, dtype=np.float32
        )

        self.refractory_periods = np.ones(
            self.aligned_capacity, dtype=np.int32
        )
        self.refractory_counters = np.zeros(
            self.aligned_capacity, dtype=np.int32
        )

        #  MEMORY OPTIMIZATION: Coordinate arrays using uint16 (supports
        #  0-65,535 per dimension)
        self.coordinates_x = np.zeros(self.aligned_capacity, dtype=np.uint16)
        self.coordinates_y = np.zeros(self.aligned_capacity, dtype=np.uint16)
        self.coordinates_z = np.zeros(self.aligned_capacity, dtype=np.uint16)

        # Area mapping and activation
        self.cortical_idxs = np.zeros(self.aligned_capacity, dtype=np.uint16)
        self.is_active = np.zeros(self.aligned_capacity, dtype=np.bool_)
        self.valid_mask = np.zeros(self.aligned_capacity, dtype=np.bool_)

        # Additional arrays
        self.last_fired = np.zeros(self.aligned_capacity, dtype=np.int32)
        self.neuron_types = np.zeros(self.aligned_capacity, dtype=np.int32)
        self.enabled_flags = np.ones(self.aligned_capacity, dtype=np.int32)

        # PERFORMANCE: Per-cortical-area tracking of probabilistic firing needs
        # Maps cortical_idx -> bool (True if area needs probabilistic firing)
        self._area_probabilistic_firing = {}

        logger.info(
            f"Initialized NeuronArray: {self.backend_type} backend, capacity: {self.aligned_capacity}"
        )

    def set_cortical_area_excitability(
        self,
        cortical_idx: int,
        start_idx: int,
        end_idx: int,
        excitability: float,
    ) -> None:
        """Set excitability for a cortical area and update performance
        tracking.

        Args:
            cortical_idx: Cortical area index
            start_idx: Start neuron index
            end_idx: End neuron index (exclusive)
            excitability: Excitability value (0.0 to 1.0)
        """
        # Clamp excitability to valid range
        excitability = max(0.0, min(1.0, excitability))

        # Set excitability values for the cortical area
        self.excitability[start_idx:end_idx] = excitability

        # Update per-area probabilistic firing tracking
        self._area_probabilistic_firing[cortical_idx] = excitability < 0.999

    def set_excitability_range(
        self, start_idx: int, end_idx: int, excitability: float
    ) -> None:
        """
        DEPRECATED: Use set_cortical_area_excitability instead for proper area tracking.

        This method is kept for backward compatibility but doesn't update area tracking.
        """
        # Clamp excitability to valid range
        excitability = max(0.0, min(1.0, excitability))

        # Set excitability values
        self.excitability[start_idx:end_idx] = excitability

    def get_probabilistic_areas(self) -> set:
        """Get set of cortical area indices that use probabilistic firing."""
        return {
            idx
            for idx, needs_prob in self._area_probabilistic_firing.items()
            if needs_prob
        }

    def allocate_neuron(self, neuron_id: int) -> int:
        """Allocate space for a neuron and return its array index."""
        if self._has_neuron(neuron_id):
            logger.warning(f"Neuron ID {neuron_id} already exists")
            return self._get_neuron_index(neuron_id)

        # Reuse a free index if available, otherwise use next_index
        if self.free_indices:
            index = self.free_indices.pop()
        else:
            if self.next_index >= self.aligned_capacity:
                raise ValueError(
                    f"Maximum neuron capacity ({self.aligned_capacity}) reached"
                )
            index = self.next_index
            self.next_index += 1

        # Mark this index as valid and store the ID mapping
        self.valid_mask[index] = True

        # Store bidirectional mapping via provider
        self._set_neuron_mapping(neuron_id, index)
        self.neuron_count += 1

        return index

    def embedded_optimized_neural_update(
        self, timestep: int, connectivity_matrix=None
    ) -> List[int]:
        """Ultra-optimized neural update for embedded single-core operation."""
        # CRITICAL DEBUG: Test if this method is being called at all
        logger.info(f"[COORD-DEBUG] *** NEURAL UPDATE CALLED *** timestep={timestep}")
        print(f"[COORD-DEBUG] *** NEURAL UPDATE CALLED *** timestep={timestep}")  # Force output
        if self._use_rust:
            return self._rust_backend.embedded_neural_update(
                timestep, connectivity_matrix
            )

        # PHASE 1: Create properly filtered mask
        valid_neurons = self.valid_mask
        can_update_mask = valid_neurons & (self.refractory_counters == 0)

        # PHASE 2: Membrane potential decay (SIMD-optimized)
        simd_membrane_decay(
            self.membrane_potentials, self.decay_rates, can_update_mask
        )

        # PHASE 3: Refractory period updates (SIMD-optimized)
        simd_refractory_update(self.refractory_counters, valid_neurons)

        #  PHASE 4: Enhanced threshold checking and probabilistic firing
        #  (SIMD-optimized)
        #  PERFORMANCE OPTIMIZATION: Check if any cortical areas need
        #  probabilistic firing
        probabilistic_areas = self.get_probabilistic_areas()

        if not probabilistic_areas:
            #  Fast path: No areas use probabilistic firing - use original
            #  deterministic function
            fired_mask = simd_firing_check(
                self.membrane_potentials, self.thresholds, can_update_mask
            )
        else:
            # Mixed path: Some areas need probabilistic firing, some don't
            #  Use enhanced function but it will automatically handle
            #  deterministic neurons efficiently
            if NUMBA_AVAILABLE:
                fired_mask = simd_firing_check_with_excitability_numba(
                    self.membrane_potentials,
                    self.thresholds,
                    can_update_mask,
                    self.excitability,
                )
            else:
                fired_mask = simd_firing_check_with_excitability(
                    self.membrane_potentials,
                    self.thresholds,
                    can_update_mask,
                    self.excitability,
                    rng=self._rng,
                )

        # PHASE 5: Process firing consequences (CRITICAL FIX!)
        fired_indices = np.where(fired_mask)[0]
        if len(fired_indices) > 0:
            # COORDINATE DEBUG: Log fired neurons and their IDs
            fired_neuron_ids = []
            if hasattr(self, 'mapping_provider') and self.mapping_provider:
                # Convert indices to neuron IDs
                for idx in fired_indices:
                    neuron_id = self.mapping_provider.get_neuron_id(idx)
                    if neuron_id is not None:
                        fired_neuron_ids.append(neuron_id)
                        # Log specific neurons we're tracking
                        if neuron_id in [4495, 4496]:
                            logger.info(f"[COORD-DEBUG] FIRING DETECTED: Neuron {neuron_id} (index {idx}) fired - membrane_potential={self.membrane_potentials[idx]}, threshold={self.thresholds[idx]}")
                
                if fired_neuron_ids:
                    logger.info(f"[COORD-DEBUG] Neural update detected {len(fired_neuron_ids)} firing neurons: {fired_neuron_ids}")
            
            # DEBUG LOGGING FOR REFRACTORY PERIOD BUG INVESTIGATION
            # Check cortical areas of fired neurons (needed for debug analysis)
            cortical_areas = {}
            for idx in fired_indices:
                cortical_idx = (
                    self.cortical_idxs[idx]
                    if idx < len(self.cortical_idxs)
                    else "N/A"
                )
                if cortical_idx not in cortical_areas:
                    cortical_areas[cortical_idx] = []
                cortical_areas[cortical_idx].append(idx)

            if self._debug_refractory:
                print(
                    f"\n🔥 NEURON DEBUG: Timestep {timestep} - {len(fired_indices)} neurons firing"
                )
                print(f"   Fired indices: {fired_indices}")
                print(f"   Fired by cortical area: {cortical_areas}")

                # Show refractory counters BEFORE setting
                print(
                    f"   Refractory counters BEFORE: {self.refractory_counters[fired_indices]}"
                )
                print(
                    f"   Refractory periods to set: {self.refractory_periods[fired_indices]}"
                )

            # Reset membrane potentials of fired neurons to resting potential
            self.membrane_potentials[fired_indices] = self.resting_potentials[
                fired_indices
            ]

            #  Set refractory counters to refractory periods (CRITICAL FOR
            #  REFRACTORY PERIOD ENFORCEMENT!)
            self.refractory_counters[fired_indices] = self.refractory_periods[
                fired_indices
            ]

            if self._debug_refractory:
                # Show refractory counters AFTER setting
                print(
                    f"   Refractory counters AFTER: {self.refractory_counters[fired_indices]}"
                )

                #  CRITICAL DEBUG: Check if OTHER neurons in same cortical
                #  areas are affected
                for cortical_idx, fired_in_area in cortical_areas.items():
                    if cortical_idx == "N/A":
                        continue

                    # Find ALL neurons in this cortical area
                    all_in_area = np.where(self.cortical_idxs == cortical_idx)[
                        0
                    ]
                    non_fired_in_area = [
                        idx for idx in all_in_area if idx not in fired_indices
                    ]

                    if len(non_fired_in_area) > 0:
                        print(
                            f"   🧪 Area {cortical_idx}: {len(fired_in_area)} fired, {len(non_fired_in_area)} didn't fire"
                        )
                        print(
                            f"      Non-fired indices: {non_fired_in_area[:5]}..."
                        )  # Show first 5
                        print(
                            f"      Non-fired refractory counters: {self.refractory_counters[non_fired_in_area[:5]]}"
                        )

                        #  BUG DETECTION: Check if non-fired neurons
                        #  incorrectly became refractory
                        incorrect_refractory = (
                            self.refractory_counters[non_fired_in_area] > 0
                        )
                        if np.any(incorrect_refractory):
                            buggy_indices = np.array(non_fired_in_area)[
                                incorrect_refractory
                            ]
                            print(
                                f"   🚨 BUG DETECTED: Non-fired neurons became refractory: {buggy_indices}"
                            )
                            print(
                                f"      Their refractory counters: {self.refractory_counters[buggy_indices]}"
                            )
                            print(
                                "      This confirms AREA-WIDE REFRACTORY SUPPRESSION!"
                            )

                            #  Check memory addresses to see if they're sharing
                            #  memory
                            fired_addr = id(
                                self.refractory_counters[fired_indices[0]]
                                if fired_indices
                                else None
                            )
                            buggy_addr = id(
                                self.refractory_counters[buggy_indices[0]]
                                if len(buggy_indices) > 0
                                else None
                            )
                            print(
                                f"      Memory check - Fired neuron addr: {fired_addr}, Buggy neuron addr: {buggy_addr}"
                            )

                            if fired_addr == buggy_addr:
                                print(
                                    "      🎯 ROOT CAUSE: SHARED MEMORY - neurons sharing same refractory counter!"
                                )
                            else:
                                print(
                                    "      🎯 ROOT CAUSE: INDEXING BUG - incorrect array slicing/indexing!"
                                )

        # PHASE 6: Extract fired neuron IDs
        fired_neurons = []
        for idx in fired_indices:
            neuron_id = self._get_neuron_id(idx)
            if neuron_id is not None:
                fired_neurons.append(neuron_id)

        return fired_neurons

    def enable_refractory_debug(self):
        """Enable debug logging for refractory period behavior."""
        self._debug_refractory = True
        print("🔬 Refractory period debug logging ENABLED")

    def disable_refractory_debug(self):
        """Disable debug logging for refractory period behavior."""
        self._debug_refractory = False
        print("🔇 Refractory period debug logging DISABLED")

    def update_membrane_potentials(
        self,
        synapse_indices=None,
        synapse_data=None,
        timestep: Optional[int] = None,
        decay_factor: Optional[float] = None,
    ):
        """High-performance membrane potential update with embedded
        optimization."""
        # If called with old-style parameters, provide backward compatibility
        if (
            decay_factor is not None
            and synapse_indices is None
            and synapse_data is None
        ):
            # Legacy mode: just apply decay and return empty list
            simd_membrane_decay(
                self.membrane_potentials,
                np.full_like(self.decay_rates, decay_factor),
                self.valid_mask,
            )
            return []

        # Use embedded optimization for full neural update
        if timestep is None:
            timestep = getattr(self, "_current_timestep", 0)

        # Perform complete optimized neural update
        return self.embedded_optimized_neural_update(timestep, synapse_data)

    def batch_create_neurons(
        self,
        cortical_idx: Optional[int],
        positions: List[Tuple[int, int, int]],
        thresholds: Union[float, List[float]] = 1.0,
        membrane_potentials: Union[float, List[float]] = 0.0,
        resting_potentials: Union[float, List[float]] = 0.0,
        decay_rates: Union[float, List[float]] = 0.5,
        refractory_periods: Union[int, List[int]] = 1,
    ) -> List[int]:
        """Create multiple neurons with the same or different properties in
        batch."""
        if cortical_idx is None:
            cortical_idx = 0

        num_neurons = len(positions)
        if num_neurons == 0:
            return []

        # Find indices for all neurons
        if len(self.free_indices) >= num_neurons:
            indices = np.array(
                list(self.free_indices)[:num_neurons], dtype=np.int32
            )
            self.free_indices = self.free_indices - set(indices)
        else:
            # Use free indices + new indices
            num_new_indices = num_neurons - len(self.free_indices)
            if self.next_index + num_new_indices > self.max_neurons:
                raise ValueError(
                    f"Maximum number of neurons ({self.max_neurons}) exceeded"
                )

            free_indices = (
                np.array(list(self.free_indices), dtype=np.int32)
                if self.free_indices
                else np.array([], dtype=np.int32)
            )
            new_indices = np.arange(
                self.next_index,
                self.next_index + num_new_indices,
                dtype=np.int32,
            )
            indices = np.concatenate([free_indices, new_indices])

            self.free_indices = set()
            self.next_index += num_new_indices

        # Generate unique neuron IDs
        if not hasattr(self, "_next_neuron_id"):
            self._next_neuron_id = 1
        neuron_ids = list(
            range(self._next_neuron_id, self._next_neuron_id + num_neurons)
        )
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
        """Create a new neuron and add it to the array.

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

        # Make sure we have capacity
        if self.next_index >= self.max_neurons:
            raise ValueError(
                f"Maximum number of neurons ({self.max_neurons}) exceeded"
            )

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

        # Set coordinates with uint16 clamping
        self.coordinates_x[idx] = np.uint16(max(0, min(65535, position[0])))
        self.coordinates_y[idx] = np.uint16(max(0, min(65535, position[1])))
        self.coordinates_z[idx] = np.uint16(max(0, min(65535, position[2])))

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
            self.cortical_idxs[index] = (
                INVALID_CORTICAL_IDX  # Invalid cortical area
            )

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

    def vectorized_indices_to_neuron_ids(
        self, indices: np.ndarray, filter_invalid: bool = True
    ) -> np.ndarray:
        """Convert array indices to neuron IDs using vectorized operations.

        Args:
            indices: Array of neuron indices
            filter_invalid: If True, filter out invalid/missing mappings

        Returns:
            Array of neuron IDs corresponding to the indices
        """
        if not self.mapping_provider:
            # Fallback: return indices as IDs if no mapping provider
            logger.warning(
                "No mapping provider available - returning indices as IDs"
            )
            return indices.astype(np.int64)

        # Convert indices to list for individual lookups
        indices_list = indices.astype(int).tolist()
        neuron_ids = []

        for idx in indices_list:
            neuron_id = self.mapping_provider.get_neuron_id(idx)
            if neuron_id is not None:
                neuron_ids.append(neuron_id)
            elif not filter_invalid:
                neuron_ids.append(-1)  # Use -1 for invalid mappings

        return np.array(neuron_ids, dtype=np.int64)

    def set_neuron_property(
        self, neuron_id: int, property_name: str, value: Any
    ) -> bool:
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
            logger.warning(
                "No mapping provider available - cannot get neuron index"
            )
            return False

        neuron_index = self.mapping_provider.get_neuron_index(neuron_id)
        if neuron_index is None:
            logger.warning(f"Neuron {neuron_id} not found in mapping")
            return False

        # Set the property directly in the array based on property name
        try:
            if property_name == "membrane_potential":
                # COORDINATE DEBUG: Log membrane potential changes for neurons 4495 and 4496
                if neuron_id in [4495, 4496]:
                    import traceback
                    logger.info(f"[COORD-DEBUG] *** MEMBRANE POTENTIAL CHANGE *** Neuron {neuron_id}: {self.membrane_potentials[neuron_index]} → {value}")
                    logger.info(f"[COORD-DEBUG] Call stack: {traceback.format_stack()[-3:-1]}")
                    print(f"[COORD-DEBUG] *** MEMBRANE POTENTIAL CHANGE *** Neuron {neuron_id}: {self.membrane_potentials[neuron_index]} → {value}")  # Force output
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
            elif property_name == "is_active":
                self.is_active[neuron_index] = bool(value)
            elif property_name == "position":
                if isinstance(value, (tuple, list)) and len(value) >= 3:
                    #  MEMORY OPTIMIZATION: Use uint16 with range clamping
                    #  (0-65535)
                    self.coordinates_x[neuron_index] = np.uint16(
                        max(0, min(65535, value[0]))
                    )
                    self.coordinates_y[neuron_index] = np.uint16(
                        max(0, min(65535, value[1]))
                    )
                    self.coordinates_z[neuron_index] = np.uint16(
                        max(0, min(65535, value[2]))
                    )
                else:
                    logger.error(
                        f"Invalid position value for neuron {neuron_id}: {value}"
                    )
                    return False
            else:
                logger.warning(
                    f"Unknown property '{property_name}' for neuron {neuron_id}"
                )
                return False

            return True

        except Exception as e:
            logger.error(
                f"Failed to set property {property_name} for neuron {neuron_id}: {e}"
            )
            return False

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary for monitoring and optimization.

        Returns:
            Dictionary containing performance metrics and backend information
        """
        try:
            summary = {
                "backend": (
                    self.backend.name
                    if hasattr(self.backend, "name")
                    else str(self.backend)
                ),
                "simd_enabled": getattr(self.backend, "simd_enabled", False),
                "alignment": getattr(self.backend, "alignment", 64),
                "neuron_count": self.neuron_count,
                "max_neurons": self.max_neurons,
                "memory_usage_mb": self._estimate_memory_usage(),
                "use_rust": self._use_rust,
                "precision": getattr(self.backend, "precision", "fp32"),
                "device": getattr(self.backend, "device", "cpu"),
            }

            # Add backend-specific metrics if available
            if hasattr(self.backend, "get_performance_metrics"):
                backend_metrics = self.backend.get_performance_metrics()
                summary.update(backend_metrics)

            return summary

        except Exception as e:
            logger.error(f"Failed to get performance summary: {e}")
            return {
                "backend": "unknown",
                "simd_enabled": False,
                "alignment": 64,
                "neuron_count": 0,
                "max_neurons": self.max_neurons,
                "memory_usage_mb": 0.0,
                "use_rust": self._use_rust,
                "precision": "fp32",
                "device": "cpu",
            }

    def _estimate_memory_usage(self) -> float:
        """Estimate memory usage in megabytes."""
        try:
            if self._use_rust and hasattr(
                self._rust_backend, "get_memory_usage"
            ):
                return self._rust_backend.get_memory_usage() / (1024 * 1024)
            else:
                # Estimate based on array sizes
                bytes_per_neuron = (
                    4
                    * 8  # 8 float32 arrays (membrane_potentials, thresholds, etc.)
                    + 4 * 3  # 3 uint32 coordinate arrays
                    + 4
                    * 2  # 2 int32 arrays (cortical_idxs, refractory_periods)
                    + 1 * 2  # 2 bool arrays (valid_mask, is_active)
                )
                total_bytes = bytes_per_neuron * self.max_neurons
                return total_bytes / (1024 * 1024)
        except Exception:
            return 0.0


# For backwards compatibility
class Neuron:
    """Wrapper class for individual neurons.

    This class provides an object-oriented interface to individual neurons for
    API compatibility. It delegates most operations to the underlying
    NeuronArray for efficiency.
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
        """Initialize a Neuron object.

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
