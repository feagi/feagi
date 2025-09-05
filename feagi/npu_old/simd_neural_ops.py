# Copyright 2016-2025 Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#  ==============================================================================
"""
SIMD Neural Operations - NPU Module

This module provides SIMD-optimized neural operations for high-performance
neural processing. Designed for Rust migration and RTOS compatibility.

Key Features:
- Vectorized membrane potential decay
- SIMD-optimized refractory period updates
- Vectorized firing threshold checks
- Cache-aligned memory operations
- Zero memory allocation during runtime

Design Principles:
- All operations are vectorized using NumPy
- Functions are pure (no side effects)
- Pre-allocated output arrays
- SIMD-friendly memory layout
- Rust FFI compatible
"""

import logging
import numpy as np

logger = logging.getLogger(__name__)


def simd_membrane_decay(
    potentials: np.ndarray,
    decay_rates: np.ndarray,
    mask: np.ndarray
) -> None:
    """SIMD-optimized membrane potential decay.
    
    Applies exponential decay to membrane potentials using vectorized operations.
    Only processes neurons specified by the mask for efficiency.
    
    Args:
        potentials: Membrane potentials array [N] (modified in-place)
        decay_rates: Decay rates for each neuron [N]
        mask: Boolean mask for neurons to update [N]
    
    Note:
        Potentials array is modified in-place for performance.
        Only neurons where mask[i] == True are updated.
    """
    if not np.any(mask):
        return
    
    # Vectorized decay: V(t+1) = V(t) * decay_rate
    potentials[mask] *= decay_rates[mask]


def simd_refractory_update(
    counters: np.ndarray,
    mask: np.ndarray
) -> None:
    """SIMD-optimized refractory period counter updates.
    
    Decrements refractory counters for neurons that are in refractory period.
    Uses vectorized operations for maximum performance.
    
    Args:
        counters: Refractory counters array [N] (modified in-place)
        mask: Boolean mask for valid neurons [N]
    
    Note:
        Counters array is modified in-place for performance.
        Only decrements counters > 0 to avoid underflow.
    """
    if not np.any(mask):
        return
    
    # Find neurons that are in refractory period (counter > 0) and valid
    refractory_mask = mask & (counters > 0)
    
    if np.any(refractory_mask):
        # Vectorized decrement
        counters[refractory_mask] -= 1


def simd_firing_check(
    potentials: np.ndarray,
    thresholds: np.ndarray,
    can_fire_mask: np.ndarray
) -> np.ndarray:
    """SIMD-optimized firing threshold check.
    
    Determines which neurons should fire based on membrane potential
    exceeding threshold and not being in refractory period.
    
    Args:
        potentials: Membrane potentials array [N]
        thresholds: Firing thresholds array [N]
        can_fire_mask: Mask for neurons that can fire (not refractory) [N]
    
    Returns:
        Boolean array indicating which neurons should fire [N]
    
    Note:
        Returns a new array. Input arrays are not modified.
    """
    # Vectorized threshold check
    threshold_met = potentials >= thresholds
    
    # Combine with can_fire_mask (not in refractory)
    firing_mask = threshold_met & can_fire_mask
    
    return firing_mask


def simd_firing_check_with_excitability(
    potentials: np.ndarray,
    thresholds: np.ndarray,
    can_fire_mask: np.ndarray,
    excitability,
    rng: np.random.Generator = None
) -> np.ndarray:
    """SIMD-optimized firing check with probabilistic excitability.
    
    Enhanced firing check that includes probabilistic firing based on
    excitability values. Uses vectorized random number generation.
    
    Args:
        potentials: Membrane potentials array [N]
        thresholds: Firing thresholds array [N]
        can_fire_mask: Mask for neurons that can fire [N]
        excitability: Excitability values (0.0 to 1.0) [N]
        rng: Random number generator (optional)
    
    Returns:
        Boolean array indicating which neurons should fire [N]
    
    Note:
        Fast path when all excitability >= 0.999 (bypasses random generation).
        Uses vectorized random number generation for efficiency.
    """
    # Fast path decision: when excitability is provided as per-area cache tuple
    # excitability may be either:
    # - a numpy array of per-neuron values (legacy), or
    # - a tuple: (area_ex_map: Dict[int,float], cortical_idxs: np.ndarray[uint16], any_low_flag: bool)
    if isinstance(excitability, tuple):
        area_ex_map, cortical_idxs, any_low_flag = excitability
        if not any_low_flag:
            return simd_firing_check(potentials, thresholds, can_fire_mask)
    else:
        # Legacy array path
        if np.all(excitability >= 0.999):
            return simd_firing_check(potentials, thresholds, can_fire_mask)
    
    # Standard threshold check
    threshold_met = (potentials >= thresholds) & can_fire_mask
    
    # Early exit if no neurons met threshold
    if not np.any(threshold_met):
        return threshold_met
    
    # Find neurons that need probabilistic check
    threshold_indices = np.where(threshold_met)[0]
    
    # Build area excitability values for the ready indices
    if isinstance(excitability, tuple):
        area_ex_map, cortical_idxs, any_low_flag = excitability
        relevant_ex_values = np.fromiter(
            (area_ex_map.get(int(cidx), 1.0) for cidx in cortical_idxs[threshold_indices].tolist()),
            dtype=np.float32,
            count=threshold_indices.size,
        )
    else:
        relevant_ex_values = excitability[threshold_indices]
    
    # Fast path for neurons with excitability >= 0.999
    certain_fire_mask = relevant_ex_values >= 0.999
    
    # Probabilistic check for neurons with excitability < 0.999
    uncertain_mask = ~certain_fire_mask
    
    if np.any(uncertain_mask):
        if rng is None:
            rng = np.random.default_rng()
        
        uncertain_count = int(np.sum(uncertain_mask))
        random_values = rng.random(uncertain_count)
        uncertain_ex_vals = relevant_ex_values[uncertain_mask]
        probabilistic_fire = random_values < uncertain_ex_vals
        firing_decisions = np.copy(certain_fire_mask)
        firing_decisions[uncertain_mask] = probabilistic_fire
    else:
        firing_decisions = certain_fire_mask
    
    # Create final firing mask
    final_firing_mask = np.zeros_like(threshold_met, dtype=bool)
    final_firing_mask[threshold_indices] = firing_decisions
    
    return final_firing_mask


def simd_reset_fired_neurons(
    potentials: np.ndarray,
    resting_potentials: np.ndarray,
    refractory_periods: np.ndarray,
    refractory_counters: np.ndarray,
    firing_mask: np.ndarray
) -> None:
    """SIMD-optimized reset of fired neurons.
    
    Resets membrane potentials to resting values and sets refractory counters
    for neurons that fired. Uses vectorized operations.
    
    Args:
        potentials: Membrane potentials array [N] (modified in-place)
        resting_potentials: Resting potential values [N]
        refractory_periods: Refractory period durations [N]
        refractory_counters: Refractory counters [N] (modified in-place)
        firing_mask: Boolean mask of neurons that fired [N]
    
    Note:
        Potentials and counters arrays are modified in-place.
    """
    if not np.any(firing_mask):
        return
    
    # Reset membrane potentials to resting values
    potentials[firing_mask] = resting_potentials[firing_mask]
    
    # Set refractory counters
    refractory_counters[firing_mask] = refractory_periods[firing_mask]


def simd_batch_neural_update(
    potentials: np.ndarray,
    thresholds: np.ndarray,
    decay_rates: np.ndarray,
    resting_potentials: np.ndarray,
    refractory_periods: np.ndarray,
    refractory_counters: np.ndarray,
    excitability: np.ndarray,
    valid_mask: np.ndarray,
    output_firing_mask: np.ndarray,
    rng: np.random.Generator = None
) -> int:
    """Complete SIMD-optimized neural update in one function.
    
    Performs membrane decay, refractory updates, firing checks, and neuron
    reset in a single vectorized operation for maximum performance.
    
    Args:
        potentials: Membrane potentials [N] (modified in-place)
        thresholds: Firing thresholds [N]
        decay_rates: Decay rates [N]
        resting_potentials: Resting potentials [N]
        refractory_periods: Refractory periods [N]
        refractory_counters: Refractory counters [N] (modified in-place)
        excitability: Excitability values [N]
        valid_mask: Valid neuron mask [N]
        output_firing_mask: Output firing mask [N] (pre-allocated)
        rng: Random number generator (optional)
    
    Returns:
        Number of neurons that fired
    
    Note:
        This is the most efficient way to perform neural updates as it
        minimizes memory access and maximizes vectorization.
    """
    # Step 1: Create can-fire mask (valid and not in refractory)
    can_fire_mask = valid_mask & (refractory_counters == 0)
    
    # Step 2: Apply membrane decay to neurons that can update
    simd_membrane_decay(potentials, decay_rates, can_fire_mask)
    
    # Step 3: Update refractory counters
    simd_refractory_update(refractory_counters, valid_mask)
    
    # Step 4: Check firing with excitability
    np.copyto(output_firing_mask, simd_firing_check_with_excitability(
        potentials, thresholds, can_fire_mask, excitability, rng
    ))
    
    # Step 5: Reset fired neurons
    simd_reset_fired_neurons(
        potentials, resting_potentials, refractory_periods,
        refractory_counters, output_firing_mask
    )
    
    # Return firing count
    return int(np.sum(output_firing_mask))
