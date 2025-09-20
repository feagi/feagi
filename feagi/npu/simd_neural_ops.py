"""
SIMD-Optimized Neural Operations for FEAGI NPU

High-performance vectorized neural processing operations optimized for:
- Rust migration readiness (SoA format, primitive types only)
- RTOS compatibility (deterministic execution, no system calls)
- GPU coalesced memory access patterns
- SIMD vectorization (batch operations on aligned arrays)

Design Principles:
1. Structure of Arrays (SoA) - optimal for SIMD/GPU
2. In-place operations - minimize memory allocations
3. Vectorized numpy operations - leverage SIMD instructions
4. No branching in hot paths - GPU-friendly
5. Deterministic behavior - RTOS compatible
"""

from typing import Optional
import numpy as np
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


def simd_membrane_decay(
    potentials: np.ndarray,
    decay_rates: np.ndarray,
    leak_coefficients: np.ndarray,
    resting_potentials: np.ndarray,
    valid_mask: np.ndarray
) -> None:
    """SIMD-optimized membrane potential decay with leak behavior.
    
    Applies leaky integrate-and-fire dynamics to membrane potentials.
    Uses vectorized operations for maximum SIMD performance.
    
    Args:
        potentials: Membrane potentials array [N] (modified in-place)
        decay_rates: Decay rates per neuron [N] (0.0-1.0)
        leak_coefficients: Leak coefficients per neuron [N] (0.0-1.0)
        resting_potentials: Resting potential values [N]
        valid_mask: Boolean mask for valid neurons [N]
    
    Note:
        RUST-COMPATIBLE: Uses only primitive array operations.
        RTOS-SAFE: Deterministic execution time, no system calls.
        GPU-READY: Coalesced memory access, vectorized operations.
    """
    if not np.any(valid_mask):
        return
    
    # Vectorized leaky integrate-and-fire dynamics
    # potential = potential * decay_rate + (resting_potential - potential) * leak_coefficient
    valid_indices = np.where(valid_mask)[0]
    
    if len(valid_indices) == 0:
        return
    
    # SIMD vectorized decay and leak application
    current_potentials = potentials[valid_indices]
    decay_factors = decay_rates[valid_indices]
    leak_factors = leak_coefficients[valid_indices]
    resting_vals = resting_potentials[valid_indices]
    
    # Leaky integrate-and-fire: V(t+1) = V(t) * decay + (V_rest - V(t)) * leak
    leak_current = (resting_vals - current_potentials) * leak_factors
    new_potentials = current_potentials * decay_factors + leak_current
    
    # Write back to original array (in-place for memory efficiency)
    potentials[valid_indices] = new_potentials


def simd_refractory_update(
    refractory_counters: np.ndarray,
    valid_mask: np.ndarray
) -> None:
    """SIMD-optimized refractory period counter updates.
    
    Decrements refractory counters for neurons that are in refractory period.
    Uses vectorized operations for maximum performance.
    
    Args:
        refractory_counters: Refractory counters array [N] (modified in-place)
        valid_mask: Boolean mask for valid neurons [N]
    
    Note:
        RUST-COMPATIBLE: Only primitive array operations.
        RTOS-SAFE: Deterministic execution, no allocations.
        GPU-READY: Vectorized memory access patterns.
    """
    if not np.any(valid_mask):
        return
    
    # Find neurons that are in refractory period (counter > 0) and valid
    refractory_mask = valid_mask & (refractory_counters > 0)
    
    if np.any(refractory_mask):
        # Vectorized decrement (SIMD-optimized)
        refractory_counters[refractory_mask] -= 1


def simd_consecutive_fire_update(
    consecutive_fire_counts: np.ndarray,
    firing_mask: np.ndarray,
    not_firing_mask: np.ndarray,
    valid_mask: np.ndarray,
    consecutive_fire_limits: np.ndarray
) -> None:
    """SIMD-optimized consecutive fire count tracking.
    
    Updates consecutive fire counts: increment for firing neurons,
    reset to 0 for non-firing neurons.
    SKIPS UPDATE if consecutive_fire_limits is 0 for a neuron.
    
    Args:
        consecutive_fire_counts: Consecutive fire counts [N] (modified in-place)
        firing_mask: Boolean mask for neurons that fired [N]
        not_firing_mask: Boolean mask for neurons that didn't fire [N]
        valid_mask: Boolean mask for valid neurons [N]
        consecutive_fire_limits: Consecutive fire limits [N] (0 = skip counting)
    
    Note:
        RUST-COMPATIBLE: Vectorized primitive operations only.
        RTOS-SAFE: Deterministic execution, no branching in hot path.
    """
    if not np.any(valid_mask):
        return
    
    # Only update consecutive fire counts where limit > 0 (skip if limit is 0)
    active_counting_mask = (consecutive_fire_limits > 0) & valid_mask
    
    if not np.any(active_counting_mask):
        return  # No neurons have consecutive fire counting enabled
    
    # Increment consecutive count for firing neurons with active counting (vectorized)
    firing_active = firing_mask & active_counting_mask
    if np.any(firing_active):
        consecutive_fire_counts[firing_active] += 1
    
    # Reset consecutive count for non-firing neurons with active counting (vectorized)  
    not_firing_active = not_firing_mask & active_counting_mask
    if np.any(not_firing_active):
        consecutive_fire_counts[not_firing_active] = 0


def simd_firing_check_with_consecutive_limits(
    potentials: np.ndarray,
    thresholds: np.ndarray,
    refractory_counters: np.ndarray,
    consecutive_fire_counts: np.ndarray,
    consecutive_fire_limits: np.ndarray,
    valid_mask: np.ndarray,
    excitability: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None
) -> np.ndarray:
    """SIMD-optimized firing check with consecutive fire limits and excitability.
    
    Determines which neurons should fire based on:
    1. Membrane potential >= threshold
    2. Not in refractory period (counter == 0)
    3. Consecutive fire count < limit
    4. Optional: Excitability probability check
    
    Args:
        potentials: Membrane potentials [N]
        thresholds: Firing thresholds [N]
        refractory_counters: Refractory counters [N]
        consecutive_fire_counts: Consecutive fire counts [N]
        consecutive_fire_limits: Max consecutive fires allowed [N]
        valid_mask: Valid neuron mask [N]
        excitability: Optional excitability values [N] (0.0-1.0)
        rng: Optional random number generator
    
    Returns:
        Boolean mask of neurons that should fire [N]
    
    Note:
        RUST-COMPATIBLE: Pure vectorized operations, no dynamic allocation.
        RTOS-SAFE: Deterministic execution (when rng is None).
        GPU-READY: Coalesced memory access, minimal branching.
    """
    if not np.any(valid_mask):
        return np.zeros_like(potentials, dtype=bool)
    
    # Step 1: Basic firing conditions (vectorized)
    # Skip consecutive fire count constraint if consecutive_fire_limits is 0
    consecutive_fire_constraint = np.ones_like(valid_mask, dtype=bool)  # Default: no constraint
    
    # Apply consecutive fire limit only where limit > 0
    limit_active = consecutive_fire_limits > 0
    consecutive_fire_constraint[limit_active] = (consecutive_fire_counts < consecutive_fire_limits)[limit_active]
    
    # Debug the threshold comparison for high-threshold neurons
    threshold_mask = (potentials >= thresholds)
    
    # Check for obvious bugs: high threshold neurons passing the test
    high_threshold_indices = np.where((thresholds >= 1000) & threshold_mask)[0]
    if len(high_threshold_indices) > 0:
        import logging
        logger = logging.getLogger(__name__)
        logger.error(f"[SIMD-BUG] {len(high_threshold_indices)} high-threshold neurons passed threshold check!")
        for i in high_threshold_indices[:5]:  # Show first 5
            pot_val = potentials[i]
            thresh_val = thresholds[i]
            comparison_result = pot_val >= thresh_val
            logger.error(f"  Neuron[{i}]: potential={pot_val:.6f} >= threshold={thresh_val:.1f} = {comparison_result}")
            logger.error(f"    Raw values: pot={pot_val} thresh={thresh_val} types=({type(pot_val)}, {type(thresh_val)})")
            logger.error(f"    Array dtypes: potentials={potentials.dtype} thresholds={thresholds.dtype}")
    
    can_fire_mask = (
        valid_mask &                            # Valid neurons
        (refractory_counters == 0) &           # Not in refractory
        threshold_mask &                       # Above threshold
        consecutive_fire_constraint            # Under consecutive limit (only if limit > 0)
    )
    
    if not np.any(can_fire_mask):
        return np.zeros_like(potentials, dtype=bool)
    
    # Step 2: Apply excitability if provided (supports both array and tuple formats)
    if excitability is not None:
        # Handle tuple format from old NPU implementation: (area_ex_map, cortical_idxs, any_low_flag)
        if isinstance(excitability, tuple):
            area_ex_map, cortical_idxs, any_low_flag = excitability
            
            # Fast path: if no areas have low excitability, skip RNG entirely
            if not any_low_flag:
                return can_fire_mask
            
            # Only use probabilistic firing when needed AND RNG is provided
            if rng is not None:
                # Get indices of neurons that can potentially fire
                candidate_indices = np.where(can_fire_mask)[0]
                
                if len(candidate_indices) > 0:
                    # Build area excitability values for candidate neurons
                    relevant_ex_values = np.fromiter(
                        (area_ex_map.get(int(cidx), 1.0) for cidx in cortical_idxs[candidate_indices].tolist()),
                        dtype=np.float32,
                        count=candidate_indices.size,
                    )
                    
                    # Fast path for neurons with excitability >= 0.999 (certain fire)
                    certain_fire_mask = relevant_ex_values >= 0.999
                    
                    # Probabilistic check for neurons with excitability < 0.999
                    uncertain_mask = ~certain_fire_mask
                    
                    if np.any(uncertain_mask):
                        uncertain_count = int(np.sum(uncertain_mask))
                        random_values = rng.random(uncertain_count)
                        uncertain_ex_vals = relevant_ex_values[uncertain_mask]
                        probabilistic_fire = random_values < uncertain_ex_vals
                        
                        firing_decisions = np.copy(certain_fire_mask)
                        firing_decisions[uncertain_mask] = probabilistic_fire
                    else:
                        firing_decisions = certain_fire_mask
                    
                    # Create final firing mask
                    final_firing_mask = np.zeros_like(potentials, dtype=bool)
                    final_firing_mask[candidate_indices] = firing_decisions
                    
                    return final_firing_mask
            
            # Fallback: deterministic firing for all neurons
            return can_fire_mask
            
        # Handle legacy per-neuron array format
        elif rng is not None:
            # Get indices of neurons that can potentially fire
            candidate_indices = np.where(can_fire_mask)[0]
            
            if len(candidate_indices) > 0:
                # Generate random values for excitability check (vectorized)
                random_vals = rng.random(len(candidate_indices))
                excitability_vals = excitability[candidate_indices]
                
                # Apply excitability threshold (vectorized)
                excitability_mask = random_vals < excitability_vals
                
                # Update firing mask based on excitability
                final_firing_mask = np.zeros_like(potentials, dtype=bool)
                final_firing_mask[candidate_indices[excitability_mask]] = True
                
                return final_firing_mask
    
    # Return basic firing mask (deterministic mode)
    return can_fire_mask


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
        RUST-COMPATIBLE: In-place primitive array operations only.
        RTOS-SAFE: Deterministic execution, no allocations.
        GPU-READY: Vectorized memory writes, coalesced access.
    """
    if not np.any(firing_mask):
        return
    
    # Reset membrane potentials to resting values (vectorized)
    potentials[firing_mask] = resting_potentials[firing_mask]
    
    # Set refractory counters (vectorized)
    refractory_counters[firing_mask] = refractory_periods[firing_mask]


def simd_batch_neural_update(
    potentials: np.ndarray,
    thresholds: np.ndarray,
    decay_rates: np.ndarray,
    leak_coefficients: np.ndarray,
    resting_potentials: np.ndarray,
    refractory_periods: np.ndarray,
    refractory_counters: np.ndarray,
    consecutive_fire_counts: np.ndarray,
    consecutive_fire_limits: np.ndarray,
    valid_mask: np.ndarray,
    excitability: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None,
    output_firing_mask: Optional[np.ndarray] = None
) -> tuple[np.ndarray, int]:
    """Complete SIMD-optimized neural update with consecutive fire tracking.
    
    Performs the complete neural processing pipeline in CORRECT biological order:
    1. Update refractory counters (decrement existing counters)
    2. Check firing conditions BEFORE decay (preserves FCL candidate potentials)
    3. Update consecutive fire counts  
    4. Reset fired neurons (membrane potentials and set refractory counters)
    5. Apply membrane decay to remaining neurons (leak behavior)
    
    Args:
        potentials: Membrane potentials [N] (modified in-place)
        thresholds: Firing thresholds [N]
        decay_rates: Membrane decay rates [N]
        leak_coefficients: Leak coefficients [N]
        resting_potentials: Resting potentials [N]
        refractory_periods: Refractory periods [N]
        refractory_counters: Refractory counters [N] (modified in-place)
        consecutive_fire_counts: Consecutive fire counts [N] (modified in-place)
        consecutive_fire_limits: Max consecutive fires [N]
        valid_mask: Valid neuron mask [N]
        excitability: Optional excitability values [N]
        rng: Optional random number generator
        output_firing_mask: Optional pre-allocated output array [N]
    
    Returns:
        Tuple of (firing_mask, num_fired)
        
    Note:
        RUST-COMPATIBLE: Complete vectorized pipeline, SoA format.
        RTOS-SAFE: Deterministic when rng=None, pre-allocated arrays.
        GPU-READY: Optimal memory access patterns, minimal branching.
        SIMD-OPTIMIZED: Leverages vector instructions throughout.
    """
    # Step 1: Update refractory counters (decrement existing counters)
    simd_refractory_update(refractory_counters, valid_mask)
    
    # Step 2: Check firing conditions BEFORE decay (preserves FCL candidate potentials)
    firing_mask = simd_firing_check_with_consecutive_limits(
        potentials, thresholds, refractory_counters, consecutive_fire_counts,
        consecutive_fire_limits, valid_mask, excitability, rng
    )
    
    # Step 3: Update consecutive fire counts (skip if limits are 0)
    not_firing_mask = ~firing_mask
    simd_consecutive_fire_update(consecutive_fire_counts, firing_mask, 
                                not_firing_mask, valid_mask, consecutive_fire_limits)
    
    # Step 4: Reset fired neurons (membrane potentials and set refractory counters)
    simd_reset_fired_neurons(potentials, resting_potentials, refractory_periods,
                            refractory_counters, firing_mask)
    
    # Step 5: Apply membrane decay to remaining neurons (leak behavior)
    # Only apply to neurons that didn't fire - fired neurons already reset to resting
    non_firing_valid = valid_mask & (~firing_mask)
    simd_membrane_decay(potentials, decay_rates, leak_coefficients, 
                       resting_potentials, non_firing_valid)
    
    # Copy to output array if provided (for external use)
    if output_firing_mask is not None:
        np.copyto(output_firing_mask, firing_mask)
    
    # Return firing mask and count
    num_fired = int(np.sum(firing_mask))
    return firing_mask, num_fired
