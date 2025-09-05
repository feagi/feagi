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
Synaptic Pruning Operations - NPU Module

This module provides efficient synaptic pruning operations for maintaining
network health and performance. Designed for Rust migration and RTOS compatibility.

Key Features:
- Weight-based pruning (remove weak synapses)
- Activity-based pruning (remove inactive synapses)  
- Age-based pruning (remove old unused synapses)
- Vectorized operations for GPU acceleration
- Zero memory allocation during runtime
- Deterministic execution paths

Design Principles:
- All operations are in-place where possible
- Pre-allocated output arrays
- No dynamic memory allocation
- SIMD/GPU friendly operations
- Rust FFI compatible
"""

import logging
from typing import Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Pruning constants (Rust-compatible)
DEFAULT_WEIGHT_THRESHOLD = 0.1
DEFAULT_ACTIVITY_THRESHOLD = 0.01
DEFAULT_AGE_THRESHOLD = 1000  # timesteps


def prune_by_weight(
    weights: np.ndarray,
    threshold: float,
    valid_mask: np.ndarray,
    output_mask: np.ndarray
) -> int:
    """Prune synapses below weight threshold.
    
    Identifies synapses with weights below the threshold for removal.
    Uses vectorized operations for maximum performance.
    
    Args:
        weights: Synaptic weights [N]
        threshold: Minimum weight threshold
        valid_mask: Current validity mask [N] (input)
        output_mask: Updated validity mask [N] (output, pre-allocated)
    
    Returns:
        Number of synapses pruned
    
    Note:
        Output mask is modified in-place. Input arrays are not modified.
    """
    # Find synapses above threshold
    above_threshold = weights >= threshold
    
    # Update validity mask (only keep synapses above threshold AND currently valid)
    np.logical_and(valid_mask, above_threshold, out=output_mask)
    
    # Count pruned synapses
    pruned_count = np.sum(valid_mask) - np.sum(output_mask)
    
    return int(pruned_count)


def prune_by_activity(
    activity_levels: np.ndarray,
    threshold: float,
    valid_mask: np.ndarray,
    output_mask: np.ndarray
) -> int:
    """Prune synapses with low activity levels.
    
    Removes synapses that haven't been active recently.
    Activity levels should be normalized to [0, 1] range.
    
    Args:
        activity_levels: Recent activity levels [N]
        threshold: Minimum activity threshold
        valid_mask: Current validity mask [N] (input)
        output_mask: Updated validity mask [N] (output, pre-allocated)
    
    Returns:
        Number of synapses pruned
    """
    # Find synapses with sufficient activity
    active_enough = activity_levels >= threshold
    
    # Update validity mask
    np.logical_and(valid_mask, active_enough, out=output_mask)
    
    # Count pruned synapses
    pruned_count = np.sum(valid_mask) - np.sum(output_mask)
    
    return int(pruned_count)


def prune_by_age(
    last_used_timesteps: np.ndarray,
    current_timestep: int,
    max_age: int,
    valid_mask: np.ndarray,
    output_mask: np.ndarray
) -> int:
    """Prune synapses that haven't been used recently.
    
    Removes synapses that haven't been active within the age threshold.
    
    Args:
        last_used_timesteps: Last timestep each synapse was used [N]
        current_timestep: Current simulation timestep
        max_age: Maximum age in timesteps
        valid_mask: Current validity mask [N] (input)
        output_mask: Updated validity mask [N] (output, pre-allocated)
    
    Returns:
        Number of synapses pruned
    """
    # Calculate age of each synapse
    ages = current_timestep - last_used_timesteps
    
    # Find synapses within age limit
    young_enough = ages <= max_age
    
    # Update validity mask
    np.logical_and(valid_mask, young_enough, out=output_mask)
    
    # Count pruned synapses
    pruned_count = np.sum(valid_mask) - np.sum(output_mask)
    
    return int(pruned_count)


def combined_pruning(
    weights: np.ndarray,
    activity_levels: np.ndarray,
    last_used_timesteps: np.ndarray,
    current_timestep: int,
    weight_threshold: float = DEFAULT_WEIGHT_THRESHOLD,
    activity_threshold: float = DEFAULT_ACTIVITY_THRESHOLD,
    age_threshold: int = DEFAULT_AGE_THRESHOLD,
    valid_mask: np.ndarray = None,
    output_mask: np.ndarray = None,
    temp_mask: np.ndarray = None
) -> Tuple[int, int, int, int]:
    """Combined pruning using multiple criteria.
    
    Applies weight, activity, and age-based pruning in sequence.
    Uses pre-allocated temporary buffers to avoid memory allocation.
    
    Args:
        weights: Synaptic weights [N]
        activity_levels: Recent activity levels [N]
        last_used_timesteps: Last used timesteps [N]
        current_timestep: Current simulation timestep
        weight_threshold: Minimum weight threshold
        activity_threshold: Minimum activity threshold
        age_threshold: Maximum age in timesteps
        valid_mask: Current validity mask [N] (input, optional)
        output_mask: Final validity mask [N] (output, pre-allocated)
        temp_mask: Temporary mask for intermediate results [N] (pre-allocated)
    
    Returns:
        Tuple of (weight_pruned, activity_pruned, age_pruned, total_pruned)
    """
    synapse_count = len(weights)
    
    # Initialize masks if not provided
    if valid_mask is None:
        valid_mask = np.ones(synapse_count, dtype=np.bool_)
    if output_mask is None:
        output_mask = np.empty(synapse_count, dtype=np.bool_)
    if temp_mask is None:
        temp_mask = np.empty(synapse_count, dtype=np.bool_)
    
    initial_count = np.sum(valid_mask)
    
    # Step 1: Prune by weight
    weight_pruned = prune_by_weight(weights, weight_threshold, valid_mask, temp_mask)
    
    # Step 2: Prune by activity (using result from step 1)
    activity_pruned = prune_by_activity(activity_levels, activity_threshold, temp_mask, output_mask)
    
    # Step 3: Prune by age (using result from step 2)  
    age_pruned = prune_by_age(last_used_timesteps, current_timestep, age_threshold, output_mask, temp_mask)
    
    # Final result
    np.copyto(output_mask, temp_mask)
    
    final_count = np.sum(output_mask)
    total_pruned = initial_count - final_count
    
    return weight_pruned, activity_pruned, age_pruned, int(total_pruned)


def compact_synapse_arrays(
    source_neurons: np.ndarray,
    target_neurons: np.ndarray,
    weights: np.ndarray,
    valid_mask: np.ndarray,
    output_sources: np.ndarray,
    output_targets: np.ndarray,
    output_weights: np.ndarray
) -> int:
    """Compact synapse arrays by removing invalid entries.
    
    Removes pruned synapses and compacts arrays to eliminate gaps.
    All output arrays must be pre-allocated with sufficient size.
    
    Args:
        source_neurons: Source neuron IDs [N]
        target_neurons: Target neuron IDs [N]
        weights: Synaptic weights [N]
        valid_mask: Validity mask [N]
        output_sources: Compacted source neurons [M] (pre-allocated)
        output_targets: Compacted target neurons [M] (pre-allocated)
        output_weights: Compacted weights [M] (pre-allocated)
    
    Returns:
        Number of valid synapses after compaction
    
    Note:
        Output arrays must be at least as large as the number of valid synapses.
        Only the first 'valid_count' elements of output arrays are meaningful.
    """
    # Find valid indices
    valid_indices = np.where(valid_mask)[0]
    valid_count = len(valid_indices)
    
    if valid_count == 0:
        return 0
    
    # Compact arrays using advanced indexing
    output_sources[:valid_count] = source_neurons[valid_indices]
    output_targets[:valid_count] = target_neurons[valid_indices]
    output_weights[:valid_count] = weights[valid_indices]
    
    return valid_count


def update_activity_tracking(
    firing_neurons: np.ndarray,
    source_neurons: np.ndarray,
    target_neurons: np.ndarray,
    current_timestep: int,
    activity_levels: np.ndarray,
    last_used_timesteps: np.ndarray,
    decay_rate: float = 0.95
) -> None:
    """Update activity tracking for synapses.
    
    Updates activity levels and last-used timestamps for synapses
    based on which neurons fired in the current timestep.
    
    Args:
        firing_neurons: Neurons that fired this timestep
        source_neurons: Source neuron IDs for all synapses [N]
        target_neurons: Target neuron IDs for all synapses [N]
        current_timestep: Current simulation timestep
        activity_levels: Activity levels to update [N] (modified in-place)
        last_used_timesteps: Last used timestamps [N] (modified in-place)
        decay_rate: Decay rate for activity levels
    
    Note:
        Activity levels and timestamps are modified in-place.
    """
    # Decay all activity levels
    activity_levels *= decay_rate
    
    if len(firing_neurons) == 0:
        return
    
    # Find synapses with firing source neurons
    firing_set = set(firing_neurons)
    
    # Vectorized approach: create mask for active synapses
    active_mask = np.isin(source_neurons, firing_neurons)
    
    if np.any(active_mask):
        # Update activity levels for active synapses
        activity_levels[active_mask] = 1.0
        
        # Update last used timestamps
        last_used_timesteps[active_mask] = current_timestep
