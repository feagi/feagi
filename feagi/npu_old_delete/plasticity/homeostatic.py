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
Homeostatic Plasticity Operations - NPU Module

This module provides homeostatic plasticity mechanisms to maintain neural
network stability and prevent runaway excitation or complete silence.

Key Features:
- Synaptic scaling based on neuron activity
- Weight normalization to prevent saturation
- Activity-dependent threshold adjustment
- Intrinsic excitability regulation
- Vectorized operations for GPU acceleration

Design Principles:
- Maintains network stability without external intervention
- Prevents both hyperexcitation and neural silence
- Uses local information only (no global state)
- All operations are vectorized and GPU-friendly
- Zero memory allocation during runtime
- Rust FFI compatible
"""

import logging
from typing import Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Homeostatic constants (Rust-compatible)
DEFAULT_TARGET_ACTIVITY = 0.1  # Target firing rate (10%)
DEFAULT_SCALING_RATE = 0.01    # Rate of homeostatic scaling
DEFAULT_MIN_WEIGHT = 0.01      # Minimum weight after scaling
DEFAULT_MAX_WEIGHT = 10.0      # Maximum weight after scaling


def synaptic_scaling(
    weights: np.ndarray,
    target_activity: float,
    actual_activity: np.ndarray,
    scaling_rate: float,
    output_weights: np.ndarray,
    min_weight: float = DEFAULT_MIN_WEIGHT,
    max_weight: float = DEFAULT_MAX_WEIGHT
) -> None:
    """Apply synaptic scaling to maintain target activity levels.
    
    Scales synaptic weights based on the difference between target and
    actual activity levels. Neurons with low activity get stronger synapses,
    neurons with high activity get weaker synapses.
    
    Args:
        weights: Current synaptic weights [N]
        target_activity: Target activity level (0-1)
        actual_activity: Actual activity levels per neuron [M]
        scaling_rate: Rate of scaling adjustment
        output_weights: Updated weights [N] (pre-allocated)
        min_weight: Minimum allowed weight
        max_weight: Maximum allowed weight
    
    Note:
        This assumes weights are organized by target neuron.
        actual_activity[i] corresponds to activity of neuron i.
    """
    # Calculate scaling factors based on activity deviation
    activity_ratio = target_activity / (actual_activity + 1e-8)  # Avoid division by zero
    
    # Apply gradual scaling (not immediate)
    scaling_factors = 1.0 + scaling_rate * (activity_ratio - 1.0)
    
    # Apply scaling to weights
    # Note: This assumes weights are grouped by target neuron
    # For more complex arrangements, additional indexing would be needed
    np.multiply(weights, scaling_factors, out=output_weights)
    
    # Apply bounds
    np.clip(output_weights, min_weight, max_weight, out=output_weights)


def weight_normalization(
    weights: np.ndarray,
    source_neurons: np.ndarray,
    target_neurons: np.ndarray,
    normalization_factor: float,
    output_weights: np.ndarray
) -> None:
    """Normalize synaptic weights to maintain total synaptic strength.
    
    Normalizes the total synaptic input to each neuron to prevent
    saturation or complete silence.
    
    Args:
        weights: Current synaptic weights [N]
        source_neurons: Source neuron indices [N]
        target_neurons: Target neuron indices [N]
        normalization_factor: Target total synaptic strength per neuron
        output_weights: Normalized weights [N] (pre-allocated)
    
    Note:
        This operation maintains the relative strength of synapses
        while controlling the total input strength.
    """
    # Copy weights to output initially
    np.copyto(output_weights, weights)
    
    # Get unique target neurons
    unique_targets = np.unique(target_neurons)
    
    # Normalize weights for each target neuron
    for target in unique_targets:
        # Find synapses targeting this neuron
        target_mask = target_neurons == target
        target_weights = weights[target_mask]
        
        # Calculate current total strength
        current_total = np.sum(target_weights)
        
        if current_total > 1e-8:  # Avoid division by zero
            # Calculate normalization factor
            norm_factor = normalization_factor / current_total
            
            # Apply normalization
            output_weights[target_mask] = target_weights * norm_factor


def intrinsic_excitability_update(
    thresholds: np.ndarray,
    target_activity: float,
    actual_activity: np.ndarray,
    adaptation_rate: float,
    output_thresholds: np.ndarray,
    min_threshold: float = 0.1,
    max_threshold: float = 10.0
) -> None:
    """Update intrinsic excitability (firing thresholds) homeostatically.
    
    Adjusts neuron firing thresholds to maintain target activity levels.
    Neurons with low activity get lower thresholds (easier to fire),
    neurons with high activity get higher thresholds (harder to fire).
    
    Args:
        thresholds: Current firing thresholds [N]
        target_activity: Target activity level (0-1)
        actual_activity: Actual activity levels [N]
        adaptation_rate: Rate of threshold adaptation
        output_thresholds: Updated thresholds [N] (pre-allocated)
        min_threshold: Minimum allowed threshold
        max_threshold: Maximum allowed threshold
    """
    # Calculate threshold adjustment based on activity deviation
    activity_error = actual_activity - target_activity
    
    # Adjust thresholds: increase for overactive neurons, decrease for underactive
    threshold_adjustment = adaptation_rate * activity_error
    
    # Apply adjustment
    np.subtract(thresholds, threshold_adjustment, out=output_thresholds)
    
    # Apply bounds
    np.clip(output_thresholds, min_threshold, max_threshold, out=output_thresholds)


def activity_dependent_scaling(
    weights: np.ndarray,
    pre_activity: np.ndarray,
    post_activity: np.ndarray,
    source_neurons: np.ndarray,
    target_neurons: np.ndarray,
    scaling_rate: float,
    output_weights: np.ndarray
) -> None:
    """Apply activity-dependent synaptic scaling.
    
    Scales synapses based on both pre- and post-synaptic activity levels.
    Implements a form of BCM (Bienenstock-Cooper-Munro) rule.
    
    Args:
        weights: Current synaptic weights [N]
        pre_activity: Pre-synaptic activity levels [M]
        post_activity: Post-synaptic activity levels [M]
        source_neurons: Source neuron indices [N]
        target_neurons: Target neuron indices [N]
        scaling_rate: Rate of activity-dependent scaling
        output_weights: Updated weights [N] (pre-allocated)
    """
    # Copy weights initially
    np.copyto(output_weights, weights)
    
    # Calculate activity-dependent scaling for each synapse
    for i in range(len(weights)):
        pre_idx = source_neurons[i]
        post_idx = target_neurons[i]
        
        pre_act = pre_activity[pre_idx]
        post_act = post_activity[post_idx]
        
        # BCM-like rule: strengthen if both active, weaken if only post active
        if post_act > 0.5:  # Post-synaptic neuron is active
            if pre_act > 0.5:  # Both active - strengthen
                scaling_factor = 1.0 + scaling_rate * pre_act * post_act
            else:  # Only post active - weaken
                scaling_factor = 1.0 - scaling_rate * post_act
        else:
            scaling_factor = 1.0  # No change if post-synaptic not active
        
        output_weights[i] = weights[i] * scaling_factor
    
    # Apply bounds
    np.clip(output_weights, DEFAULT_MIN_WEIGHT, DEFAULT_MAX_WEIGHT, out=output_weights)


def homeostatic_update_batch(
    weights: np.ndarray,
    thresholds: np.ndarray,
    source_neurons: np.ndarray,
    target_neurons: np.ndarray,
    pre_activity: np.ndarray,
    post_activity: np.ndarray,
    target_activity: float = DEFAULT_TARGET_ACTIVITY,
    scaling_rate: float = DEFAULT_SCALING_RATE,
    threshold_rate: float = 0.001,
    output_weights: np.ndarray = None,
    output_thresholds: np.ndarray = None
) -> Tuple[int, float]:
    """Comprehensive homeostatic update combining multiple mechanisms.
    
    Applies synaptic scaling, weight normalization, and threshold adaptation
    in a coordinated manner to maintain network stability.
    
    Args:
        weights: Current synaptic weights [N]
        thresholds: Current firing thresholds [M]
        source_neurons: Source neuron indices [N]
        target_neurons: Target neuron indices [N]
        pre_activity: Pre-synaptic activity levels [M]
        post_activity: Post-synaptic activity levels [M]
        target_activity: Target activity level
        scaling_rate: Rate of synaptic scaling
        threshold_rate: Rate of threshold adaptation
        output_weights: Updated weights [N] (pre-allocated)
        output_thresholds: Updated thresholds [M] (pre-allocated)
    
    Returns:
        Tuple of (synapses_updated, average_scaling_factor)
    """
    synapse_count = len(weights)
    neuron_count = len(thresholds)
    
    # Allocate output arrays if not provided
    if output_weights is None:
        output_weights = np.empty_like(weights)
    if output_thresholds is None:
        output_thresholds = np.empty_like(thresholds)
    
    # Step 1: Apply synaptic scaling based on post-synaptic activity
    synaptic_scaling(
        weights, target_activity, post_activity, scaling_rate,
        output_weights
    )
    
    # Step 2: Update intrinsic excitability (thresholds)
    intrinsic_excitability_update(
        thresholds, target_activity, post_activity, threshold_rate,
        output_thresholds
    )
    
    # Step 3: Apply weight normalization to prevent saturation
    weight_normalization(
        output_weights, source_neurons, target_neurons,
        target_activity * 10.0,  # Normalization factor
        output_weights
    )
    
    # Calculate statistics
    scaling_factors = output_weights / (weights + 1e-8)
    average_scaling = np.mean(scaling_factors)
    
    return synapse_count, float(average_scaling)
