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
Core Synaptic Plasticity Operations - NPU Module

This module provides the fundamental plasticity operations for synaptic weight updates.
Designed for Rust migration and RTOS compatibility with:

- Small, focused functions (< 50 lines each)
- No dynamic memory allocation during runtime
- Vectorized operations for SIMD/GPU acceleration
- Deterministic execution paths
- Zero external dependencies beyond NumPy

Key Design Principles:
- Functions are pure (no side effects)
- All arrays are pre-allocated
- No Python objects created during runtime
- Compatible with Rust FFI boundaries
- RTOS-friendly (no blocking operations)
"""

import logging
from typing import Tuple
from enum import IntEnum

import numpy as np

logger = logging.getLogger(__name__)


class PlasticityType(IntEnum):
    """Plasticity types for synaptic updates.
    
    Using IntEnum for Rust FFI compatibility.
    """
    NONE = 0
    STP = 1   # Short-Term Plasticity
    LTP = 2   # Long-Term Potentiation
    LTD = 3   # Long-Term Depression


# Constants for weight bounds (Rust-compatible)
MIN_WEIGHT = 0.0
MAX_WEIGHT = 255.0
EPSILON = 1e-6


def stp_weight_update(
    current_weights: np.ndarray,
    plasticity_coeffs: np.ndarray,
    activity_factors: np.ndarray,
    decay_rates: np.ndarray,
    scaling_exponents: np.ndarray,
    dt: float,
    output_weights: np.ndarray
) -> None:
    """Short-Term Plasticity weight update (multiplicative).
    
    Implements STP using multiplicative scaling based on recent activity.
    All operations are vectorized for SIMD/GPU acceleration.
    
    Args:
        current_weights: Current synaptic weights [N]
        plasticity_coeffs: Plasticity coefficients [N]
        activity_factors: Recent activity levels [N]
        decay_rates: Decay rates for plasticity [N]
        scaling_exponents: Scaling exponents [N]
        dt: Time step (seconds)
        output_weights: Output array for updated weights [N] (pre-allocated)
    
    Note:
        All arrays must be the same length. No bounds checking for performance.
        Output array must be pre-allocated by caller.
    """
    # Vectorized STP computation
    # Formula: w_new = w_old * (coeff^exponent) * activity * (decay^dt)
    np.multiply(
        current_weights,
        np.power(plasticity_coeffs, scaling_exponents),
        out=output_weights
    )
    np.multiply(output_weights, activity_factors, out=output_weights)
    np.multiply(output_weights, np.power(decay_rates, dt), out=output_weights)
    
    # Apply bounds (vectorized)
    np.clip(output_weights, MIN_WEIGHT, MAX_WEIGHT, out=output_weights)


def ltp_weight_update(
    current_weights: np.ndarray,
    plasticity_coeffs: np.ndarray,
    activity_factors: np.ndarray,
    decay_rates: np.ndarray,
    scaling_exponents: np.ndarray,
    dt: float,
    output_weights: np.ndarray
) -> None:
    """Long-Term Potentiation weight update (additive).
    
    Implements LTP using additive updates based on correlated activity.
    All operations are vectorized for SIMD/GPU acceleration.
    
    Args:
        current_weights: Current synaptic weights [N]
        plasticity_coeffs: Plasticity coefficients [N]
        activity_factors: Activity correlation factors [N]
        decay_rates: Decay rates for plasticity [N]
        scaling_exponents: Scaling exponents [N]
        dt: Time step (seconds)
        output_weights: Output array for updated weights [N] (pre-allocated)
    
    Note:
        All arrays must be the same length. No bounds checking for performance.
        Output array must be pre-allocated by caller.
    """
    # Vectorized LTP computation
    # Formula: w_new = w_old + (coeff^exponent) * activity * (decay^dt) * w_old
    plasticity_delta = np.power(plasticity_coeffs, scaling_exponents)
    np.multiply(plasticity_delta, activity_factors, out=plasticity_delta)
    np.multiply(plasticity_delta, np.power(decay_rates, dt), out=plasticity_delta)
    np.multiply(plasticity_delta, current_weights, out=plasticity_delta)
    
    np.add(current_weights, plasticity_delta, out=output_weights)
    
    # Apply bounds (vectorized)
    np.clip(output_weights, MIN_WEIGHT, MAX_WEIGHT, out=output_weights)


def ltd_weight_update(
    current_weights: np.ndarray,
    plasticity_coeffs: np.ndarray,
    activity_factors: np.ndarray,
    decay_rates: np.ndarray,
    scaling_exponents: np.ndarray,
    dt: float,
    output_weights: np.ndarray
) -> None:
    """Long-Term Depression weight update (subtractive).
    
    Implements LTD using subtractive updates based on uncorrelated activity.
    All operations are vectorized for SIMD/GPU acceleration.
    
    Args:
        current_weights: Current synaptic weights [N]
        plasticity_coeffs: Plasticity coefficients [N]
        activity_factors: Activity correlation factors [N]
        decay_rates: Decay rates for plasticity [N]
        scaling_exponents: Scaling exponents [N]
        dt: Time step (seconds)
        output_weights: Output array for updated weights [N] (pre-allocated)
    
    Note:
        All arrays must be the same length. No bounds checking for performance.
        Output array must be pre-allocated by caller.
    """
    # Vectorized LTD computation (negative plasticity)
    # Formula: w_new = w_old - (coeff^exponent) * activity * (decay^dt) * w_old
    plasticity_delta = np.power(plasticity_coeffs, scaling_exponents)
    np.multiply(plasticity_delta, activity_factors, out=plasticity_delta)
    np.multiply(plasticity_delta, np.power(decay_rates, dt), out=plasticity_delta)
    np.multiply(plasticity_delta, current_weights, out=plasticity_delta)
    
    np.subtract(current_weights, plasticity_delta, out=output_weights)
    
    # Apply bounds (vectorized)
    np.clip(output_weights, MIN_WEIGHT, MAX_WEIGHT, out=output_weights)


def batch_plasticity_update(
    current_weights: np.ndarray,
    plasticity_types: np.ndarray,
    plasticity_coeffs: np.ndarray,
    activity_factors: np.ndarray,
    decay_rates: np.ndarray,
    scaling_exponents: np.ndarray,
    dt: float,
    output_weights: np.ndarray,
    temp_buffer: np.ndarray
) -> int:
    """Batch update synaptic weights based on plasticity types.
    
    Processes all synapses in vectorized batches for maximum performance.
    Uses pre-allocated temporary buffer to avoid memory allocation.
    
    Args:
        current_weights: Current synaptic weights [N]
        plasticity_types: Plasticity type for each synapse [N]
        plasticity_coeffs: Plasticity coefficients [N]
        activity_factors: Activity factors [N]
        decay_rates: Decay rates [N]
        scaling_exponents: Scaling exponents [N]
        dt: Time step (seconds)
        output_weights: Output array for updated weights [N] (pre-allocated)
        temp_buffer: Temporary buffer for calculations [N] (pre-allocated)
    
    Returns:
        Number of synapses updated
    
    Note:
        All arrays must be the same length and pre-allocated.
        No memory allocation occurs during execution.
    """
    synapse_count = len(current_weights)
    updated_count = 0
    
    # Copy current weights to output (default case)
    np.copyto(output_weights, current_weights)
    
    # Process STP synapses
    stp_mask = plasticity_types == PlasticityType.STP
    if np.any(stp_mask):
        stp_count = np.sum(stp_mask)
        stp_weight_update(
            current_weights[stp_mask],
            plasticity_coeffs[stp_mask],
            activity_factors[stp_mask],
            decay_rates[stp_mask],
            scaling_exponents[stp_mask],
            dt,
            temp_buffer[:stp_count]
        )
        output_weights[stp_mask] = temp_buffer[:stp_count]
        updated_count += stp_count
    
    # Process LTP synapses
    ltp_mask = plasticity_types == PlasticityType.LTP
    if np.any(ltp_mask):
        ltp_count = np.sum(ltp_mask)
        ltp_weight_update(
            current_weights[ltp_mask],
            plasticity_coeffs[ltp_mask],
            activity_factors[ltp_mask],
            decay_rates[ltp_mask],
            scaling_exponents[ltp_mask],
            dt,
            temp_buffer[:ltp_count]
        )
        output_weights[ltp_mask] = temp_buffer[:ltp_count]
        updated_count += ltp_count
    
    # Process LTD synapses
    ltd_mask = plasticity_types == PlasticityType.LTD
    if np.any(ltd_mask):
        ltd_count = np.sum(ltd_mask)
        ltd_weight_update(
            current_weights[ltd_mask],
            plasticity_coeffs[ltd_mask],
            activity_factors[ltd_mask],
            decay_rates[ltd_mask],
            scaling_exponents[ltd_mask],
            dt,
            temp_buffer[:ltd_count]
        )
        output_weights[ltd_mask] = temp_buffer[:ltd_count]
        updated_count += ltd_count
    
    return updated_count


def validate_plasticity_arrays(
    weights: np.ndarray,
    plasticity_types: np.ndarray,
    coeffs: np.ndarray,
    activity: np.ndarray,
    decay: np.ndarray,
    exponents: np.ndarray
) -> bool:
    """Validate plasticity arrays for consistency.
    
    Performs fast validation checks for array compatibility.
    Designed for debug builds only - can be compiled out in release.
    
    Args:
        weights: Weight array
        plasticity_types: Plasticity type array
        coeffs: Coefficient array
        activity: Activity array
        decay: Decay array
        exponents: Exponent array
    
    Returns:
        True if all arrays are compatible
    """
    if not all(arr.shape == weights.shape for arr in [plasticity_types, coeffs, activity, decay, exponents]):
        return False
    
    if not all(arr.dtype in [np.float32, np.int32, np.uint8] for arr in [weights, coeffs, activity, decay, exponents]):
        return False
    
    if plasticity_types.dtype not in [np.int32, np.uint8]:
        return False
    
    return True
