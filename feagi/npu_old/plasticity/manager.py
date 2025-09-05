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
Plasticity Manager - NPU Module

This module provides high-level coordination of all plasticity operations.
Designed as a lightweight coordinator that schedules and manages different
types of plasticity updates without blocking neural processing.

Key Features:
- Coordinates STP, LTP, LTD, and homeostatic plasticity
- Manages pruning schedules and thresholds
- Provides unified interface for all plasticity operations
- Zero-allocation runtime operation
- RTOS-friendly scheduling
- Rust FFI compatible

Design Principles:
- Lightweight coordinator (< 300 lines)
- All heavy computation delegated to specialized modules
- Pre-allocated buffers for all operations
- Configurable update schedules
- Non-blocking operation
"""

import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

import numpy as np

from .core import PlasticityType, batch_plasticity_update, validate_plasticity_arrays
from .pruning import combined_pruning, compact_synapse_arrays, update_activity_tracking
from .homeostatic import homeostatic_update_batch

logger = logging.getLogger(__name__)


@dataclass
class PlasticityConfig:
    """Configuration for plasticity operations.
    
    All parameters are Rust-compatible primitive types.
    """
    # Update schedules (in timesteps)
    plasticity_update_interval: int = 1
    pruning_update_interval: int = 100
    homeostatic_update_interval: int = 1000
    
    # Plasticity parameters
    stp_enabled: bool = True
    ltp_enabled: bool = True
    ltd_enabled: bool = True
    homeostatic_enabled: bool = True
    
    # Pruning thresholds
    weight_prune_threshold: float = 0.01
    activity_prune_threshold: float = 0.001
    age_prune_threshold: int = 10000
    
    # Homeostatic parameters
    target_activity: float = 0.1
    scaling_rate: float = 0.01
    threshold_adaptation_rate: float = 0.001
    
    # Performance parameters
    max_synapses_per_update: int = 100000
    enable_compaction: bool = True
    compaction_threshold: float = 0.1  # Compact when >10% synapses pruned


class PlasticityManager:
    """High-level plasticity coordination and scheduling.
    
    This class coordinates all plasticity operations while maintaining
    high performance and RTOS compatibility.
    """
    
    def __init__(self, max_synapses: int, config: Optional[PlasticityConfig] = None):
        """Initialize plasticity manager.
        
        Args:
            max_synapses: Maximum number of synapses to support
            config: Plasticity configuration (optional)
        """
        self.max_synapses = max_synapses
        self.config = config or PlasticityConfig()
        
        # Pre-allocate all working buffers (zero allocation during runtime)
        self._temp_weights = np.empty(max_synapses, dtype=np.float32)
        self._temp_mask = np.empty(max_synapses, dtype=np.bool_)
        self._temp_mask2 = np.empty(max_synapses, dtype=np.bool_)
        self._activity_buffer = np.zeros(max_synapses, dtype=np.float32)
        self._last_used = np.zeros(max_synapses, dtype=np.int32)
        
        # Scheduling state
        self._last_plasticity_update = 0
        self._last_pruning_update = 0
        self._last_homeostatic_update = 0
        
        # Statistics
        self._total_plasticity_updates = 0
        self._total_synapses_pruned = 0
        self._total_homeostatic_updates = 0
        
        logger.info(f"PlasticityManager initialized: {max_synapses:,} max synapses")
    
    def should_update_plasticity(self, timestep: int) -> bool:
        """Check if plasticity should be updated this timestep."""
        return (timestep - self._last_plasticity_update) >= self.config.plasticity_update_interval
    
    def should_update_pruning(self, timestep: int) -> bool:
        """Check if pruning should be updated this timestep."""
        return (timestep - self._last_pruning_update) >= self.config.pruning_update_interval
    
    def should_update_homeostatic(self, timestep: int) -> bool:
        """Check if homeostatic plasticity should be updated this timestep."""
        return (timestep - self._last_homeostatic_update) >= self.config.homeostatic_update_interval
    
    def update_plasticity(
        self,
        timestep: int,
        weights: np.ndarray,
        plasticity_types: np.ndarray,
        plasticity_coeffs: np.ndarray,
        activity_factors: np.ndarray,
        decay_rates: np.ndarray,
        scaling_exponents: np.ndarray,
        dt: float,
        output_weights: Optional[np.ndarray] = None
    ) -> int:
        """Update synaptic plasticity if scheduled.
        
        Args:
            timestep: Current simulation timestep
            weights: Current synaptic weights [N]
            plasticity_types: Plasticity type for each synapse [N]
            plasticity_coeffs: Plasticity coefficients [N]
            activity_factors: Activity factors [N]
            decay_rates: Decay rates [N]
            scaling_exponents: Scaling exponents [N]
            dt: Time step (seconds)
            output_weights: Output weights [N] (optional, uses internal buffer if None)
        
        Returns:
            Number of synapses updated (0 if not scheduled)
        """
        if not self.should_update_plasticity(timestep):
            return 0
        
        # Use internal buffer if no output provided
        if output_weights is None:
            output_weights = weights  # Update in-place
        
        # Validate arrays in debug mode
        if __debug__:
            if not validate_plasticity_arrays(
                weights, plasticity_types, plasticity_coeffs,
                activity_factors, decay_rates, scaling_exponents
            ):
                logger.error("Plasticity array validation failed")
                return 0
        
        # Perform batch plasticity update
        updated_count = batch_plasticity_update(
            weights, plasticity_types, plasticity_coeffs,
            activity_factors, decay_rates, scaling_exponents,
            dt, output_weights, self._temp_weights
        )
        
        # Update scheduling state
        self._last_plasticity_update = timestep
        self._total_plasticity_updates += 1
        
        return updated_count
    
    def update_pruning(
        self,
        timestep: int,
        weights: np.ndarray,
        activity_levels: np.ndarray,
        valid_mask: np.ndarray,
        output_mask: Optional[np.ndarray] = None
    ) -> Tuple[int, int, int, int]:
        """Update synaptic pruning if scheduled.
        
        Args:
            timestep: Current simulation timestep
            weights: Synaptic weights [N]
            activity_levels: Activity levels [N]
            valid_mask: Current validity mask [N]
            output_mask: Output validity mask [N] (optional)
        
        Returns:
            Tuple of (weight_pruned, activity_pruned, age_pruned, total_pruned)
        """
        if not self.should_update_pruning(timestep):
            return 0, 0, 0, 0
        
        # Use internal buffer if no output provided
        if output_mask is None:
            output_mask = valid_mask  # Update in-place
        
        # Perform combined pruning
        weight_pruned, activity_pruned, age_pruned, total_pruned = combined_pruning(
            weights, activity_levels, self._last_used, timestep,
            self.config.weight_prune_threshold,
            self.config.activity_prune_threshold,
            self.config.age_prune_threshold,
            valid_mask, output_mask, self._temp_mask
        )
        
        # Update scheduling state
        self._last_pruning_update = timestep
        self._total_synapses_pruned += total_pruned
        
        return weight_pruned, activity_pruned, age_pruned, total_pruned
    
    def update_homeostatic(
        self,
        timestep: int,
        weights: np.ndarray,
        thresholds: np.ndarray,
        source_neurons: np.ndarray,
        target_neurons: np.ndarray,
        pre_activity: np.ndarray,
        post_activity: np.ndarray,
        output_weights: Optional[np.ndarray] = None,
        output_thresholds: Optional[np.ndarray] = None
    ) -> Tuple[int, float]:
        """Update homeostatic plasticity if scheduled.
        
        Args:
            timestep: Current simulation timestep
            weights: Synaptic weights [N]
            thresholds: Neuron thresholds [M]
            source_neurons: Source neuron indices [N]
            target_neurons: Target neuron indices [N]
            pre_activity: Pre-synaptic activity [M]
            post_activity: Post-synaptic activity [M]
            output_weights: Output weights [N] (optional)
            output_thresholds: Output thresholds [M] (optional)
        
        Returns:
            Tuple of (synapses_updated, average_scaling_factor)
        """
        if not self.should_update_homeostatic(timestep):
            return 0, 1.0
        
        if not self.config.homeostatic_enabled:
            return 0, 1.0
        
        # Use input arrays if no output provided (in-place update)
        if output_weights is None:
            output_weights = weights
        if output_thresholds is None:
            output_thresholds = thresholds
        
        # Perform homeostatic update
        synapses_updated, avg_scaling = homeostatic_update_batch(
            weights, thresholds, source_neurons, target_neurons,
            pre_activity, post_activity,
            self.config.target_activity,
            self.config.scaling_rate,
            self.config.threshold_adaptation_rate,
            output_weights, output_thresholds
        )
        
        # Update scheduling state
        self._last_homeostatic_update = timestep
        self._total_homeostatic_updates += 1
        
        return synapses_updated, avg_scaling
    
    def update_activity_tracking(
        self,
        firing_neurons: List[int],
        source_neurons: np.ndarray,
        target_neurons: np.ndarray,
        timestep: int
    ) -> None:
        """Update activity tracking for synapses.
        
        This should be called every timestep to maintain accurate
        activity levels for pruning decisions.
        
        Args:
            firing_neurons: Neurons that fired this timestep
            source_neurons: Source neuron indices [N]
            target_neurons: Target neuron indices [N]
            timestep: Current simulation timestep
        """
        if len(firing_neurons) == 0:
            return
        
        firing_array = np.array(firing_neurons, dtype=np.int32)
        
        update_activity_tracking(
            firing_array, source_neurons, target_neurons,
            timestep, self._activity_buffer, self._last_used
        )
    
    def get_statistics(self) -> Dict[str, int]:
        """Get plasticity statistics.
        
        Returns:
            Dictionary of plasticity statistics
        """
        return {
            'total_plasticity_updates': self._total_plasticity_updates,
            'total_synapses_pruned': self._total_synapses_pruned,
            'total_homeostatic_updates': self._total_homeostatic_updates,
            'last_plasticity_update': self._last_plasticity_update,
            'last_pruning_update': self._last_pruning_update,
            'last_homeostatic_update': self._last_homeostatic_update
        }
    
    def reset_statistics(self) -> None:
        """Reset all statistics counters."""
        self._total_plasticity_updates = 0
        self._total_synapses_pruned = 0
        self._total_homeostatic_updates = 0
