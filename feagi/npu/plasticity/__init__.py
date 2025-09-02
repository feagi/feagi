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
NPU Plasticity Module - Modular Synaptic Plasticity System

This package provides a highly modular, RTOS-friendly synaptic plasticity system
designed for Rust migration and high-performance neural processing.

Architecture:
- core.py: Fundamental plasticity operations (STP, LTP, LTD)
- pruning.py: Synaptic pruning and maintenance operations  
- homeostatic.py: Homeostatic plasticity for network stability
- manager.py: High-level plasticity coordination and scheduling

Key Design Principles:
- Small, focused modules (< 200 lines each)
- Zero memory allocation during runtime
- Vectorized operations for SIMD/GPU acceleration
- Deterministic execution paths
- Rust FFI compatible interfaces
- RTOS-friendly (no blocking operations)

Usage:
    from feagi.npu.plasticity import PlasticityManager
    
    # Initialize plasticity system
    plasticity = PlasticityManager(max_synapses=1_000_000)
    
    # Update plasticity during neural processing
    updated_count = plasticity.update_plasticity(
        timestep=current_timestep,
        firing_neurons=fired_neurons,
        dt=0.001
    )
"""

from .core import (
    PlasticityType,
    stp_weight_update,
    ltp_weight_update,
    ltd_weight_update,
    batch_plasticity_update,
    validate_plasticity_arrays
)

from .pruning import (
    prune_by_weight,
    prune_by_activity,
    prune_by_age,
    combined_pruning,
    compact_synapse_arrays,
    update_activity_tracking
)

from .homeostatic import (
    synaptic_scaling,
    weight_normalization,
    intrinsic_excitability_update,
    activity_dependent_scaling,
    homeostatic_update_batch
)

from .manager import (
    PlasticityManager,
    PlasticityConfig
)

__all__ = [
    # Core plasticity
    'PlasticityType',
    'stp_weight_update',
    'ltp_weight_update', 
    'ltd_weight_update',
    'batch_plasticity_update',
    'validate_plasticity_arrays',
    
    # Pruning operations
    'prune_by_weight',
    'prune_by_activity',
    'prune_by_age',
    'combined_pruning',
    'compact_synapse_arrays',
    'update_activity_tracking',
    
    # Homeostatic plasticity
    'synaptic_scaling',
    'weight_normalization',
    'intrinsic_excitability_update',
    'activity_dependent_scaling',
    'homeostatic_update_batch',
    
    # Manager and configuration
    'PlasticityManager',
    'PlasticityConfig'
]
