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
FEAGI Neural Processing Unit (NPU) - High-Performance Neural Computation

This package provides the core neural processing components for FEAGI 2.0,
designed for high-performance neural computation and Rust migration readiness.

Key Components:
- NeuralProcessor: Primary neural processing engine
- FCLManager: Fire Candidate List management
- PlasticityManager: Synaptic plasticity operations
- BurstEngine Integration: NPU integration with burst processing

Architecture:
- NPU is PRIMARY OWNER of neural data structures
- BDU gets controlled access during sleep periods
- Memory neurons remain BDU-owned (CPU-based)
- All runtime neural processing happens in NPU
"""

# Core NPU components
from .fcl_manager import FCLManager

# New NPU Interface and Data Structures (Single Source of Truth)
from .interface import NPUInterface, OperationResult, BatchOperationResult
from .data_structures import NeuronArray, MemoryNeuronArray, SynapseArray, BackendType

# Plasticity system
from .plasticity import (
    PlasticityManager,
    PlasticityConfig,
    PlasticityType
)

# Archived components removed - migration to new NPU Interface complete

# SIMD operations
from .simd_neural_ops import (
    simd_membrane_decay,
    simd_refractory_update,
    simd_firing_check,
    simd_firing_check_with_excitability,
    simd_batch_neural_update
)

__all__ = [
    # Core NPU
    'FCLManager',
    
    # New NPU Interface and Data Structures (Single Source of Truth)
    'NPUInterface',
    'OperationResult',
    'BatchOperationResult',
    'NeuronArray',
    'MemoryNeuronArray',
    'SynapseArray',
    'BackendType',
    
    # Plasticity system
    'PlasticityManager',
    'PlasticityConfig',
    'PlasticityType',
    
    # Archived components removed - migration complete
    
    # SIMD operations
    'simd_membrane_decay',
    'simd_refractory_update',
    'simd_firing_check',
    'simd_firing_check_with_excitability',
    'simd_batch_neural_update'
]
