"""
FEAGI Neural Processing Unit (NPU) - Clean Architecture

This is a complete rewrite of the FEAGI NPU with clean separation of concerns:

Components:
- Fire Candidate List (FCL): Pre-burst candidate collection
- Fire Queue: Current timestep firing neurons  
- Fire Ledger: Historical firing data (Rust-optimized)
- Burst Engine: Clean orchestration of neural processing

Architecture Principles:
1. Single responsibility per component
2. No mixed concerns or timing issues
3. Rust/RTOS friendly data structures
4. Clear data flow: FCL → Fire Queue → Fire Ledger
5. High-performance SoA (Structure of Arrays) format
"""

from .fire_queue import FireQueue, FiringNeuron
from .burst_engine import BurstEngine
# FQSampler is now in Rust - use RustFQSamplerWrapper from rust_fq_sampler_wrapper
from .rust_fq_sampler_wrapper import RustFQSamplerWrapper

# Critical: Import data structures and interfaces that other parts of FEAGI depend on
from .data_structures import (
    NeuronArray,
    MemoryNeuronArray,
    SynapseArray,
    BackendType,
    SIMDConfig,
    SIMDDetector,
    MemoryPatternKey
)

from .interface import (
    NPUInterface,
    OperationResult,
    BatchOperationResult,
    SynapseCreationRequest,
    NeuronCreationRequest,
    NeuronUpdateRequest
)

__all__ = [
    # Core NPU Components
    'FireQueue',
    'FiringNeuron', 
    'BurstEngine',
    'RustFQSamplerWrapper',  # Rust FQ Sampler wrapper
    
    # Core Data Structures (for FEAGI compatibility)
    'NeuronArray',
    'MemoryNeuronArray',
    'SynapseArray',
    'BackendType',
    'SIMDConfig',
    'SIMDDetector',
    'MemoryPatternKey',
    
    # NPU Interface
    'NPUInterface',
    'OperationResult',
    'BatchOperationResult',
    'SynapseCreationRequest',
    'NeuronCreationRequest',
    'NeuronUpdateRequest'
]
