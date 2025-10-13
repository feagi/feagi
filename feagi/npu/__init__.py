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

from .coordinate_converter import CoordinateConverter
from .fire_candidate_list import FireCandidateList, FCLCandidate
from .fire_queue import FireQueue, FiringNeuron
from .fire_ledger import FireLedgerInterface
from .burst_engine import BurstEngine
from .fq_sampler import FQSampler, UnifiedFQSampler
from .fcl_injector import FCLInjector
from .fcl_manager import (
    FCLManager,
    EnhancedFCLManager, 
    BitMap,
    TimestepOutOfRangeError,
    FCLError,
    MembraneUpdate,
    NeuronCollection,
    NeuronCollectionType,
    example_enhanced_fcl_usage,
    example_fcl_usage,
    inject_neurons_into_fcl
)
from .gpu_fcl_adapter import (
    GPUBitMap,
    GPUAcceleratedFCL,
    create_gpu_accelerated_fcl,
    get_backend
)

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

from .special_area_handler import (
    SpecialAreaHandler,
    SpecialAreaConfig,
    CorticalId,
    NeuronId
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
    # Clean NPU Architecture
    'CoordinateConverter',
    'FireCandidateList', 
    'FCLCandidate',
    'FireQueue',
    'FiringNeuron', 
    'FireLedgerInterface',
    'BurstEngine',
    'FQSampler',
    'UnifiedFQSampler',
    'FCLInjector',
    
    # Legacy compatibility (FCL Managers)
    'FCLManager',
    'EnhancedFCLManager',
    'BitMap',
    'TimestepOutOfRangeError',
    'FCLError',
    'MembraneUpdate',
    'NeuronCollection',
    'NeuronCollectionType',
    'example_enhanced_fcl_usage',
    'example_fcl_usage',
    'inject_neurons_into_fcl',
    
    # GPU acceleration
    'GPUBitMap',
    'GPUAcceleratedFCL',
    'create_gpu_accelerated_fcl',
    'get_backend',
    
    # Core Data Structures (for FEAGI compatibility)
    'NeuronArray',
    'MemoryNeuronArray',
    'SynapseArray',
    'BackendType',
    'SIMDConfig',
    'SIMDDetector',
    'MemoryPatternKey',
    
    # Special Area Handling
    'SpecialAreaHandler',
    'SpecialAreaConfig', 
    'CorticalId',
    'NeuronId',
    
    # NPU Interface
    'NPUInterface',
    'OperationResult',
    'BatchOperationResult',
    'SynapseCreationRequest',
    'NeuronCreationRequest',
    'NeuronUpdateRequest'
]
