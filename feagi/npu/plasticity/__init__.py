"""
Plasticity package for FEAGI NPU

Deterministic, RTOS-friendly, Rust-ready plasticity and memory formation
implemented against the Fire Ledger history interface.

Enhanced memory system features:
- RoaringBitmap-based pattern detection with SHA-256 hashing
- Memory neuron lifecycle management with SoA data structures
- CPU-optimized processing with minimal GPU interaction
- Global unique ID allocation with range partitioning

This package contains no burst-loop logic and no timing primitives. All
functions are pure or bounded, operate on pre-allocated arrays where
possible, and are designed to be migrated to a standalone Rust crate.
"""

from .stdp import STDPConfig, STDPComputer
from .memory import (
    MemoryConfig,
    MemoryFormationManager,
)
from .orchestrator import PlasticityOrchestrator
from .service import PlasticityService, PlasticityConfig
from .pattern_detector import PatternDetector, BatchPatternDetector, PatternConfig, TemporalPattern
from .memory_neuron_array import MemoryNeuronArray, MemoryNeuronLifecycleConfig, MemoryNeuronStats
from .neuron_id_manager import NeuronIdManager, get_neuron_id_manager, NeuronIdRanges

__all__ = [
    # Core plasticity components
    "STDPConfig",
    "STDPComputer",
    "MemoryConfig",
    "MemoryFormationManager",
    "PlasticityOrchestrator",
    "PlasticityService",
    "PlasticityConfig",
    
    # Memory system components
    "PatternDetector",
    "BatchPatternDetector",
    "PatternConfig",
    "TemporalPattern",
    "MemoryNeuronArray",
    "MemoryNeuronLifecycleConfig",
    "MemoryNeuronStats",
    
    # ID management
    "NeuronIdManager",
    "get_neuron_id_manager",
    "NeuronIdRanges",
]


