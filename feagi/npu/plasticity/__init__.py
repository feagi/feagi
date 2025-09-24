"""
Plasticity package for FEAGI NPU

Deterministic, RTOS-friendly, Rust-ready plasticity and memory formation
implemented against the Fire Ledger history interface.

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

__all__ = [
    "STDPConfig",
    "STDPComputer",
    "MemoryConfig",
    "MemoryFormationManager",
    "PlasticityOrchestrator",
]


