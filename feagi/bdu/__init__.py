"""
BDU (Brain Development Unit) - Core data structures and connectome management.

This module provides the fundamental data structures for FEAGI's neural simulation,
including the optimized ConnectomeManager.

All connectivity/morphology logic is now in Rust (feagi_bdu extension).
"""

# Core connectome management
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType

# Neuroembryogenesis
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

# Core models
from feagi.bdu.models.brain_region import BrainRegion
from feagi.bdu.models.cortical_area import CorticalArea

__all__ = [
    # Core connectome management
    "ConnectomeManager",
    "NeuronPropertyType",
    # Data models
    "CorticalArea",
    "BrainRegion",
    # Neuroembryogenesis
    "NeuroEmbryogenesis",
]
