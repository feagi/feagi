"""
BDU (Brain Data Unit) - Core data structures and connectome management.

This module provides the fundamental data structures for FEAGI's neural simulation,
including the optimized ConnectomeManager with Structure of Arrays (SoA) storage.
"""

# Import connectivity modules
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.mapping_utils import (
    build_power_connections,
    get_detailed_cortical_map,
)

# Function-based connectivity rules are imported via synaptogenesis
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType

# Import neuroembryogenesis
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

# Import core models
from feagi.bdu.models.brain_region import BrainRegion
from feagi.bdu.models.cortical_area import CorticalArea
from feagi.bdu.models.neuron import NeuronMappingProvider
from feagi.bdu.models.synapse import SynapseManager

__all__ = [
    # Core connectome management - optimized SoA implementation
    "ConnectomeManager",
    "NeuronPropertyType",
    # Data models
    "CorticalArea",
    "NeuronMappingProvider",
    # Core models
    "BrainRegion",
    "SynapseManager",
    # Connectivity modules
    "CorticalMapping",
    # Mapping utilities
    "build_power_connections",
    "get_detailed_cortical_map",
    # Neuroembryogenesis
    "NeuroEmbryogenesis",
]
