"""
BDU (Brain Data Unit) - Core data structures and connectome management.

This module provides the fundamental data structures for FEAGI's neural simulation,
including the optimized ConnectomeManager with Structure of Arrays (SoA) storage.
"""

# Import connectivity modules
from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.mapping_utils import (
    build_power_connections,
    get_detailed_cortical_map,
)
from feagi.bdu.connectivity.synapse_rule import SynapseRule
from feagi.bdu.connectivity.synaptogenesis import SynaptogenesisRule
from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType

# Import neuroembryogenesis
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

# Import core models
from feagi.bdu.models.brain_region import BrainRegion
from feagi.bdu.models.cortical_area import CorticalArea
from feagi.bdu.models.neuron import Neuron, NeuronArray
from feagi.bdu.models.synapse import SynapseManager

__all__ = [
    # Core connectome management - optimized SoA implementation
    "ConnectomeManager",
    "NeuronPropertyType",
    # Data models
    "CorticalArea",
    "Neuron",
    "NeuronArray",
    # Core models
    "BrainRegion",
    "SynapseManager",
    # Connectivity modules
    "ConnectivityRule",
    "CorticalMapping",
    "SynaptogenesisRule",
    "SynapseRule",
    # Neuroembryogenesis
    "NeuroEmbryogenesis",
]
