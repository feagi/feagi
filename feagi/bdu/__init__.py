"""
BDU (Brain Data Unit) - Core data structures and connectome management.

This module provides the fundamental data structures for FEAGI's neural simulation,
including the optimized ConnectomeManager with Structure of Arrays (SoA) storage.
"""

from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType
from feagi.bdu.models.cortical_area import CorticalArea
from feagi.bdu.models.neuron import Neuron, NeuronArray

# Import core models
from feagi.bdu.models.brain_region import BrainRegion
from feagi.bdu.models.synapse import SynapseManager

# Import connectivity modules
from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.synaptogenesis import SynaptogenesisRule
from feagi.bdu.connectivity.synapse_rule import SynapseRule
from feagi.bdu.connectivity.mapping_utils import get_detailed_cortical_map, build_power_connections

# Import neuroembryogenesis
from feagi.bdu.embryogenesis.neuroembryogenesis import NeuroEmbryogenesis

__all__ = [
    # Core connectome management - optimized SoA implementation
    'ConnectomeManager',
    'NeuronPropertyType',
    
    # Data models
    'CorticalArea',
    'Neuron',
    'NeuronArray',
    
    # Core models
    'BrainRegion',
    'SynapseManager',
    
    # Connectivity modules
    'ConnectivityRule',
    'CorticalMapping',
    'SynaptogenesisRule',
    'SynapseRule',
    
    # Neuroembryogenesis
    'NeuroEmbryogenesis'
]
