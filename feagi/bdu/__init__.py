"""Brain Development Unit (BDU) for FEAGI.

The BDU is responsible for developing and managing the neural connectome
based on genomic instructions.
"""

from feagi.bdu.connectome_manager import ConnectomeManager

# Import core models
from feagi.bdu.models.neuron import Neuron, NeuronArray
from feagi.bdu.models.cortical_area import CorticalArea
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
    'ConnectomeManager',
    'Neuron',
    'NeuronArray',
    'CorticalArea',
    'BrainRegion',
    'SynapseManager',
    'ConnectivityRule',
    'CorticalMapping',
    'SynaptogenesisRule',
    'SynapseRule',
    'NeuroEmbryogenesis'
]
