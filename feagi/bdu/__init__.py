"""Brain Development Unit (BDU) for FEAGI.

The BDU is responsible for developing and managing the neural connectome
based on genomic instructions.
"""

from feagi.bdu.connectome_manager import ConnectomeManager

# Import core models
from feagi.bdu.models.neuron import Neuron
from feagi.bdu.models.cortical_area import CorticalArea

# Import connectivity modules
from feagi.bdu.connectivity.connectivity_rules import ConnectivityRule
from feagi.bdu.connectivity.cortical_mappings import CorticalMapping
from feagi.bdu.connectivity.synaptogenesis import SynaptogenesisRule

# Import neuroembryogenesis
from feagi.bdu.neuroembryogenesis import Neuroembryogenesis

__all__ = [
    'ConnectomeManager',
    'Neuron',
    'CorticalArea',
    'ConnectivityRule',
    'CorticalMapping',
    'SynaptogenesisRule',
    'Neuroembryogenesis'
]
