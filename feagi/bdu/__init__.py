"""
BDU (Brain Dynamics Unit) package.

This package provides components for managing brain dynamics in FEAGI,
including the Connectome Manager for neuron and synapse operations.
"""

from feagi.bdu.connectome_manager import ConnectomeManager, NeuronPropertyType, CorticalArea

__all__ = [
    'ConnectomeManager', 
    'NeuronPropertyType', 
    'CorticalArea'
]
