"""Expansion Services Module.

This module provides complete synaptic rebuilding during cortical area
dimensional changes (expansion, contraction, reshaping), implementing 
morphology-based connection reconstruction.
"""

from .connection_analyzer import ConnectionAnalyzer
from .synaptic_rebuilder import SynapticRebuilder

__all__ = ["SynapticRebuilder", "ConnectionAnalyzer"]
