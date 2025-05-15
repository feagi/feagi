"""
FEAGI Communication Protocols

This package implements the common protocols used for communication
between FEAGI and agents.
"""

# Protocol identifiers
class ProtocolID:
    """Protocol identifiers for FEAGI communication protocols."""
    FCP = 1  # FEAGI Control Protocol
    FVP = 2  # FEAGI Visualization Protocol
    FSMP = 3  # FEAGI Sensorimotor Protocol

# Import key components for easier access
from .fcp import FCPMessageType
from .fsmp import FSMPChannel
from .fvp import FVPFrameType
from .serialization import BinarySerializer

__all__ = [
    'ProtocolID',
    'FCPMessageType',
    'FSMPChannel',
    'FVPFrameType',
    'BinarySerializer',
] 