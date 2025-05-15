"""
FEAGI Communication Protocols

This package implements the common protocols used for communication
between FEAGI and agents using byte structures.
"""

# Byte structure identifiers
class ByteStructureID:
    """Byte structure identifiers for FEAGI communication."""
    JSON = 1
    RAW_IMAGE = 8
    MULTI_HOLDER = 9
    NEURON_FLAT = 10
    NEURON_CATEGORIES = 11

# Protocol types for clarity and backwards compatibility
class ProtocolType:
    """Protocol types for FEAGI communication."""
    FCP = "fcp"  # FEAGI Control Protocol
    FVP = "fvp"  # FEAGI Visualization Protocol
    FSMP = "fsmp"  # FEAGI Sensorimotor Protocol

# Import key components for easier access
from .fcp import FCPMessageType
from .fsmp import FSMPChannel
from .fvp import FVPFrameType
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureTranslator

__all__ = [
    'ByteStructureID',
    'ProtocolType',
    'FCPMessageType',
    'FSMPChannel',
    'FVPFrameType',
    'ByteStructureEncoder',
    'ByteStructureDecoder',
    'ByteStructureTranslator',
] 