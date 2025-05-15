"""
FEAGI Bytes - Binary serialization for FEAGI protocols

This package provides binary serialization and deserialization for FEAGI communication protocols
using custom byte structures optimized for neural data.
"""

__version__ = "0.1.0"

# Import key components for easier access
from .constants import ByteStructureID
from .serialization import ByteStructureEncoder, ByteStructureDecoder
from .translator import ByteStructureTranslator

__all__ = [
    'ByteStructureID',
    'ByteStructureEncoder',
    'ByteStructureDecoder',
    'ByteStructureTranslator',
] 