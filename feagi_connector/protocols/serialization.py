"""
Binary serialization for FEAGI protocols

This module provides serialization and deserialization functions for
the binary byte structures used in FEAGI communication protocols.

This is a compatibility module that re-exports classes from the feagi_bytes package.
"""

# Import from the PyPI feagi_bytes package
from feagi_bytes.serialization import (
    ByteStructureEncoder,
    ByteStructureDecoder,
    SUPPORTED_VERSIONS
)
from feagi_bytes.translator import ByteStructureTranslator

# Re-export the classes for backward compatibility
__all__ = [
    'ByteStructureEncoder',
    'ByteStructureDecoder',
    'ByteStructureTranslator',
    'SUPPORTED_VERSIONS'
] 