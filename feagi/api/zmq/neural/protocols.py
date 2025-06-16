"""
Neural data protocol definitions for FEAGI ZMQ communication.

This module defines the protocol identifiers and structures for neural data
transmission. All structures are designed to be fixed-size and compatible
with Rust FFI for future migration.
"""

from enum import IntEnum
from typing import Final

__all__ = [
    "NeuralProtocolID",
    "NEURAL_HEADER_SIZE",
    "MAX_NEURONS_PER_MESSAGE",
    "MAX_CORTICAL_AREAS",
]


class NeuralProtocolID(IntEnum):
    """Neural data protocol identifiers."""

    # Dense array formats
    NEURON_FLAT = 10  # Dense neural arrays (most common)
    NEURON_SPARSE = 11  # Sparse neural data (compressed)
    NEURON_MULTI = 12  # Multi-modal neural data

    # Structural formats
    CORTICAL_MAP = 13  # Cortical topology updates
    NEURAL_BURST = 14  # High-frequency burst data
    NEURAL_DELTA = 15  # Delta updates (changes only)

    # Control formats
    NEURAL_CONFIG = 20  # Neural configuration
    NEURAL_SYNC = 21  # Synchronization markers

    # Special formats
    NEURAL_COMPRESSED = 30  # Generic compressed format
    NEURAL_ENCRYPTED = 31  # Encrypted neural data


# Protocol constants
NEURAL_HEADER_SIZE: Final[int] = 32  # Fixed header size in bytes
MAX_NEURONS_PER_MESSAGE: Final[int] = 1_000_000  # 1M neurons max
MAX_CORTICAL_AREAS: Final[int] = 1024  # Max cortical areas
NEURAL_MAGIC: Final[bytes] = b"FEAG"  # Magic bytes for validation


# Compression types
class CompressionType(IntEnum):
    """Compression algorithms for neural data."""

    NONE = 0
    LZ4 = 1
    ZSTD = 2
    SNAPPY = 3


# Data precision types
class NeuralPrecision(IntEnum):
    """Neural data precision levels."""

    FLOAT32 = 0  # Full precision (default)
    FLOAT16 = 1  # Half precision
    INT16 = 2  # Quantized to 16-bit
    INT8 = 3  # Quantized to 8-bit
    BINARY = 4  # Binary spikes only


# Priority levels for QoS
class NeuralPriority(IntEnum):
    """Priority levels for neural data transmission."""

    REALTIME = 0  # Highest priority (motor commands)
    HIGH = 1  # High priority (sensory data)
    NORMAL = 2  # Normal priority (visualization)
    LOW = 3  # Low priority (logging, debug)
    BULK = 4  # Bulk transfer (topology updates)
