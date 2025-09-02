"""Neural data processing components for ZMQ.

This package provides neural-specific data structures, protocols, and
optimizations for high-performance neural data transmission.
"""

from .headers import (
    NeuralDataHeader,
    NeuralHeaderError,
    create_header,
    parse_header,
)
from .protocols import (
    MAX_CORTICAL_AREAS,
    MAX_NEURONS_PER_MESSAGE,
    NEURAL_HEADER_SIZE,
    NEURAL_MAGIC,
    CompressionType,
    NeuralPrecision,
    NeuralPriority,
    NeuralProtocolID,
)
from .ring_buffer import BufferSlot, RingBufferError, ZeroCopyRingBuffer

__all__ = [
    # Protocols
    "NeuralProtocolID",
    "CompressionType",
    "NeuralPrecision",
    "NeuralPriority",
    "NEURAL_HEADER_SIZE",
    "MAX_NEURONS_PER_MESSAGE",
    "MAX_CORTICAL_AREAS",
    "NEURAL_MAGIC",
    # Headers
    "NeuralDataHeader",
    "NeuralHeaderError",
    "parse_header",
    "create_header",
    # Ring Buffer
    "ZeroCopyRingBuffer",
    "BufferSlot",
    "RingBufferError",
]
