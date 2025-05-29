"""
Neural data processing components for ZMQ.

This package provides neural-specific data structures, protocols,
and optimizations for high-performance neural data transmission.
"""

from .protocols import (
    NeuralProtocolID,
    CompressionType,
    NeuralPrecision,
    NeuralPriority,
    NEURAL_HEADER_SIZE,
    MAX_NEURONS_PER_MESSAGE,
    MAX_CORTICAL_AREAS,
    NEURAL_MAGIC,
)

from .headers import (
    NeuralDataHeader,
    NeuralHeaderError,
    parse_header,
    create_header,
)

from .ring_buffer import (
    ZeroCopyRingBuffer,
    BufferSlot,
    RingBufferError,
)

__all__ = [
    # Protocols
    'NeuralProtocolID',
    'CompressionType',
    'NeuralPrecision',
    'NeuralPriority',
    'NEURAL_HEADER_SIZE',
    'MAX_NEURONS_PER_MESSAGE',
    'MAX_CORTICAL_AREAS',
    'NEURAL_MAGIC',
    
    # Headers
    'NeuralDataHeader',
    'NeuralHeaderError',
    'parse_header',
    'create_header',
    
    # Ring Buffer
    'ZeroCopyRingBuffer',
    'BufferSlot',
    'RingBufferError',
] 