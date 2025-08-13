"""
Memory management components for zero-allocation neural data processing.

This package provides static buffer pools and memory management utilities
to eliminate dynamic allocation in the neural data critical path.
"""

from .buffer_pool import (
    Buffer,
    BufferPoolError,
    FixedBufferPool,
    NeuralBufferPool,
)

__all__ = [
    "FixedBufferPool",
    "NeuralBufferPool",
    "Buffer",
    "BufferPoolError",
]
