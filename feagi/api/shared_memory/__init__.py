"""
Shared Memory IPC for FEAGI.

This module provides shared memory-based inter-process communication for FEAGI,
replacing the ZMQ-based approach for higher performance and lower resource usage.
"""

from .data_structures import (
    SharedConfigDict,
    SharedNeuronArray,
    SharedSynapseArray,
)
from .events import EventNotificationSystem
from .manager import SharedMemoryManager

__all__ = [
    "SharedMemoryManager",
    "EventNotificationSystem",
    "SharedNeuronArray",
    "SharedSynapseArray",
    "SharedConfigDict",
]
