"""
Shared Memory IPC for FEAGI.

This module provides shared memory-based inter-process communication for FEAGI,
replacing the ZMQ-based approach for higher performance and lower resource usage.
"""

from .manager import SharedMemoryManager
from .events import EventNotificationSystem
from .data_structures import SharedNeuronArray, SharedSynapseArray, SharedConfigDict

__all__ = [
    "SharedMemoryManager",
    "EventNotificationSystem", 
    "SharedNeuronArray",
    "SharedSynapseArray",
    "SharedConfigDict"
] 