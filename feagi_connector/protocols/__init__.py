"""
FEAGI Communication Protocols

This module defines the communication protocols used by FEAGI:

- FSMP (FEAGI Sensorimotor Protocol): For exchanging sensory and motor data
- FVP (FEAGI Visualization Protocol): For receiving neural activity and structure data
- FCP (FEAGI Control Protocol): For agent registration and control
"""

from enum import Enum, auto


class FSMPChannel(Enum):
    """
    FSMP Channel IDs for different sensory and motor modalities.
    """
    # Sensory channels
    VISION = 1
    AUDIO = 2
    TACTILE = 3
    PROPRIOCEPTION = 4
    OLFACTORY = 5
    GUSTATORY = 6
    TEXT = 7
    
    # Motor channels
    MOTOR_ARM = 101
    MOTOR_LEG = 102
    MOTOR_HAND = 103
    MOTOR_SPEECH = 104
    MOTOR_EYE = 105


class FCPMessageType(Enum):
    """
    FCP Message types for control protocol.
    """
    HELLO = auto()
    WELCOME = auto()
    CAPABILITIES = auto()
    ACK = auto()
    HEARTBEAT = auto()
    BYE = auto()
    ERROR = auto()


class FVPMessageType(Enum):
    """
    FVP Message types for visualization protocol.
    """
    ACTIVITY_REQUEST = auto()
    ACTIVITY_DATA = auto()
    STRUCTURE_REQUEST = auto()
    STRUCTURE_DATA = auto()


__all__ = ["FSMPChannel", "FCPMessageType", "FVPMessageType"] 