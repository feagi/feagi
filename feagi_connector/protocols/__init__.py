"""
Protocol definitions for FEAGI communication.
"""

from feagi_connector.protocols.constants import ProtocolID

# Simple FSMPChannel enum for backward compatibility
class FSMPChannel:
    VISION = 1
    AUDIO = 2
    TACTILE = 3
    PROPRIOCEPTION = 4
    OLFACTORY = 5
    GUSTATORY = 6
    TEXT = 7
    MOTOR_ARM = 101
    MOTOR_LEG = 102
    MOTOR_HAND = 103
    MOTOR_SPEECH = 104
    MOTOR_EYE = 105

__all__ = [
    "ProtocolID",
    "FSMPChannel",
] 