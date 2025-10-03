"""
FEAGI Protocol Constants

This module defines constants used throughout the FEAGI protocol implementation,
including protocol identifiers.
"""

from enum import IntEnum


class ProtocolID(IntEnum):
    """Protocol IDs for different FEAGI protocols."""

    FCP = 1   # FEAGI Control Protocol
    FVP = 2   # FEAGI Visualization Protocol  
    FSMP = 3  # FEAGI Sensorimotor Protocol 