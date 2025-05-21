"""
FEAGI API Client Library

This module provides the client implementations for FEAGI.
"""

from .command_client import FeagiControlClient
from .sensory_client import FeagiSensoryClient
from .motor_client import FeagiMotorClient
from .viz_client import FeagiVizClient

__all__ = [
    'FeagiControlClient',
    'FeagiSensoryClient',
    'FeagiMotorClient',
    'FeagiVizClient'
] 