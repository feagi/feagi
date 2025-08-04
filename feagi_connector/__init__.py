"""
FEAGI Connector SDK

Complete SDK library for connecting to FEAGI (Fractal Evolutionary Adaptive General Intelligence).
"""

from feagi_connector.client import FeagiClient
from feagi_connector.capabilities.manager import CapabilitiesManager
from feagi_connector.motor.processor import MotorProcessor
from feagi_connector.state.connection import ConnectionState
from feagi_connector.logging.setup import setup_agent_logging

# Export main classes and functions
__all__ = [
    "FeagiClient",
    "CapabilitiesManager", 
    "MotorProcessor",
    "ConnectionState",
    "setup_agent_logging",
]

__version__ = "1.0.0" 