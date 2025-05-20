"""
FEAGI API Clients

This package contains client implementations for the different FEAGI APIs:
- Command API (REQ/REP pattern on port 5555)
- Sensorimotor API (DEALER/ROUTER pattern on port 5558)
- Visualization API (DEALER/ROUTER pattern on port 5560)
"""

from feagi_connector.api.command_client import FeagiCommandClient
from feagi_connector.api.sensory_client import FeagiSensoryClient
from feagi_connector.api.viz_client import FeagiVizClient

__all__ = ["FeagiCommandClient", "FeagiSensoryClient", "FeagiVizClient"] 