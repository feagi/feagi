"""
FEAGI Connector - Agent-side integration for FEAGI

This package provides client-side integration for agents connecting to FEAGI,
with support for all FEAGI communication protocols.
"""

__version__ = "0.1.0"

from .client import FeagiClient
from .zmq_rest_client import FeagiZmqRestClient 