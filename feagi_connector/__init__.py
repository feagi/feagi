"""
FEAGI Connector
--------------

Client-side integration library for agents connecting to FEAGI.

FEAGI Connector provides a simple, high-level API for agents to connect to FEAGI
(Flexible & Extensible Artificial General Intelligence) and exchange data using
its communication protocols.
"""

__version__ = "0.1.0"

from feagi_connector.client import FeagiClient

__all__ = ["FeagiClient"] 