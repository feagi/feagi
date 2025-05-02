"""ZeroMQ API implementation for FEAGI.

This module provides a ZeroMQ interface to FEAGI's functionality,
primarily for high-performance streaming data and real-time operations.
"""

from feagi.api.zmq.server import create_zmq_server
from feagi.api.zmq.client import create_zmq_client

__all__ = ["create_zmq_server", "create_zmq_client"] 