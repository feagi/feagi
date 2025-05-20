"""
ZeroMQ integration for FEAGI Connector.

This module provides ZeroMQ-based connectivity to FEAGI for agents.
"""

# Standard import - no excessive mocking
try:
    import zmq
    import zmq.asyncio
except ImportError:
    import logging
    import zmq
    logging.getLogger(__name__).warning(
        "zmq.asyncio not available - full ZMQ functionality will be limited"
    )

from .client import ZmqFeagiClient 