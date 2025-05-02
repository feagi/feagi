"""Top-level ZMQ package for FEAGI.

This module provides a compatibility layer that forwards to the new
implementation in feagi.api.zmq.
"""
import logging
from typing import Any, List, Optional

# Forward to new implementation
from feagi.api.zmq import (
    create_zmq_server,
    create_zmq_client,
    ZmqServer,
    ZmqClient
)

__all__ = ["create_zmq_server", "create_zmq_client", "ZmqServer", "ZmqClient"]

logger = logging.getLogger(__name__)
logger.warning("The 'feagi.zmq' module is deprecated. Please use 'feagi.api.zmq' instead.") 