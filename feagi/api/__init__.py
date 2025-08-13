"""
FEAGI API Module

This module provides the main API interface for FEAGI, including both REST and ZMQ endpoints.
"""

import os

# Check if embedded mode is enabled
EMBEDDED_MODE = os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1"

if not EMBEDDED_MODE:
    from feagi.api.rest import create_rest_app
else:
    # In embedded mode, provide a stub function
    def create_rest_app(*args, **kwargs):
        raise RuntimeError("REST API is disabled in embedded mode")


# Core API exports
from feagi.api.core.services import CoreAPIService

# Gateway exports
from feagi.api.gateway import APIGateway, get_api_gateway

# REST API exports
from feagi.api.rest import create_rest_app

# ZMQ API exports
# ZMQ API is always available
from feagi.api.zmq import ZmqServer, create_zmq_client, create_zmq_server

__all__ = [
    "CoreAPIService",
    "APIGateway",
    "get_api_gateway",
    "create_rest_app",
    "create_zmq_server",
    "create_zmq_client",
    "ZmqServer",
]
