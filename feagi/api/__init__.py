"""API module for FEAGI."""

# Core API exports
from feagi.api.core.services import CoreAPIService

# Gateway exports
from feagi.api.gateway import APIGateway, get_api_gateway

# REST API exports
from feagi.api.rest import create_rest_app

# ZMQ API exports
from feagi.api.zmq import create_zmq_server, create_zmq_client

__all__ = [
    "CoreAPIService",
    "APIGateway",
    "get_api_gateway",
    "create_rest_app",
    "create_zmq_server", 
    "create_zmq_client"
] 