"""API Gateway for FEAGI.

This module provides the API Gateway implementation for FEAGI,
handling authentication, routing, and protocol translation.

This implementation consolidates the functionality previously split between
feagi/api/gateway.py and feagi/api/gateway/api_gateway.py into a unified implementation.
"""

from feagi.api.gateway.api_gateway import APIGateway, get_api_gateway

__all__ = ["APIGateway", "get_api_gateway"] 