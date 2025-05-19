"""
FEAGI API Module

This package provides the API for interacting with FEAGI, including:
- Core API for brain simulation
- REST API
- Binary protocols
- Client libraries
"""

import importlib.util
import sys
from typing import Optional, Dict, Any, List, Union

# Import core components
# Import directly from api_gateway.py to avoid circular imports
from feagi.api.gateway.api_gateway import APIGateway, get_api_gateway

# Public exports
__all__ = [
    'APIGateway',
    'get_api_gateway'
] 