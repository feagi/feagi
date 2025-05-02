"""REST API implementation for FEAGI.

This module provides a REST API interface to FEAGI's functionality,
primarily for management operations and configuration.
"""

from feagi.api.rest.app import create_rest_app

__all__ = ["create_rest_app"] 