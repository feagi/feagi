"""Common utilities for FEAGI REST API."""

from feagi.api.rest.common.errors import ErrorResponse, APIError
from feagi.api.rest.common.rate_limit import RateLimiter

__all__ = ["ErrorResponse", "APIError", "RateLimiter"]

def raw_response(data):
    """
    Returns data directly without wrapping it in a standard response format.
    Used for maintaining backward compatibility with v1 API endpoints.
    """
    return data 