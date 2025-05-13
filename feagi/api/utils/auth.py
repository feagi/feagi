"""
Authentication utilities for FEAGI API.

This module provides authentication mechanisms for API access.
"""

from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import os
import time
from typing import Dict, Optional, List, Any, Union


# Simple in-memory token store for demonstration
# In a real implementation, this would use a more secure method
_TOKENS = {}

async def validate_token(token: str) -> bool:
    """
    Verify if a token is valid.
    
    Args:
        token: The token to verify
        
    Returns:
        True if the token is valid, False otherwise
    """
    # If no token is provided, allow access if authentication is disabled
    if not token:
        return not os.environ.get("FEAGI_AUTH_REQUIRED", "false").lower() == "true"
    
    # Check if token exists and is not expired
    token_data = _TOKENS.get(token)
    if not token_data:
        return False
    
    # Check expiration
    if token_data.get("expires_at", 0) < time.time():
        # Clean up expired token
        del _TOKENS[token]
        return False
    
    return True

async def generate_token(user_id: str, expires_in: int = 3600) -> str:
    """
    Create a new authentication token.
    
    Args:
        user_id: User identifier
        expires_in: Token expiration time in seconds
        
    Returns:
        New token string
    """
    import uuid
    
    token = str(uuid.uuid4())
    _TOKENS[token] = {
        "user_id": user_id,
        "created_at": time.time(),
        "expires_at": time.time() + expires_in
    }
    
    return token

async def invalidate_token(token: str) -> bool:
    """
    Invalidate an authentication token.
    
    Args:
        token: The token to invalidate
        
    Returns:
        True if the token was found and invalidated, False otherwise
    """
    if token in _TOKENS:
        del _TOKENS[token]
        return True
    return False

async def get_token_info(token: str) -> Optional[Dict[str, Any]]:
    """
    Get information about a token.
    
    Args:
        token: The token to get information about
        
    Returns:
        Token information or None if the token is invalid
    """
    return _TOKENS.get(token) 