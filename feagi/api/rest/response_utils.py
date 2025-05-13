"""
Standardized response utilities for FEAGI REST API.
"""
from typing import Any, Dict, Optional, TypeVar, Generic, List
from pydantic import BaseModel
from datetime import datetime
import json
from fastapi.responses import JSONResponse

T = TypeVar('T')

class ApiResponse(BaseModel, Generic[T]):
    success: bool
    data: Optional[T] = None
    message: Optional[str] = None
    error_code: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    timestamp: str = datetime.now().isoformat()

def success_response(data=None, message=None, metadata=None):
    """Create a standardized success response"""
    return {
        "success": True,
        "data": data,
        "message": message,
        "metadata": metadata or {},
        "timestamp": datetime.now().isoformat()
    }

def error_response(message, error_code=None, metadata=None):
    """Create a standardized error response"""
    return {
        "success": False,
        "message": message,
        "error_code": error_code,
        "metadata": metadata or {},
        "timestamp": datetime.now().isoformat()
    }

def raw_response(data):
    """
    Mark a response to bypass standardization.
    Used for legacy v1 endpoints that need to maintain original format.
    
    Args:
        data: The response data to be returned without standardization
        
    Returns:
        The data with a special marker that the middleware will detect
    """
    if isinstance(data, dict):
        data["__raw_response__"] = True
    return data 