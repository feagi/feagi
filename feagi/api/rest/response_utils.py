"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Standardized response utilities for FEAGI REST API.
"""
import json
from datetime import datetime
from typing import Any, Dict, Generic, List, Optional, TypeVar

from fastapi.responses import JSONResponse
from pydantic import BaseModel

T = TypeVar("T")


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
        "timestamp": datetime.now().isoformat(),
    }


def error_response(message, error_code=None, metadata=None):
    """Create a standardized error response"""
    return {
        "success": False,
        "message": message,
        "error_code": error_code,
        "metadata": metadata or {},
        "timestamp": datetime.now().isoformat(),
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
