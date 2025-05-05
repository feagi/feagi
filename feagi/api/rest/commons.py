"""
Common utilities and classes for the REST API.
"""

import queue
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger("feagi.api.rest.commons")

# Queue for API requests processing
api_queue = queue.Queue()


class CustomError(Exception):
    """
    Custom error class for API-specific exceptions.
    
    Attributes:
        message: Error message
        status_code: HTTP status code
        details: Additional error details
    """
    
    def __init__(
        self, 
        message: str, 
        status_code: int = 500, 
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize the custom error.
        
        Args:
            message: Error message
            status_code: HTTP status code
            details: Additional error details
        """
        self.message = message
        self.status_code = status_code
        self.details = details or {}
        super().__init__(self.message)
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the error to a dictionary representation.
        
        Returns:
            Dictionary containing error information
        """
        result = {
            "message": self.message,
            "status_code": self.status_code
        }
        
        if self.details:
            result["details"] = self.details
            
        return result 