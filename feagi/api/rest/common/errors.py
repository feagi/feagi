"""Error handling utilities for FEAGI REST API."""

from typing import Optional, Dict, Any
from pydantic import BaseModel

class ErrorResponse(BaseModel):
    """Standardized error response model."""
    code: int
    message: str
    details: Optional[Dict[str, Any]] = None

class APIError(Exception):
    """API error exception class for custom error handling."""
    
    def __init__(self, code: int, message: str, details: Optional[Dict[str, Any]] = None):
        """
        Initialize an API error.
        
        Args:
            code: HTTP status code.
            message: Error message.
            details: Optional details about the error.
        """
        self.code = code
        self.message = message
        self.details = details
        super().__init__(message)
        
    def to_response(self) -> ErrorResponse:
        """
        Convert the exception to an ErrorResponse.
        
        Returns:
            ErrorResponse instance.
        """
        return ErrorResponse(
            code=self.code,
            message=self.message,
            details=self.details
        ) 