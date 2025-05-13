"""
Common utilities and classes for the REST API.
"""

import queue
from feagi.utils.logger import setup_logger
logger = setup_logger("feagi.api.rest.commons")
from typing import Dict, Any, Optional
from fastapi import Request, HTTPException


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

async def check_burst_engine_or_allow_genome_ops(request: Request):
    """
    Similar to check_burst_engine, but also allows genome operations 
    when the burst engine is not yet running.
    """
    from feagi.core.state_manager import FeagiStateManager, ServiceState
    
    # Skip the check for genome loading/initial operations
    if request.url.path.endswith("/v1/genome/upload") or \
       request.url.path.endswith("/v1/genome/download") or \
       request.url.path.endswith("/v1/genome/genome_number"):
        return
        
    # Otherwise perform the standard check
    state_manager = FeagiStateManager.instance()
    if state_manager.get_burst_engine_state() != ServiceState.READY:
        raise HTTPException(status_code=400, detail="Burst engine is not running!") 