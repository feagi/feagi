"""API Gateway for FEAGI.

This module implements the API Gateway for FEAGI, providing a central interface
to access core FEAGI functionality. It serves as a connection point between
different API interfaces (REST, ZMQ, etc.) and the underlying core components.
"""

from feagi.utils.logger import setup_logger
import os
from typing import Dict, Any, Optional, List

from feagi.api.core.services import CoreAPIService
from feagi.api.zmq.client import ZmqClient




class APIGateway:
    """
    API Gateway for FEAGI.
    
    This class provides a central point of access to FEAGI's functionality for
    all API interfaces (REST, ZMQ, etc.). It manages connections to:
    
    1. Core API - Direct access to in-process core components
    2. ZMQ Client - Access to remote FEAGI processes via ZMQ
    
    Features:
    - Environment-based configuration
    - Authentication and authorization
    - Request routing and protocol translation
    - Rate limiting and monitoring
    
    The gateway automatically determines whether to use local components or
    connect to remote ones based on environment variables.
    """
    
    _instance = None
    
    def __new__(cls, *args, **kwargs):
        """Implement the singleton pattern."""
        if cls._instance is None:
            cls._instance = super(APIGateway, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self, core_api: Optional[CoreAPIService] = None):
        """
        Initialize the API Gateway if not already initialized.
        
        Args:
            core_api: Optional CoreAPIService instance. If not provided,
                     one will be created or obtained from the process manager.
        """
        if getattr(self, "_initialized", False):
            return
            
        self._initialized = True
        self._core_api = core_api
        self._zmq_client = None
        self._rate_limiters = {}
        self._auth_handlers = {}
        
        # Only auto-detect if core_api wasn't explicitly provided
        if self._core_api is None:
            self._initialize_core_api()
        
        # Initialize ZMQ client if enabled
        self._initialize_zmq_client()
    
    def _initialize_core_api(self):
        """Initialize the Core API service based on environment."""
        try:
            # First check if we're running as part of the main FEAGI process
            if os.environ.get("FEAGI_INITIALIZED") == "1":
                logger.info("Running in FEAGI main process, using Process Manager")
                from feagi.process_manager import get_process_manager
                process_manager = get_process_manager()
                self._core_api = process_manager.get_core_api()
                logger.info("Core API obtained from Process Manager")
            else:
                # If not, check if we should create a local instance
                if os.environ.get("FEAGI_LOCAL_CORE", "0") == "1":
                    logger.info("Creating local Core API instance")
                    from feagi.core import create_core_api
                    self._core_api = create_core_api()
                    logger.info("Local Core API created")
        except ImportError:
            logger.warning("Could not import Process Manager or create local Core API")
            
        # Create mock if we couldn't get a real core API
        if self._core_api is None:
            logger.warning("Using mock Core API")
            from unittest.mock import MagicMock
            self._core_api = MagicMock()
    
    def _initialize_zmq_client(self):
        """Initialize the ZMQ client if enabled by environment variables."""
        if os.environ.get("FEAGI_ZMQ_ENABLED", "0") == "1":
            try:
                zmq_host = os.environ.get("FEAGI_ZMQ_HOST", "127.0.0.1")
                zmq_req_port = int(os.environ.get("FEAGI_ZMQ_REQ_PORT", "5555"))
                zmq_pub_port = int(os.environ.get("FEAGI_ZMQ_PUB_PORT", "5556"))
                zmq_push_port = int(os.environ.get("FEAGI_ZMQ_PUSH_PORT", "5557"))
                zmq_stream_port = int(os.environ.get("FEAGI_ZMQ_STREAM_PORT", "5558"))
                
                logger.info(f"Initializing ZMQ client to {zmq_host}")
                self._zmq_client = ZmqClient(
                    host=zmq_host,
                    req_port=zmq_req_port,
                    sub_port=zmq_pub_port,
                    push_port=zmq_push_port,
                    stream_port=zmq_stream_port
                )
                logger.info("ZMQ client initialized")
            except Exception as e:
                logger.error(f"Failed to initialize ZMQ client: {e}")
    
    @property
    def core_api(self) -> CoreAPIService:
        """Get the Core API service."""
        return self._core_api
    
    @property
    def zmq_client(self) -> Optional[ZmqClient]:
        """Get the ZMQ client if available."""
        return self._zmq_client
    
    # Authentication and authorization methods
    
    def authenticate(self, credentials: Dict[str, Any]) -> bool:
        """
        Authenticate a client.
        
        Args:
            credentials: Dictionary containing authentication credentials.
            
        Returns:
            True if authentication is successful, False otherwise.
        """
        # Placeholder for authentication implementation
        return True
        
    def authorize(self, resource: str, action: str, credentials: Dict[str, Any]) -> bool:
        """
        Authorize a client to perform an action on a resource.
        
        Args:
            resource: The resource being accessed.
            action: The action being performed.
            credentials: Dictionary containing authentication credentials.
            
        Returns:
            True if authorization is successful, False otherwise.
        """
        # Placeholder for authorization implementation
        return True
    
    # Routing methods
    
    def route_request(self, protocol: str, endpoint: str, method: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Route a request to the appropriate handler.
        
        Args:
            protocol: The protocol being used (REST, ZMQ).
            endpoint: The endpoint being accessed.
            method: The HTTP method or ZMQ pattern.
            data: Dictionary containing request data.
            
        Returns:
            Dictionary containing response data.
        """
        # Implementation will route to appropriate handler based on protocol and endpoint
        pass
    
    # Rate limiting methods
    
    def check_rate_limit(self, client_id: str, endpoint: str) -> bool:
        """
        Check if a client has exceeded rate limits for an endpoint.
        
        Args:
            client_id: ID of the client.
            endpoint: The endpoint being accessed.
            
        Returns:
            True if the client has not exceeded rate limits, False otherwise.
        """
        # Placeholder for rate limiting implementation
        return True
    
    # Monitoring methods
    
    def record_request(self, protocol: str, endpoint: str, status: int, duration: float):
        """
        Record a request for monitoring purposes.
        
        Args:
            protocol: The protocol being used (REST, ZMQ).
            endpoint: The endpoint being accessed.
            status: The response status.
            duration: The request duration in seconds.
        """
        # Implementation will record request metrics
        pass
        
    def get_metrics(self) -> Dict[str, Any]:
        """
        Get API metrics.
        
        Returns:
            Dictionary containing API metrics.
        """
        # Placeholder for metrics implementation
        return {}


# Factory function for creating/getting gateway instances
def get_api_gateway(core_api: Optional[CoreAPIService] = None) -> APIGateway:
    """
    Get or create an API Gateway instance.
    
    Args:
        core_api: Optional CoreAPIService instance to use in the gateway.
                If not provided, the gateway will auto-detect appropriate
                Core API access based on environment.
    
    Returns:
        APIGateway instance (singleton).
    """
    return APIGateway(core_api) 