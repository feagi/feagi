"""
ZMQ Transport Adapter for FEAGI v1 API

This adapter provides ZMQ REST API access to FEAGI's v1 business logic.
It translates ZMQ REST requests to v1 API calls and returns ZMQ responses.

The adapter ensures that ZMQ clients get identical behavior to HTTP
clients by using the same underlying v1 business logic.
"""

import json
import time
import asyncio
import traceback
from typing import Dict, Any, Optional, Callable

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.system import create_system_api, SystemAPI
from feagi.api.v1.schemas import (
    UserPreferencesRequest, UserPreferencesResponse,
    VersionsResponse, HealthCheckResponse, ConfigurationResponse,
    InfluxDBTestResponse, CorticalAreaTypesResponse,
    SuccessResponse, ErrorResponse,
    RegistrationRequest, LogsRequest, SubscriberRequest,
    CircuitLibraryPathRequest
)
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class ZMQRestAdapter:
    """
    ZMQ Transport Adapter for FEAGI v1 API.
    
    This adapter converts ZMQ messages formatted like REST API calls into
    v1 API business logic calls, ensuring identical behavior to FastAPI.
    """
    
    def __init__(self, core_api_service: CoreAPIService):
        """
        Initialize the ZMQ REST adapter.
        
        Args:
            core_api_service: Core API service instance for processing requests
        """
        self.core_api_service = core_api_service
        self.system_api = create_system_api(core_api_service)
        self.route_handlers = {}
        self._initialize_route_handlers()
    
    def _initialize_route_handlers(self):
        """Initialize the mapping of routes to handler methods using v1 API."""
        self.route_handlers = {
            # ===== System Endpoints (using v1 API) =====
            "GET:/v1/system/user_preferences": self._handle_get_user_preferences,
            "PUT:/v1/system/user_preferences": self._handle_update_user_preferences,
            "GET:/v1/system/versions": self._handle_get_versions,
            "GET:/v1/system/health_check": self._handle_get_health_check,
            "GET:/v1/system/configuration": self._handle_get_configuration,
            "GET:/v1/system/db/influxdb/test": self._handle_test_influxdb,
            "POST:/v1/system/circuit_library_path": self._handle_set_circuit_library_path,
            "GET:/v1/system/cortical_area_types": self._handle_get_cortical_area_types,
            "POST:/v1/system/fcl_reset": self._handle_reset_fcl,
            "POST:/v1/system/register": self._handle_register_system,
            "POST:/v1/system/logs": self._handle_manage_logs,
            "GET:/v1/system/beacon/subscribers": self._handle_get_beacon_subscribers,
            "POST:/v1/system/beacon/subscribe": self._handle_subscribe_to_beacon,
            "DELETE:/v1/system/beacon/unsubscribe": self._handle_unsubscribe_from_beacon,
            "GET:/v1/system/version": self._handle_get_version,
        }
    
    async def process_message(self, message_data: bytes) -> bytes:
        """
        Process a REST API-style message received over ZMQ.
        
        Args:
            message_data: ZMQ message data containing REST API request
            
        Returns:
            Response data in REST API format
        """
        try:
            # Parse the message
            request = self._parse_message(message_data)
            if request is None:
                return self._create_error_response(400, "Invalid request format")
            
            # Process the request
            response = await self._process_request(request)
            
            # Return the response
            return json.dumps(response).encode('utf-8')
            
        except Exception as e:
            logger.error(f"Error processing REST API message: {str(e)}")
            logger.error(traceback.format_exc())
            return self._create_error_response(500, f"Internal server error: {str(e)}")
    
    def _parse_message(self, message_data: bytes) -> Optional[Dict[str, Any]]:
        """Parse ZMQ message data into a structured request."""
        try:
            request = json.loads(message_data.decode('utf-8'))
            
            if not isinstance(request, dict):
                logger.error(f"Request is not a dictionary: {type(request)}")
                return None
                
            # Required fields
            required_fields = ['route', 'method']
            for field in required_fields:
                if field not in request:
                    logger.error(f"Missing required field: {field}")
                    return None
            
            # Initialize optional fields if not present
            for field in ['params', 'query', 'body', 'headers']:
                if field not in request:
                    request[field] = {}
            
            # Add timestamp if not present
            if 'timestamp' not in request:
                request['timestamp'] = int(time.time() * 1000)
                
            return request
            
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse JSON: {str(e)}")
            return None
        except Exception as e:
            logger.error(f"Error parsing message: {str(e)}")
            return None
    
    async def _process_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """Process a parsed REST API request using v1 business logic."""
        route = request['route']
        method = request['method']
        params = request.get('params', {})
        query = request.get('query', {})
        body = request.get('body', {})
        headers = request.get('headers', {})
        
        # Create route key
        route_key = f"{method}:{route}"
        
        # Find handler
        handler = self.route_handlers.get(route_key)
        if handler:
            try:
                # Call the handler with the request components
                result = await handler(params, query, body, headers)
                return self._create_success_response(result)
            except ValueError as e:
                logger.error(f"Business logic error for {route_key}: {str(e)}")
                return self._create_error_response(400, str(e))
            except Exception as e:
                logger.error(f"Error in handler for {route_key}: {str(e)}")
                logger.error(traceback.format_exc())
                return self._create_error_response(500, f"Handler error: {str(e)}")
        else:
            logger.error(f"No handler found for route: {route_key}")
            return self._create_error_response(404, f"Endpoint not found: {route}")
    
    def _create_success_response(self, body: Any) -> Dict[str, Any]:
        """Create a success response."""
        # Handle Pydantic models
        if hasattr(body, 'dict'):
            body = body.dict()
        
        return {
            "status": 200,
            "headers": {"content-type": "application/json"},
            "body": body,
            "timestamp": int(time.time() * 1000)
        }
    
    def _create_error_response(self, status: int, message: str) -> Dict[str, Any]:
        """Create an error response."""
        return {
            "status": status,
            "headers": {"content-type": "application/json"},
            "body": {
                "type": "error",
                "code": f"ERROR_{status}",
                "message": message
            },
            "timestamp": int(time.time() * 1000)
        }
    
    # ===== System Handler Implementations (using v1 API) =====
    
    async def _handle_get_user_preferences(self, params, query, body, headers):
        """Handler for GET /v1/system/user_preferences"""
        return self.system_api.get_user_preferences()
    
    async def _handle_update_user_preferences(self, params, query, body, headers):
        """Handler for PUT /v1/system/user_preferences"""
        request = UserPreferencesRequest(**body)
        return self.system_api.update_user_preferences(request)
    
    async def _handle_get_versions(self, params, query, body, headers):
        """Handler for GET /v1/system/versions"""
        return self.system_api.get_versions()
    
    async def _handle_get_health_check(self, params, query, body, headers):
        """Handler for GET /v1/system/health_check"""
        return await self.system_api.get_health_check()
    
    async def _handle_get_configuration(self, params, query, body, headers):
        """Handler for GET /v1/system/configuration"""
        return self.system_api.get_configuration()
    
    async def _handle_test_influxdb(self, params, query, body, headers):
        """Handler for GET /v1/system/db/influxdb/test"""
        return self.system_api.test_influxdb()
    
    async def _handle_set_circuit_library_path(self, params, query, body, headers):
        """Handler for POST /v1/system/circuit_library_path"""
        path = body.get('path') if body else query.get('path')
        if not path:
            raise ValueError("Missing required parameter: path")
        return self.system_api.set_circuit_library_path(path)
    
    async def _handle_get_cortical_area_types(self, params, query, body, headers):
        """Handler for GET /v1/system/cortical_area_types"""
        return self.system_api.get_cortical_area_types()
    
    async def _handle_reset_fcl(self, params, query, body, headers):
        """Handler for POST /v1/system/fcl_reset"""
        return self.system_api.reset_fcl()
    
    async def _handle_register_system(self, params, query, body, headers):
        """Handler for POST /v1/system/register"""
        return self.system_api.register_system(body or {})
    
    async def _handle_manage_logs(self, params, query, body, headers):
        """Handler for POST /v1/system/logs"""
        return self.system_api.manage_logs(body or {})
    
    async def _handle_get_beacon_subscribers(self, params, query, body, headers):
        """Handler for GET /v1/system/beacon/subscribers"""
        return self.system_api.get_beacon_subscribers()
    
    async def _handle_subscribe_to_beacon(self, params, query, body, headers):
        """Handler for POST /v1/system/beacon/subscribe"""
        subscriber_address = body.get('subscriber_address') if body else query.get('subscriber_address')
        if not subscriber_address:
            raise ValueError("Missing required parameter: subscriber_address")
        return self.system_api.subscribe_to_beacon(subscriber_address)
    
    async def _handle_unsubscribe_from_beacon(self, params, query, body, headers):
        """Handler for DELETE /v1/system/beacon/unsubscribe"""
        subscriber_address = body.get('subscriber_address') if body else query.get('subscriber_address')
        if not subscriber_address:
            raise ValueError("Missing required parameter: subscriber_address")
        return self.system_api.unsubscribe_from_beacon(subscriber_address)
    
    async def _handle_get_version(self, params, query, body, headers):
        """Handler for GET /v1/system/version (legacy endpoint)"""
        versions = self.system_api.get_versions()
        return {"version": versions.feagi_core} 