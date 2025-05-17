"""
ZMQ REST API Adapter

This module implements an adapter that allows REST API requests to be sent over ZMQ,
using the same format and structure as the HTTP-based REST API.

The adapter:
1. Receives ZMQ messages in REST API format
2. Translates them to internal API calls
3. Returns responses in the same format as the REST API

This approach provides a unified API experience regardless of transport.
"""

import json
import time
import asyncio
import traceback
from typing import Dict, Any, Optional, Tuple, Union, List, Callable

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

class ZMQRestAPIAdapter:
    """
    Adapter for handling REST API-style requests over ZMQ.
    
    This adapter converts ZMQ messages formatted like REST API calls into internal
    API service calls, and formats responses to match REST API responses.
    """
    
    def __init__(self, core_api_service):
        """
        Initialize the ZMQ REST API adapter.
        
        Args:
            core_api_service: Core API service instance for processing requests
        """
        self.core_api_service = core_api_service
        self.route_handlers = {}
        self._initialize_route_handlers()
        
    def _initialize_route_handlers(self):
        """Initialize the mapping of routes to handler methods."""
        # This could also be done with introspection of the core_api_service
        # or by importing the route mapping from the REST API module
        
        # Basic handlers for now, expand as needed
        self.route_handlers = {
            # System endpoints
            "GET:/v1/system/health_check": self._handle_health_check,
            "GET:/v1/system/configuration": self._handle_get_configuration,
            "PUT:/v1/system/configuration": self._handle_update_configuration,
            "GET:/v1/system/versions": self._handle_get_versions,
            "GET:/v1/system/cortical_area_types": self._handle_get_cortical_area_types,
            
            # Genome endpoints
            "GET:/v1/genome/blueprint": self._handle_get_genome_blueprint,
            "GET:/v1/genome": self._handle_get_genome,
            
            # Connectome endpoints
            "GET:/v1/connectome/cortical_areas": self._handle_get_cortical_areas,
            "GET:/v1/connectome/cortical_area/{cortical_id}": self._handle_get_cortical_area,
            
            # Status endpoint
            "GET:/v1/status": self._handle_status,
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
        """
        Parse ZMQ message data into a structured request.
        
        Args:
            message_data: Message data from ZMQ
            
        Returns:
            Parsed request or None if invalid
        """
        try:
            # Parse JSON data
            request = json.loads(message_data.decode('utf-8'))
            
            # Basic validation
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
        """
        Process a parsed REST API request.
        
        Args:
            request: Parsed request dictionary
            
        Returns:
            Response dictionary
        """
        # Extract request components
        route = request['route']
        method = request['method']
        params = request.get('params', {})
        query = request.get('query', {})
        body = request.get('body', {})
        headers = request.get('headers', {})
        
        # Create route key
        # We need to be careful with how we handle parameters in the route
        route_with_params = route
        
        # For parameterized routes like "/v1/connectome/cortical_area/123" where 123 is a parameter value
        # We need the route key to be "/v1/connectome/cortical_area/{cortical_id}" for handler matching
        
        # Start with basic route segmentation
        route_segments = route.split('/')
        
        # Detect and standardize segments that look like parameters
        # This is a heuristic based on common REST API patterns
        for i, segment in enumerate(route_segments):
            # Skip empty segments
            if not segment:
                continue
                
            # Check if this segment might be a parameter value (simple heuristic)
            # A parameter value is often numeric or a UUID, but could be any string that's not a common API path
            if segment.isdigit() or (len(segment) > 1 and not segment.startswith('v') and segment not in ['api', 'system', 'genome', 'connectome', 'cortical', 'area', 'areas', 'status', 'configuration', 'health_check', 'versions', 'cortical_area_types', 'blueprint']):
                # Try to find parameter name based on position in path
                param_name = None
                
                # This is a heuristic based on common route patterns - expand as needed
                if 'cortical_area' in route and i > 0 and route_segments[i-1] == 'cortical_area':
                    param_name = 'cortical_id'
                elif 'neuron' in route and i > 0 and route_segments[i-1] == 'neuron':
                    param_name = 'neuron_id'
                
                # If we identified a parameter, update both the route template and params
                if param_name:
                    # Add to params if not already present
                    if param_name not in params:
                        params[param_name] = segment
                    
                    # Replace in the route segments
                    route_segments[i] = '{' + param_name + '}'
        
        # Reconstruct the route with parameters
        route_with_params = '/'.join(route_segments)
        
        # Create the final route key
        route_key = f"{method}:{route_with_params}"
        
        # Find handler
        handler = self._find_handler(route_key, params)
        if handler:
            try:
                # Call the handler with the request components
                result = await handler(params, query, body, headers)
                return self._create_success_response(result)
            except Exception as e:
                logger.error(f"Error in handler for {route_key}: {str(e)}")
                logger.error(traceback.format_exc())
                return self._create_error_response(500, f"Handler error: {str(e)}")
        else:
            logger.error(f"No handler found for route: {route_key}")
            return self._create_error_response(404, f"Endpoint not found: {route}")
    
    def _find_handler(self, route_key: str, params: Dict[str, Any]) -> Optional[Callable]:
        """
        Find the appropriate handler for a route key.
        
        Args:
            route_key: Route key to find handler for
            params: Path parameters
            
        Returns:
            Handler function or None if not found
        """
        # Log route key and available handlers
        logger.debug(f"Looking for handler for route key: {route_key}")
        logger.debug(f"Available handlers: {list(self.route_handlers.keys())}")
        
        # Direct match
        if route_key in self.route_handlers:
            return self.route_handlers[route_key]
        
        # Try to match routes with parameters
        for potential_route, handler in self.route_handlers.items():
            method, path = potential_route.split(':', 1)
            
            # Skip if methods don't match
            route_method, route_path = route_key.split(':', 1)
            if method != route_method:
                continue
                
            # Check if this is a parameterized route that could match
            if '{' in path and '}' in path:
                # Split path into segments for comparison
                template_parts = path.split('/')
                actual_parts = route_path.split('/')
                
                # Skip if part count doesn't match
                if len(template_parts) != len(actual_parts):
                    continue
                
                logger.debug(f"Comparing template {path} with actual {route_path}")
                
                # Check if the pattern matches by comparing each segment
                matches = True
                for tp, ap in zip(template_parts, actual_parts):
                    # If template segment has a parameter (e.g., {cortical_id})
                    if '{' in tp and '}' in tp:
                        # Always matches, but we need to check if the parameter name exists in params
                        param_name = tp.strip('{}')
                        if param_name not in params:
                            # Parameter not provided, but should use route part as the parameter value
                            # This handles cases where the client just includes the value in the route
                            # but doesn't explicitly set it in params
                            logger.debug(f"Adding missing param {param_name} = {ap}")
                            params[param_name] = ap
                    elif tp != ap:
                        matches = False
                        break
                
                if matches:
                    logger.debug(f"Found matching route: {potential_route}")
                    return handler
        
        logger.error(f"No handler found for route: {route_key}")
        return None
    
    def _create_success_response(self, body: Any) -> Dict[str, Any]:
        """
        Create a success response.
        
        Args:
            body: Response body
            
        Returns:
            Formatted response
        """
        return {
            "status": 200,
            "headers": {
                "content-type": "application/json"
            },
            "body": body,
            "timestamp": int(time.time() * 1000)
        }
    
    def _create_error_response(self, status: int, message: str) -> Dict[str, Any]:
        """
        Create an error response.
        
        Args:
            status: HTTP status code
            message: Error message
            
        Returns:
            Formatted error response
        """
        return {
            "status": status,
            "headers": {
                "content-type": "application/json"
            },
            "body": {
                "type": "error",
                "code": f"ERROR_{status}",
                "message": message
            },
            "timestamp": int(time.time() * 1000)
        }
    
    # Handler implementations
    
    async def _handle_health_check(self, params, query, body, headers):
        """Handler for GET /v1/system/health_check"""
        return await self.core_api_service.get_system_health()
    
    async def _handle_get_configuration(self, params, query, body, headers):
        """Handler for GET /v1/system/configuration"""
        return self.core_api_service.get_configuration()
    
    async def _handle_update_configuration(self, params, query, body, headers):
        """Handler for PUT /v1/system/configuration"""
        success = self.core_api_service.update_configuration(body)
        if success:
            return {"status": "success", "message": "Configuration updated successfully"}
        else:
            raise ValueError("Failed to update configuration")
    
    async def _handle_get_versions(self, params, query, body, headers):
        """Handler for GET /v1/system/versions"""
        return self.core_api_service.get_versions()
    
    async def _handle_get_cortical_area_types(self, params, query, body, headers):
        """Handler for GET /v1/system/cortical_area_types"""
        return self.core_api_service.get_cortical_area_types()
    
    async def _handle_get_genome_blueprint(self, params, query, body, headers):
        """Handler for GET /v1/genome/blueprint"""
        genome = self.core_api_service.get_genome()
        return genome.get('cortical_areas', {})
    
    async def _handle_get_genome(self, params, query, body, headers):
        """Handler for GET /v1/genome"""
        return self.core_api_service.get_genome()
    
    async def _handle_get_cortical_areas(self, params, query, body, headers):
        """Handler for GET /v1/connectome/cortical_areas"""
        return self.core_api_service.get_cortical_areas()
    
    async def _handle_get_cortical_area(self, params, query, body, headers):
        """Handler for GET /v1/connectome/cortical_area/{cortical_id}"""
        cortical_id = params.get('cortical_id')
        if not cortical_id:
            raise ValueError("Missing required parameter: cortical_id")
        
        area = self.core_api_service.get_cortical_area(cortical_id)
        if not area:
            raise ValueError(f"Cortical area not found: {cortical_id}")
        
        return area
    
    async def _handle_status(self, params, query, body, headers):
        """Handler for GET /v1/status"""
        # Return the combined status of all FEAGI components
        return {
            "genome_availability": self.core_api_service.genome_is_loaded(),
            "genome_validity": self.core_api_service.genome_is_loaded(),  # Simplified for now
            "brain_readiness": self.core_api_service.get_state_manager().is_ready(),
            "burst_engine_status": str(self.core_api_service.get_state_manager().get_burst_engine_state()),
            "timestamp": int(time.time() * 1000)
        } 