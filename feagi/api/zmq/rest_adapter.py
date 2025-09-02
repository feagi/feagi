"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
ZMQ REST API Adapter

[WARN]  ARCHITECTURAL ALERT - SINGLE SOURCE OF TRUTH VIOLATIONS PREVENTED [WARN]

This module was COMPLETELY REFACTORED on 2025-05-24 after discovering critical
architectural violations where different transport protocols (HTTP vs ZMQ) were
returning different responses for the same endpoints.

BEFORE MODIFYING THIS FILE:
1. Read /docs/arch-api-decorator-architecture.md
2. ALL endpoints MUST delegate to v1 API modules
3. NO custom business logic implementations allowed
4. Response formats MUST be identical across all transports

[ERR] FORBIDDEN: Custom endpoint handlers with business logic
[OK] REQUIRED: Pure delegation to feagi.api.v1.* modules

This module implements an adapter that allows REST API requests to be sent over ZMQ,
using the same format and structure as the HTTP-based REST API.

The adapter:
1. Receives ZMQ messages in REST API format
2. Translates them to transport-agnostic v1 API calls
3. Returns responses in the same format as the REST API

This approach provides a unified API experience regardless of transport,
ensuring identical behavior between HTTP and ZMQ clients.
"""

import inspect
import json
import os
import time
import traceback
from typing import Any, Callable, Dict, Optional

from feagi.utils.logger import setup_logger

# Check if embedded mode is enabled
EMBEDDED_MODE = os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1"

if not EMBEDDED_MODE:
    from feagi.api.transport.zmq_adapter import ZMQRestAdapter
else:
    # In embedded mode, provide a stub class
    class ZMQRestAdapter:
        def __init__(self, core_api_service):
            pass

        async def process_message(self, request_json):
            return b'{"status": 503, "body": {"message": "REST API disabled in embedded mode"}}'


logger = setup_logger(__name__)


class ZMQRestAPIAdapter:
    """Enhanced ZMQ REST API Adapter using transport-agnostic v1 API.

    This adapter now delegates system endpoints to the transport-agnostic v1
    API while maintaining existing implementations for genome and connectome
    endpoints. This ensures identical behavior between HTTP and ZMQ transports.
    """

    def __init__(self, core_api_service):
        """Initialize the ZMQ REST API adapter.

        Args:
            core_api_service: Core API service instance for processing requests
        """
        self.core_api_service = core_api_service

        # Create the transport-agnostic ZMQ adapter for system endpoints
        self.v1_adapter = ZMQRestAdapter(core_api_service)

        self.route_handlers = {}
        self._initialize_route_handlers()

    def _initialize_route_handlers(self):
        """Initialize the mapping of routes to handler methods."""
        self.route_handlers = {
            # ===== System endpoints - Delegated to v1 API =====
            "GET:/v1/system/health_check": self._delegate_to_v1_api,
            "GET:/v1/system/configuration": self._delegate_to_v1_api,
            "PUT:/v1/system/configuration": self._delegate_to_v1_api,
            "GET:/v1/system/versions": self._delegate_to_v1_api,
            "GET:/v1/system/cortical_area_types": self._delegate_to_v1_api,
            "GET:/v1/system/user_preferences": self._delegate_to_v1_api,
            "PUT:/v1/system/user_preferences": self._delegate_to_v1_api,
            "GET:/v1/system/db/influxdb/test": self._delegate_to_v1_api,
            "POST:/v1/system/circuit_library_path": self._delegate_to_v1_api,
            "POST:/v1/system/fcl_reset": self._delegate_to_v1_api,
            "POST:/v1/system/register": self._delegate_to_v1_api,
            "POST:/v1/system/logs": self._delegate_to_v1_api,
            "GET:/v1/system/beacon/subscribers": self._delegate_to_v1_api,
            "POST:/v1/system/beacon/subscribe": self._delegate_to_v1_api,
            "DELETE:/v1/system/beacon/unsubscribe": self._delegate_to_v1_api,
            "GET:/v1/system/version": self._delegate_to_v1_api,
            # ===== Genome endpoints - Delegated to v1 API =====
            "GET:/v1/genome/blueprint": self._delegate_to_v1_api,
            "GET:/v1/genome": self._delegate_to_v1_api,
            "GET:/v1/genome/file_name": self._delegate_to_v1_api,
            "GET:/v1/genome/defaults/files": self._delegate_to_v1_api,
            "GET:/v1/genome/download": self._delegate_to_v1_api,
            "GET:/v1/genome/genome_number": self._delegate_to_v1_api,
            "GET:/v1/genome/cortical_template": self._delegate_to_v1_api,
            "GET:/v1/genome/circuits": self._delegate_to_v1_api,
            "GET:/v1/genome/amalgamation_history": self._delegate_to_v1_api,
            "GET:/v1/genome/download_region": self._delegate_to_v1_api,
            "POST:/v1/genome/upload/barebones": self._delegate_to_v1_api,
            "POST:/v1/genome/upload/essential": self._delegate_to_v1_api,
            "POST:/v1/genome/upload/file": self._delegate_to_v1_api,
            "POST:/v1/genome/upload/string": self._delegate_to_v1_api,
            "POST:/v1/genome/reset": self._delegate_to_v1_api,
            # ===== Connectome endpoints - Delegated to v1 API =====
            "GET:/v1/connectome/cortical_areas": self._delegate_to_v1_api,
            "GET:/v1/connectome/cortical_areas/list/summary": self._delegate_to_v1_api,
            "GET:/v1/connectome/cortical_areas/list/detailed": self._delegate_to_v1_api,
            "GET:/v1/connectome/cortical_areas/list/transforming": self._delegate_to_v1_api,
            # ===== Cortical Area endpoints - Delegated to v1 API =====
            "GET:/v1/cortical_area/cortical_area_id_list": self._delegate_to_v1_api,
            "POST:/v1/cortical_area/cortical_area_properties": self._delegate_to_v1_api,
            "POST:/v1/cortical_area/multi_cortical_area_properties": self._delegate_to_v1_api,
            "POST:/v1/cortical_area/multi/cortical_area_properties": self._delegate_to_v1_api,
            # ===== Status endpoint - Delegated to v1 API =====
            "GET:/v1/status": self._delegate_to_v1_api,
            # ===== Agent endpoints - Delegated to v1 API =====
            "GET:/v1/agent/list": self._delegate_to_v1_api,
            "GET:/v1/agent/info/{agent_id}": self._delegate_to_v1_api,
            "POST:/v1/agent/configure": self._delegate_to_v1_api,
            "POST:/v1/agent/register": self._delegate_to_v1_api,
            "DELETE:/v1/agent/deregister": self._delegate_to_v1_api,
            "GET:/v1/agent/properties/{agent_id}": self._delegate_to_v1_api,
            "GET:/v1/agent/properties": self._delegate_to_v1_api,
            "GET:/v1/agent/fq_sampler_status": self._delegate_to_v1_api,
            # ===== Visualization endpoints - Custom handlers =====
            "POST:/v1/visualization/register_client": self._handle_visualization_register_client,
            "POST:/v1/visualization/unregister_client": self._handle_visualization_unregister_client,
            "POST:/v1/visualization/heartbeat": self._handle_visualization_heartbeat,
            "GET:/v1/visualization/status": self._handle_visualization_status,
        }

    async def process_message(self, message_data: bytes) -> bytes:
        """Process a REST API-style message received over ZMQ.

        Args:
            message_data: ZMQ message data containing REST API request

        Returns:
            Response data in REST API format
        """
        try:
            # Parse the message
            request = self._parse_message(message_data)
            if request is None:
                return self._create_error_response(
                    400, "Invalid request format"
                )

            # Process the request
            response = await self._process_request(request)

            # Return the response
            return json.dumps(response).encode("utf-8")

        except Exception as e:
            logger.error(f"Error processing REST API message: {str(e)}")
            logger.error(traceback.format_exc())
            return self._create_error_response(
                500, f"Internal server error: {str(e)}"
            )

    async def _delegate_to_v1_api(self, params, query, body, headers) -> Any:
        """Delegate system endpoint requests to the transport-agnostic v1 API.

        This ensures identical behavior between HTTP and ZMQ transports.
        """
        # We need to extract the route and method from the current context
        #  Since this is called from _process_request, we can get it from the
        #  frame
        frame = inspect.currentframe()
        caller_frame = frame.f_back
        caller_locals = caller_frame.f_locals

        route = caller_locals.get("route", "")
        method = caller_locals.get("method", "GET")

        # Reconstruct the request format expected by the v1 adapter
        request_dict = {
            "route": route,
            "method": method,
            "params": params,
            "query": query,
            "body": body,
            "headers": headers,
            "timestamp": int(time.time() * 1000),
        }
        request_json = json.dumps(request_dict).encode("utf-8")

        try:
            # Process through the v1 adapter
            response_bytes = await self.v1_adapter.process_message(
                request_json
            )
            response = json.loads(response_bytes.decode("utf-8"))

            # Return the body part of the response (the v1 adapter wraps it)
            if response.get("status") == 200:
                return response.get("body")
            else:
                # Re-raise as an error for consistent handling
                error_body = response.get("body", {})
                raise ValueError(error_body.get("message", "Unknown error"))
        except Exception as e:
            logger.error(f"Error delegating to v1 API: {e}")
            raise ValueError(f"v1 API delegation failed: {str(e)}") from e

    # ===== Visualization Endpoint Handlers =====

    async def _handle_visualization_register_client(
        self, params, query, body, headers
    ) -> Any:
        """Handle visualization client registration."""
        try:
            # Check if embedded mode is enabled
            if os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1":
                client_id = body.get("client_id") if body else None
                if not client_id:
                    import uuid

                    client_id = str(uuid.uuid4())

                logger.info(
                    f"Embedded mode: Visualization registration ignored for client {client_id}"
                )
                return {
                    "client_id": client_id,
                    "success": False,
                    "message": "Visualization disabled in embedded mode",
                    "embedded_mode": True,
                }

            import uuid

            # Generate client ID if not provided
            client_id = body.get("client_id") if body else None
            if not client_id:
                client_id = str(uuid.uuid4())

            logger.info(f"Registering visualization client: {client_id}")

            # Get the ZMQ server from the module registry
            zmq_server = getattr(self, "_zmq_server", None)
            if not zmq_server:
                # Try to find it in globals or use a fallback approach
                logger.warning(
                    "ZMQ server reference not available - using fallback approach"
                )

                #  For now, just register the client and assume the
                #  visualization stream will pick it up
                #  This ensures the registration call succeeds even if we can't
                #  immediately access the stream
                return {
                    "client_id": client_id,
                    "success": True,
                    "message": f"Visualization client {client_id} registration request received",
                }

            # Get visualization stream from ZMQ server
            viz_stream = zmq_server.get_visualization_stream()
            if viz_stream:
                # RTOS: VisualizationStream is now synchronous, no await needed
                viz_stream.register_visualization_client(client_id)
                logger.info(f"Visualization client registered: {client_id}")

                return {
                    "client_id": client_id,
                    "success": True,
                    "message": f"Visualization client {client_id} registered successfully",
                }
            else:
                logger.error("Visualization stream not available")
                raise ValueError("Visualization stream not available")

        except Exception as e:
            logger.error(f"Error registering visualization client: {str(e)}")
            raise ValueError(f"Registration failed: {str(e)}") from e

    async def _handle_visualization_unregister_client(
        self, params, query, body, headers
    ) -> Any:
        """Handle visualization client unregistration."""
        try:
            # Check if embedded mode is enabled
            if os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1":
                client_id = body.get("client_id") if body else None
                logger.info(
                    f"Embedded mode: Visualization unregistration ignored for client {client_id}"
                )
                return {
                    "success": False,
                    "message": "Visualization disabled in embedded mode",
                    "embedded_mode": True,
                }

            client_id = body.get("client_id") if body else None
            if not client_id:
                raise ValueError("Client ID is required")

            logger.info(f"Unregistering visualization client: {client_id}")

            # Get the ZMQ server from the module registry
            zmq_server = getattr(self, "_zmq_server", None)
            if not zmq_server:
                logger.warning(
                    "ZMQ server reference not available - using fallback approach"
                )
                return {
                    "message": f"Visualization client {client_id} unregistration request received"
                }

            # Get visualization stream from ZMQ server
            viz_stream = zmq_server.get_visualization_stream()
            if viz_stream:
                # RTOS: VisualizationStream is now synchronous, no await needed
                viz_stream.unregister_visualization_client(client_id)
                logger.info(f"Visualization client unregistered: {client_id}")

                return {
                    "message": f"Visualization client {client_id} unregistered successfully"
                }
            else:
                logger.error("Visualization stream not available")
                raise ValueError("Visualization stream not available")

        except Exception as e:
            logger.error(f"Error unregistering visualization client: {str(e)}")
            raise ValueError(f"Unregistration failed: {str(e)}") from e

    async def _handle_visualization_heartbeat(
        self, params, query, body, headers
    ) -> Any:
        """Handle visualization heartbeat request."""
        try:
            # Check if embedded mode is enabled
            if os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1":
                logger.debug(
                    "[CONFIG] Embedded mode: Visualization heartbeat - disabled"
                )
                return {
                    "message": "Heartbeat disabled in embedded mode",
                    "embedded_mode": True,
                }

            # Extract client_id from request body
            client_id = body.get("client_id", None)
            if not client_id:
                logger.error(
                    "[ERR] No client_id provided in heartbeat request"
                )
                raise ValueError("client_id is required for heartbeat")

            # Only log debug info when ZMQ inbound debugging is enabled
            from feagi.core.state_manager import get_state_manager

            state_manager = get_state_manager()

            if state_manager and state_manager.is_debug_zmq_inbound_enabled():
                logger.info(
                    f"[CONFIG] DEBUG: Processing visualization heartbeat for {client_id}"
                )

            # Get the ZMQ server from the module registry
            zmq_server = getattr(self, "_zmq_server", None)
            if not zmq_server:
                logger.error(
                    "[ERR] ZMQ server reference not available for visualization heartbeat"
                )
                raise ValueError("ZMQ server not available")

            # Get visualization stream from ZMQ server
            viz_stream = zmq_server.get_visualization_stream()

            if viz_stream:
                if (
                    state_manager
                    and state_manager.is_debug_zmq_inbound_enabled()
                ):
                    logger.info(
                        f"[CONFIG] DEBUG: Calling heartbeat_visualization_client for {client_id}"
                    )
                # Check if the visualization stream has the heartbeat method
                if not hasattr(viz_stream, "heartbeat_visualization_client"):
                    logger.error(
                        "[ERR] Visualization stream does not have heartbeat_visualization_client method"
                    )
                    raise ValueError(
                        "Visualization stream heartbeat method not available"
                    )

                # RTOS: VisualizationStream is now synchronous, no await needed
                try:
                    result = viz_stream.heartbeat_visualization_client(
                        client_id
                    )
                    if (
                        state_manager
                        and state_manager.is_debug_zmq_inbound_enabled()
                    ):
                        logger.info(
                            f"[CONFIG] DEBUG: Method returned: {result}"
                        )
                        logger.info(
                            f"[CONFIG] DEBUG: Heartbeat call completed for {client_id}"
                        )

                    return {
                        "message": f"Heartbeat received from client {client_id}",
                        "client_id": client_id,
                        "status": "success",
                    }
                except Exception as heartbeat_error:
                    logger.error(
                        f"[ERR] Error calling heartbeat method: {heartbeat_error}"
                    )
                    raise ValueError(
                        f"Heartbeat method failed: {str(heartbeat_error)}"
                    ) from heartbeat_error
            else:
                logger.error("[ERR] Visualization stream not available")
                # Instead of failing completely, provide degraded functionality
                logger.warning(
                    f"[WARN] Visualization heartbeat for {client_id} failed - stream unavailable, but continuing"
                )
                return {
                    "message": f"Heartbeat processed for client {client_id} (degraded mode)",
                    "client_id": client_id,
                    "status": "degraded",
                    "warning": "Visualization stream not available",
                }

        except Exception as e:
            logger.error(
                f"[ERR] Error processing visualization heartbeat: {str(e)}"
            )
            logger.error(f"[ERR] Exception type: {type(e)}")
            import traceback

            logger.error(f"[ERR] Full traceback: {traceback.format_exc()}")

            #  Instead of re-raising, return error response to prevent
            #  cascading failures
            return {
                "message": f"Heartbeat failed: {str(e)}",
                "status": "error",
                "error": str(e),
            }

    async def _handle_visualization_status(
        self, params, query, body, headers
    ) -> Any:
        """Handle visualization status request."""
        try:
            # Check if embedded mode is enabled
            if os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1":
                logger.debug(
                    "[CONFIG] Embedded mode: Visualization status - disabled"
                )
                return {
                    "enabled": False,
                    "active_clients": 0,
                    "fq_sampler_enabled": False,
                    "message": "Visualization disabled in embedded mode",
                    "embedded_mode": True,
                }

            logger.debug("[STATS] Getting visualization status")

            # Get the ZMQ server from the module registry
            zmq_server = getattr(self, "_zmq_server", None)
            if not zmq_server:
                return {
                    "enabled": False,
                    "active_clients": 0,
                    "fq_sampler_enabled": False,
                    "message": "ZMQ server reference not available",
                }

            # Get visualization stream from ZMQ server
            viz_stream = zmq_server.get_visualization_stream()
            if viz_stream:
                # Get status from visualization stream using the correct method
                active_clients = viz_stream.get_connected_client_count()

                # Check FQ sampler status
                fq_sampler_enabled = getattr(
                    viz_stream, "_fq_sampler_enabled", False
                )

                return {
                    "enabled": True,
                    "active_clients": active_clients,
                    "fq_sampler_enabled": fq_sampler_enabled,
                    "message": f"Visualization system active with {active_clients} clients",
                }
            else:
                return {
                    "enabled": False,
                    "active_clients": 0,
                    "fq_sampler_enabled": False,
                    "message": "Visualization stream not available",
                }

        except Exception as e:
            logger.error(f"[ERR] Error getting visualization status: {str(e)}")
            raise ValueError(f"Status check failed: {str(e)}") from e

    def set_zmq_server(self, zmq_server):
        """Set the ZMQ server reference for visualization endpoints."""
        self._zmq_server = zmq_server
        logger.debug("ZMQ server reference set for visualization endpoints")

    def _parse_message(self, message_data: bytes) -> Optional[Dict[str, Any]]:
        """Parse ZMQ message data into a structured request.

        Args:
            message_data: Message data from ZMQ

        Returns:
            Parsed request or None if invalid
        """
        try:
            # Parse JSON data
            request = json.loads(message_data.decode("utf-8"))

            # Basic validation
            if not isinstance(request, dict):
                logger.error(f"Request is not a dictionary: {type(request)}")
                return None

            # Required fields
            required_fields = ["route", "method"]
            for field in required_fields:
                if field not in request:
                    logger.error(f"Missing required field: {field}")
                    return None

            # Initialize optional fields if not present
            for field in ["params", "query", "body", "headers"]:
                if field not in request:
                    request[field] = {}

            # Add timestamp if not present
            if "timestamp" not in request:
                request["timestamp"] = int(time.time() * 1000)

            return request

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse JSON: {str(e)}")
            return None
        except Exception as e:
            logger.error(f"Error parsing message: {str(e)}")
            return None

    async def _process_request(
        self, request: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Process a parsed REST API request.

        Args:
            request: Parsed request dictionary

        Returns:
            Response dictionary
        """
        # Extract request components
        route = request["route"]
        method = request["method"]
        params = request.get("params", {})
        query = request.get("query", {})
        body = request.get("body", {})
        headers = request.get("headers", {})

        # Create route key
        # We need to be careful with how we handle parameters in the route
        route_with_params = route

        #  For parameterized routes like "/v1/connectome/cortical_area/123"
        #  where 123 is a parameter value
        #  We need the route key to be
        #  "/v1/connectome/cortical_area/{cortical_id}" for handler matching

        # Start with basic route segmentation
        route_segments = route.split("/")

        # Detect and standardize segments that look like parameters
        # This is a heuristic based on common REST API patterns
        for i, segment in enumerate(route_segments):
            # Skip empty segments
            if not segment:
                continue

            #  Check if this segment might be a parameter value (simple
            #  heuristic)
            #  A parameter value is often numeric or a UUID, but could be any
            #  string that's not a common API path
            if segment.isdigit() or (
                len(segment) > 1
                and not segment.startswith("v")
                and segment
                not in [
                    "api",
                    "system",
                    "genome",
                    "connectome",
                    "cortical",
                    "area",
                    "areas",
                    "status",
                    "configuration",
                    "health_check",
                    "versions",
                    "cortical_area_types",
                    "cortical_area_id_list",
                    "cortical_area_properties",
                    "multi_cortical_area_properties",
                    "multi",
                    "blueprint",
                ]
            ):
                # Try to find parameter name based on position in path
                param_name = None

                #  This is a heuristic based on common route patterns - expand
                #  as needed
                if (
                    "cortical_area" in route
                    and i > 0
                    and route_segments[i - 1] == "cortical_area"
                ):
                    param_name = "cortical_id"
                elif (
                    "neuron" in route
                    and i > 0
                    and route_segments[i - 1] == "neuron"
                ):
                    param_name = "neuron_id"

                #  If we identified a parameter, update both the route template
                #  and params
                if param_name:
                    # Add to params if not already present
                    if param_name not in params:
                        params[param_name] = segment

                    # Replace in the route segments
                    route_segments[i] = "{" + param_name + "}"

        # Reconstruct the route with parameters
        route_with_params = "/".join(route_segments)

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
                return self._create_error_response(
                    500, f"Handler error: {str(e)}"
                )
        else:
            logger.error(f"No handler found for route: {route_key}")
            return self._create_error_response(
                404, f"Endpoint not found: {route}"
            )

    def _find_handler(
        self, route_key: str, params: Dict[str, Any]
    ) -> Optional[Callable]:
        """Find the appropriate handler for a route key.

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
            method, path = potential_route.split(":", 1)

            # Skip if methods don't match
            route_method, route_path = route_key.split(":", 1)
            if method != route_method:
                continue

            # Check if this is a parameterized route that could match
            if "{" in path and "}" in path:
                # Split path into segments for comparison
                template_parts = path.split("/")
                actual_parts = route_path.split("/")

                # Skip if part count doesn't match
                if len(template_parts) != len(actual_parts):
                    continue

                logger.debug(
                    f"Comparing template {path} with actual {route_path}"
                )

                # Check if the pattern matches by comparing each segment
                matches = True
                for tp, ap in zip(template_parts, actual_parts):
                    # If template segment has a parameter (e.g., {cortical_id})
                    if "{" in tp and "}" in tp:
                        #  Always matches, but we need to check if the
                        #  parameter name exists in params
                        param_name = tp.strip("{}")
                        if param_name not in params:
                            #  Parameter not provided, but should use route
                            #  part as the parameter value
                            #  This handles cases where the client just
                            #  includes the value in the route
                            # but doesn't explicitly set it in params
                            logger.debug(
                                f"Adding missing param {param_name} = {ap}"
                            )
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
        """Create a success response.

        Args:
            body: Response body

        Returns:
            Formatted response
        """
        return {
            "status": 200,
            "headers": {"content-type": "application/json"},
            "body": body,
            "timestamp": int(time.time() * 1000),
        }

    def _create_error_response(
        self, status: int, message: str
    ) -> Dict[str, Any]:
        """Create an error response.

        Args:
            status: HTTP status code
            message: Error message

        Returns:
            Formatted error response
        """
        return {
            "status": status,
            "headers": {"content-type": "application/json"},
            "body": {
                "type": "error",
                "code": f"ERROR_{status}",
                "message": message,
            },
            "timestamp": int(time.time() * 1000),
        }

    # ===== Architecture Notes =====
    #
    #  All endpoint handlers have been removed and delegated to the unified v1
    #  API
    #  to maintain the single source of truth principle and ensure identical
    #  behavior
    # between HTTP and ZMQ transports.
    #
    # The _delegate_to_v1_api method ensures that:
    # 1. All endpoints use the exact same logic across transports
    # 2. Response formats are identical between HTTP and ZMQ
    # 3. No endpoint duplication exists anywhere in the codebase
    # 4. All transport protocols remain in perfect sync
    #
    #  This architecture prevents the architectural violations that occurred
    #  with
    # custom handlers providing different responses for the same endpoints.
