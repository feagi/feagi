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
ZMQ Transport Adapter for FEAGI v1 API

[OK] ARCHITECTURAL COMPLIANCE - SINGLE SOURCE OF TRUTH ENFORCED [OK]

This adapter provides ZMQ REST API access to FEAGI's v1 business logic.
It translates ZMQ REST requests to v1 API calls and returns ZMQ responses.

🏗️ ARCHITECTURE PRINCIPLES:
1. All business logic resides in feagi.api.v1.* modules
2. This adapter provides PURE DELEGATION to v1 APIs
3. NO custom business logic implementations
4. Identical responses guaranteed across all transports

The adapter ensures that ZMQ clients get identical behavior to HTTP
clients by using the same underlying v1 business logic.

📖 See: /docs/arch-api-decorator-architecture.md for complete guidelines
"""

import json
import os
import time
import traceback
from typing import Any, Dict, Optional

# Check if embedded mode is enabled
EMBEDDED_MODE = os.environ.get("FEAGI_EMBEDDED_MODE", "0") == "1"

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.burst_engine import BurstEngineAPI
from feagi.api.v1.connectome import create_connectome_api
from feagi.api.v1.cortical_area import create_cortical_area_api
from feagi.api.v1.feagi_agent import create_feagi_agent_api
from feagi.api.v1.genome import create_genome_api
from feagi.api.v1.schemas import (
    AgentConfigRequest,
    AgentDeregistrationRequest,
    AgentRegistrationRequest,
    CorticalIdListRequest,
    CorticalIdRequest,
    UserPreferencesRequest,
)
from feagi.api.v1.system import create_system_api
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

if EMBEDDED_MODE:
    #  In embedded mode, provide a stub class that handles basic system
    #  endpoints
    # but disables full REST API functionality
    class ZMQRestAdapter:
        def __init__(self, core_api_service):
            self.core_api_service = core_api_service

        async def process_message(self, request_json):
            """Handle basic system endpoints in embedded mode."""
            try:
                import json

                request = json.loads(request_json.decode("utf-8"))
                route = request.get("route", "")
                method = request.get("method", "GET")

                #  Handle basic system endpoints that should work in embedded
                #  mode
                if route == "/v1/system/health_check" and method == "GET":
                    return self._create_success_response(
                        {
                            "status": "healthy",
                            "mode": "embedded",
                            "message": "FEAGI running in embedded mode",
                        }
                    )
                elif route == "/v1/system/version" and method == "GET":
                    return self._create_success_response(
                        {"version": "2.0.0", "mode": "embedded"}
                    )
                elif route == "/v1/system/configuration" and method == "GET":
                    return self._create_success_response(
                        {
                            "mode": "embedded",
                            "features": [
                                "zmq_control",
                                "zmq_sensory",
                                "zmq_motor",
                            ],
                            "disabled": [
                                "rest_api",
                                "visualization",
                                "web_interface",
                            ],
                        }
                    )
                else:
                    # For all other endpoints, return embedded mode error
                    return self._create_error_response(
                        503, "Advanced REST API disabled in embedded mode"
                    )

            except Exception as e:
                return self._create_error_response(
                    500, f"Error in embedded mode handler: {str(e)}"
                )

        def _create_success_response(self, body):
            """Create a success response."""
            import json
            import time

            response = {
                "status": 200,
                "headers": {"content-type": "application/json"},
                "body": body,
                "timestamp": int(time.time() * 1000),
            }
            return json.dumps(response).encode("utf-8")

        def _create_error_response(self, status, message):
            """Create an error response."""
            import json
            import time

            response = {
                "status": status,
                "headers": {"content-type": "application/json"},
                "body": {
                    "type": "error",
                    "code": f"ERROR_{status}",
                    "message": message,
                    "embedded_mode": True,
                },
                "timestamp": int(time.time() * 1000),
            }
            return json.dumps(response).encode("utf-8")

else:

    class ZMQRestAdapter:
        """ZMQ Transport Adapter for FEAGI v1 API.

        This adapter converts ZMQ messages formatted like REST API calls into
        v1 API business logic calls, ensuring identical behavior to FastAPI.
        """

        def __init__(self, core_api_service: CoreAPIService):
            """Initialize the ZMQ REST adapter.

            Args:
                core_api_service: Core API service instance for processing requests
            """
            self.core_api_service = core_api_service
            self.system_api = create_system_api(core_api_service)
            self.cortical_area_api = create_cortical_area_api(core_api_service)
            self.genome_api = create_genome_api(core_api_service)
            self.connectome_api = create_connectome_api(core_api_service)
            self.agent_api = create_feagi_agent_api(core_api_service)
            self.burst_engine_api = BurstEngineAPI(core_api_service)
            self.route_handlers = {}
            self._initialize_route_handlers()

        def _initialize_route_handlers(self):
            """Initialize the mapping of routes to handler methods using v1
            API."""
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
                # ===== Cortical Area Endpoints (using v1 API) =====
                "GET:/v1/cortical_area/cortical_area_id_list": self._handle_get_cortical_area_id_list,
                "POST:/v1/cortical_area/cortical_area_properties": self._handle_get_cortical_area_properties_body,
                "POST:/v1/cortical_area/multi_cortical_area_properties": self._handle_get_multi_cortical_area_properties,
                "POST:/v1/cortical_area/multi/cortical_area_properties": self._handle_get_multi_cortical_area_properties,
                # ===== Genome Endpoints (using v1 API) =====
                "GET:/v1/genome": self._handle_get_genome,
                "GET:/v1/genome/blueprint": self._handle_get_genome_blueprint,
                "GET:/v1/genome/file_name": self._handle_get_genome_file_name,
                "GET:/v1/genome/defaults/files": self._handle_get_genome_defaults,
                "GET:/v1/genome/download": self._handle_download_genome,
                "GET:/v1/genome/genome_number": self._handle_get_genome_number,
                "GET:/v1/genome/cortical_template": self._handle_get_cortical_template,
                "GET:/v1/genome/circuits": self._handle_get_circuits,
                "GET:/v1/genome/amalgamation_history": self._handle_get_amalgamation_history,
                "GET:/v1/genome/download_region": self._handle_download_genome_region,
                "POST:/v1/genome/upload/barebones": self._handle_upload_barebones_genome,
                "POST:/v1/genome/upload/essential": self._handle_upload_essential_genome,
                "POST:/v1/genome/upload/file": self._handle_upload_genome_file,
                "POST:/v1/genome/upload/string": self._handle_upload_genome_string,
                "POST:/v1/genome/reset": self._handle_reset_genome,
                # ===== Agent Endpoints (using v1 API) =====
                "GET:/v1/agent/list": self._handle_list_agents,
                "GET:/v1/agent/info/{agent_id}": self._handle_get_agent_info,
                "POST:/v1/agent/configure": self._handle_configure_agent,
                "POST:/v1/agent/register": self._handle_register_agent,
                "DELETE:/v1/agent/deregister": self._handle_deregister_agent,
                "GET:/v1/agent/properties/{agent_id}": self._handle_get_agent_properties,
                "GET:/v1/agent/shared_mem": self._handle_get_agent_shared_mem,
                "GET:/v1/agent/properties": self._handle_get_agent_properties_query,
                "GET:/v1/agent/fq_sampler_status": self._handle_get_fq_sampler_status,
                "POST:/v1/agent/heartbeat": self._handle_agent_heartbeat,
                # ===== Connectome Endpoints (using v1 API) =====
                "GET:/v1/connectome/cortical_areas": self._handle_get_connectome_cortical_areas,
                "GET:/v1/connectome/cortical_areas/list/summary": self._handle_get_cortical_areas_summary,
                "GET:/v1/connectome/cortical_areas/list/detailed": self._handle_get_cortical_areas_detailed,
                "GET:/v1/connectome/cortical_areas/list/transforming": self._handle_get_transforming_cortical_areas,
                # ===== Burst Engine Endpoints (using v1 API) =====
                "GET:/v1/burst_engine/simulation_timestep": self._handle_get_simulation_timestep,
                "POST:/v1/burst_engine/simulation_timestep": self._handle_change_simulation_timestep,
                "GET:/v1/burst_engine/status": self._handle_get_burst_engine_status,
                # ===== Status Endpoint (maps to health_check) =====
                "GET:/v1/status": self._handle_get_health_check,
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

        def _parse_message(
            self, message_data: bytes
        ) -> Optional[Dict[str, Any]]:
            """Parse ZMQ message data into a structured request."""
            try:
                request = json.loads(message_data.decode("utf-8"))

                if not isinstance(request, dict):
                    logger.error(
                        f"Request is not a dictionary: {type(request)}"
                    )
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
            """Process a parsed REST API request using v1 business logic."""
            route = request["route"]
            method = request["method"]
            params = request.get("params", {})
            query = request.get("query", {})
            body = request.get("body", {})
            headers = request.get("headers", {})

            # Create route key - handle parameterized routes
            route_key = f"{method}:{route}"

            # Check for exact match first
            handler = self.route_handlers.get(route_key)

            # If no exact match, try to match parameterized routes
            if not handler:
                for (
                    potential_route,
                    potential_handler,
                ) in self.route_handlers.items():
                    potential_method, potential_path = potential_route.split(
                        ":", 1
                    )

                    # Skip if methods don't match
                    if method != potential_method:
                        continue

                    # Check if this is a parameterized route that could match
                    if "{" in potential_path and "}" in potential_path:
                        # Split path into segments for comparison
                        template_parts = potential_path.split("/")
                        actual_parts = route.split("/")

                        # Skip if part count doesn't match
                        if len(template_parts) != len(actual_parts):
                            continue

                        #  Check if the pattern matches by comparing each
                        #  segment
                        matches = True
                        extracted_params = {}

                        for template_part, actual_part in zip(
                            template_parts, actual_parts
                        ):
                            #  If template segment has a parameter (e.g.,
                            #  {cortical_id})
                            if "{" in template_part and "}" in template_part:
                                param_name = template_part.strip("{}")
                                extracted_params[param_name] = actual_part
                            elif template_part != actual_part:
                                matches = False
                                break

                        if matches:
                            # Add extracted parameters to the params dict
                            params.update(extracted_params)
                            handler = potential_handler
                            route_key = potential_route
                            break

            if handler:
                try:
                    # Call the handler with the request components
                    result = await handler(params, query, body, headers)
                    return self._create_success_response(result)
                except ValueError as e:
                    logger.error(
                        f"Business logic error for {route_key}: {str(e)}"
                    )
                    return self._create_error_response(400, str(e))
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

        def _create_success_response(self, body: Any) -> Dict[str, Any]:
            """Create a success response."""
            # Handle Pydantic models
            if hasattr(body, "dict"):
                body = body.dict()

            return {
                "status": 200,
                "headers": {"content-type": "application/json"},
                "body": body,
                "timestamp": int(time.time() * 1000),
            }

        def _create_error_response(
            self, status: int, message: str
        ) -> Dict[str, Any]:
            """Create an error response."""
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

        # ===== System Handler Implementations (using v1 API) =====

        async def _handle_get_user_preferences(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/system/user_preferences."""
            return self.system_api.get_user_preferences()

        async def _handle_update_user_preferences(
            self, params, query, body, headers
        ):
            """Handler for PUT /v1/system/user_preferences."""
            request = UserPreferencesRequest(**body)
            return self.system_api.update_user_preferences(request)

        async def _handle_get_versions(self, params, query, body, headers):
            """Handler for GET /v1/system/versions."""
            return self.system_api.get_versions()

        async def _handle_get_health_check(self, params, query, body, headers):
            """Handler for GET /v1/system/health_check."""
            return await self.system_api.get_health_check()

        async def _handle_get_configuration(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/system/configuration."""
            return self.system_api.get_configuration()

        async def _handle_test_influxdb(self, params, query, body, headers):
            """Handler for GET /v1/system/db/influxdb/test."""
            return self.system_api.test_influxdb()

        async def _handle_set_circuit_library_path(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/system/circuit_library_path."""
            path = body.get("path") if body else query.get("path")
            if not path:
                raise ValueError("Missing required parameter: path")
            return self.system_api.set_circuit_library_path(path)

        async def _handle_get_cortical_area_types(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/system/cortical_area_types."""
            return self.system_api.get_cortical_area_types()

        async def _handle_reset_fcl(self, params, query, body, headers):
            """Handler for POST /v1/system/fcl_reset."""
            return self.system_api.reset_fcl()

        async def _handle_register_system(self, params, query, body, headers):
            """Handler for POST /v1/system/register."""
            return self.system_api.register_system(body or {})

        async def _handle_manage_logs(self, params, query, body, headers):
            """Handler for POST /v1/system/logs."""
            return self.system_api.manage_logs(body or {})

        async def _handle_get_beacon_subscribers(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/system/beacon/subscribers."""
            return self.system_api.get_beacon_subscribers()

        async def _handle_subscribe_to_beacon(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/system/beacon/subscribe."""
            subscriber_address = (
                body.get("subscriber_address")
                if body
                else query.get("subscriber_address")
            )
            if not subscriber_address:
                raise ValueError(
                    "Missing required parameter: subscriber_address"
                )
            return self.system_api.subscribe_to_beacon(subscriber_address)

        async def _handle_unsubscribe_from_beacon(
            self, params, query, body, headers
        ):
            """Handler for DELETE /v1/system/beacon/unsubscribe."""
            subscriber_address = (
                body.get("subscriber_address")
                if body
                else query.get("subscriber_address")
            )
            if not subscriber_address:
                raise ValueError(
                    "Missing required parameter: subscriber_address"
                )
            return self.system_api.unsubscribe_from_beacon(subscriber_address)

        async def _handle_get_version(self, params, query, body, headers):
            """Handler for GET /v1/system/version."""
            return self.system_api.get_version()

        # ===== Cortical Area Handler Implementations (using v1 API) =====

        async def _handle_get_cortical_area_id_list(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/cortical_area/cortical_area_id_list."""
            return self.cortical_area_api.get_cortical_area_id_list_legacy()

        async def _handle_get_cortical_area_properties_body(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/cortical_area/cortical_area_properties."""
            cortical_id = body.get("cortical_id") if body else None
            if not cortical_id:
                raise ValueError("Missing required parameter: cortical_id")

            request = CorticalIdRequest(cortical_id=cortical_id)
            return self.cortical_area_api.get_cortical_area_properties(request)

        async def _handle_get_multi_cortical_area_properties(
            self, params, query, body, headers
        ):
            """Handler for POST
            /v1/cortical_area/multi_cortical_area_properties."""
            cortical_id_list = []

            if body:
                if "cortical_id_list" in body:
                    cortical_id_list = body["cortical_id_list"]
                elif "cortical_ids" in body:
                    cortical_id_list = body["cortical_ids"]
                elif "cortical_id" in body:
                    cortical_id_list = [body["cortical_id"]]
                elif isinstance(body, list):
                    cortical_id_list = body

            if not cortical_id_list:
                # Get all cortical areas if no specific list provided
                cortical_id_list = (
                    self.cortical_area_api.get_cortical_area_id_list_legacy()
                )

            # Use the correct field name 'cortical_ids' for the schema
            request = CorticalIdListRequest(cortical_ids=cortical_id_list)
            return self.cortical_area_api.get_multiple_cortical_properties(
                request
            )

        # ===== Genome Handler Implementations (using v1 API) =====

        async def _handle_get_genome(self, params, query, body, headers):
            """Handler for GET /v1/genome."""
            #  This endpoint doesn't exist in v1 API yet, delegate to core
            #  service
            return self.core_api_service.get_genome()

        async def _handle_get_genome_blueprint(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/blueprint."""
            #  This endpoint doesn't exist in v1 API yet, delegate to core
            #  service
            genome = self.core_api_service.get_genome()
            return genome.get("cortical_areas", {}) if genome else {}

        async def _handle_get_genome_file_name(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/file_name."""
            return self.genome_api.get_genome_file_name_direct()

        async def _handle_get_genome_defaults(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/defaults/files."""
            return self.genome_api.get_default_genome_files()

        async def _handle_download_genome(self, params, query, body, headers):
            """Handler for GET /v1/genome/download."""
            return self.genome_api.download_genome()

        async def _handle_get_genome_number(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/genome_number."""
            return self.genome_api.get_genome_number()

        async def _handle_get_cortical_template(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/cortical_template."""
            return self.genome_api.get_cortical_template()

        async def _handle_get_circuits(self, params, query, body, headers):
            """Handler for GET /v1/genome/circuits."""
            return self.genome_api.get_circuit_library()

        async def _handle_get_amalgamation_history(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/amalgamation_history."""
            return self.genome_api.get_amalgamation_history()

        async def _handle_download_genome_region(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/genome/download_region."""
            region_id = query.get("region_id")
            if not region_id:
                raise ValueError("Missing required query parameter: region_id")
            return await self.genome_api.download_genome_from_region(region_id)

        async def _handle_upload_barebones_genome(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/genome/upload/barebones."""
            return await self.genome_api.upload_barebones_genome()

        async def _handle_upload_essential_genome(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/genome/upload/essential."""
            return await self.genome_api.upload_essential_genome()

        async def _handle_upload_genome_file(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/genome/upload/file."""
            if not body:
                raise ValueError("Missing genome data in request body")
            return await self.genome_api.upload_genome_file(body)

        async def _handle_upload_genome_string(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/genome/upload/string."""
            if not body:
                raise ValueError("Missing genome data in request body")
            return self.genome_api.upload_genome_string(body)

        async def _handle_reset_genome(self, params, query, body, headers):
            """Handler for POST /v1/genome/reset."""
            return await self.genome_api.reset_genome()

        # ===== Connectome Handler Implementations (using v1 API) =====

        async def _handle_get_connectome_cortical_areas(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/connectome/cortical_areas."""
            #  Use the working cortical area service method like the working
            #  endpoint
            cortical_area_ids = (
                self.cortical_area_api.get_cortical_area_id_list_legacy()
            )

            # Build cortical_areas dict with basic info for each area
            cortical_areas = {}
            for area_id in cortical_area_ids:
                try:
                    # Get basic properties for each area
                    from feagi.api.core.models.cortical_area_models import (
                        CorticalIdRequest,
                    )

                    request = CorticalIdRequest(cortical_id=area_id)
                    area_properties = (
                        self.cortical_area_api.get_cortical_area_properties(
                            request
                        )
                    )
                    cortical_areas[area_id] = area_properties
                except Exception as e:
                    # If properties fail, include basic info
                    cortical_areas[area_id] = {
                        "cortical_id": area_id,
                        "error": str(e),
                    }

            return {"cortical_areas": cortical_areas}

        async def _handle_get_cortical_areas_summary(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/connectome/cortical_areas/list/summary."""
            return await self.connectome_api.get_cortical_areas_summary()

        async def _handle_get_cortical_areas_detailed(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/connectome/cortical_areas/list/detailed."""
            return await self.connectome_api.get_cortical_areas_detailed()

        async def _handle_get_transforming_cortical_areas(
            self, params, query, body, headers
        ):
            """Handler for GET
            /v1/connectome/cortical_areas/list/transforming."""
            return await self.connectome_api.get_transforming_cortical_areas()

        # ===== Burst Engine Handler Implementations (using v1 API) =====

        async def _handle_get_simulation_timestep(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/burst_engine/simulation_timestep."""
            return self.burst_engine_api.get_simulation_timestep()

        async def _handle_change_simulation_timestep(
            self, params, query, body, headers
        ):
            """Handler for POST /v1/burst_engine/simulation_timestep."""
            #  Create SimulationTimestepRequest from the body using the new
            #  simplified format
            from feagi.api.v1.burst_engine import SimulationTimestepRequest

            #  Create the request object by unpacking the body (expects
            #  {"simulation_timestep": 0.1})
            request = SimulationTimestepRequest(**body)
            return self.burst_engine_api.change_simulation_timestep(request)

        async def _handle_get_burst_engine_status(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/burst_engine/status."""
            return await self.burst_engine_api.get_burst_engine_status()

        # ===== Agent Handler Implementations (using v1 API) =====

        async def _handle_list_agents(self, params, query, body, headers):
            """Handler for GET /v1/agent/list."""
            return await self.agent_api.list_agents()

        async def _handle_get_agent_info(self, params, query, body, headers):
            """Handler for GET /v1/agent/info/{agent_id}"""
            agent_id = params.get("agent_id")
            if not agent_id:
                raise ValueError("Missing required parameter: agent_id")
            return await self.agent_api.get_agent_info(agent_id)

        async def _handle_configure_agent(self, params, query, body, headers):
            """Handler for POST /v1/agent/configure."""
            request = (
                AgentConfigRequest(**body) if body else AgentConfigRequest()
            )
            return await self.agent_api.configure_agent(request)

        async def _handle_register_agent(self, params, query, body, headers):
            """Handler for POST /v1/agent/register."""
            if not body:
                raise ValueError(
                    "Missing agent registration data in request body"
                )
            request = AgentRegistrationRequest(**body)
            return await self.agent_api.register_agent(request)

        async def _handle_deregister_agent(self, params, query, body, headers):
            """Handler for DELETE /v1/agent/deregister."""
            if not body:
                raise ValueError(
                    "Missing agent deregistration data in request body"
                )
            request = AgentDeregistrationRequest(**body)
            return await self.agent_api.deregister_agent(request)

        async def _handle_agent_heartbeat(self, params, query, body, headers):
            """Handler for POST /v1/agent/heartbeat."""
            if not body:
                raise ValueError(
                    "Missing agent heartbeat data in request body"
                )
            request = AgentDeregistrationRequest(**body)  # Reuse minimal model with agent_id
            return await self.agent_api.heartbeat(request)

        async def _handle_get_agent_properties(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/agent/properties/{agent_id}"""
            agent_id = params.get("agent_id")
            if not agent_id:
                raise ValueError("Missing required parameter: agent_id")
            return await self.agent_api.get_agent_properties(agent_id)

        async def _handle_get_agent_properties_query(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/agent/properties?agent_id=<agent_id>"""
            agent_id = query.get("agent_id")
            if not agent_id:
                raise ValueError("Missing required query parameter: agent_id")
            return await self.agent_api.get_agent_properties(agent_id)

        async def _handle_get_agent_shared_mem(self, params, query, body, headers):
            """Handler for GET /v1/agent/shared_mem."""
            return await self.agent_api.list_agents_with_shared_mem()

        async def _handle_get_fq_sampler_status(
            self, params, query, body, headers
        ):
            """Handler for GET /v1/agent/fq_sampler_status."""
            return await self.agent_api.get_fq_sampler_status()
