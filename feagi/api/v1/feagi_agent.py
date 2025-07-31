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
FEAGI v1 Agent API - Single Source of Truth
"""


from fastapi import HTTPException

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

try:
    from feagi.config.toml_loader import load_feagi_config, get_agent_config
except ImportError:
    # Handle cases where configuration might not be available
    load_feagi_config = None
    get_agent_config = None

from .decorators import endpoint
from .schemas import (
    AgentConfigRequest,
    AgentDeregistrationRequest,
    AgentInfoResponse,
    AgentListResponse,
    AgentPropertiesResponse,
    AgentRegistrationRequest,
    SuccessResponse,
)

logger = setup_logger(__name__)


def agent_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="feagi_agent",
    )


class FeagiAgentAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
        self.logger = logger

    @agent_endpoint("GET", "/list", response_model=AgentListResponse)
    async def list_agents(self) -> AgentListResponse:
        try:
            # Delegate to Registration Manager for consistent agent listing
            from feagi.pns.registration_manager import get_registration_manager

            registration_manager = get_registration_manager()
            if not registration_manager:
                self.logger.error("Registration Manager not available")
                raise ValueError("Registration service unavailable")

            agents_data = registration_manager.list_agents()

            # Extract just the agent IDs for the simple list format expected by AgentListResponse
            agent_ids = [agent["agent_id"] for agent in agents_data.get("agents", [])]

            return AgentListResponse(root=agent_ids)
        except Exception as e:
            self.logger.error(f"Error listing agents: {e}")
            raise ValueError(f"Failed to list agents: {str(e)}")

    @agent_endpoint("GET", "/info/{agent_id}", response_model=AgentInfoResponse)
    async def get_agent_info(self, agent_id: str) -> AgentInfoResponse:
        try:
            agent_info = self.core_api_service.get_agent_properties(agent_id)
            if not agent_info:
                raise ValueError(f"Agent {agent_id} not found")
            return AgentInfoResponse(agent_info=agent_info)
        except Exception as e:
            raise ValueError(f"Failed to get agent info: {str(e)}")

    @agent_endpoint(
        "POST",
        "/configure",
        request_model=AgentConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_agent(self, request: AgentConfigRequest) -> SuccessResponse:
        try:
            success = self.core_api_service.configure_agent(
                request.agent_id, request.config
            )
            if not success:
                raise ValueError("Failed to configure agent")
            return SuccessResponse(message="Agent configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure agent: {str(e)}")

    @agent_endpoint(
        "POST",
        "/register",
        request_model=AgentRegistrationRequest,
        response_model=SuccessResponse,
    )
    async def register_agent(
        self, request: AgentRegistrationRequest
    ) -> SuccessResponse:
        try:
            # Delegate to Registration Manager for centralized agent coordination
            from feagi.pns.registration_manager import (
                AgentRegistrationRequest as RegistrationRequest,
            )
            from feagi.pns.registration_manager import get_registration_manager

            registration_manager = get_registration_manager()
            if not registration_manager:
                self.logger.error("Registration Manager not available")
                raise HTTPException(
                    status_code=503, detail="Registration service unavailable"
                )

            # Create registration request for Registration Manager
            reg_request = RegistrationRequest(
                agent_id=request.agent_id,
                agent_type=request.agent_type,
                capabilities=request.capabilities,
                agent_data_port=request.agent_data_port,
                agent_version=request.agent_version,
                controller_version=request.controller_version,
                agent_ip=request.agent_ip,  # Let RegistrationRequest handle None with configuration
            )

            # Process registration through Registration Manager
            response = registration_manager.register_agent(reg_request)

            if response.success:
                self.logger.info(
                    f"✅ Agent '{request.agent_id}' registered via Registration Manager - "
                    f"FQ samplers coordinated: {response.fq_samplers_enabled}"
                )

                return SuccessResponse(message=response.message, success=True)
            else:
                self.logger.error(f"❌ Registration Manager failed: {response.message}")

                # Map Registration Manager error codes to HTTP status codes
                status_map = {
                    "DUPLICATE_AGENT_ID": 409,  # Conflict
                    "VALIDATION_FAILED": 400,  # Bad Request
                    "FEAGI_NOT_READY": 503,  # Service Unavailable
                    "MISSING_AGENT_ID": 400,  # Bad Request
                    "MISSING_CAPABILITIES": 400,  # Bad Request
                    "INVALID_CAPABILITIES_FORMAT": 400,  # Bad Request
                }

                status_code = status_map.get(response.error_code, 500)

                raise HTTPException(status_code=status_code, detail=response.message)

        except HTTPException:
            # Re-raise HTTP exceptions as-is
            raise
        except Exception as e:
            self.logger.error(f"Error registering agent: {e}")
            raise HTTPException(
                status_code=500, detail=f"Error registering agent: {str(e)}"
            )

    @agent_endpoint(
        "DELETE",
        "/deregister",
        request_model=AgentDeregistrationRequest,
        response_model=SuccessResponse,
    )
    async def deregister_agent(
        self, request: AgentDeregistrationRequest
    ) -> SuccessResponse:
        try:
            # Delegate to Registration Manager for centralized agent coordination
            from feagi.pns.registration_manager import get_registration_manager

            registration_manager = get_registration_manager()
            if not registration_manager:
                self.logger.error("Registration Manager not available")
                raise HTTPException(
                    status_code=503, detail="Registration service unavailable"
                )

            # Process deregistration through Registration Manager
            response = registration_manager.deregister_agent(request.agent_id)

            if response.success:
                self.logger.info(
                    f"✅ Agent '{request.agent_id}' deregistered via Registration Manager - "
                    f"FQ samplers coordinated: {response.fq_samplers_enabled}"
                )

                return SuccessResponse(message=response.message, success=True)
            else:
                self.logger.error(
                    f"❌ Registration Manager deregistration failed: {response.message}"
                )

                # Map Registration Manager error codes to HTTP status codes
                status_map = {
                    "AGENT_NOT_FOUND": 404,  # Not Found
                    "DEREGISTRATION_ERROR": 500,  # Internal Server Error
                }

                status_code = status_map.get(response.error_code, 500)

                raise HTTPException(status_code=status_code, detail=response.message)

        except HTTPException:
            # Re-raise HTTP exceptions as-is
            raise
        except Exception as e:
            self.logger.error(f"Error deregistering agent: {e}")
            raise HTTPException(
                status_code=500, detail=f"Error deregistering agent: {str(e)}"
            )

    @agent_endpoint(
        "GET", "/properties/{agent_id}", response_model=AgentPropertiesResponse
    )
    async def get_agent_properties(self, agent_id: str) -> AgentPropertiesResponse:
        try:
            # Get configured default agent IP
            default_agent_ip = "127.0.0.1"  # @architecture:acceptable - emergency fallback
            try:
                if load_feagi_config and get_agent_config:
                    config = load_feagi_config()
                    agent_config = get_agent_config(config)
                    default_agent_ip = agent_config.default_host
            except Exception as e:
                self.logger.warning(f"Could not load agent configuration, using fallback: {e}")
            
            # Delegate to Registration Manager for consistent agent information
            from feagi.pns.registration_manager import get_registration_manager

            registration_manager = get_registration_manager()
            if not registration_manager:
                self.logger.error("Registration Manager not available")
                raise ValueError("Registration service unavailable")

            properties = registration_manager.get_agent_properties(agent_id)
            if not properties:
                raise ValueError(f"Agent {agent_id} not found")

            # Map Registration Manager data to AgentPropertiesResponse schema
            agent_router_address = properties.get("agent_router_address")
            if (
                not agent_router_address
                and properties.get("agent_ip")
                and properties.get("agent_data_port")
            ):
                agent_router_address = (
                    f"tcp://{properties['agent_ip']}:{properties['agent_data_port']}"
                )

            return AgentPropertiesResponse(
                agent_type=properties.get("agent_type", ""),
                agent_ip=properties.get("agent_ip", default_agent_ip),
                agent_data_port=properties.get("agent_data_port", 0),
                agent_router_address=agent_router_address or "",
                agent_version=properties.get("agent_version", ""),
                controller_version=properties.get("controller_version", ""),
                capabilities=properties.get("capabilities", {}),
            )
        except Exception as e:
            self.logger.error(
                f"Error getting agent properties for {agent_id}: {str(e)}"
            )
            raise ValueError(f"Failed to get agent properties: {str(e)}")

    # Manual query parameter version for FastAPI compatibility
    async def get_agent_properties_query(
        self, agent_id: str
    ) -> AgentPropertiesResponse:
        """
        Get agent properties using query parameter format.
        This endpoint supports the query parameter format: /v1/agent/properties?agent_id=<agent_id>
        This method is manually registered to FastAPI to support query parameters.
        """
        # Delegate to the path parameter version for consistency
        return await self.get_agent_properties(agent_id)

    @agent_endpoint("GET", "/fq_sampler_status")
    async def get_fq_sampler_status(self) -> dict:
        """
        Get comprehensive FQ sampler status with agent registry integration.

        Returns:
            Dictionary with FQ sampler status and related agent information
        """
        try:
            # Delegate to Registration Manager for comprehensive FQ sampler coordination status
            from feagi.pns.registration_manager import get_registration_manager

            registration_manager = get_registration_manager()
            if not registration_manager:
                self.logger.error("Registration Manager not available")
                raise ValueError("Registration service unavailable")

            # Get comprehensive coordination status from Registration Manager
            coordination_status = (
                registration_manager.get_fq_sampler_coordination_status()
            )

            # Get agents list for additional context
            agents_info = registration_manager.list_agents()

            return {
                "fq_sampler_coordination": coordination_status,
                "agent_registry": agents_info,
                "system_status": "coordinated_via_registration_manager",
            }

        except Exception as e:
            self.logger.error(f"Error getting FQ sampler status: {e}")
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            raise ValueError(f"Failed to get FQ sampler status: {str(e)}")


# NOTE: FQ sampler management methods removed - now handled by Registration Manager
# All agent registration, deregistration, and FQ sampler coordination is now
# centralized in the Registration Manager (feagi.pns.registration_manager)
# This eliminates the complexity of multiple coordination points.


def create_feagi_agent_api(core_api_service: CoreAPIService) -> FeagiAgentAPI:
    return FeagiAgentAPI(core_api_service)
