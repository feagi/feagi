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
FEAGI v1 Agent API - Single Source of Truth
"""

from typing import Any, Dict

from fastapi import HTTPException

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

try:
    from feagi.config.toml_loader import get_agent_config, load_feagi_config
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
    ManualStimulationRequest,
    SuccessResponse,
)
from feagi.api.shared_memory.capabilities import (
    SharedMemoryCapability,
    CAPABILITY_DESCRIPTIONS,
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
                self.logger.warning("Registration Manager not available - returning empty agent list")
                return AgentListResponse(root=[])

            agents_data = registration_manager.list_agents()

            #  Extract just the agent IDs for the simple list format expected
            #  by AgentListResponse
            agent_ids = [
                agent["agent_id"] for agent in agents_data.get("agents", [])
            ]

            return AgentListResponse(root=agent_ids)
        except Exception as e:
            # Log error but return empty list instead of raising ValueError (which becomes 400 Bad Request)
            self.logger.error(f"Error listing agents: {e} - returning empty list")
            return AgentListResponse(root=[])

    @agent_endpoint(
        "GET", "/info/{agent_id}", response_model=AgentInfoResponse
    )
    async def get_agent_info(self, agent_id: str) -> AgentInfoResponse:
        try:
            agent_info = self.core_api_service.get_agent_properties(agent_id)
            if not agent_info:
                raise ValueError(f"Agent {agent_id} not found")
            return AgentInfoResponse(agent_info=agent_info)
        except Exception as e:
            raise ValueError(f"Failed to get agent info: {str(e)}") from e

    @agent_endpoint(
        "POST",
        "/configure",
        request_model=AgentConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_agent(
        self, request: AgentConfigRequest
    ) -> SuccessResponse:
        try:
            success = self.core_api_service.configure_agent(
                request.agent_id, request.config
            )
            if not success:
                raise ValueError("Failed to configure agent")
            return SuccessResponse(message="Agent configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure agent: {str(e)}") from e

    @agent_endpoint(
        "POST",
        "/register",
        request_model=AgentRegistrationRequest,
        response_model=SuccessResponse,
    )
    async def register_agent(
        self, request: AgentRegistrationRequest
    ) -> SuccessResponse:
        """Agent registration with automatic capability rate handling."""
        try:
            # Process capability rates if needed
            capability_configs = self._process_agent_capabilities(request)
            
            # Perform standard registration
            success = self.core_api_service.register_agent(
                agent_id=request.agent_id,
                agent_type=request.agent_type,
                capabilities=request.capabilities,
                agent_data_port=request.agent_data_port,
                agent_version=request.agent_version,
                controller_version=request.controller_version,
                agent_ip=request.agent_ip
            )
            
            if not success:
                raise HTTPException(
                    status_code=500,
                    detail="Agent registration failed"
                )
            
            # Store capability rate configs if capability manager is available
            if capability_configs:
                self._store_capability_rates(request.agent_id, capability_configs)
            
            return SuccessResponse(
                success=True,
                message=f"Agent {request.agent_id} registered successfully"
            )
            
        except HTTPException:
            raise
        except Exception as e:
            self.logger.error(f"Agent registration failed: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Registration failed: {str(e)}"
            )

    def _process_agent_capabilities(self, request: AgentRegistrationRequest) -> Optional[List[Any]]:
        """Process agent capabilities and convert to rate specifications."""
        if not request.capabilities:
            return None
            
        try:
            from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec
            
            capability_specs = []
            seen_capability_types = set()
            
            # Default rates
            default_rates = {
                "sensory": 10.0,
                "motor": 20.0, 
                "visualization": 5.0,
                "neurons_stream": 10.0,
                "control": 1.0
            }
            
            for cap_name, cap_config in request.capabilities.items():
                # Handle sensorimotor as combined capability
                if cap_name.lower() == "sensorimotor":
                    sensory_rate = default_rates["sensory"]
                    motor_rate = default_rates["motor"]
                    
                    if isinstance(cap_config, dict):
                        if "sensory_rate_hz" in cap_config:
                            sensory_rate = float(cap_config["sensory_rate_hz"])
                        if "motor_rate_hz" in cap_config:
                            motor_rate = float(cap_config["motor_rate_hz"])
                        elif "rate_hz" in cap_config:
                            sensory_rate = motor_rate = float(cap_config["rate_hz"])
                    
                    if CapabilityType.SENSORY not in seen_capability_types:
                        capability_specs.append(CapabilityRateSpec(
                            capability_type=CapabilityType.SENSORY,
                            requested_rate_hz=sensory_rate,
                            required=True
                        ))
                        seen_capability_types.add(CapabilityType.SENSORY)
                    
                    if CapabilityType.MOTOR not in seen_capability_types:
                        capability_specs.append(CapabilityRateSpec(
                            capability_type=CapabilityType.MOTOR,
                            requested_rate_hz=motor_rate,
                            required=True
                        ))
                        seen_capability_types.add(CapabilityType.MOTOR)
                    continue
                
                # Map capability name to type
                cap_type = None
                default_rate = 10.0
                
                if cap_name.lower() in ["sensory", "sensor", "input", "sensors"]:
                    cap_type = CapabilityType.SENSORY
                    default_rate = default_rates["sensory"]
                elif cap_name.lower() in ["motor", "output", "actuator", "motors", "actuators"]:
                    cap_type = CapabilityType.MOTOR
                    default_rate = default_rates["motor"]
                elif cap_name.lower() in ["visualization", "viz", "visual", "display"]:
                    cap_type = CapabilityType.VISUALIZATION
                    default_rate = default_rates["visualization"]
                elif cap_name.lower() in ["neurons_stream", "neuron_stream", "neural_stream"]:
                    cap_type = CapabilityType.NEURONS_STREAM
                    default_rate = default_rates["neurons_stream"]
                elif cap_name.lower() in ["control", "command", "commands"]:
                    cap_type = CapabilityType.CONTROL
                    default_rate = default_rates["control"]
                else:
                    cap_type = CapabilityType.SENSORY
                    default_rate = default_rates["sensory"]
                
                # Add capability if not already present
                if cap_type not in seen_capability_types:
                    final_rate = default_rate
                    if isinstance(cap_config, dict) and "rate_hz" in cap_config:
                        final_rate = float(cap_config["rate_hz"])
                    
                    capability_specs.append(CapabilityRateSpec(
                        capability_type=cap_type,
                        requested_rate_hz=final_rate,
                        required=True
                    ))
                    seen_capability_types.add(cap_type)
            
            return capability_specs
            
        except Exception as e:
            self.logger.warning(f"Could not process capability rates for {request.agent_id}: {e}")
            return None
    
    def _store_capability_rates(self, agent_id: str, capability_specs: List[Any]) -> None:
        """Store capability rate configurations in the capability manager."""
        try:
            from feagi.core.capability_rate_manager import get_capability_rate_manager
            
            capability_manager = get_capability_rate_manager()
            if capability_manager:
                approved_configs, rejections = capability_manager.register_agent_capabilities(
                    agent_id, capability_specs
                )
                
                if rejections:
                    self.logger.warning(
                        f"Some capabilities rejected for agent {agent_id}: {rejections}"
                    )
                
                self.logger.info(
                    f"Registered {len(approved_configs)} capability rates for agent {agent_id}"
                )
        except Exception as e:
            self.logger.warning(f"Could not store capability rates for {agent_id}: {e}")

    @agent_endpoint("GET", "/shared_mem")
    async def list_agents_with_shared_mem(self) -> Dict[str, Any]:
        """Return agents that provided shared memory details during registration.

        Response format:
            { "agent_id": { "video_preview_shared_mem_path": str, "metadata": {...} }, ... }
        """
        try:
            # Prefer authoritative StateManager SHM registry
            from feagi.core.state_manager import FeagiStateManager

            sm = FeagiStateManager.instance()
            result: Dict[str, Any] = {}
            if hasattr(sm, "_agent_shared_memory"):
                seen_signatures = set()
                for aid, mapping in getattr(sm, "_agent_shared_memory", {}).items():
                    if not mapping:
                        continue
                    # Create a stable signature of the mapping to collapse duplicates
                    try:
                        signature_parts = []
                        for k in sorted(mapping.keys()):
                            v = mapping.get(k, "")
                            signature_parts.append(f"{k}={v}")
                        signature = "|".join(signature_parts)
                    except Exception:
                        signature = str(mapping)
                    if signature in seen_signatures:
                        # Duplicate mapping under a different agent_id; keep first
                        try:
                            self.logger.info(
                                f"[SHM] Deduplicating shared_mem entry for agent '{aid}' (duplicate mapping)"
                            )
                        except Exception:
                            pass
                        continue
                    seen_signatures.add(signature)
                    result[aid] = {**mapping}
            return result
        except Exception as e:
            self.logger.error(f"Error listing shared memory agents: {e}")
            return {}

    @agent_endpoint("GET", "/capabilities")
    async def list_capabilities(self) -> Dict[str, Any]:
        """List canonical shared-memory capability types recognized by FEAGI.

        Returns a JSON object with the allowed capability keys and descriptions.
        Agents should use ONLY these keys when requesting or advertising shared
        memory capabilities.
        """
        try:
            caps = {
                "capabilities": [c.value for c in SharedMemoryCapability],
                "descriptions": CAPABILITY_DESCRIPTIONS,
            }
            return caps
        except Exception as e:
            self.logger.error(f"Error listing capabilities: {e}")
            return {"capabilities": [], "descriptions": {}}

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
            #  Delegate to Registration Manager for centralized agent
            #  coordination
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

                # Cleanup per-agent SHM files
                try:
                    from feagi.core.state_manager import FeagiStateManager

                    sm = FeagiStateManager.instance()
                    if hasattr(sm, "delete_agent_shm"):
                        sm.delete_agent_shm(request.agent_id)
                except Exception as e:
                    self.logger.warning(
                        f"SHM cleanup skipped for agent {request.agent_id}: {e}"
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

                raise HTTPException(
                    status_code=status_code, detail=response.message
                )

        except HTTPException:
            # Re-raise HTTP exceptions as-is
            raise
        except Exception as e:
            self.logger.error(f"Error deregistering agent: {e}")
            raise HTTPException(
                status_code=500, detail=f"Error deregistering agent: {str(e)}"
            ) from e

    @agent_endpoint(
        "GET", "/properties/{agent_id}", response_model=AgentPropertiesResponse
    )
    async def get_agent_properties(
        self, agent_id: str
    ) -> AgentPropertiesResponse:
        try:
            # Get configured default agent IP
            default_agent_ip = (
                "127.0.0.1"  # @architecture:acceptable - emergency fallback
            )
            try:
                if load_feagi_config and get_agent_config:
                    config = load_feagi_config()
                    agent_config = get_agent_config(config)
                    default_agent_ip = agent_config.default_host
            except Exception as e:
                self.logger.warning(
                    f"Could not load agent configuration, using fallback: {e}"
                )

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
                agent_router_address = f"tcp://{properties['agent_ip']}:{properties['agent_data_port']}"

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
            raise ValueError(
                f"Failed to get agent properties: {str(e)}"
            ) from e

    # Manual query parameter version for FastAPI compatibility
    async def get_agent_properties_query(
        self, agent_id: str
    ) -> AgentPropertiesResponse:
        """Get agent properties using query parameter format.

        This endpoint supports the query parameter format: /v1/agent/properties?agent_id=<agent_id>
        This method is manually registered to FastAPI to support query parameters.
        """
        # Delegate to the path parameter version for consistency
        return await self.get_agent_properties(agent_id)

    @agent_endpoint("GET", "/fq_sampler_status")
    async def get_fq_sampler_status(self) -> dict:
        """Get comprehensive FQ sampler status with agent registry integration.

        Returns:
            Dictionary with FQ sampler status and related agent information
        """
        try:
            #  Delegate to Registration Manager for comprehensive FQ sampler
            #  coordination status
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
            raise ValueError(
                f"Failed to get FQ sampler status: {str(e)}"
            ) from e

    @agent_endpoint(
        "POST",
        "/manual_stimulation",
        request_model=ManualStimulationRequest,
        response_model=Dict[str, Any],
    )
    async def manual_stimulation(
        self, request: ManualStimulationRequest
    ) -> Dict[str, Any]:
        """Trigger manual neural stimulation across multiple cortical areas.

        Injects neuron activations associated with the payload data into the fire candidate list.

        Args:
            request: Manual stimulation request containing stimulation payload with cortical areas
                    mapped to coordinate lists

        Returns:
            Dictionary containing stimulation results and statistics

        Example request body:
        {
            "stimulation_payload": {
                "_power": [[1, 0, 0], [2, 4, 3]],
                "cx3212": [[1, 1, 0], [12, 24, 33], [0, 0, 0]]
            }
        }
        """
        try:
            self.logger.info(
                f"Manual stimulation request received for {len(request.stimulation_payload)} cortical areas"
            )

            # Delegate to CoreAPIService for processing
            result = self.core_api_service.trigger_multi_area_stimulation(
                request.stimulation_payload
            )

            self.logger.info(
                f"Manual stimulation completed: {result.get('success', False)}"
            )

            return result

        except Exception as e:
            self.logger.error(f"Error processing manual stimulation: {e}")
            return {"success": False, "error": str(e)}


#  NOTE: FQ sampler management methods removed - now handled by Registration
#  Manager
# All agent registration, deregistration, and FQ sampler coordination is now
# centralized in the Registration Manager (feagi.pns.registration_manager)
# This eliminates the complexity of multiple coordination points.


def create_feagi_agent_api(core_api_service: CoreAPIService) -> FeagiAgentAPI:
    return FeagiAgentAPI(core_api_service)
