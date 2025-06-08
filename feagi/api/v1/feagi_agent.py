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

from typing import Dict, Any, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (AgentListResponse, AgentInfoResponse, AgentConfigRequest, SuccessResponse,
                      AgentRegistrationRequest, AgentDeregistrationRequest, 
                      AgentPropertiesRequest, AgentPropertiesResponse)
from .decorators import endpoint

logger = setup_logger(__name__)

def agent_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='feagi_agent')

class FeagiAgentAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @agent_endpoint('GET', '/list', response_model=AgentListResponse)
    async def get_agents_list(self) -> AgentListResponse:
        try:
            # Get all connected agents from the state manager
            connected_agents = self.core_api_service.get_connected_agents()
            
            # Convert set to list of agent IDs
            agent_ids = list(connected_agents)
            
            return AgentListResponse(root=agent_ids)
        except Exception as e:
            raise ValueError(f"Failed to get agents list: {str(e)}")
    
    @agent_endpoint('GET', '/info/{agent_id}', response_model=AgentInfoResponse)
    async def get_agent_info(self, agent_id: str) -> AgentInfoResponse:
        try:
            agent_info = self.core_api_service.get_agent_properties(agent_id)
            if not agent_info:
                raise ValueError(f"Agent {agent_id} not found")
            return AgentInfoResponse(agent_info=agent_info)
        except Exception as e:
            raise ValueError(f"Failed to get agent info: {str(e)}")
    
    @agent_endpoint('POST', '/configure', request_model=AgentConfigRequest, response_model=SuccessResponse)
    async def configure_agent(self, request: AgentConfigRequest) -> SuccessResponse:
        try:
            success = self.core_api_service.configure_agent(request.agent_id, request.config)
            if not success:
                raise ValueError("Failed to configure agent")
            return SuccessResponse(message="Agent configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure agent: {str(e)}")
    
    @agent_endpoint('POST', '/register', request_model=AgentRegistrationRequest, response_model=SuccessResponse)
    async def register_agent(self, request: AgentRegistrationRequest) -> SuccessResponse:
        try:
            # For now, use the agent_ip from the request model or default
            # In a full implementation, we could access the HTTP request context
            agent_ip = request.agent_ip or "127.0.0.1"  # Default fallback
                
            success = self.core_api_service.register_agent(
                agent_id=request.agent_id,
                agent_type=request.agent_type,
                capabilities=request.capabilities,
                agent_data_port=request.agent_data_port,
                agent_version=request.agent_version,
                controller_version=request.controller_version,
                agent_ip=agent_ip
            )
            if not success:
                raise ValueError("Failed to register agent")
            return SuccessResponse(message=f"Agent {request.agent_id} registered successfully")
        except Exception as e:
            raise ValueError(f"Failed to register agent: {str(e)}")
    
    @agent_endpoint('DELETE', '/deregister', request_model=AgentDeregistrationRequest, response_model=SuccessResponse)
    async def deregister_agent(self, request: AgentDeregistrationRequest) -> SuccessResponse:
        try:
            success = self.core_api_service.unregister_agent(request.agent_id)
            if not success:
                raise ValueError("Failed to deregister agent")
            return SuccessResponse(message=f"Agent {request.agent_id} deregistered successfully")
        except Exception as e:
            raise ValueError(f"Failed to deregister agent: {str(e)}")
    
    @agent_endpoint('GET', '/properties/{agent_id}', response_model=AgentPropertiesResponse)
    async def get_agent_properties(self, agent_id: str) -> AgentPropertiesResponse:
        try:
            properties = self.core_api_service.get_agent_properties(agent_id)
            if not properties:
                raise ValueError(f"Agent {agent_id} not found")
                
            # Format response to match expected ecosystem structure
            return AgentPropertiesResponse(
                agent_type=properties.get("type", "unknown"),
                agent_ip=properties.get("agent_ip", ""),
                agent_data_port=properties.get("agent_data_port", 0),
                agent_router_address=properties.get("agent_router_address", ""),
                agent_version=properties.get("agent_version", ""),
                controller_version=properties.get("controller_version", ""),
                capabilities=properties.get("capabilities", {})
            )
        except Exception as e:
            raise ValueError(f"Failed to get agent properties: {str(e)}")

def create_feagi_agent_api(core_api_service: CoreAPIService) -> FeagiAgentAPI:
    return FeagiAgentAPI(core_api_service) 