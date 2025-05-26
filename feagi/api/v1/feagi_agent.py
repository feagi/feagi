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
from .schemas import AgentListResponse, AgentInfoResponse, AgentConfigRequest, SuccessResponse
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
            agents = self.core_api_service.get_connected_agents()
            return AgentListResponse(agents=agents)
        except Exception as e:
            raise ValueError(f"Failed to get agents list: {str(e)}")
    
    @agent_endpoint('GET', '/info/{agent_id}', response_model=AgentInfoResponse)
    async def get_agent_info(self, agent_id: str) -> AgentInfoResponse:
        try:
            agent_info = self.core_api_service.get_agent_details(agent_id)
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

def create_feagi_agent_api(core_api_service: CoreAPIService) -> FeagiAgentAPI:
    return FeagiAgentAPI(core_api_service) 