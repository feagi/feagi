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
from fastapi import HTTPException

logger = setup_logger(__name__)

def agent_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='feagi_agent')

class FeagiAgentAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
        self.logger = logger
    
    @agent_endpoint('GET', '/list', response_model=AgentListResponse)
    async def list_agents(self) -> AgentListResponse:
        try:
            agents = self.core_api_service.get_connected_agents()
            return AgentListResponse.model_validate(agents)
        except Exception as e:
            self.logger.error(f"Error listing agents: {e}")
            raise ValueError(f"Failed to list agents: {str(e)}")
    
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
            
            if success:
                # Automatically enable FQ samplers based on agent capabilities
                await self._manage_fq_samplers_for_registration(request.capabilities)
                
                return SuccessResponse(
                    message=f"Agent {request.agent_id} registered successfully",
                    success=True
                )
            else:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to register agent {request.agent_id}"
                )
                
        except Exception as e:
            self.logger.error(f"Error registering agent: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Error registering agent: {str(e)}"
            )
    
    @agent_endpoint('DELETE', '/deregister', request_model=AgentDeregistrationRequest, response_model=SuccessResponse)
    async def deregister_agent(self, request: AgentDeregistrationRequest) -> SuccessResponse:
        try:
            # Get agent capabilities before deregistration for FQ sampler management
            agent_properties = self.core_api_service.get_agent_properties(request.agent_id)
            agent_capabilities = agent_properties.get('capabilities', {}) if agent_properties else {}
            
            success = self.core_api_service.unregister_agent(request.agent_id)
            
            if success:
                # Automatically disable FQ samplers if no more agents need them
                await self._manage_fq_samplers_for_deregistration(agent_capabilities)
                
                return SuccessResponse(
                    message=f"Agent {request.agent_id} deregistered successfully",
                    success=True
                )
            else:
                raise HTTPException(
                    status_code=404,
                    detail=f"Agent {request.agent_id} not found"
                )
                
        except Exception as e:
            self.logger.error(f"Error deregistering agent: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Error deregistering agent: {str(e)}"
            )
    
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

    @agent_endpoint('GET', '/fq_sampler_status')
    async def get_fq_sampler_status(self) -> dict:
        """
        Get comprehensive FQ sampler status with agent registry integration.
        
        Returns:
            Dictionary with FQ sampler status and related agent information
        """
        try:
            from feagi.process_manager import get_process_manager
            
            # Get agent registry information
            registry_summary = self.core_api_service.get_agent_registry_summary()
            
            # Get process manager and FQ sampler status
            process_manager = get_process_manager()
            fq_status = {}
            
            if process_manager:
                # Get visualization FQ sampler status
                viz_sampler = getattr(process_manager, '_viz_fq_sampler', None)
                if viz_sampler:
                    fq_status['visualization'] = {
                        'enabled': getattr(viz_sampler, '_has_visualization_subscribers', False),
                        'running': getattr(viz_sampler, 'running', False),
                        'frequency_hz': getattr(viz_sampler, 'sample_frequency', 0),
                        'sampling_mode': getattr(viz_sampler, 'sampling_mode', 'unknown'),
                        'available': True
                    }
                else:
                    fq_status['visualization'] = {'enabled': False, 'available': False, 'error': 'sampler not found'}
                
                # Get motor FQ sampler status  
                motor_sampler = getattr(process_manager, '_motor_fq_sampler', None)
                if motor_sampler:
                    fq_status['motor'] = {
                        'enabled': getattr(motor_sampler, '_has_motor_subscribers', False),
                        'running': getattr(motor_sampler, 'running', False),
                        'frequency_hz': getattr(motor_sampler, 'sample_frequency', 0),
                        'sampling_mode': getattr(motor_sampler, 'sampling_mode', 'unknown'),
                        'available': True
                    }
                else:
                    fq_status['motor'] = {'enabled': False, 'available': False, 'error': 'sampler not found'}
            else:
                fq_status = {'error': 'Process manager not available'}
            
            # Calculate coordination status
            viz_agent_count = registry_summary.get('agent_count_viz', 0)
            motor_agent_count = registry_summary.get('agent_count_sensorimotor', 0)
            viz_sampler_enabled = fq_status.get('visualization', {}).get('enabled', False)
            motor_sampler_enabled = fq_status.get('motor', {}).get('enabled', False)
            
            return {
                'fq_samplers': fq_status,
                'agent_registry': registry_summary,
                'coordination': {
                    'viz_agents_vs_sampler': {
                        'agent_count': viz_agent_count,
                        'sampler_enabled': viz_sampler_enabled,
                        'status': 'synced' if (viz_agent_count > 0) == viz_sampler_enabled else 'out_of_sync'
                    },
                    'motor_agents_vs_sampler': {
                        'agent_count': motor_agent_count,
                        'sampler_enabled': motor_sampler_enabled,
                        'status': 'synced' if (motor_agent_count > 0) == motor_sampler_enabled else 'out_of_sync'
                    }
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error getting FQ sampler status: {e}")
            import traceback
            self.logger.error(f"Traceback: {traceback.format_exc()}")
            raise ValueError(f"Failed to get FQ sampler status: {str(e)}")

    async def _manage_fq_samplers_for_registration(self, capabilities: dict) -> None:
        """
        Enable FQ samplers when agents with relevant capabilities register.
        
        Args:
            capabilities: Agent capabilities dictionary
        """
        try:
            from feagi.process_manager import get_process_manager
            
            process_manager = get_process_manager()
            if not process_manager:
                self.logger.warning("Process manager not available for FQ sampler management")
                return
                
            # Check if agent has visualization capabilities
            if self._has_visualization_capabilities(capabilities):
                self.logger.info("🎨 Agent has visualization capabilities - enabling visualization FQ sampler")
                success = process_manager.enable_viz_fq_sampler()
                if success:
                    self.logger.info("✅ Visualization FQ sampler enabled for new agent")
                else:
                    self.logger.warning("⚠️ Failed to enable visualization FQ sampler")
                    
            # Check if agent has motor/sensorimotor capabilities  
            if self._has_motor_capabilities(capabilities):
                self.logger.info("🚗 Agent has motor capabilities - enabling motor FQ sampler")
                success = process_manager.enable_motor_fq_sampler()
                if success:
                    self.logger.info("✅ Motor FQ sampler enabled for new agent")
                else:
                    self.logger.warning("⚠️ Failed to enable motor FQ sampler")
                    
        except Exception as e:
            self.logger.error(f"Error managing FQ samplers for registration: {e}")

    async def _manage_fq_samplers_for_deregistration(self, capabilities: dict) -> None:
        """
        Disable FQ samplers when no more agents need them after deregistration.
        
        Args:
            capabilities: Capabilities of the agent being deregistered
        """
        try:
            from feagi.process_manager import get_process_manager
            
            process_manager = get_process_manager()
            if not process_manager:
                self.logger.warning("Process manager not available for FQ sampler management")
                return
            
            # Get current agent registry state
            registry_summary = self.core_api_service.get_agent_registry_summary()
            
            # Check if we should disable visualization FQ sampler
            if self._has_visualization_capabilities(capabilities):
                remaining_viz_agents = registry_summary.get('agent_count_viz', 0)
                if remaining_viz_agents == 0:
                    self.logger.info("🎨 No more visualization agents - disabling visualization FQ sampler")
                    process_manager.disable_viz_fq_sampler()
                    self.logger.info("⏹️ Visualization FQ sampler disabled")
                else:
                    self.logger.info(f"🎨 {remaining_viz_agents} visualization agents still connected - keeping FQ sampler enabled")
                    
            # Check if we should disable motor FQ sampler
            if self._has_motor_capabilities(capabilities):
                remaining_motor_agents = registry_summary.get('agent_count_sensorimotor', 0)
                if remaining_motor_agents == 0:
                    self.logger.info("🚗 No more motor agents - disabling motor FQ sampler")
                    process_manager.disable_motor_fq_sampler()
                    self.logger.info("⏹️ Motor FQ sampler disabled")
                else:
                    self.logger.info(f"🚗 {remaining_motor_agents} motor agents still connected - keeping FQ sampler enabled")
                    
        except Exception as e:
            self.logger.error(f"Error managing FQ samplers for deregistration: {e}")

    def _has_visualization_capabilities(self, capabilities: dict) -> bool:
        """Check if capabilities include visualization features."""
        return bool(capabilities.get('visualization', False))
        
    def _has_motor_capabilities(self, capabilities: dict) -> bool:
        """Check if capabilities include motor/sensorimotor features."""
        return bool(
            capabilities.get('output', False) or 
            capabilities.get('motor', False) or
            capabilities.get('sensorimotor', False)
        )

def create_feagi_agent_api(core_api_service: CoreAPIService) -> FeagiAgentAPI:
    return FeagiAgentAPI(core_api_service) 