"""
FEAGI v1 Simulation API - Single Source of Truth
"""

from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import SimulationStatusResponse, SimulationConfigRequest, SimulationStatsResponse, SuccessResponse
from .decorators import endpoint

logger = setup_logger(__name__)

def simulation_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='simulation')

class SimulationAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @simulation_endpoint('GET', '/status', response_model=SimulationStatusResponse)
    async def get_simulation_status(self) -> SimulationStatusResponse:
        try:
            status = self.core_api_service.get_simulation_status()
            return SimulationStatusResponse(status=status.get('status', 'unknown'), is_running=status.get('is_running', False), config=status.get('config'))
        except Exception as e:
            raise ValueError(f"Failed to get simulation status: {str(e)}")
    
    @simulation_endpoint('POST', '/configure', request_model=SimulationConfigRequest, response_model=SuccessResponse)
    async def configure_simulation(self, request: SimulationConfigRequest) -> SuccessResponse:
        try:
            success = self.core_api_service.configure_simulation(request.config)
            if not success:
                raise ValueError("Failed to configure simulation")
            return SuccessResponse(message="Simulation configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure simulation: {str(e)}")
    
    @simulation_endpoint('GET', '/stats', response_model=SimulationStatsResponse)
    async def get_simulation_stats(self) -> SimulationStatsResponse:
        try:
            stats = self.core_api_service.get_simulation_stats()
            return SimulationStatsResponse(stats=stats)
        except Exception as e:
            raise ValueError(f"Failed to get simulation stats: {str(e)}")

def create_simulation_api(core_api_service: CoreAPIService) -> SimulationAPI:
    return SimulationAPI(core_api_service) 