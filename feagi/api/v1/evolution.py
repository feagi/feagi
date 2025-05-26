"""FEAGI v1 Evolution API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import EvolutionStatusResponse, EvolutionConfigRequest, SuccessResponse
from .decorators import endpoint

def evolution_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='evolution')

class EvolutionAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @evolution_endpoint('GET', '/status', response_model=EvolutionStatusResponse)
    async def get_evolution_status(self) -> EvolutionStatusResponse:
        status = self.core_api_service.get_evolution_status()
        return EvolutionStatusResponse(status=status.get('status', 'unknown'), generation=status.get('generation'), config=status.get('config'))
    
    @evolution_endpoint('POST', '/configure', request_model=EvolutionConfigRequest, response_model=SuccessResponse)
    async def configure_evolution(self, request: EvolutionConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_evolution(request.config)
        if not success:
            raise ValueError("Failed to configure evolution")
        return SuccessResponse(message="Evolution configured successfully")

def create_evolution_api(core_api_service: CoreAPIService) -> EvolutionAPI:
    return EvolutionAPI(core_api_service) 