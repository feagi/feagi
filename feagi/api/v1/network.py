"""FEAGI v1 Network API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import NetworkStatusResponse, NetworkConfigRequest, SuccessResponse
from .decorators import endpoint

def network_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='network')

class NetworkAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @network_endpoint('GET', '/status', response_model=NetworkStatusResponse)
    async def get_network_status(self) -> NetworkStatusResponse:
        status = self.core_api_service.get_network_status()
        return NetworkStatusResponse(status=status)
    
    @network_endpoint('POST', '/configure', request_model=NetworkConfigRequest, response_model=SuccessResponse)
    async def configure_network(self, request: NetworkConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_network(request.config)
        if not success:
            raise ValueError("Failed to configure network")
        return SuccessResponse(message="Network configured successfully")

def create_network_api(core_api_service: CoreAPIService) -> NetworkAPI:
    return NetworkAPI(core_api_service) 