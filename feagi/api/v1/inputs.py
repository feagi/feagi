"""FEAGI v1 Inputs API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import InputSourcesResponse, IOConfigRequest, SuccessResponse
from .decorators import endpoint

def inputs_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='inputs')

class InputsAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @inputs_endpoint('GET', '/sources', response_model=InputSourcesResponse)
    async def get_input_sources(self) -> InputSourcesResponse:
        sources = self.core_api_service.get_input_sources()
        return InputSourcesResponse(sources=sources)
    
    @inputs_endpoint('POST', '/configure', request_model=IOConfigRequest, response_model=SuccessResponse)
    async def configure_inputs(self, request: IOConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_inputs(request.config)
        if not success:
            raise ValueError("Failed to configure inputs")
        return SuccessResponse(message="Inputs configured successfully")

def create_inputs_api(core_api_service: CoreAPIService) -> InputsAPI:
    return InputsAPI(core_api_service) 