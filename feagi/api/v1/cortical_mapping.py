"""FEAGI v1 Cortical Mapping API"""
from typing import Dict, Any
from feagi.api.core.services.core_api_service import CoreAPIService
from .schemas import SuccessResponse
from .decorators import endpoint

def cortical_mapping_endpoint(methods, path, request_model=None, response_model=None, description=None):
    return endpoint(methods=methods, path=path, request_model=request_model, response_model=response_model, description=description, module='cortical_mapping')

class CorticalMappingAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service
    
    @cortical_mapping_endpoint('GET', '/mapping', response_model=Dict[str, Any])
    async def get_cortical_mapping(self) -> Dict[str, Any]:
        mapping = self.core_api_service.get_cortical_mapping()
        return mapping
    
    @cortical_mapping_endpoint('PUT', '/mapping', response_model=SuccessResponse)
    async def update_cortical_mapping(self, mapping: Dict[str, Any]) -> SuccessResponse:
        success = self.core_api_service.update_cortical_mapping(mapping)
        if not success:
            raise ValueError("Failed to update cortical mapping")
        return SuccessResponse(message="Cortical mapping updated successfully")

def create_cortical_mapping_api(core_api_service: CoreAPIService) -> CorticalMappingAPI:
    return CorticalMappingAPI(core_api_service) 