"""
FEAGI v1 Region API - Single Source of Truth

This module contains the ONLY definitions of brain region API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Dict, Any, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (
    RegionInfoResponse, RegionListResponse, CreateRegionRequest,
    UpdateRegionRequest, SuccessResponse, ErrorResponse
)
from .decorators import endpoint

logger = setup_logger(__name__)


# Define the convenience decorator for region endpoints
def region_endpoint(methods, path, request_model=None, response_model=None, description=None):
    """Convenience decorator for region endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module='region'
    )


class RegionAPI:
    """
    Region API - Single Source of Truth for ALL Transports
    
    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.
    
    This ensures identical behavior across all transports with zero duplication.
    """
    
    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service
    
    # ===== Region Information =====
    
    @region_endpoint('GET', '/list', response_model=RegionListResponse)
    async def get_regions_list(self) -> RegionListResponse:
        """Get list of all brain regions."""
        try:
            regions = self.core_api_service.get_brain_regions()
            return RegionListResponse(regions=regions)
        except Exception as e:
            logger.error(f"Error getting regions list: {e}")
            raise ValueError(f"Failed to get regions list: {str(e)}")
    
    @region_endpoint('GET', '/info/{region_id}', response_model=RegionInfoResponse)
    async def get_region_info(self, region_id: str) -> RegionInfoResponse:
        """Get information about a specific brain region."""
        try:
            region_info = self.core_api_service.get_brain_region_info(region_id)
            return RegionInfoResponse(region_info=region_info)
        except Exception as e:
            logger.error(f"Error getting region info: {e}")
            raise ValueError(f"Failed to get region info: {str(e)}")
    
    # ===== Region Management =====
    
    @region_endpoint('POST', '/create', 
                    request_model=CreateRegionRequest,
                    response_model=SuccessResponse)
    async def create_region(self, request: CreateRegionRequest) -> SuccessResponse:
        """Create a new brain region."""
        try:
            success = self.core_api_service.create_brain_region(request.region_data)
            if not success:
                raise ValueError("Failed to create brain region")
            
            return SuccessResponse(message="Brain region created successfully")
        except Exception as e:
            logger.error(f"Error creating region: {e}")
            raise ValueError(f"Failed to create region: {str(e)}")
    
    @region_endpoint('PUT', '/update', 
                    request_model=UpdateRegionRequest,
                    response_model=SuccessResponse)
    async def update_region(self, request: UpdateRegionRequest) -> SuccessResponse:
        """Update an existing brain region."""
        try:
            success = self.core_api_service.update_brain_region(request.region_id, request.updates)
            if not success:
                raise ValueError("Failed to update brain region")
            
            return SuccessResponse(message="Brain region updated successfully")
        except Exception as e:
            logger.error(f"Error updating region: {e}")
            raise ValueError(f"Failed to update region: {str(e)}")
    
    @region_endpoint('DELETE', '/delete/{region_id}', response_model=SuccessResponse)
    async def delete_region(self, region_id: str) -> SuccessResponse:
        """Delete a brain region."""
        try:
            success = self.core_api_service.delete_brain_region(region_id)
            if not success:
                raise ValueError("Failed to delete brain region")
            
            return SuccessResponse(message="Brain region deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting region: {e}")
            raise ValueError(f"Failed to delete region: {str(e)}")


# ===== Factory Function =====

def create_region_api(core_api_service: CoreAPIService) -> RegionAPI:
    """
    Factory function to create a RegionAPI instance.
    
    This function can be used by transport adapters to get a configured
    RegionAPI instance with the required dependencies.
    """
    return RegionAPI(core_api_service) 