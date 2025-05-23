"""
FEAGI v1 Morphology API - Single Source of Truth

This module contains the ONLY definitions of morphology API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Dict, Any, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (
    MorphologyListResponse, MorphologyInfoResponse, CreateMorphologyRequest,
    UpdateMorphologyRequest, SuccessResponse, ErrorResponse
)
from .decorators import endpoint

logger = setup_logger(__name__)


# Define the convenience decorator for morphology endpoints
def morphology_endpoint(methods, path, request_model=None, response_model=None, description=None):
    """Convenience decorator for morphology endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module='morphology'
    )


class MorphologyAPI:
    """
    Morphology API - Single Source of Truth for ALL Transports
    
    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.
    
    This ensures identical behavior across all transports with zero duplication.
    """
    
    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service
    
    # ===== Morphology Information =====
    
    @morphology_endpoint('GET', '/list', response_model=MorphologyListResponse)
    async def get_morphologies_list(self) -> MorphologyListResponse:
        """Get list of all morphologies."""
        try:
            morphologies = self.core_api_service.get_morphologies()
            return MorphologyListResponse(morphologies=morphologies)
        except Exception as e:
            logger.error(f"Error getting morphologies list: {e}")
            raise ValueError(f"Failed to get morphologies list: {str(e)}")
    
    @morphology_endpoint('GET', '/info/{morphology_id}', response_model=MorphologyInfoResponse)
    async def get_morphology_info(self, morphology_id: str) -> MorphologyInfoResponse:
        """Get information about a specific morphology."""
        try:
            morphology = self.core_api_service.get_morphology_info(morphology_id)
            return MorphologyInfoResponse(morphology=morphology)
        except Exception as e:
            logger.error(f"Error getting morphology info: {e}")
            raise ValueError(f"Failed to get morphology info: {str(e)}")
    
    # ===== Morphology Management =====
    
    @morphology_endpoint('POST', '/create',
                        request_model=CreateMorphologyRequest,
                        response_model=SuccessResponse)
    async def create_morphology(self, request: CreateMorphologyRequest) -> SuccessResponse:
        """Create a new morphology."""
        try:
            success = self.core_api_service.create_morphology(request.morphology_data)
            if not success:
                raise ValueError("Failed to create morphology")
            
            return SuccessResponse(message="Morphology created successfully")
        except Exception as e:
            logger.error(f"Error creating morphology: {e}")
            raise ValueError(f"Failed to create morphology: {str(e)}")
    
    @morphology_endpoint('PUT', '/update',
                        request_model=UpdateMorphologyRequest,
                        response_model=SuccessResponse)
    async def update_morphology(self, request: UpdateMorphologyRequest) -> SuccessResponse:
        """Update an existing morphology."""
        try:
            success = self.core_api_service.update_morphology(request.morphology_id, request.updates)
            if not success:
                raise ValueError("Failed to update morphology")
            
            return SuccessResponse(message="Morphology updated successfully")
        except Exception as e:
            logger.error(f"Error updating morphology: {e}")
            raise ValueError(f"Failed to update morphology: {str(e)}")
    
    @morphology_endpoint('DELETE', '/delete/{morphology_id}', response_model=SuccessResponse)
    async def delete_morphology(self, morphology_id: str) -> SuccessResponse:
        """Delete a morphology."""
        try:
            success = self.core_api_service.delete_morphology(morphology_id)
            if not success:
                raise ValueError("Failed to delete morphology")
            
            return SuccessResponse(message="Morphology deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting morphology: {e}")
            raise ValueError(f"Failed to delete morphology: {str(e)}")


# ===== Factory Function =====

def create_morphology_api(core_api_service: CoreAPIService) -> MorphologyAPI:
    """
    Factory function to create a MorphologyAPI instance.
    
    This function can be used by transport adapters to get a configured
    MorphologyAPI instance with the required dependencies.
    """
    return MorphologyAPI(core_api_service) 