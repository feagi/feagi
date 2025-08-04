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
FEAGI v1 Morphology API - Single Source of Truth

This module contains the ONLY definitions of morphology API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    CreateMorphologyRequest,
    DirectMorphologyRequest,
    MorphologyInfoResponse,
    MorphologyListResponse,
    MorphologyNameRequest,
    SuccessResponse,
    UpdateMorphologyRequest,
)

logger = setup_logger(__name__)


# Define the convenience decorator for morphology endpoints
def morphology_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for morphology endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="morphology",
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

    def _auto_detect_dimension_sensitive(self, morphology_type: str) -> bool:
        """
        Auto-detect dimension_sensitive based on morphology type.
        
        Args:
            morphology_type: The type of morphology (patterns, vectors, functions, etc.)
            
        Returns:
            bool: True if dimension-sensitive, False if dimension-agnostic
        """
        if morphology_type in ["patterns", "vectors"]:
            return False  # Dimension-agnostic
        elif morphology_type == "functions":
            return True   # Dimension-sensitive (e.g., projectors)
        else:
            return False  # Conservative default for composite/unknown types

    # ===== Morphology Information =====

    @morphology_endpoint("GET", "/morphology_list", response_model=List[str])
    async def get_morphology_list(self) -> List[str]:
        """Get list of all morphology names."""
        try:
            morphologies = self.core_api_service.get_morphology_list()
            return morphologies
        except Exception as e:
            logger.error(f"Error getting morphology list: {e}")
            raise ValueError(f"Failed to get morphology list: {str(e)}")

    @morphology_endpoint("GET", "/morphology_types", response_model=List[str])
    async def get_morphology_types(self) -> List[str]:
        """Get list of all morphology types."""
        try:
            types = self.core_api_service.get_morphology_types()
            return types
        except Exception as e:
            logger.error(f"Error getting morphology types: {e}")
            raise ValueError(f"Failed to get morphology types: {str(e)}")

    @morphology_endpoint("GET", "/list/types", response_model=List[str])
    async def get_morphology_list_types(self) -> List[str]:
        """Get list of all morphology types (alias)."""
        try:
            types = self.core_api_service.get_morphology_types()
            return types
        except Exception as e:
            logger.error(f"Error getting morphology list types: {e}")
            raise ValueError(f"Failed to get morphology list types: {str(e)}")

    @morphology_endpoint("GET", "/morphologies", response_model=Dict[str, Any])
    async def get_morphologies(self) -> Dict[str, Any]:
        """Get all morphologies with detailed information."""
        try:
            morphologies = self.core_api_service.get_morphologies()
            return morphologies
        except Exception as e:
            logger.error(f"Error getting morphologies: {e}")
            raise ValueError(f"Failed to get morphologies: {str(e)}")

    @morphology_endpoint("GET", "/list", response_model=MorphologyListResponse)
    async def get_morphologies_list(self) -> MorphologyListResponse:
        """Get list of all morphologies."""
        try:
            morphologies = self.core_api_service.get_morphologies()
            return MorphologyListResponse(morphologies=morphologies)
        except Exception as e:
            logger.error(f"Error getting morphologies list: {e}")
            raise ValueError(f"Failed to get morphologies list: {str(e)}")

    @morphology_endpoint(
        "GET", "/info/{morphology_id}", response_model=MorphologyInfoResponse
    )
    async def get_morphology_info(self, morphology_id: str) -> MorphologyInfoResponse:
        """Get information about a specific morphology."""
        try:
            morphology = self.core_api_service.get_morphology_info(morphology_id)
            return MorphologyInfoResponse(morphology=morphology)
        except Exception as e:
            logger.error(f"Error getting morphology info: {e}")
            raise ValueError(f"Failed to get morphology info: {str(e)}")

    # ===== Morphology Management =====

    @morphology_endpoint(
        "POST",
        "/create",
        request_model=CreateMorphologyRequest,
        response_model=SuccessResponse,
    )
    async def create_morphology(
        self, request: CreateMorphologyRequest
    ) -> SuccessResponse:
        """Create a new morphology."""
        try:
            # Auto-detect dimension_sensitive if not provided
            if request.dimension_sensitive is None:
                morphology_type = request.morphology_data.get("type", "")
                dimension_sensitive = self._auto_detect_dimension_sensitive(morphology_type)
                logger.info(f"Auto-detected dimension_sensitive={dimension_sensitive} for type '{morphology_type}'")
            else:
                dimension_sensitive = request.dimension_sensitive
                logger.info(f"Using provided dimension_sensitive={dimension_sensitive}")

            # Add dimension_sensitive to morphology data
            morphology_data = request.morphology_data.copy()
            morphology_data["dimension_sensitive"] = dimension_sensitive

            success = self.core_api_service.create_morphology(morphology_data)
            if not success:
                raise ValueError("Failed to create morphology")

            return SuccessResponse(message="Morphology created successfully")
        except Exception as e:
            logger.error(f"Error creating morphology: {e}")
            raise ValueError(f"Failed to create morphology: {str(e)}")

    @morphology_endpoint(
        "POST",
        "/morphology",
        request_model=DirectMorphologyRequest,
        response_model=SuccessResponse,
    )
    async def create_morphology_direct(
        self, request: DirectMorphologyRequest
    ) -> SuccessResponse:
        """Create a new morphology with direct client format."""
        try:
            # Auto-detect dimension_sensitive if not provided
            if request.dimension_sensitive is None:
                dimension_sensitive = self._auto_detect_dimension_sensitive(request.morphology_type)
                logger.info(f"Auto-detected dimension_sensitive={dimension_sensitive} for type '{request.morphology_type}'")
            else:
                dimension_sensitive = request.dimension_sensitive
                logger.info(f"Using provided dimension_sensitive={dimension_sensitive}")

            # Convert client format to internal format
            morphology_data = {
                "name": request.morphology_name,
                "type": request.morphology_type,
                "parameters": request.morphology_parameters,
                "dimension_sensitive": dimension_sensitive,
            }

            success = self.core_api_service.create_morphology(morphology_data)
            if not success:
                raise ValueError("Failed to create morphology")

            return SuccessResponse(message="Morphology created successfully")
        except Exception as e:
            logger.error(f"Error creating morphology: {e}")
            raise ValueError(f"Failed to create morphology: {str(e)}")

    @morphology_endpoint(
        "PUT",
        "/update",
        request_model=UpdateMorphologyRequest,
        response_model=SuccessResponse,
    )
    async def update_morphology(
        self, request: UpdateMorphologyRequest
    ) -> SuccessResponse:
        """Update an existing morphology."""
        try:
            success = self.core_api_service.update_morphology(
                request.morphology_id, request.updates
            )
            if not success:
                raise ValueError("Failed to update morphology")

            return SuccessResponse(message="Morphology updated successfully")
        except Exception as e:
            logger.error(f"Error updating morphology: {e}")
            raise ValueError(f"Failed to update morphology: {str(e)}")

    @morphology_endpoint(
        "DELETE", "/delete/{morphology_id}", response_model=SuccessResponse
    )
    async def delete_morphology(self, morphology_id: str) -> SuccessResponse:
        """Delete a morphology by ID."""
        try:
            success = self.core_api_service.delete_morphology(morphology_id)
            if not success:
                raise ValueError("Failed to delete morphology")

            return SuccessResponse(message="Morphology deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting morphology: {e}")
            raise ValueError(f"Failed to delete morphology: {str(e)}")

    @morphology_endpoint(
        "DELETE",
        "/morphology",
        request_model=MorphologyNameRequest,
        response_model=SuccessResponse,
    )
    async def delete_morphology_by_name(
        self, request: MorphologyNameRequest
    ) -> SuccessResponse:
        """Delete a morphology by name (client-compatible endpoint)."""
        try:
            success = self.core_api_service.delete_morphology(request.morphology_name)
            if not success:
                raise ValueError("Failed to delete morphology")

            return SuccessResponse(message="Morphology deleted successfully")
        except Exception as e:
            logger.error(f"Error deleting morphology: {e}")
            raise ValueError(f"Failed to delete morphology: {str(e)}")

    @morphology_endpoint(
        "POST",
        "/morphology_properties",
        request_model=MorphologyNameRequest,
        response_model=Dict[str, Any],
    )
    async def get_morphology_properties(
        self, request: MorphologyNameRequest
    ) -> Dict[str, Any]:
        """Get properties of a specific morphology."""
        try:
            properties = self.core_api_service.get_morphology_properties(
                request.morphology_name
            )
            return properties
        except Exception as e:
            logger.error(f"Error getting morphology properties: {e}")
            raise ValueError(f"Failed to get morphology properties: {str(e)}")

    @morphology_endpoint(
        "POST",
        "/morphology_usage",
        request_model=MorphologyNameRequest,
        response_model=List[List[str]],
    )
    async def get_morphology_usage(
        self, request: MorphologyNameRequest
    ) -> List[List[str]]:
        """Get usage report for a specific morphology."""
        try:
            usage = self.core_api_service.get_morphology_usage(request.morphology_name)
            return usage
        except Exception as e:
            logger.error(f"Error getting morphology usage: {e}")
            raise ValueError(f"Failed to get morphology usage: {str(e)}")


# ===== Factory Function =====


def create_morphology_api(core_api_service: CoreAPIService) -> MorphologyAPI:
    """
    Factory function to create a MorphologyAPI instance.

    This function can be used by transport adapters to get a configured
    MorphologyAPI instance with the required dependencies.
    """
    return MorphologyAPI(core_api_service)
