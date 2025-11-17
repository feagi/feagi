"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI v1 Neuroplasticity API - Single Source of Truth

This module contains the ONLY definitions of neuroplasticity API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import SuccessResponse

logger = setup_logger(__name__)


# Define the convenience decorator for neuroplasticity endpoints
def neuroplasticity_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for neuroplasticity endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="neuroplasticity",
    )


class NeuroplasticityAPI:
    """
    Neuroplasticity API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== Plasticity Status and Configuration =====

    @neuroplasticity_endpoint("GET", "/status", response_model=Dict[str, Any])
    async def get_plasticity_status(self) -> Dict[str, Any]:
        """Get the current neuroplasticity status."""
        try:
            plasticity_info = self.core_api_service.get_plasticity_info()
            return plasticity_info
        except Exception as e:
            logger.error(f"Error getting plasticity status: {e}")
            raise ValueError(
                f"Failed to get plasticity status: {str(e)}"
            ) from e

    @neuroplasticity_endpoint(
        "POST", "/configure", response_model=SuccessResponse
    )
    async def configure_plasticity(
        self, config: Dict[str, Any]
    ) -> SuccessResponse:
        """Configure neuroplasticity settings."""
        try:
            success = self.core_api_service.update_plasticity_config(config)
            if not success:
                raise ValueError("Failed to update plasticity configuration")

            return SuccessResponse(
                message="Plasticity configuration updated successfully"
            )
        except Exception as e:
            logger.error(f"Error configuring plasticity: {e}")
            raise ValueError(
                f"Failed to configure plasticity: {str(e)}"
            ) from e

    # ===== Area-Specific Plasticity Control =====

    @neuroplasticity_endpoint(
        "POST", "/enable/{area_id}", response_model=SuccessResponse
    )
    async def enable_area_plasticity(
        self, area_id: str, settings: Dict[str, Any] = None
    ) -> SuccessResponse:
        """Enable neuroplasticity for a specific cortical area."""
        try:
            if settings is None:
                settings = {}

            success = self.core_api_service.enable_area_plasticity(
                area_id, settings
            )
            if not success:
                raise ValueError(
                    f"Failed to enable plasticity for area {area_id}"
                )

            return SuccessResponse(
                message=f"Plasticity enabled for area {area_id}"
            )
        except Exception as e:
            logger.error(f"Error enabling area plasticity: {e}")
            raise ValueError(
                f"Failed to enable plasticity for area {area_id}: {str(e)}"
            ) from e

    @neuroplasticity_endpoint(
        "POST", "/disable/{area_id}", response_model=SuccessResponse
    )
    async def disable_area_plasticity(self, area_id: str) -> SuccessResponse:
        """Disable neuroplasticity for a specific cortical area."""
        try:
            success = self.core_api_service.disable_area_plasticity(area_id)
            if not success:
                raise ValueError(
                    f"Failed to disable plasticity for area {area_id}"
                )

            return SuccessResponse(
                message=f"Plasticity disabled for area {area_id}"
            )
        except Exception as e:
            logger.error(f"Error disabling area plasticity: {e}")
            raise ValueError(
                f"Failed to disable plasticity for area {area_id}: {str(e)}"
            ) from e

    # ===== Transforming Areas =====

    @neuroplasticity_endpoint("GET", "/transforming", response_model=List[str])
    async def get_transforming_areas(self) -> List[str]:
        """Get a list of all cortical areas currently undergoing
        transformation."""
        try:
            return self.core_api_service.get_transforming_areas()
        except Exception as e:
            logger.error(f"Error getting transforming areas: {e}")
            raise ValueError(
                f"Failed to get transforming areas: {str(e)}"
            ) from e

    # ===== Queue Management =====

    @neuroplasticity_endpoint(
        "GET", "/plasticity_queue_depth", response_model=int
    )
    async def get_plasticity_queue_depth(self) -> int:
        """Get the current plasticity queue depth value."""
        try:
            return self.core_api_service.get_plasticity_queue_depth()
        except Exception as e:
            logger.error(f"Error getting plasticity queue depth: {e}")
            raise ValueError(
                f"Failed to get plasticity queue depth: {str(e)}"
            ) from e

    @neuroplasticity_endpoint(
        "PUT", "/plasticity_queue_depth", response_model=SuccessResponse
    )
    async def update_plasticity_queue_depth(
        self, queue_depth: int
    ) -> SuccessResponse:
        """Update the plasticity queue depth setting."""
        try:
            success = self.core_api_service.update_plasticity_queue_depth(
                queue_depth
            )
            if not success:
                raise ValueError("Failed to update plasticity queue depth")

            return SuccessResponse(
                message="Plasticity queue depth updated successfully"
            )
        except Exception as e:
            logger.error(f"Error updating plasticity queue depth: {e}")
            raise ValueError(
                f"Failed to update plasticity queue depth: {str(e)}"
            ) from e


# ===== Factory Function =====


def create_neuroplasticity_api(
    core_api_service: CoreAPIService,
) -> NeuroplasticityAPI:
    """Factory function to create a NeuroplasticityAPI instance.

    This function can be used by transport adapters to get a configured
    NeuroplasticityAPI instance with the required dependencies.
    """
    return NeuroplasticityAPI(core_api_service)
