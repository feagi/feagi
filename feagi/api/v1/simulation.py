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
FEAGI v1 Simulation API - Single Source of Truth

This module contains the ONLY definitions of simulation API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict

from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    SimulationConfigRequest,
    SimulationStatsResponse,
    SimulationStatusResponse,
    SuccessResponse,
)

logger = setup_logger(__name__)

# ===== Simulation-specific Schemas =====


class Stimulation(BaseModel):
    """Request model for stimulation script."""

    stimulation_script: Dict[str, Any]


def simulation_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="simulation",
    )


class SimulationAPI:
    """
    Simulation API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    # ===== Legacy Simulation Endpoints =====

    @simulation_endpoint(
        "POST",
        "/upload/string",
        request_model=Stimulation,
        response_model=SuccessResponse,
    )
    def stimulation_string_upload(
        self, stimulation_script: Stimulation
    ) -> SuccessResponse:
        """
        Upload stimulation script.

        Example stimulation_script:
        {
            "IR_pain": {
                "repeat": 10,
                "definition": [
                    [{"i__pro": ["0-0-3"], "o__mot": ["2-0-7"]}, 10],
                    [{"i__pro": ["0-0-8"]}, 5],
                    [{"i__bat": ["0-0-7"]}, 1],
                    [{}, 50]
                ]
            },
            "exploration": {
                "definition": []
            },
            "move_forward": {
                "definition": []
            },
            "charge_batteries": {
                "repeat": 1000,
                "definition": [
                    [{"i__inf": ["2-0-0"]}, 2]
                ]
            }
        }
        """
        try:
            success = self.core_api_service.upload_stimulation_script(
                stimulation_script.stimulation_script
            )
            if not success:
                raise ValueError("Failed to upload stimulation script")

            return SuccessResponse(message="Stimulation script uploaded successfully")
        except Exception as e:
            logger.error(f"Error uploading stimulation script: {e}")
            raise ValueError(f"Failed to upload stimulation script: {str(e)}")

    @simulation_endpoint("POST", "/reset", response_model=SuccessResponse)
    def reset_simulation(self) -> SuccessResponse:
        """Reset simulation by clearing stimulation script."""
        try:
            success = self.core_api_service.reset_simulation()
            if not success:
                raise ValueError("Failed to reset simulation")

            return SuccessResponse(message="Simulation reset successfully")
        except Exception as e:
            logger.error(f"Error resetting simulation: {e}")
            raise ValueError(f"Failed to reset simulation: {str(e)}")

    # ===== New API Endpoints (for future use) =====

    @simulation_endpoint("GET", "/status", response_model=SimulationStatusResponse)
    async def get_simulation_status(self) -> SimulationStatusResponse:
        """Get current simulation status."""
        try:
            status = self.core_api_service.get_simulation_status()
            return SimulationStatusResponse(
                status=status.get("status", "unknown"),
                is_running=status.get("is_running", False),
                config=status.get("config"),
            )
        except Exception as e:
            raise ValueError(f"Failed to get simulation status: {str(e)}")

    @simulation_endpoint(
        "POST",
        "/configure",
        request_model=SimulationConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_simulation(
        self, request: SimulationConfigRequest
    ) -> SuccessResponse:
        """Configure simulation parameters."""
        try:
            success = self.core_api_service.configure_simulation(request.config)
            if not success:
                raise ValueError("Failed to configure simulation")
            return SuccessResponse(message="Simulation configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure simulation: {str(e)}")

    @simulation_endpoint("GET", "/stats", response_model=SimulationStatsResponse)
    async def get_simulation_stats(self) -> SimulationStatsResponse:
        """Get simulation statistics."""
        try:
            stats = self.core_api_service.get_simulation_stats()
            return SimulationStatsResponse(stats=stats)
        except Exception as e:
            raise ValueError(f"Failed to get simulation stats: {str(e)}")


def create_simulation_api(core_api_service: CoreAPIService) -> SimulationAPI:
    """
    Factory function to create a SimulationAPI instance.

    This function can be used by transport adapters to get a configured
    SimulationAPI instance with the required dependencies.
    """
    return SimulationAPI(core_api_service)
