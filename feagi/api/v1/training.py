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
FEAGI v1 Training API - Single Source of Truth

This module contains the ONLY definitions of training API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService

from .decorators import endpoint
from .schemas import (
    SuccessResponse,
    TrainingConfigRequest,
    TrainingStatsResponse,
    TrainingStatusResponse,
)

# ===== Training-specific Schemas =====


class Shock(BaseModel):
    """Request model for shock scenarios."""

    shock: List[str]


class Intensity(BaseModel):
    """Request model for reward/punishment intensity."""

    intensity: float


class FitnessCriteria(BaseModel):
    """Request model for fitness criteria configuration."""

    criteria: Dict[str, float]


class FitnessStats(BaseModel):
    """Request model for fitness statistics."""

    FITNESS_KEYS: Dict[str, Any]
    METADATA: Optional[Dict[str, Any]] = None


def training_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="training",
    )


class TrainingAPI:
    """
    Training API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    # ===== Legacy Training Endpoints =====

    @training_endpoint(
        "DELETE", "/reset_fitness_stats", response_model=SuccessResponse
    )
    def delete_fitness_stats_from_db(self) -> SuccessResponse:
        """Erases the fitness statistics from the database."""
        try:
            success = self.core_api_service.delete_fitness_stats()
            if not success:
                raise ValueError("Failed to delete fitness statistics")

            return SuccessResponse(
                message="Fitness statistics deleted successfully"
            )
        except Exception as e:
            raise ValueError(
                f"Failed to delete fitness statistics: {str(e)}"
            ) from e

    @training_endpoint("GET", "/shock/options")
    def list_available_shock_scenarios(self) -> Dict[str, Any]:
        """Get a list of available shock scenarios."""
        try:
            shock_options = self.core_api_service.get_shock_scenarios_options()
            if shock_options:
                return shock_options
            else:
                return {}
        except Exception as e:
            raise ValueError(
                f"Failed to get shock scenario options: {str(e)}"
            ) from e

    @training_endpoint("GET", "/shock/status")
    def list_activated_shock_scenarios(self) -> Dict[str, Any]:
        """Get currently activated shock scenarios."""
        try:
            shock_scenarios = (
                self.core_api_service.get_activated_shock_scenarios()
            )
            if shock_scenarios:
                return shock_scenarios
            else:
                raise ValueError("No shock scenario is defined")
        except Exception as e:
            raise ValueError(f"Failed to get shock scenarios: {str(e)}") from e

    @training_endpoint(
        "POST",
        "/shock/activate",
        request_model=Shock,
        response_model=SuccessResponse,
    )
    def activate_shock_scenarios(self, shock: Shock) -> SuccessResponse:
        """
        Enables shock for given scenarios. One or many shock scenario could coexist.

        Example:
        {
          "shock": [
            "shock_scenario_1",
            "shock_scenario_2"
          ]
        }
        """
        try:
            message = shock.dict()
            success = self.core_api_service.activate_shock_scenarios(message)
            if not success:
                raise ValueError("Failed to activate shock scenarios")

            return SuccessResponse(
                message="Shock scenarios activated successfully"
            )
        except Exception as e:
            raise ValueError(
                f"Failed to activate shock scenarios: {str(e)}"
            ) from e

    @training_endpoint(
        "POST",
        "/reward",
        request_model=Intensity,
        response_model=SuccessResponse,
    )
    def reward_intensity(self, intensity: Intensity) -> SuccessResponse:
        """Captures feedback from the environment during training."""
        try:
            message = {"reward": intensity.intensity}
            success = self.core_api_service.send_training_feedback(message)
            if not success:
                raise ValueError("Failed to send reward feedback")

            return SuccessResponse(message="Reward feedback sent successfully")
        except Exception as e:
            raise ValueError(
                f"Failed to send reward feedback: {str(e)}"
            ) from e

    @training_endpoint(
        "POST",
        "/punishment",
        request_model=Intensity,
        response_model=SuccessResponse,
    )
    def punishment_intensity(self, intensity: Intensity) -> SuccessResponse:
        """Captures feedback from the environment during training."""
        try:
            message = {"punishment": intensity.intensity}
            success = self.core_api_service.send_training_feedback(message)
            if not success:
                raise ValueError("Failed to send punishment feedback")

            return SuccessResponse(
                message="Punishment feedback sent successfully"
            )
        except Exception as e:
            raise ValueError(
                f"Failed to send punishment feedback: {str(e)}"
            ) from e

    @training_endpoint("POST", "/gameover", response_model=SuccessResponse)
    def gameover_signal(self) -> SuccessResponse:
        """Captures feedback from the environment during training."""
        try:
            message = {"gameover": True}
            success = self.core_api_service.send_training_feedback(message)
            if not success:
                raise ValueError("Failed to send gameover signal")

            return SuccessResponse(message="Gameover signal sent successfully")
        except Exception as e:
            raise ValueError(
                f"Failed to send gameover signal: {str(e)}"
            ) from e

    @training_endpoint("GET", "/training_report")
    def training_report(self) -> Dict[str, Any]:
        """Returns stats associated with training."""
        try:
            return self.core_api_service.get_training_report()
        except Exception as e:
            raise ValueError(f"Failed to get training report: {str(e)}") from e

    @training_endpoint("GET", "/brain_fitness")
    def brain_average_fitness_value(self) -> float:
        """Calculates fitness score based on the defined fitness criteria."""
        try:
            fitness_score = self.core_api_service.calculate_brain_fitness()
            return fitness_score
        except Exception as e:
            raise ValueError(
                f"Failed to calculate brain fitness: {str(e)}"
            ) from e

    @training_endpoint("GET", "/fitness_criteria")
    def fetch_fitness_criteria(self) -> Dict[str, float]:
        """Returns the effective fitness criteria."""
        try:
            return self.core_api_service.get_fitness_criteria()
        except Exception as e:
            raise ValueError(
                f"Failed to get fitness criteria: {str(e)}"
            ) from e

    @training_endpoint(
        "POST", "/fitness_criteria", response_model=SuccessResponse
    )
    def configure_fitness_criteria(
        self, fitness_criteria: Dict[str, float]
    ) -> SuccessResponse:
        """
        Configure the weights associated with each fitness criteria.

        Important: Total weights has to equal to 1.

        Actual game stats will be weighted based on the defined criteria and produce a single fitness value between 0 and 1.

        Example:
        {
            "time_alive": 0.4,
            "max_level_reached": 0.2,
            "score_trying_to_max": 1.0,
            "score_trying_to_min": -1.0,
            "something_custom": 0.4
        }
        """
        try:
            # Validate that weights sum to 1
            key_sum = sum(fitness_criteria.values())
            if abs(key_sum - 1.0) > 0.001:  # Allow small floating point errors
                raise ValueError(
                    "The sum of all FITNESS_KEYS should be equal to 1"
                )

            success = self.core_api_service.configure_fitness_criteria(
                fitness_criteria
            )
            if not success:
                raise ValueError("Failed to configure fitness criteria")

            return SuccessResponse(
                message="Fitness criteria configured successfully"
            )
        except Exception as e:
            raise ValueError(
                f"Failed to configure fitness criteria: {str(e)}"
            ) from e

    @training_endpoint("GET", "/fitness_stats")
    def get_fitness_stats(self) -> List[Dict[str, Any]]:
        """Returns fitness stats."""
        try:
            return self.core_api_service.get_fitness_stats()
        except Exception as e:
            raise ValueError(f"Failed to get fitness stats: {str(e)}") from e

    @training_endpoint(
        "PUT",
        "/fitness_stats",
        request_model=FitnessStats,
        response_model=SuccessResponse,
    )
    def capture_fitness_stats_instance(
        self, fitness_stats: FitnessStats
    ) -> SuccessResponse:
        """
        Updates fitness stats. Data should be in a dictionary form following the structure defined under /fitness_criteria.

        Sample:
        {
        "FITNESS_KEYS":
            {
                "time_alive": 672,
                "max_level_reached": 2,
                "score_trying_to_max": 78,
                "score_trying_to_min": 42,
                "something_custom": 23
            },
        "METADATA":
            {
                "event": "changed such and such environment variable"
            }
        }

        Note: Metadata is optional and to provide additional context.
        """
        try:
            fitness_data = fitness_stats.dict()

            if "FITNESS_KEYS" not in fitness_data:
                raise ValueError(
                    "FITNESS_KEYS is not defined as a dictionary key"
                )

            if "METADATA" not in fitness_data:
                fitness_data["METADATA"] = {}

            success = self.core_api_service.capture_fitness_stats(fitness_data)
            if not success:
                raise ValueError("Failed to capture fitness stats")

            return SuccessResponse(
                message="Fitness stats captured successfully"
            )
        except Exception as e:
            raise ValueError(
                f"Failed to capture fitness stats: {str(e)}"
            ) from e

    @training_endpoint(
        "DELETE", "/fitness_stats", response_model=SuccessResponse
    )
    def reset_fitness_stats(self) -> SuccessResponse:
        """Resets fitness stats."""
        try:
            success = self.core_api_service.reset_fitness_stats()
            if not success:
                raise ValueError("Failed to reset fitness stats")

            return SuccessResponse(message="Fitness stats reset successfully")
        except Exception as e:
            raise ValueError(f"Failed to reset fitness stats: {str(e)}") from e

    # ===== New API Endpoints (for future use) =====

    @training_endpoint("GET", "/status", response_model=TrainingStatusResponse)
    async def get_training_status(self) -> TrainingStatusResponse:
        """Get current training status."""
        try:
            status = self.core_api_service.get_training_status()
            return TrainingStatusResponse(
                status=status.get("status", "unknown"),
                progress=status.get("progress"),
                config=status.get("config"),
            )
        except Exception as e:
            raise ValueError(f"Failed to get training status: {str(e)}") from e

    @training_endpoint(
        "POST",
        "/configure",
        request_model=TrainingConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_training(
        self, request: TrainingConfigRequest
    ) -> SuccessResponse:
        """Configure training parameters."""
        try:
            success = self.core_api_service.configure_training(request.config)
            if not success:
                raise ValueError("Failed to configure training")
            return SuccessResponse(message="Training configured successfully")
        except Exception as e:
            raise ValueError(f"Failed to configure training: {str(e)}") from e

    @training_endpoint("GET", "/stats", response_model=TrainingStatsResponse)
    async def get_training_stats(self) -> TrainingStatsResponse:
        """Get training statistics."""
        try:
            stats = self.core_api_service.get_training_stats()
            return TrainingStatsResponse(stats=stats)
        except Exception as e:
            raise ValueError(f"Failed to get training stats: {str(e)}") from e


def create_training_api(core_api_service: CoreAPIService) -> TrainingAPI:
    """
    Factory function to create a TrainingAPI instance.

    This function can be used by transport adapters to get a configured
    TrainingAPI instance with the required dependencies.
    """
    return TrainingAPI(core_api_service)
