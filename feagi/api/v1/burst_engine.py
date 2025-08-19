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
FEAGI v1 Burst Engine API - Single Source of Truth

This module contains the ONLY definitions of burst engine API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Any, Dict, List

from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    BurstEngineConfigRequest,
    BurstEngineStatsResponse,
    BurstEngineStatusResponse,
    FCLContentResponse,
    SuccessResponse,
)

logger = setup_logger(__name__)

# ===== Burst Engine-specific Schemas =====


class BurstEngineRequest(BaseModel):
    """Request model for burst engine configuration."""

    burst_engine_config: Dict[str, Any]


class SimulationTimestepRequest(BaseModel):
    """Request model for simulation timestep endpoint."""

    simulation_timestep: float


# Define the convenience decorator for burst engine endpoints
def burst_engine_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for burst engine endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="burst_engine",
    )


class BurstEngineAPI:
    """
    Burst Engine API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== Legacy Burst Engine Endpoints =====

    @burst_engine_endpoint("GET", "/simulation_timestep")
    def get_simulation_timestep(self) -> float:
        """Returns the simulation timestep (time between neural bursts) in
        seconds."""
        try:
            burst_timer = self.core_api_service.get_burst_timer()
            return burst_timer if burst_timer is not None else 0.0
        except Exception as e:
            logger.error(f"Error getting simulation timestep: {e}")
            raise ValueError(
                f"Failed to get simulation timestep: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "POST",
        "/simulation_timestep",
        request_model=SimulationTimestepRequest,
        response_model=SuccessResponse,
    )
    def change_simulation_timestep(
        self, message: SimulationTimestepRequest
    ) -> SuccessResponse:
        """Update the simulation timestep (neural processing period)."""
        try:
            # Extract simulation timestep directly from the request
            simulation_timestep = message.simulation_timestep

            # Validate simulation timestep
            if simulation_timestep <= 0:
                raise ValueError(
                    "Invalid simulation_timestep: must be a positive number."
                )

            # Convert simulation timestep (seconds) to frequency (Hz)
            # frequency = 1 / period
            burst_frequency_hz = 1.0 / float(simulation_timestep)

            # Update burst engine configuration using the correct method
            config_update = {"burst_frequency_hz": burst_frequency_hz}
            success = self.core_api_service.update_burst_engine_config(
                config_update
            )

            if not success:
                raise ValueError("Failed to change simulation timestep")

            logger.info(
                f"Successfully updated simulation timestep to {simulation_timestep}s (frequency: {burst_frequency_hz}Hz)"
            )
            return SuccessResponse(
                message=f"Simulation timestep changed to {simulation_timestep}s successfully"
            )

        except Exception as e:
            logger.error(f"Error changing simulation timestep: {e}")
            raise ValueError(
                f"Failed to change simulation timestep: {str(e)}"
            ) from e

    # ===== Burst Engine Status and Info =====

    @burst_engine_endpoint(
        "GET", "/status", response_model=BurstEngineStatusResponse
    )
    async def get_burst_engine_status(self) -> BurstEngineStatusResponse:
        """Get the current burst engine status."""
        try:
            status_data = self.core_api_service.get_burst_engine_status()
            return BurstEngineStatusResponse(
                status=status_data.get("status", "unknown"),
                is_running=status_data.get("is_running", False),
                config=status_data.get("config"),
            )
        except Exception as e:
            logger.error(f"Error getting burst engine status: {e}")
            raise ValueError(
                f"Failed to get burst engine status: {str(e)}"
            ) from e

    @burst_engine_endpoint("POST", "/start", response_model=SuccessResponse)
    async def start_burst_engine(self) -> SuccessResponse:
        """Start the burst engine."""
        try:
            success = self.core_api_service.start_burst_engine()
            if not success:
                raise ValueError("Failed to start burst engine")
            return SuccessResponse(message="Burst engine started successfully")
        except Exception as e:
            logger.error(f"Error starting burst engine: {e}")
            raise ValueError(f"Failed to start burst engine: {str(e)}") from e

    @burst_engine_endpoint("POST", "/stop", response_model=SuccessResponse)
    async def stop_burst_engine(self) -> SuccessResponse:
        """Stop the burst engine."""
        try:
            success = self.core_api_service.stop_burst_engine()
            if not success:
                raise ValueError("Failed to stop burst engine")
            return SuccessResponse(message="Burst engine stopped successfully")
        except Exception as e:
            logger.error(f"Error stopping burst engine: {e}")
            raise ValueError(f"Failed to stop burst engine: {str(e)}") from e

    @burst_engine_endpoint("GET", "/burst_counter", response_model=int)
    async def get_burst_counter(self) -> int:
        """Return the number associated with current FEAGI burst instance."""
        try:
            return self.core_api_service.get_burst_counter() or 0
        except Exception as e:
            logger.error(f"Error getting burst counter: {e}")
            raise ValueError(f"Failed to get burst counter: {str(e)}") from e

    @burst_engine_endpoint(
        "GET", "/stats", response_model=BurstEngineStatsResponse
    )
    async def get_burst_engine_stats(self) -> BurstEngineStatsResponse:
        """Get the burst engine statistics."""
        try:
            stats = self.core_api_service.get_burst_engine_stats()
            # Augment with cumulative activity counters from state manager
            try:
                from feagi.core.state_manager import FeagiStateManager

                counters = (
                    FeagiStateManager.instance().get_cumulative_activity()
                )
                stats = dict(stats)
                stats["cumulative_activity_bursts"] = int(
                    counters.get("bursts", 0)
                )
                stats["cumulative_activity_neurons"] = int(
                    counters.get("neurons", 0)
                )
            except Exception:
                pass
            return BurstEngineStatsResponse(stats=stats)
        except Exception as e:
            logger.error(f"Error getting burst engine stats: {e}")
            raise ValueError(
                f"Failed to get burst engine stats: {str(e)}"
            ) from e

    @burst_engine_endpoint("GET", "/fcl", response_model=FCLContentResponse)
    async def get_fcl_content(self) -> FCLContentResponse:
        """Get the complete FCL (Fire Candidate List) content at the current timestep.
        
        Returns all currently firing neurons organized by cortical areas,
        providing a comprehensive snapshot of neural activity.
        """
        try:
            fcl_manager = self.core_api_service.get_fcl_manager()
            if not fcl_manager:
                raise ValueError("FCL manager not available")
            
            # Get current timestep
            current_timestep = fcl_manager.current_timestep
            
            # Get global FCL (all firing neurons)
            global_fcl_bitmap = fcl_manager.get_global_fcl()
            global_fcl_list = list(global_fcl_bitmap) if global_fcl_bitmap else []
            
            # Get FCL organized by cortical areas
            cortical_fcl_dict = fcl_manager.get_fcl_by_cortical()
            
            # Convert cortical areas data to string keys and lists
            cortical_areas = {}
            for cortical_idx, neuron_bitmap in cortical_fcl_dict.items():
                cortical_areas[str(cortical_idx)] = list(neuron_bitmap)
            
            # Get FCL manager statistics
            window_size = fcl_manager.window_size
            active_cortical_count = len(cortical_areas)
            total_neurons = len(global_fcl_list)
            
            return FCLContentResponse(
                timestep=current_timestep,
                total_neurons=total_neurons,
                global_fcl=global_fcl_list,
                cortical_areas=cortical_areas,
                window_size=window_size,
                active_cortical_count=active_cortical_count
            )
            
        except Exception as e:
            logger.error(f"Error getting FCL content: {e}")
            raise ValueError(f"Failed to get FCL content: {str(e)}") from e

    # ===== Burst Engine Configuration =====

    @burst_engine_endpoint("GET", "/config", response_model=Dict[str, Any])
    async def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get the current burst engine configuration."""
        try:
            return self.core_api_service.get_burst_engine_config()
        except Exception as e:
            logger.error(f"Error getting burst engine config: {e}")
            raise ValueError(
                f"Failed to get burst engine config: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "PUT",
        "/config",
        request_model=BurstEngineConfigRequest,
        response_model=Dict[str, Any],
    )
    async def update_burst_engine_config(
        self, request: BurstEngineConfigRequest
    ) -> Dict[str, Any]:
        """Update the burst engine configuration."""
        try:
            result = self.core_api_service.update_burst_engine_config(
                request.config
            )
            if not result:
                raise ValueError("Failed to update burst engine configuration")
            return request.config
        except Exception as e:
            logger.error(f"Error updating burst engine config: {e}")
            raise ValueError(
                f"Failed to update burst engine config: {str(e)}"
            ) from e

    # ===== FCL Sampler Configuration =====

    @burst_engine_endpoint(
        "GET", "/fcl_sampler/config", response_model=Dict[str, Any]
    )
    async def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """Get the FCLSampler configuration (frequency, consumer)."""
        try:
            config = self.core_api_service.get_fcl_sampler_config()
            return {
                "frequency": config["frequency"],
                "consumer": config["consumer"],
            }
        except Exception as e:
            logger.error(f"Error getting FCL sampler config: {e}")
            raise ValueError(
                f"Failed to get FCL sampler config: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "POST", "/fcl_sampler/config", response_model=Dict[str, Any]
    )
    async def update_fcl_sampler_config(
        self, config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Update the FCLSampler configuration (frequency, consumer)."""
        try:
            success = self.core_api_service.update_fcl_sampler_config(
                frequency=config.get("frequency"),
                consumer=config.get("consumer"),
            )

            if not success:
                raise ValueError("Failed to update FCL sampler configuration")

            return config
        except Exception as e:
            logger.error(f"Error updating FCL sampler config: {e}")
            raise ValueError(
                f"Failed to update FCL sampler config: {str(e)}"
            ) from e

    # ===== FCL Sample Rate Management =====

    @burst_engine_endpoint(
        "GET",
        "/fcl_sampler/area/{area_id}/sample_rate",
        response_model=Dict[str, Any],
    )
    async def get_area_fq_sample_rate(self, area_id: int) -> Dict[str, Any]:
        """Get the FCL sample rate for a specific cortical area."""
        try:
            rate = self.core_api_service.get_area_fq_sample_rate(area_id)
            return {"sample_rate": rate}
        except KeyError:
            raise ValueError("Cortical area not found") from None
        except Exception as e:
            logger.error(f"Error getting area FCL sample rate: {e}")
            raise ValueError(
                f"Failed to get area FCL sample rate: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "POST",
        "/fcl_sampler/area/{area_id}/sample_rate",
        response_model=Dict[str, Any],
    )
    async def set_area_fq_sample_rate(
        self, area_id: int, config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Set the FCL sample rate for a specific cortical area."""
        try:
            sample_rate = config.get("sample_rate")
            if sample_rate is None or sample_rate <= 0:
                raise ValueError("Sample rate must be positive")

            success = self.core_api_service.set_area_fq_sample_rate(
                area_id, sample_rate
            )
            if not success:
                raise ValueError("Failed to update FCL sample rate")

            return {"sample_rate": sample_rate}
        except ValueError:
            raise
        except KeyError:
            raise ValueError("Cortical area not found") from None
        except Exception as e:
            logger.error(f"Error setting area FCL sample rate: {e}")
            raise ValueError(
                f"Failed to set area FCL sample rate: {str(e)}"
            ) from e

    # ===== Membrane Potentials =====

    @burst_engine_endpoint(
        "GET", "/membrane_potentials", response_model=Dict[str, float]
    )
    async def get_membrane_potentials(
        self, neuron_ids: List[int]
    ) -> Dict[str, float]:
        """Get membrane potentials for specific neurons."""
        try:
            return self.core_api_service.get_membrane_potentials(neuron_ids)
        except Exception as e:
            logger.error(f"Error getting membrane potentials: {e}")
            raise ValueError(
                f"Failed to get membrane potentials: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "PUT", "/membrane_potentials", response_model=Dict[str, Any]
    )
    async def update_membrane_potentials(
        self, potentials: Dict[str, float]
    ) -> Dict[str, Any]:
        """Update membrane potentials for specific neurons."""
        try:
            # Convert string keys to integers
            neuron_potentials = {int(k): v for k, v in potentials.items()}

            result = self.core_api_service.update_membrane_potentials(
                neuron_potentials
            )
            if not result:
                raise ValueError("Failed to update membrane potentials")

            return {"success": True, "updated_count": len(potentials)}
        except Exception as e:
            logger.error(f"Error updating membrane potentials: {e}")
            raise ValueError(
                f"Failed to update membrane potentials: {str(e)}"
            ) from e

    @burst_engine_endpoint("POST", "/hold", response_model=SuccessResponse)
    async def hold_burst_engine(self) -> SuccessResponse:
        """Put burst engine on hold (pause neural processing)."""
        try:
            success = self.core_api_service.hold_burst_engine()
            if not success:
                raise ValueError("Failed to put burst engine on hold")
            return SuccessResponse(
                message="Burst engine put on hold - neural processing paused"
            )
        except Exception as e:
            logger.error(f"Error putting burst engine on hold: {e}")
            raise ValueError(
                f"Failed to put burst engine on hold: {str(e)}"
            ) from e

    @burst_engine_endpoint("POST", "/resume", response_model=SuccessResponse)
    async def resume_burst_engine(self) -> SuccessResponse:
        """Resume burst engine from hold (resume neural processing)."""
        try:
            success = self.core_api_service.resume_burst_engine()
            if not success:
                raise ValueError("Failed to resume burst engine")
            return SuccessResponse(
                message="Burst engine resumed - neural processing active"
            )
        except Exception as e:
            logger.error(f"Error resuming burst engine: {e}")
            raise ValueError(f"Failed to resume burst engine: {str(e)}") from e

    # ===== Frequency Measurement =====

    @burst_engine_endpoint(
        "POST", "/measure_frequency", response_model=Dict[str, Any]
    )
    async def trigger_frequency_measurement(
        self, duration_seconds: float = 5.0, sample_count: int = 100
    ) -> Dict[str, Any]:
        """Trigger an on-demand burst frequency measurement.

        This is an expensive operation that measures actual burst engine performance
        over a specified period. Use sparingly for monitoring/debugging purposes.

        Args:
            duration_seconds: How long to measure (default 5.0 seconds)
            sample_count: Number of burst samples to collect (default 100)

        Returns:
            Dictionary with measurement results and performance statistics
        """
        try:
            # Validate parameters
            if duration_seconds <= 0 or duration_seconds > 60:
                raise ValueError("Duration must be between 0 and 60 seconds")
            if sample_count <= 0 or sample_count > 1000:
                raise ValueError("Sample count must be between 1 and 1000")

            # Trigger measurement via state manager
            result = self.core_api_service.trigger_frequency_measurement(
                duration_seconds=duration_seconds, sample_count=sample_count
            )

            return result

        except Exception as e:
            logger.error(f"Error triggering frequency measurement: {e}")
            raise ValueError(
                f"Failed to trigger frequency measurement: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "GET", "/frequency_history", response_model=Dict[str, Any]
    )
    async def get_frequency_measurement_history(
        self, limit: int = 10
    ) -> Dict[str, Any]:
        """Get the history of frequency measurements.

        Args:
            limit: Maximum number of recent measurements to return (default 10, max 100)

        Returns:
            Dictionary with measurement history and summary statistics
        """
        try:
            # Validate limit
            if limit <= 0 or limit > 100:
                raise ValueError("Limit must be between 1 and 100")

            history = self.core_api_service.get_frequency_measurement_history(
                limit=limit
            )
            summary = self.core_api_service.get_frequency_status_summary()

            return {
                "history": history,
                "summary": summary,
                "count": len(history),
            }

        except Exception as e:
            logger.error(f"Error getting frequency measurement history: {e}")
            raise ValueError(
                f"Failed to get frequency measurement history: {str(e)}"
            ) from e

    @burst_engine_endpoint(
        "GET", "/frequency_status", response_model=Dict[str, Any]
    )
    async def get_frequency_status(self) -> Dict[str, Any]:
        """Get current frequency status and latest measurement.

        Returns:
            Dictionary with frequency status, target frequency, and latest measurement
        """
        try:
            summary = self.core_api_service.get_frequency_status_summary()
            return summary

        except Exception as e:
            logger.error(f"Error getting frequency status: {e}")
            raise ValueError(
                f"Failed to get frequency status: {str(e)}"
            ) from e


# ===== Factory Function =====


def create_burst_engine_api(
    core_api_service: CoreAPIService,
) -> BurstEngineAPI:
    """Factory function to create a BurstEngineAPI instance.

    This function can be used by transport adapters to get a configured
    BurstEngineAPI instance with the required dependencies.
    """
    return BurstEngineAPI(core_api_service)
