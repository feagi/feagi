"""
FEAGI v1 Burst Engine API - Single Source of Truth

This module contains the ONLY definitions of burst engine API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

from typing import Dict, Any, List
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
from .schemas import (
    BurstEngineStatusResponse, BurstEngineConfigRequest, BurstEngineStatsResponse,
    SuccessResponse, ErrorResponse
)
from .decorators import endpoint
from pydantic import BaseModel

logger = setup_logger(__name__)

# ===== Burst Engine-specific Schemas =====

class BurstEngineRequest(BaseModel):
    """Request model for burst engine configuration."""
    burst_engine_config: Dict[str, Any]


# Define the convenience decorator for burst engine endpoints
def burst_engine_endpoint(methods, path, request_model=None, response_model=None, description=None):
    """Convenience decorator for burst engine endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module='burst_engine'
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
    
    @burst_engine_endpoint('GET', '/stimulation_period')
    def get_stimulation_period(self) -> float:
        """Returns the time it takes for each burst to execute in seconds."""
        try:
            burst_timer = self.core_api_service.get_burst_timer()
            return burst_timer if burst_timer is not None else 0.0
        except Exception as e:
            logger.error(f"Error getting stimulation period: {e}")
            raise ValueError(f"Failed to get stimulation period: {str(e)}")
    
    @burst_engine_endpoint('POST', '/stimulation_period', 
                          request_model=BurstEngineRequest,
                          response_model=SuccessResponse)
    def change_stimulation_period(self, message: BurstEngineRequest) -> SuccessResponse:
        """Enables changes against various Burst Engine parameters."""
        try:
            burst_config = {'burst_management': message.burst_engine_config}
            success = self.core_api_service.send_burst_management_message(burst_config)
            
            if not success:
                raise ValueError("Failed to change stimulation period")
            
            return SuccessResponse(message="Stimulation period changed successfully")
        except Exception as e:
            logger.error(f"Error changing stimulation period: {e}")
            raise ValueError(f"Failed to change stimulation period: {str(e)}")

    # ===== Burst Engine Status and Info =====
    
    @burst_engine_endpoint('GET', '/burst_counter', response_model=int)
    async def get_burst_counter(self) -> int:
        """Return the number associated with current FEAGI burst instance."""
        try:
            return self.core_api_service.get_burst_counter() or 0
        except Exception as e:
            logger.error(f"Error getting burst counter: {e}")
            raise ValueError(f"Failed to get burst counter: {str(e)}")
    
    @burst_engine_endpoint('GET', '/stats', response_model=BurstEngineStatsResponse)
    async def get_burst_engine_stats(self) -> BurstEngineStatsResponse:
        """Get the burst engine statistics."""
        try:
            stats = self.core_api_service.get_burst_engine_stats()
            return BurstEngineStatsResponse(stats=stats)
        except Exception as e:
            logger.error(f"Error getting burst engine stats: {e}")
            raise ValueError(f"Failed to get burst engine stats: {str(e)}")
    
    # ===== Burst Engine Configuration =====
    
    @burst_engine_endpoint('GET', '/config', response_model=Dict[str, Any])
    async def get_burst_engine_config(self) -> Dict[str, Any]:
        """Get the current burst engine configuration."""
        try:
            return self.core_api_service.get_burst_engine_config()
        except Exception as e:
            logger.error(f"Error getting burst engine config: {e}")
            raise ValueError(f"Failed to get burst engine config: {str(e)}")
    
    @burst_engine_endpoint('PUT', '/config', 
                          request_model=BurstEngineConfigRequest,
                          response_model=Dict[str, Any])
    async def update_burst_engine_config(self, request: BurstEngineConfigRequest) -> Dict[str, Any]:
        """Update the burst engine configuration."""
        try:
            result = self.core_api_service.update_burst_engine_config(request.config)
            if not result:
                raise ValueError("Failed to update burst engine configuration")
            return request.config
        except Exception as e:
            logger.error(f"Error updating burst engine config: {e}")
            raise ValueError(f"Failed to update burst engine config: {str(e)}")
    
    # ===== FCL Sampler Configuration =====
    
    @burst_engine_endpoint('GET', '/fcl_sampler/config', response_model=Dict[str, Any])
    async def get_fcl_sampler_config(self) -> Dict[str, Any]:
        """Get the FCLSampler configuration (frequency, consumer)."""
        try:
            config = self.core_api_service.get_fcl_sampler_config()
            return {
                "frequency": config["frequency"],
                "consumer": config["consumer"]
            }
        except Exception as e:
            logger.error(f"Error getting FCL sampler config: {e}")
            raise ValueError(f"Failed to get FCL sampler config: {str(e)}")
    
    @burst_engine_endpoint('POST', '/fcl_sampler/config', response_model=Dict[str, Any])
    async def update_fcl_sampler_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Update the FCLSampler configuration (frequency, consumer)."""
        try:
            success = self.core_api_service.update_fcl_sampler_config(
                frequency=config.get("frequency"),
                consumer=config.get("consumer")
            )
            
            if not success:
                raise ValueError("Failed to update FCL sampler configuration")
                
            return config
        except Exception as e:
            logger.error(f"Error updating FCL sampler config: {e}")
            raise ValueError(f"Failed to update FCL sampler config: {str(e)}")
    
    # ===== FCL Sample Rate Management =====
    
    @burst_engine_endpoint('GET', '/fcl_sampler/area/{area_id}/sample_rate', response_model=Dict[str, Any])
    async def get_area_fcl_sample_rate(self, area_id: int) -> Dict[str, Any]:
        """Get the FCL sample rate for a specific cortical area."""
        try:
            rate = self.core_api_service.get_area_fcl_sample_rate(area_id)
            return {"sample_rate": rate}
        except KeyError:
            raise ValueError("Cortical area not found")
        except Exception as e:
            logger.error(f"Error getting area FCL sample rate: {e}")
            raise ValueError(f"Failed to get area FCL sample rate: {str(e)}")
    
    @burst_engine_endpoint('POST', '/fcl_sampler/area/{area_id}/sample_rate', response_model=Dict[str, Any])
    async def set_area_fcl_sample_rate(self, area_id: int, config: Dict[str, Any]) -> Dict[str, Any]:
        """Set the FCL sample rate for a specific cortical area."""
        try:
            sample_rate = config.get("sample_rate")
            if sample_rate is None or sample_rate <= 0:
                raise ValueError("Sample rate must be positive")
                
            success = self.core_api_service.set_area_fcl_sample_rate(area_id, sample_rate)
            if not success:
                raise ValueError("Failed to update FCL sample rate")
                
            return {"sample_rate": sample_rate}
        except ValueError:
            raise
        except KeyError:
            raise ValueError("Cortical area not found")
        except Exception as e:
            logger.error(f"Error setting area FCL sample rate: {e}")
            raise ValueError(f"Failed to set area FCL sample rate: {str(e)}")
    
    # ===== Membrane Potentials =====
    
    @burst_engine_endpoint('GET', '/membrane_potentials', response_model=Dict[str, float])
    async def get_membrane_potentials(self, neuron_ids: List[int]) -> Dict[str, float]:
        """Get membrane potentials for specific neurons."""
        try:
            return self.core_api_service.get_membrane_potentials(neuron_ids)
        except Exception as e:
            logger.error(f"Error getting membrane potentials: {e}")
            raise ValueError(f"Failed to get membrane potentials: {str(e)}")
    
    @burst_engine_endpoint('PUT', '/membrane_potentials', response_model=Dict[str, Any])
    async def update_membrane_potentials(self, potentials: Dict[str, float]) -> Dict[str, Any]:
        """Update membrane potentials for specific neurons."""
        try:
            # Convert string keys to integers
            neuron_potentials = {int(k): v for k, v in potentials.items()}
            
            result = self.core_api_service.update_membrane_potentials(neuron_potentials)
            if not result:
                raise ValueError("Failed to update membrane potentials")
            
            return {
                "success": True,
                "updated_count": len(potentials)
            }
        except Exception as e:
            logger.error(f"Error updating membrane potentials: {e}")
            raise ValueError(f"Failed to update membrane potentials: {str(e)}")


# ===== Factory Function =====

def create_burst_engine_api(core_api_service: CoreAPIService) -> BurstEngineAPI:
    """
    Factory function to create a BurstEngineAPI instance.
    
    This function can be used by transport adapters to get a configured
    BurstEngineAPI instance with the required dependencies.
    """
    return BurstEngineAPI(core_api_service) 