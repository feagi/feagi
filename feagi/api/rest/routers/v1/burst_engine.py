#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================


from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, List, Optional

from ...schemas import BurstEngineConfig, FCLSamplerConfig, FCLSamplerConsumer, FCLSampleRateConfig
from ...commons import *
from feagi.core.state_manager import FeagiStateManager
from feagi.process_manager import get_process_manager
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.schemas import BurstEngineStats

router = APIRouter()
state = FeagiStateManager.instance()


# ######  Burst-Engine Endpoints #########
# ########################################
@router.get("/burst_counter", response_model=int)
async def burst_engine_counter(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Return the number associated with current FEAGI burst instance.
    """
    return core_api_service.get_burst_counter() or 0


@router.get("/config", response_model=BurstEngineConfig)
async def get_burst_engine_config(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get the current burst engine configuration."""
    return core_api_service.get_burst_engine_config()


@router.put("/config", response_model=BurstEngineConfig)
async def update_burst_engine_config(
    config: BurstEngineConfig,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Update the burst engine configuration."""
    result = core_api_service.update_burst_engine_config(config.dict())
    if not result:
        raise HTTPException(status_code=500, detail="Failed to update burst engine configuration")
    return config


@router.get("/fcl_sampler/config", response_model=FCLSamplerConfig)
async def get_fcl_sampler_config(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the FCLSampler configuration (frequency, consumer).
    """
    config = core_api_service.get_fcl_sampler_config()
    return FCLSamplerConfig(
        frequency=config["frequency"],
        consumer=FCLSamplerConsumer(config["consumer"])
    )


@router.post("/fcl_sampler/config", response_model=FCLSamplerConfig)
async def update_fcl_sampler_config(
    config: FCLSamplerConfig,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Updates the FCLSampler configuration (frequency, consumer).
    Only supports Visualization and Motor as consumer options for now.
    """
    success = core_api_service.update_fcl_sampler_config(
        frequency=config.frequency,
        consumer=config.consumer.value
    )
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update FCL sampler configuration")
        
    return config


@router.get("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def get_area_fcl_sample_rate(
    area_id: int,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get the FCL sample rate for a specific cortical area.
    """
    try:
        # Access via CoreAPIService
        rate = core_api_service.get_area_fcl_sample_rate(area_id)
        return FCLSampleRateConfig(sample_rate=rate)
    except KeyError:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving FCL sample rate: {str(e)}")


@router.post("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def set_area_fcl_sample_rate(
    area_id: int,
    config: FCLSampleRateConfig,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Set the FCL sample rate for a specific cortical area.
    """
    try:
        # Access via CoreAPIService
        if config.sample_rate <= 0:
            raise ValueError("Sample rate must be positive")
            
        success = core_api_service.set_area_fcl_sample_rate(area_id, config.sample_rate)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to update FCL sample rate")
            
        return FCLSampleRateConfig(sample_rate=config.sample_rate)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except KeyError:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error setting FCL sample rate: {str(e)}")


@router.get("/stats", response_model=BurstEngineStats)
async def get_burst_engine_stats(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get the burst engine statistics."""
    return core_api_service.get_burst_engine_stats()


@router.get("/membrane_potentials")
async def get_membrane_potentials(
    neuron_ids: List[int],
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get membrane potentials for specific neurons.
    
    Args:
        neuron_ids: List of neuron IDs to get potentials for
        
    Returns:
        Dictionary mapping neuron IDs to their membrane potentials
    """
    return core_api_service.get_membrane_potentials(neuron_ids)


@router.put("/membrane_potentials")
async def update_membrane_potentials(
    potentials: Dict[str, float],
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Update membrane potentials for specific neurons.
    
    Args:
        potentials: Dictionary mapping neuron IDs to membrane potential values
        
    Returns:
        Success message
    """
    # Convert string keys to integers
    neuron_potentials = {int(k): v for k, v in potentials.items()}
    
    result = core_api_service.update_membrane_potentials(neuron_potentials)
    if not result:
        raise HTTPException(status_code=500, detail="Failed to update membrane potentials")
    return {"success": True, "updated_count": len(potentials)}
