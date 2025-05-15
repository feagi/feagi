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
from feagi.api.rest.dependencies import get_core_api
from feagi.api.rest.schemas import BurstEngineStats

router = APIRouter()
state = FeagiStateManager.instance()


# ######  Burst-Engine Endpoints #########
# ########################################
@router.get("/burst_counter", response_model=int)
async def burst_engine_counter():
    """
    Return the number associated with current FEAGI burst instance.
    """
    # Mock data for now
    return 0


@router.get("/config", response_model=BurstEngineConfig)
async def get_burst_engine_config():
    """Get the current burst engine configuration."""
    return get_core_api().get_burst_engine_config()


@router.put("/config", response_model=BurstEngineConfig)
async def update_burst_engine_config(config: BurstEngineConfig):
    """Update the burst engine configuration."""
    result = get_core_api().update_burst_engine_config(config.dict())
    if not result:
        raise HTTPException(status_code=500, detail="Failed to update burst engine configuration")
    return config


@router.get("/fcl_sampler/config", response_model=FCLSamplerConfig)
async def get_fcl_sampler_config():
    """
    Returns the FCLSampler configuration (frequency, consumer).
    """
    return FCLSamplerConfig(
        frequency=state.get_fcl_sampler_frequency(),
        consumer=FCLSamplerConsumer(state.get_fcl_sampler_consumer())
    )


@router.post("/fcl_sampler/config", response_model=FCLSamplerConfig)
async def update_fcl_sampler_config(config: FCLSamplerConfig):
    """
    Updates the FCLSampler configuration (frequency, consumer).
    Only supports Visualization and Motor as consumer options for now.
    """
    state.set_fcl_sampler_frequency(config.frequency)
    state.set_fcl_sampler_consumer(config.consumer.value)
    # TODO: Notify process manager/FCLSampler to update live config if running
    return config


@router.get("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def get_area_fcl_sample_rate(area_id: int):
    """
    Get the FCL sample rate for a specific cortical area.
    """
    try:
        # Access via CoreAPIService instead of directly using connectome_manager
        rate = get_core_api().get_area_fcl_sample_rate(area_id)
        return FCLSampleRateConfig(sample_rate=rate)
    except KeyError:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving FCL sample rate: {str(e)}")


@router.post("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def set_area_fcl_sample_rate(area_id: int, config: FCLSampleRateConfig):
    """
    Set the FCL sample rate for a specific cortical area.
    """
    try:
        # Access via CoreAPIService instead of directly using connectome_manager
        if config.sample_rate <= 0:
            raise ValueError("Sample rate must be positive")
            
        success = get_core_api().set_area_fcl_sample_rate(area_id, config.sample_rate)
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
async def get_burst_engine_stats():
    """Get the burst engine statistics."""
    return get_core_api().get_burst_engine_stats()


@router.get("/membrane_potentials")
async def get_membrane_potentials(neuron_ids: List[int]):
    """
    Get membrane potentials for specific neurons.
    
    Args:
        neuron_ids: List of neuron IDs to get potentials for
        
    Returns:
        Dictionary mapping neuron IDs to their membrane potentials
    """
    return get_core_api().get_membrane_potentials(neuron_ids)


@router.put("/membrane_potentials")
async def update_membrane_potentials(potentials: Dict[str, float]):
    """
    Update membrane potentials for specific neurons.
    
    Args:
        potentials: Dictionary mapping neuron IDs to membrane potential values
        
    Returns:
        Success message
    """
    # Convert string keys to integers
    neuron_potentials = {int(k): v for k, v in potentials.items()}
    
    result = get_core_api().update_membrane_potentials(neuron_potentials)
    if not result:
        raise HTTPException(status_code=500, detail="Failed to update membrane potentials")
    return {"success": True, "updated_count": len(potentials)}
