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


from fastapi import APIRouter, HTTPException

from ...schemas import BurstEngineConfig, FCLSamplerConfig, FCLSamplerConsumer, FCLSampleRateConfig
from ...commons import *
from feagi.core.state_manager import FeagiStateManager
from feagi.process_manager import get_process_manager

router = APIRouter()


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
    """
    Returns the burst engine configuration.
    """
    # Mock data for now
    return BurstEngineConfig()


@router.post("/config", response_model=BurstEngineConfig)
async def update_burst_engine_config(config: BurstEngineConfig):
    """
    Updates the burst engine configuration.
    """
    message = config.dict()
    message = {'burst_management': message}
    api_queue.put(item=message)
    return config


@router.get("/fcl_sampler/config", response_model=FCLSamplerConfig)
async def get_fcl_sampler_config():
    """
    Returns the FCLSampler configuration (frequency, consumer).
    """
    state = FeagiStateManager.instance()
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
    state = FeagiStateManager.instance()
    state.set_fcl_sampler_frequency(config.frequency)
    state.set_fcl_sampler_consumer(config.consumer.value)
    # TODO: Notify process manager/FCLSampler to update live config if running
    return config


@router.get("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def get_area_fcl_sample_rate(area_id: int):
    """
    Get the FCL sample rate for a specific cortical area.
    """
    # Assume connectome_manager is globally accessible or injected
    area = connectome_manager.cortical_areas.get(area_id)
    if area is None:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    rate = area.properties.get('fcl_sample_rate', None)
    if rate is None:
        rate = FeagiStateManager.instance().get_fcl_sampler_frequency()  # fallback to global
    return FCLSampleRateConfig(sample_rate=rate)


@router.post("/fcl_sampler/area/{area_id}/sample_rate", response_model=FCLSampleRateConfig)
async def set_area_fcl_sample_rate(area_id: int, config: FCLSampleRateConfig):
    """
    Set the FCL sample rate for a specific cortical area.
    """
    if config.sample_rate <= 0:
        raise HTTPException(status_code=400, detail="Sample rate must be positive")
    area = connectome_manager.cortical_areas.get(area_id)
    if area is None:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    area.properties['fcl_sample_rate'] = config.sample_rate
    # Live reconfiguration: notify process manager/FCLSampler if running
    process_mgr = get_process_manager()  # Get the instance
    process_mgr.update_area_sample_rate(area_id, config.sample_rate)
    return FCLSampleRateConfig(sample_rate=config.sample_rate)
