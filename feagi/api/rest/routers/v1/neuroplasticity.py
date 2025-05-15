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

"""
Neuroplasticity router for managing neuroplasticity settings and operations.
"""
from fastapi import APIRouter, Depends, HTTPException, Body
from typing import Dict, Any, List, Optional

from feagi.api.dependencies import (
    check_active_genome,
    check_connectome_ready,
    check_plasticity_enabled,
    check_cortical_area_exists,
    check_burst_engine_running,
    check_fully_operational
)
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.response_utils import success_response, error_response
from feagi.core.state_manager import FeagiStateManager

router = APIRouter()
state = FeagiStateManager.instance()

@router.get("/status")
async def get_plasticity_status(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready)
):
    """
    Get the current neuroplasticity status.
    
    This endpoint requires a connectome to be ready, but plasticity doesn't
    need to be enabled (as the status will report whether it's enabled or not).
    """
    try:
        # Get the plasticity info from the core service
        plasticity_info = core_api_service.get_plasticity_info()
        
        # Return the full plasticity status
        return plasticity_info
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get plasticity status: {str(e)}"
        )

@router.post("/configure")
async def configure_plasticity(
    config: Dict[str, Any] = Body(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_fully_operational)
):
    """
    Configure neuroplasticity settings.
    
    This endpoint requires the full system to be operational
    (genome loaded, connectome ready, burst engine running).
    
    Body:
        config: Dictionary with plasticity configuration
    """
    try:
        # Update the plasticity configuration
        success = core_api_service.update_plasticity_config(config)
        
        if not success:
            raise HTTPException(
                status_code=500,
                detail="Failed to update plasticity configuration"
            )
            
        return success_response(
            message="Plasticity configuration updated successfully",
            data={"config": config}
        )
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(
            status_code=500,
            detail=f"Failed to configure plasticity: {str(e)}"
        )

@router.post("/enable/{area_id}")
async def enable_area_plasticity(
    area_id: str,
    settings: Dict[str, Any] = Body({}),
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_plasticity_enabled),
    __: str = Depends(lambda: check_cortical_area_exists(area_id))
):
    """
    Enable neuroplasticity for a specific cortical area.
    
    This endpoint requires plasticity to be enabled globally,
    and checks that the specified cortical area exists.
    
    Args:
        area_id: ID of the cortical area to enable plasticity for
        settings: Optional plasticity settings specific to this area
    """
    try:
        # Enable plasticity for the area
        success = core_api_service.enable_area_plasticity(area_id, settings)
        
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to enable plasticity for area {area_id}"
            )
            
        return success_response(
            message=f"Plasticity enabled for area {area_id}",
            data={"area_id": area_id, "settings": settings}
        )
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(
            status_code=500,
            detail=f"Failed to enable plasticity for area {area_id}: {str(e)}"
        )

@router.post("/disable/{area_id}")
async def disable_area_plasticity(
    area_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready),
    __: str = Depends(lambda: check_cortical_area_exists(area_id))
):
    """
    Disable neuroplasticity for a specific cortical area.
    
    This only requires the connectome to be ready (not full plasticity),
    as disabling should be possible even if plasticity is globally disabled.
    
    Args:
        area_id: ID of the cortical area to disable plasticity for
    """
    try:
        # Disable plasticity for the area
        success = core_api_service.disable_area_plasticity(area_id)
        
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to disable plasticity for area {area_id}"
            )
            
        return success_response(
            message=f"Plasticity disabled for area {area_id}",
            data={"area_id": area_id}
        )
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(
            status_code=500,
            detail=f"Failed to disable plasticity for area {area_id}: {str(e)}"
        )

@router.get("/transforming")
async def get_transforming_areas(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready)
):
    """
    Get a list of all cortical areas currently undergoing transformation.
    
    This endpoint only requires the connectome to be ready.
    """
    try:
        # Get the transforming areas from the core service
        transforming_areas = core_api_service.get_transforming_areas()
        
        return transforming_areas
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get transforming areas: {str(e)}"
        )

@router.get("/plasticity_queue_depth")
async def show_plasticity_queue_depth(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome)
):
    """
    Returns the current plasticity queue depth value
    
    This endpoint requires a genome to be loaded.
    """
    try:
        queue_depth = core_api_service.get_plasticity_queue_depth()
        return queue_depth
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get plasticity queue depth: {str(e)}"
        )


@router.put("/plasticity_queue_depth")
async def update_plasticity_queue_depth(
    queue_depth: int,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome)
):
    """
    Update the plasticity queue depth setting.
    
    This endpoint requires a genome to be loaded.
    
    Args:
        queue_depth: New queue depth value
    """
    try:
        success = core_api_service.update_plasticity_queue_depth(queue_depth)
        
        if not success:
            raise HTTPException(
                status_code=500,
                detail="Failed to update plasticity queue depth"
            )
            
        return success_response(
            message="Plasticity queue depth updated successfully",
            data={"queue_depth": queue_depth}
        )
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(
            status_code=500,
            detail=f"Failed to update plasticity queue depth: {str(e)}"
        )
