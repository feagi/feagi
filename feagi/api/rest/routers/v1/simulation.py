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
from feagi.core.state_manager import FeagiStateManager
from feagi.bdu import ConnectomeManager
from pydantic import BaseModel
from feagi.api.core.services.core_api_service import CoreAPIService

from ...schemas import *
from ...commons import *


router = APIRouter()
# Get dependencies
state_manager = FeagiStateManager.instance()

# Get CoreAPIService instance
def get_api_service():
    connectome_manager = state_manager.get_connectome()
    if not connectome_manager:
        # Create a minimal version if not available
        from feagi.bdu.connectome_manager import ConnectomeManager
        connectome_manager = ConnectomeManager()
        
    return CoreAPIService(connectome_manager=connectome_manager, state_manager=state_manager)


# Stimulation model for endpoints
class Stimulation(BaseModel):
    stimulation_script: str


# ######  Stimulation #########
# #############################

@router.post("/upload/string")
async def stimulation_string_upload(stimulation_script: Stimulation):
    """
    Upload a stimulation script to be executed during simulation.
    
    The script follows a specified format for defining stimulation patterns.
    """
    api_service = get_api_service()
    try:
        success = api_service.set_stimulation_script(stimulation_script.stimulation_script)
        if success:
            return {"status": "success", "message": "Stimulation script uploaded"}
        else:
            raise HTTPException(status_code=400, detail="Failed to set stimulation script")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/reset")
async def stimulation_reset():
    """
    Reset all stimulation scripts.
    """
    api_service = get_api_service()
    try:
        success = api_service.reset_stimulation_script()
        if success:
            return {"status": "success", "message": "Stimulation scripts reset"}
        else:
            raise HTTPException(status_code=400, detail="Failed to reset stimulation script")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/start")
async def start_simulation():
    """
    Start or resume the simulation.
    """
    api_service = get_api_service()
    try:
        success = api_service.start_simulation()
        if success:
            return {"status": "success", "message": "Simulation started"}
        else:
            raise HTTPException(status_code=400, detail="Failed to start simulation")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/stop")
async def stop_simulation():
    """
    Stop the simulation.
    """
    api_service = get_api_service()
    try:
        success = api_service.stop_simulation()
        if success:
            return {"status": "success", "message": "Simulation stopped"}
        else:
            raise HTTPException(status_code=400, detail="Failed to stop simulation")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/status")
async def get_simulation_status():
    """
    Get current simulation status including running state and burst count.
    """
    api_service = get_api_service()
    try:
        status = await api_service.get_simulation_status()
        return status
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/performance")
async def get_simulation_performance():
    """
    Get performance metrics for the running simulation.
    """
    api_service = get_api_service()
    try:
        stats = await api_service.get_performance_stats()
        return stats
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
