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

import os
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from ...schemas import Registration, Logs, Subscriber, VizSkipRate, VizThreshold, BrainVisualization

from ...commons import *
from ...schemas import *

from feagi.version import __version__
from feagi.evo.templates import cortical_types
from feagi.core.state_manager import FeagiStateManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
logger = setup_logger()


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


# ######   System Endpoints #########
# ###################################

@router.get("/user_preferences")
async def get_user_preferences():
    api_service = get_api_service()
    return api_service.get_user_preferences()


@router.put("/user_preferences")
async def update_user_preferences(payload: UserPreferences):
    api_service = get_api_service()
    preferences = {
        "adv_mode": payload.adv_mode,
        "ui_magnification": payload.ui_magnification,
        "auto_pns_area_creation": getattr(payload, "auto_pns_area_creation", None)
    }
    success = api_service.update_user_preferences(preferences)
    if success:
        return {"status": "success"}
    else:
        raise HTTPException(status_code=400, detail="Failed to update user preferences")


def human_readable_version(version):
    logger.info(version, emoji="  ")
    time_portion = str(version)[-10:]
    reminder = str(version)[:-10]
    human_readable_time = datetime.utcfromtimestamp(int(time_portion))
    if reminder:
        if int(reminder) == 0:
            reminder = "C"
        else:
            reminder = "N"
    else:
        reminder = "N"
    return reminder + '-' + human_readable_time.strftime("%Y-%m-%d %H:%M:%S UTC")


@router.get("/versions")
def get_versions():
    api_service = get_api_service()
    try:
        return api_service.get_versions()
    except Exception as e:
        logger.error(f"Error during version collection: {e}")
        raise HTTPException(status_code=500, detail=f"Error retrieving versions: {str(e)}")


@router.get("/health_check")
async def feagi_health_check():
    api_service = get_api_service()
    try:
        health = await api_service.get_system_health()
        return health
    except Exception as e:
        logger.error(f"Error retrieving system health: {e}")
        raise HTTPException(status_code=500, detail=f"Error retrieving system health: {str(e)}")


@router.get("/unique_logs")
async def unique_log_entries():
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        return getattr(state, 'logs', [])
    return []


@router.post("/register")
async def feagi_registration(message: Registration):
    logger.warning("Warning! This endpoint is not doing anything at this time!")
    return "Warning! This endpoint is not doing anything at this time!"


@router.post("/logs")
async def log_management(message: Logs):
    api_service = get_api_service()
    if not hasattr(api_service._connectome_manager, 'api_message_queue'):
        raise HTTPException(status_code=400, detail="API message queue not initialized")
    
    message_dict = message.dict()
    api_message = {"log_management": message_dict}
    api_service._connectome_manager.api_message_queue.put(item=api_message)
    return {"status": "success"}


@router.get("/configuration")
async def configuration_parameters():
    api_service = get_api_service()
    return api_service.get_configuration()


@router.get("/beacon/subscribers")
async def beacon_query():
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state and getattr(state, 'beacon_sub', None):
        return tuple(state.beacon_sub)
    else:
        raise HTTPException(status_code=400, detail=f"No subscriber found")


@router.post("/beacon/subscribe")
async def beacon_subscribe(message: Subscriber):
    api_service = get_api_service()
    if not hasattr(api_service._connectome_manager, 'api_message_queue'):
        raise HTTPException(status_code=400, detail="API message queue not initialized")
    
    message_dict = {'beacon_sub': message.subscriber_address}
    api_service._connectome_manager.api_message_queue.put(item=message_dict)
    return {"status": "success"}


@router.delete("/beacon/unsubscribe")
async def beacon_unsubscribe(message: Subscriber):
    api_service = get_api_service()
    if not hasattr(api_service._connectome_manager, 'api_message_queue'):
        raise HTTPException(status_code=400, detail="API message queue not initialized")
    
    message_dict = {"beacon_unsub": message.subscriber_address}
    api_service._connectome_manager.api_message_queue.put(item=message_dict)
    return {"status": "success"}


@router.get("/db/influxdb/test")
async def test_influxdb():
    """
    Test the connection to the InfluxDB service.
    """
    api_service = get_api_service()
    influx_status = api_service.test_influxdb()
    if influx_status:
        return influx_status
    else:
        raise HTTPException(status_code=400, detail="InfluxDB service not available")


@router.post("/circuit_library_path")
async def change_circuit_library_path(circuit_library_path: str):
    api_service = get_api_service()
    try:
        success = api_service.set_circuit_library_path(circuit_library_path)
        if success:
            return {"status": "success", "message": f"{circuit_library_path} is the new circuit library path."}
        else:
            raise HTTPException(status_code=400, detail="Failed to set circuit library path")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/cortical_area_types")
async def fetch_cortical_area_types():
    api_service = get_api_service()
    return api_service.get_cortical_area_types()


@router.put("/cortical_area_types")
async def update_cortical_area_types(cortical_id: str):
    # todo
    return "Endpoint pending implementation"


@router.get("/cortical_area_visualization_skip_rate")
async def get_cortical_area_visualization_skip_rate():
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        return getattr(state, 'cortical_viz_skip_rate', None)
    return None


@router.get("/cortical_area_visualization_suppression_threshold")
async def get_cortical_area_visualization_suppression_threshold():
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        return getattr(state, 'cortical_viz_sup_threshold', None)
    return None


@router.put("/cortical_area_visualization_skip_rate")
async def update_cortical_area_visualization_skip_rate(viz_skip: VizSkipRate):
    """
    Set the FCL sample rate (Hz) for a specific cortical area for visualization purposes.
    """
    api_service = get_api_service()
    area_id = viz_skip.cortical_area
    skip_rate = viz_skip.skip_rate
    
    if skip_rate <= 0:
        raise HTTPException(status_code=400, detail="Skip rate must be positive (Hz)")
        
    try:
        success = api_service.set_area_fcl_sample_rate(int(area_id), skip_rate)
        if success:
            return {"cortical_area": area_id, "fcl_sample_rate": skip_rate}
        else:
            raise HTTPException(status_code=400, detail="Failed to update visualization skip rate")
    except KeyError:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/cortical_area_visualization_suppression_threshold")
async def update_cortical_area_visualization_suppression_threshold(visualization_threshold: VizThreshold):
    """Controls the level of voxel activity per cortical area where if exceeded will enforce visualization frequency
    control to kick in."""
    if visualization_threshold.visualization_threshold < 0:
        raise HTTPException(status_code=400, detail=f"Suppression threshold cannot be negative.")
        
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        state.cortical_viz_sup_threshold = visualization_threshold.visualization_threshold
        return {"status": "success"}
    else:
        raise HTTPException(status_code=400, detail="State manager not available")


@router.get("/global_activity_visualization")
async def fetch_cortical_area_visualization_globally():
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        return state.brain_activity_pub
    return False


@router.put("/global_activity_visualization")
async def update_cortical_area_visualization_globally(viz_ctrl: BrainVisualization):
    """Future placeholder for whole brain visualization. Currently just an API placeholder."""
    api_service = get_api_service()
    state = api_service.get_state_manager()
    if state:
        state.brain_activity_pub = viz_ctrl.visualization
        return {"status": "success"}
    else:
        raise HTTPException(status_code=400, detail="State manager not available")


@router.post("/fcl_reset")
async def reset_fire_candidate_list():
    """Reset the FCL"""
    api_service = get_api_service()
    success = api_service.reset_fcl()
    if success:
        return {"status": "success", "message": "FCL reset"}
    else:
        raise HTTPException(status_code=400, detail="Failed to reset FCL")


@router.get("/version")
async def get_version():
    return {"version": __version__}
