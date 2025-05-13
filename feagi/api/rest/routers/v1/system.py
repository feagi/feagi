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
from feagi.utils.logger import setup_logger
logger = setup_logger()



router = APIRouter()

# Helper to get state manager instance
state = FeagiStateManager.instance()


# ######   System Endpoints #########
# ###################################

@router.get("/user_preferences")
async def get_user_preferences():
    # TODO: Map user preferences to state manager if needed
    return {
        "bv_advanced_mode": getattr(state, 'bv_advanced_mode', None),
        "ui_magnification": getattr(state, 'ui_magnification', None),
        "auto_pns_area_creation": getattr(state, 'auto_pns_area_creation', None)
    }


@router.put("/user_preferences")
async def update_user_preferences(payload: UserPreferences):
    # TODO: Map user preferences to state manager if needed
    state.bv_advanced_mode = payload.adv_mode
    state.ui_magnification = payload.ui_magnification


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
    try:
        all_versions = dict()
        all_versions["feagi"] = str(__version__)
        # TODO: Map agent_registry to state manager if needed
        for agent_id in getattr(state, 'agent_registry', {}):
            if agent_id not in all_versions:
                all_versions[agent_id] = {}
            all_versions[agent_id]["agent_version"] = \
                str(state.agent_registry[agent_id]["agent_version"])
            all_versions[agent_id]["controller_version"] = \
                str(state.agent_registry[agent_id]["controller_version"])
        return all_versions
    except Exception as e:
        print(f"Error during version collection {e}")


@router.get("/health_check")
async def feagi_health_check():
    health = dict()
    # TODO: Map exit_condition, connected_agents, influxdb, etc. to state manager if needed
    health["burst_engine"] = not getattr(state, 'exit_condition', False)
    health["connected_agents"] = getattr(state, 'connected_agents', None)
    health["influxdb_availability"] = bool(getattr(state, 'influxdb', False))
    health["neuron_count_max"] = int(getattr(state, 'parameters', {}).get("Limits", {}).get("max_neuron_count", 0))
    health["synapse_count_max"] = int(getattr(state, 'parameters', {}).get("Limits", {}).get("max_synapse_count", 0))
    health["latest_changes_saved_externally"] = getattr(state, 'changes_saved_externally', False)
    if getattr(state, 'genome', None):
        health["fitness"] = getattr(state, 'genome_fitness', None)
        health["genome_availability"] = True
        connectome_neuron_count = getattr(state, 'brain_stats', {}).get("neuron_count", 0)
        connectome_synapse_count = getattr(state, 'brain_stats', {}).get("synapse_count", 0)
        connectome_size = 3E-08 * connectome_neuron_count ** 2 + 0.0011 * connectome_neuron_count + 2.9073
        health["cortical_area_count"] = len(getattr(state, 'cortical_list', []))
        health["neuron_count"] = connectome_neuron_count
        health["synapse_count"] = connectome_synapse_count
        health["estimated_brain_size_in_MB"] = connectome_size
    else:
        health["genome_availability"] = False
    health["genome_validity"] = getattr(state, 'genome_validity', None)
    health["brain_readiness"] = getattr(state, 'brain_readiness', None)
    # TODO: Map pending_amalgamation to state manager if needed
    if pending_amalgamation():
        health["amalgamation_pending"] = {
            "initiation_time": getattr(state, 'pending_amalgamation', {}).get("initiation_time", None),
            "genome_id": getattr(state, 'pending_amalgamation', {}).get("genome_id", None),
            "amalgamation_id": getattr(state, 'pending_amalgamation', {}).get("amalgamation_id", None),
            "genome_title": getattr(state, 'pending_amalgamation', {}).get("genome_title", None),
            "circuit_size": getattr(state, 'pending_amalgamation', {}).get("circuit_size", None)
        }
    return health


@router.get("/unique_logs")
async def unique_log_entries():
    # TODO: Map logs to state manager if needed
    return getattr(state, 'logs', [])


@router.post("/register")
async def feagi_registration(message: Registration):
    message = message.dict()
    source = message['source']
    host = message['host']
    capabilities = message['capabilities']

    # todo: This endpoint is currently not performing any task

    print("Warning! This endpoint is not doing anything at this time!")
    print("########## ###### >>>>>> >>>> ", source, host, capabilities)
    return "Warning! This endpoint is not doing anything at this time!"


@router.post("/logs")
async def log_management(message: Logs):
    message = message.dict()
    message = {"log_management": message}
    api_queue.put(item=message)


@router.get("/configuration")
async def configuration_parameters():
    # TODO: Map parameters to state manager if needed
    return getattr(state, 'parameters', {})


@router.get("/beacon/subscribers")
async def beacon_query():
    # TODO: Map beacon_sub to state manager if needed
    if getattr(state, 'beacon_sub', None):
        return tuple(state.beacon_sub)
    else:
        raise HTTPException(status_code=400, detail=f"No subscriber found")


@router.post("/beacon/subscribe")
async def beacon_subscribe(message: Subscriber):
    message = {'beacon_sub': message.subscriber_address}
    api_queue.put(item=message)


@router.delete("/beacon/unsubscribe")
async def beacon_unsubscribe(message: Subscriber):
    message = {"beacon_unsub": message.subscriber_address}
    api_queue.put(item=message)


@router.get("/db/influxdb/test")
async def test_influxdb():
    """
    Enables changes against various Burst Engine parameters.
    """
    # TODO: Map influxdb to state manager if needed
    influxdb = getattr(state, 'influxdb', None)
    influx_status = influxdb.test_influxdb() if influxdb else None
    if influx_status:
        return influx_status


@router.post("/circuit_library_path")
async def change_circuit_library_path(circuit_library_path: str):
    if os.path.exists(circuit_library_path):
        # TODO: Map circuit_lib_path to state manager if needed
        state.circuit_lib_path = circuit_library_path
        print(f"{circuit_library_path} is the new circuit library path.")
    else:
        raise HTTPException(status_code=400, detail=f"{circuit_library_path} is not a valid path.")


@router.get("/cortical_area_types")
async def fetch_cortical_area_types():
    return cortical_types


@router.put("/cortical_area_types")
async def update_cortical_area_types(cortical_id: str):
    # todo
    return "Endpoint pending implementation"


@router.get("/cortical_area_visualization_skip_rate")
async def update_cortical_area_visualization_skip_rate():
    # TODO: Map cortical_viz_skip_rate to state manager if needed
    return getattr(state, 'cortical_viz_skip_rate', None)


@router.get("/cortical_area_visualization_suppression_threshold")
async def update_cortical_area_visualization_suppression_threshold():
    # TODO: Map cortical_viz_sup_threshold to state manager if needed
    return getattr(state, 'cortical_viz_sup_threshold', None)


@router.put("/cortical_area_visualization_skip_rate")
async def update_cortical_area_visualization_skip_rate(viz_skip: VizSkipRate):
    """
    Set the FCL sample rate (Hz) for a specific cortical area for visualization purposes.
    """
    area_id = viz_skip.cortical_area
    skip_rate = viz_skip.skip_rate
    if skip_rate <= 0:
        raise HTTPException(status_code=400, detail="Skip rate must be positive (Hz)")
    area = connectome.cortical_areas.get(area_id)
    if area is None:
        raise HTTPException(status_code=404, detail="Cortical area not found")
    area.properties['fcl_sample_rate'] = skip_rate
    # Optionally: notify FCLSampler/process_manager for live update
    # process_manager.update_area_sample_rate(area_id, skip_rate)
    return {"cortical_area": area_id, "fcl_sample_rate": skip_rate}


@router.put("/cortical_area_visualization_suppression_threshold")
async def update_cortical_area_visualization_suppression_threshold(visualization_threshold: VizThreshold):
    """Controls the level of voxel activity per cortical area where if exceeded will enforce visualization frequency
    control to kick in."""
    if visualization_threshold.visualization_threshold < 0:
        raise HTTPException(status_code=400, detail=f"Suppression threshold cannot be negative.")
    # TODO: Map cortical_viz_sup_threshold to state manager if needed
    state.cortical_viz_sup_threshold = visualization_threshold.visualization_threshold


@router.get("/global_activity_visualization")
async def fetch_cortical_area_visualization_globally():
    """Returns the brain activity visualization status across the entire brain."""
    # TODO: Map brain_activity_pub to state manager if needed
    return getattr(state, 'brain_activity_pub', None)


@router.put("/global_activity_visualization")
async def update_cortical_area_visualization_globally(viz_ctrl: BrainVisualization):
    """Controls the brain activity visualization across the entire brain."""
    # TODO: Map brain_activity_pub to state manager if needed
    state.brain_activity_pub = viz_ctrl.global_visualization


@router.post("/fcl_reset")
async def reset_fire_candidate_list():
    """Resets fire candidate list contents"""
    # TODO: Implement reset_fire_candidate_list function
    pass


@router.get("/version")
async def get_version():
    """Returns the current FEAGI version."""
    return JSONResponse(content={"version": __version__})
