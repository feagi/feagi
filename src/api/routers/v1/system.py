# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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

from ...commons import *
from ...schemas import *

from src.version import __version__
from src.evo.templates import cortical_types


router = APIRouter()


# ######   System Endpoints #########
# ###################################

@router.get("/user_preferences")
async def get_user_preferences():
    return {
        "bv_advanced_mode": runtime_data.bv_advanced_mode,
        "ui_magnification": runtime_data.ui_magnification,
        "auto_pns_area_creation": runtime_data.auto_pns_area_creation
        }


@router.put("/user_preferences")
async def update_user_preferences(payload: UserPreferences):
    runtime_data.bv_advanced_mode = payload.adv_mode
    runtime_data.ui_magnification = payload.ui_magnification


def human_readable_version(version):
    print(version)
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
        for agent_id in runtime_data.agent_registry:
            if agent_id not in all_versions:
                all_versions[agent_id] = {}
            all_versions[agent_id]["agent_version"] = \
                str(runtime_data.agent_registry[agent_id]["agent_version"])
            all_versions[agent_id]["controller_version"] = \
                str(runtime_data.agent_registry[agent_id]["controller_version"])
        return all_versions
    except Exception as e:
        print(f"Error during version collection {e}")


@router.get("/health_check")
async def feagi_health_check():
    health = dict()
    health["burst_engine"] = not runtime_data.exit_condition

    health["connected_agents"] = runtime_data.connected_agents

    if runtime_data.influxdb:
        health["influxdb_availability"] = True
    else:
        health["influxdb_availability"] = False

    health["neuron_count_max"] = int(runtime_data.parameters["Limits"]["max_neuron_count"])
    health["synapse_count_max"] = int(runtime_data.parameters["Limits"]["max_synapse_count"])
    health["latest_changes_saved_externally"] = runtime_data.changes_saved_externally

    if runtime_data.genome:
        health["fitness"] = runtime_data.genome_fitness
        health["genome_availability"] = True
        connectome_neuron_count = runtime_data.brain_stats["neuron_count"]
        connectome_synapse_count = runtime_data.brain_stats["synapse_count"]
        connectome_size = 3E-08 * connectome_neuron_count ** 2 + 0.0011 * connectome_neuron_count + 2.9073

        health["cortical_area_count"] = len(runtime_data.cortical_list)
        health["neuron_count"] = connectome_neuron_count
        health["synapse_count"] = connectome_synapse_count
        health["estimated_brain_size_in_MB"] = connectome_size

    else:
        health["genome_availability"] = False

    health["genome_validity"] = runtime_data.genome_validity
    health["brain_readiness"] = runtime_data.brain_readiness

    if pending_amalgamation():
        health["amalgamation_pending"] = {
            "initiation_time": runtime_data.pending_amalgamation["initiation_time"],
            "genome_id": runtime_data.pending_amalgamation["genome_id"],
            "amalgamation_id": runtime_data.pending_amalgamation["amalgamation_id"],
            "genome_title": runtime_data.pending_amalgamation["genome_title"],
            "circuit_size": runtime_data.pending_amalgamation["circuit_size"]
        }

    return health


@router.get("/unique_logs")
async def unique_log_entries():
    return runtime_data.logs


@router.post("/register")
async def feagi_registration(message: Registration):
    message = message.dict()
    source = message['source']
    host = message['host']
    capabilities = message['capabilities']

    # todo: use
    print("########## ###### >>>>>> >>>> ", source, host, capabilities)


@router.post("/logs")
async def log_management(message: Logs):
    message = message.dict()
    message = {"log_management": message}
    api_queue.put(item=message)


@router.get("/configuration")
async def configuration_parameters():
    return runtime_data.parameters


@router.get("/beacon/subscribers")
async def beacon_query():
    if runtime_data.beacon_sub:
        return tuple(runtime_data.beacon_sub)
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

    influx_status = runtime_data.influxdb.test_influxdb()
    if influx_status:
        return influx_status


@router.post("/circuit_library_path")
async def change_circuit_library_path(circuit_library_path: str):
    if os.path.exists(circuit_library_path):
        runtime_data.circuit_lib_path = circuit_library_path
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
    return runtime_data.cortical_viz_skip_rate


@router.get("/cortical_area_visualization_suppression_threshold")
async def update_cortical_area_visualization_suppression_threshold():
    return runtime_data.cortical_viz_sup_threshold


@router.put("/cortical_area_visualization_skip_rate")
async def update_cortical_area_visualization_skip_rate(cortical_viz_skip_rate: VizSkipRate):
    """Set cortical area visualization skip rate. This value defines the number of skips between each instance of
    neuron firing visualization"""
    if cortical_viz_skip_rate.cortical_viz_skip_rate < 0:
        raise HTTPException(status_code=400, detail=f"Visualization skip rate cannot be negative")
    runtime_data.cortical_viz_skip_rate = cortical_viz_skip_rate.cortical_viz_skip_rate


@router.put("/cortical_area_visualization_suppression_threshold")
async def update_cortical_area_visualization_suppression_threshold(visualization_threshold: VizThreshold):
    """Controls the level of voxel activity per cortical area where if exceeded will enforce visualization frequency
    control to kick in."""
    if visualization_threshold.visualization_threshold < 0:
        raise HTTPException(status_code=400, detail=f"Suppression threshold cannot be negative.")
    runtime_data.cortical_viz_sup_threshold = visualization_threshold.visualization_threshold
