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

from fastapi import APIRouter, HTTPException, Request, Depends

from ...schemas import *
from ...commons import *
from feagi.api.rest.schemas import AgentRegistration

from feagi.core.state_manager import FeagiStateManager
from feagi.bdu import ConnectomeManager
from feagi.core.global_objects import connectome


router = APIRouter()
state = FeagiStateManager.instance()


# ######  Peripheral Nervous System Endpoints #########
# #####################################################

def assign_available_port():
    ports_used = []
    port_ranges = (40001, 40050)
    for agent_id, agent_info in state.agent_registry.items():
        print(agent_id, agent_info, agent_info['agent_type'], type(agent_info['agent_type']))
        if agent_info['agent_type'] != 'monitor':
            ports_used.append(agent_info['agent_data_port'])
    print("ports_used", ports_used)
    for port in range(port_ranges[0], port_ranges[1]):
        if port not in ports_used:
            return port
    return None


@router.get("/list")
async def agent_list():
    agents = set(state.agent_registry.keys())
    if agents:
        return agents
    else:
        return {}


@router.get("/properties")
async def agent_properties(agent_id: str):
    print("agent_id", agent_id)
    print("agent_registry", state.agent_registry)
    agent_info = {}
    if agent_id in state.agent_registry:
        agent_info["agent_type"] = state.agent_registry[agent_id]["agent_type"]
        agent_info["agent_ip"] = state.agent_registry[agent_id]["agent_ip"]
        agent_info["agent_data_port"] = state.agent_registry[agent_id]["agent_data_port"]
        agent_info["agent_router_address"] = state.agent_registry[agent_id]["agent_router_address"]
        agent_info["agent_version"] = state.agent_registry[agent_id]["agent_version"]
        agent_info["controller_version"] = state.agent_registry[agent_id]["controller_version"]
        agent_info["capabilities"] = state.agent_registry[agent_id].get("capabilities", {})
        return agent_info
    else:
        raise HTTPException(status_code=400, detail="Requested agent not found!")


@router.post("/register")
async def agent_registration(request: Request, data: AgentRegistration):
    agent_data_port = data.agent_data_port
    capabilities = {}
    if data.capabilities:
        capabilities = data.capabilities

    agent_info = dict()
    agent_info["agent_id"] = data.agent_id
    agent_info["agent_type"] = data.agent_type
    agent_info["agent_ip"] = request.client.host
    if data.agent_type == 'monitor':
        agent_router_address = f"tcp://{request.client.host}:{data.agent_data_port}"
        print("Publication of brain activity turned on!")
        state.brain_activity_pub = True
    else:
        agent_data_port = assign_available_port()
        agent_router_address = f"tcp://*:{agent_data_port}"

    agent_info["agent_data_port"] = agent_data_port
    agent_info["agent_router_address"] = agent_router_address
    agent_info["agent_version"] = data.agent_version
    agent_info["controller_version"] = data.controller_version
    agent_info["capabilities"] = capabilities

    state.agent_registry[data.agent_id] = agent_info
    state.host_info[data.agent_id] = agent_info

    if state.auto_pns_area_creation and state.get_genome():
        message = {'update_pns_areas': capabilities}
        api_queue.put(item=message)

    print("New agent has been successfully registered:", state.agent_registry[data.agent_id])

    state.evo_change_register["agent"] += 1

    agent_info = state.agent_registry[data.agent_id].copy()
    agent_info.pop('listener')
    return agent_info


@router.delete("/deregister")
async def agent_removal(agent_id: str):

    if agent_id in state.agent_registry:
        agent_info = state.agent_registry.pop(agent_id)
    else:
        raise HTTPException(status_code=400, detail="Requested agent not found!")


@router.post("/parameters")
async def robot_controller_tunner(message: RobotController):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = message.dict()
    message = {'robot_controller': message}
    api_queue.put(item=message)


@router.post("/model")
async def robot_model_modification(message: RobotModel):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = message.dict()
    message = {'robot_model': message}
    api_queue.put(item=message)


@router.get("/gazebo/files")
async def gazebo_robot_default_files():

    default_robots_path = "./evo/defaults/robot/"
    default_robots = os.listdir(default_robots_path)
    return {"robots": default_robots}


@router.post("/manual_stimulation")
async def trigger_manual_stimulation(stimulation: ManualStimulation):
    if not state.is_connectome_ready():
        raise HTTPException(status_code=400, detail="Connectome is not ready!")
    message = {'manual_stimulation': stimulation.stimulation_payload}
    api_queue.put(item=message)


@router.post("/sustained_stimulation")
async def trigger_sustained_stimulation(stimulation: ManualStimulation):
    if not state.is_connectome_ready():
        raise HTTPException(status_code=400, detail="Connectome is not ready!")
    message = {'sustained_stimulation': stimulation.stimulation_payload}
    api_queue.put(item=message)
