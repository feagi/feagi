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
from feagi.utils.logger import setup_logger
logger = setup_logger()

from fastapi import APIRouter, HTTPException, Request, Depends

from ...schemas import *
from ...commons import *
from feagi.api.rest.schemas import AgentRegistration

from feagi.core.state_manager import FeagiStateManager
from feagi.bdu import ConnectomeManager
from feagi.api.core.services.core_api_service import CoreAPIService


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

# ######  Peripheral Nervous System Endpoints #########
# #####################################################

def assign_available_port():
    ports_used = []
    port_ranges = (40001, 40050)
    for agent_id, agent_info in state_manager.agent_registry.items():
        logger.info(f"{agent_id} {agent_info} {agent_info['agent_type']} {type(agent_info['agent_type'])}")
        if agent_info['agent_type'] != 'monitor':
            ports_used.append(agent_info['agent_data_port'])
    logger.info(f"ports_used {ports_used}")
    for port in range(port_ranges[0], port_ranges[1]):
        if port not in ports_used:
            return port
    return None


@router.get("/list")
async def agent_list():
    api_service = get_api_service()
    agents = api_service.get_agent_list()
    if agents:
        return agents
    else:
        return {}


@router.get("/properties")
async def agent_properties(agent_id: str):
    api_service = get_api_service()
    try:
        agent_info = api_service.get_agent_properties(agent_id)
        return agent_info
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/register")
async def agent_registration(request: Request, data: AgentRegistration):
    api_service = get_api_service()
    
    capabilities = {}
    if data.capabilities:
        capabilities = data.capabilities
        
    # Use the CoreAPIService registration method
    try:
        result = api_service.register_agent(
            agent_id=data.agent_id,
            agent_type=data.agent_type,
            agent_ip=request.client.host,
            agent_data_port=data.agent_data_port,
            agent_version=data.agent_version,
            controller_version=data.controller_version,
            capabilities=capabilities
        )
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/deregister")
async def agent_removal(agent_id: str):
    api_service = get_api_service()
    try:
        success = api_service.deregister_agent(agent_id)
        if success:
            return {"status": "success", "message": f"Agent {agent_id} removed successfully"}
        else:
            raise HTTPException(status_code=400, detail=f"Failed to remove agent {agent_id}")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/parameters")
async def robot_controller_tunner(message: RobotController):
    """
    Enables changes against various Burst Engine parameters.
    """
    api_service = get_api_service()
    message_dict = message.dict()
    success = api_service.update_robot_controller(message_dict)
    if success:
        return {"status": "success"}
    else:
        raise HTTPException(status_code=400, detail="Failed to update robot controller")


@router.post("/model")
async def robot_model_modification(message: RobotModel):
    """
    Enables changes against various Burst Engine parameters.
    """
    api_service = get_api_service()
    message_dict = message.dict()
    success = api_service.update_robot_model(message_dict)
    if success:
        return {"status": "success"}
    else:
        raise HTTPException(status_code=400, detail="Failed to update robot model")


@router.get("/gazebo/files")
async def gazebo_robot_default_files():
    api_service = get_api_service()
    return api_service.get_gazebo_robot_files()


@router.post("/manual_stimulation")
async def trigger_manual_stimulation(stimulation: ManualStimulation):
    api_service = get_api_service()
    try:
        success = api_service.trigger_manual_stimulation(stimulation.stimulation_payload)
        if success:
            return {"status": "success"}
        else:
            raise HTTPException(status_code=400, detail="Failed to trigger manual stimulation")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/sustained_stimulation")
async def trigger_sustained_stimulation(stimulation: ManualStimulation):
    api_service = get_api_service()
    try:
        success = api_service.trigger_sustained_stimulation(stimulation.stimulation_payload)
        if success:
            return {"status": "success"}
        else:
            raise HTTPException(status_code=400, detail="Failed to trigger sustained stimulation")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
