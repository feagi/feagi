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
from feagi.api.core.services.core_api_service import CoreAPIService
from ...schemas import Network
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


# ######  Networking Endpoints #########
# ##################################

@router.get("/network")
async def network_management():
    """
    Get current network configuration settings.
    """
    api_service = get_api_service()
    try:
        network_config = api_service.get_network_config()
        if network_config:
            return network_config
        else:
            raise HTTPException(status_code=400, detail="Network configuration not available")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Network data not available: {str(e)}")


@router.post("/network")
async def update_network_management(message: Network):
    """
    Update network configuration settings.
    """
    api_service = get_api_service()
    try:
        success = api_service.update_network_config(message.dict())
        if success:
            return {"status": "success", "message": "Network configuration updated"}
        else:
            raise HTTPException(status_code=400, detail="Failed to update network configuration")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
