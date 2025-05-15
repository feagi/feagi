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
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, Tuple
from feagi.utils.logger import setup_logger

from feagi.api.rest.schemas import VisionSettings
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService

logger = setup_logger()

router = APIRouter()


# ######   Vision Input Endpoints #########
# ########################################

@router.get("/vision")
async def get_vision_tuning_parameters(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Get the current vision tuning parameters.
    
    Returns:
        Vision configuration parameters.
    """
    return core_api_service.get_vision_config()


@router.post("/vision")
async def set_vision_tuning_parameters(
    vision_settings: VisionSettings,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Set the vision tuning parameters.
    
    Args:
        vision_settings: Vision configuration parameters.
        
    Returns:
        Success message.
    """
    vision_configuration_params = vision_settings.dict(exclude_none=True)
    success = core_api_service.update_vision_config(vision_configuration_params)
    
    if success:
        return {"success": True, "message": "Vision settings updated"}
    else:
        raise HTTPException(status_code=500, detail="Failed to update vision settings")

