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


from fastapi import APIRouter

from ...schemas import BurstEngineConfig
from ...commons import *

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
