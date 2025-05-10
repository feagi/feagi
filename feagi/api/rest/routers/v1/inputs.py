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
from pydantic import BaseModel, Field
from typing import Optional, Tuple

from ...commons import *
from ...schemas import *

from feagi.version import __version__
from feagi.evo.templates import cortical_types
from feagi.pns.vision import generate_vision_configuration
from feagi.api.rest.schemas import VisionSettings


router = APIRouter()


# ######   System Endpoints #########
# ###################################

@router.get("/vision")
async def get_vision_tuning_parameters():
    vision_params = generate_vision_configuration()

    return vision_params


@router.post("/vision")
async def set_vision_tuning_parameters(vision_settings: VisionSettings):
    vision_configuration_params = vision_settings.dict(exclude_none=True)
    vision_configuration_params = {'vision': vision_configuration_params}
    print("*-----* " * 200 + "\n", vision_configuration_params)
    api_queue.put(item=vision_configuration_params)

