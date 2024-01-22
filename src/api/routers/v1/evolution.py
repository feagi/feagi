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

from fastapi import APIRouter

from src.inf import runtime_data
from src.evo import autopilot

router = APIRouter()


# ######  Evolution #########
# #############################

@router.get("/autopilot/status")
async def return_autopilot_status():
    """
    Returns the status of genome autopilot system.
    """

    if runtime_data.autopilot:
        return runtime_data.autopilot
    else:
        return False


@router.post("/autopilot/on")
async def turn_autopilot_on():
    if not runtime_data.autopilot:
        autopilot.init_generation_dict()
        if runtime_data.brain_run_id:
            autopilot.update_generation_dict()
        runtime_data.autopilot = True
        print("<" * 30, "  Autopilot has been turned on  ", ">" * 30)


@router.post("/autopilot/off", tags=["Evolution"])
async def turn_autopilot_off():
    runtime_data.autopilot = False
    return


@router.get("/generations")
async def list_generations():
    """
    Return details about all generations.
    """

    if runtime_data.generation_dict:
        return runtime_data.generation_dict
    else:
        return {}


@router.get("/change_register")
async def list_generations():
    """
    Return details about all generations.
    """

    if runtime_data.evo_change_register:
        return runtime_data.evo_change_register
    else:
        return {}
