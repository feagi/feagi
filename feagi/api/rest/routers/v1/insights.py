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


from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Tuple, Dict, Any, Optional

from feagi.utils.logger import setup_logger
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService

logger = setup_logger()

from ...schemas import *

router = APIRouter()

# Define CorticalIdList model for API endpoints
class CorticalIdList(BaseModel):
    cortical_id_list: list[str]


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@router.post("/neurons/membrane_potential_status")
async def cortical_neuron_membrane_potential_monitoring(
    cortical_area: CorticalIdList,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get membrane potential monitoring status for specified cortical areas.
    """
    result = core_api_service.get_membrane_potential_monitoring_status(cortical_area.cortical_id_list)
    return result


@router.post("/neurons/membrane_potential_set")
async def cortical_neuron_membrane_potential_monitoring(
    cortical_area: CorticalIdList, 
    state_flag: bool,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Enable or disable membrane potential monitoring for specified cortical areas.
    """
    try:
        result = core_api_service.set_membrane_potential_monitoring(
            cortical_areas=cortical_area.cortical_id_list,
            enabled=state_flag
        )
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/neuron/synaptic_potential_status")
async def cortical_synaptic_potential_monitoring(
    cortical_area: CorticalIdList,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get synaptic potential monitoring status for specified cortical areas.
    """
    result = core_api_service.get_synaptic_potential_monitoring_status(cortical_area.cortical_id_list)
    return result


@router.post("/neuron/synaptic_potential_set")
async def cortical_synaptic_potential_monitoring(
    cortical_area: CorticalIdList, 
    state_flag: bool,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Enable or disable synaptic potential monitoring for specified cortical areas.
    """
    try:
        result = core_api_service.set_synaptic_potential_monitoring(
            cortical_areas=cortical_area.cortical_id_list,
            enabled=state_flag
        )
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


# @router.get("/membrane_potential_monitoring/filter_setting")
# async def neuron_membrane_potential_collection_filters():
#     print("Membrane potential monitoring filter setting:", runtime_data.neuron_mp_collection_scope)
#
#     if runtime_data.neuron_mp_collection_scope:
#         return runtime_data.neuron_mp_collection_scope
#     else:
#         return {}
#
#
# @router.get("/postsynaptic_potential_monitoring/filter_setting")
# async def neuron_postsynaptic_potential_collection_filters():
#     print("Membrane potential monitoring filter setting:", runtime_data.neuron_psp_collection_scope)
#     if runtime_data.neuron_psp_collection_scope:
#         return runtime_data.neuron_psp_collection_scope
#     else:
#         return {}
#
#
# @router.post("/membrane_potential_monitoring/filter_setting")
# async def neuron_membrane_potential_monitoring_scope(message: dict):
#     """
#     Monitor the membrane potential of select cortical areas in Grafana.
#     Message Template:
#             {
#                 "o__mot": {
#                     "positions": [[0, 0, 0], [2, 0, 0]],
#                     "neurons": []
#                 },
#                 "i__inf": {
#                     "positions": [[1, 1, 1]],
#                     "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                 },
#                 ...
#             }
#     """
#
#     message = {'neuron_mp_collection_scope': message}
#     api_queue.put(item=message)
#
#
# @router.post("/postsynaptic_potential_monitoring")
# async def neuron_postsynaptic_potential_monitoring_scope(message: dict):
#     """
#     Monitor the post synaptic potentials of select cortical areas in Grafana.
#
#     Message Template:
#             {
#                 "o__mot": {
#                     "dst_filter": {
#                         "positions": [[0, 0, 0], [2, 0, 0]],
#                         "neurons": []
#                         },
#                     "sources": {
#                         "i__inf": {
#                             "positions": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                             },
#                         "o__inf": {
#                             "positions": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                             }
#                     },
#                 },
#                 "i__bat": {
#                     "dst_filter": {
#                         "positions": [[0, 0, 0], [2, 0, 0]],
#                         "neurons": []
#                     },
#                     "sources": {
#                         "i__inf": {
#                             "positions": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                         }
#                     }
#                 }
#             }
#     """
#
#     message = {'neuron_psp_collection_scope': message}
#     api_queue.put(item=message)
