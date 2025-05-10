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
from feagi.core.global_objects import connectome
from pydantic import BaseModel

from ...schemas import *


router = APIRouter()
state = FeagiStateManager.instance()


# Define CorticalIdList model for API endpoints
class CorticalIdList(BaseModel):
    cortical_id_list: list[str]


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@router.post("/neurons/membrane_potential_status")
async def cortical_neuron_membrane_potential_monitoring(cortical_area: CorticalIdList):
    print("Cortical membrane potential monitoring", state.neuron_mp_collection_scope)
    response = list()
    for cortical_area in cortical_area.cortical_id_list:
        if cortical_area in state.neuron_mp_collection_scope:
            response.append([cortical_area, True])
        else:
            response.append([cortical_area, False])
    return response


@router.post("/neurons/membrane_potential_set")
async def cortical_neuron_membrane_potential_monitoring(cortical_area: CorticalIdList, state_flag: bool):
    print("Cortical membrane potential monitoring", state.neuron_mp_collection_scope)
    influxdb = state.get_influxdb()
    if influxdb:
        influx_readiness = influxdb.test_influxdb()
        if influx_readiness:
            for cortical_area in cortical_area.cortical_id_list:
                if cortical_area in state.get_genome()['blueprint']:
                    if state_flag and cortical_area not in state.neuron_mp_collection_scope:
                        state.neuron_mp_collection_scope[cortical_area] = {}
                    elif not state_flag and cortical_area in state.neuron_mp_collection_scope:
                        state.neuron_mp_collection_scope.pop(cortical_area)
            return True
    else:
        raise HTTPException(status_code=400, detail="InfluxDb service is not running!")


@router.post("/neuron/synaptic_potential_status")
async def cortical_synaptic_potential_monitoring(cortical_area: CorticalIdList):
    print("Cortical synaptic potential monitoring flag", state.neuron_psp_collection_scope)
    response = list()
    for cortical_area in cortical_area.cortical_id_list:
        if cortical_area in state.neuron_psp_collection_scope:
            response.append([cortical_area, True])
        else:
            response.append([cortical_area, False])
    return response


@router.post("/neuron/synaptic_potential_set")
async def cortical_synaptic_potential_monitoring(cortical_area: CorticalIdList, state_flag: bool):
    print("Cortical synaptic potential monitoring flag", state.neuron_psp_collection_scope)
    influxdb = state.get_influxdb()
    if influxdb:
        if influxdb.test_influxdb():
            for cortical_area in cortical_area.cortical_id_list:
                if cortical_area in state.get_genome()['blueprint']:
                    if state_flag and cortical_area not in state.neuron_psp_collection_scope:
                        state.neuron_psp_collection_scope[cortical_area] = {}
                    elif not state_flag and cortical_area in state.neuron_psp_collection_scope:
                        state.neuron_psp_collection_scope.pop(cortical_area)
            return True
    else:
        raise HTTPException(status_code=400, detail="InfluxDb service is not running!")


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
