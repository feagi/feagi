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


from fastapi import APIRouter, HTTPException

from ...schemas import *

from src.inf import runtime_data


router = APIRouter()


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@router.post("/neurons/membrane_potential_status")
async def cortical_neuron_membrane_potential_monitoring(cortical_area: CorticalId):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)

    if cortical_area in runtime_data.neuron_mp_collection_scope:
        return True
    else:
        return False


@router.post("/neurons/membrane_potential_set")
async def cortical_neuron_membrane_potential_monitoring(cortical_area, state: bool):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)

    if runtime_data.influxdb:
        influx_readiness = runtime_data.influxdb.test_influxdb()
        if influx_readiness:
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope.pop(cortical_area)
                else:
                    pass
            return True
    else:
        raise HTTPException(status_code=400, detail="InfluxDb service is not running!")


@router.post("/neuron/synaptic_potential_status")
async def cortical_synaptic_potential_monitoring(cortical_area):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)

    if cortical_area in runtime_data.neuron_psp_collection_scope:
        return True
    else:
        return False


@router.post("/neuron/synaptic_potential_set")
async def cortical_synaptic_potential_monitoring(cortical_area, state: bool):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    if runtime_data.influxdb:
        if runtime_data.influxdb.test_influxdb():
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope.pop(cortical_area)
                else:
                    pass
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
#     Monitor the membrane potential of select cortical areas and voxels in Grafana.
#     Message Template:
#             {
#                 "o__mot": {
#                     "voxels": [[0, 0, 0], [2, 0, 0]],
#                     "neurons": []
#                 },
#                 "i__inf": {
#                     "voxels": [[1, 1, 1]],
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
#     Monitor the post synaptic potentials of select cortical areas and voxels in Grafana.
#
#     Message Template:
#             {
#                 "o__mot": {
#                     "dst_filter": {
#                         "voxels": [[0, 0, 0], [2, 0, 0]],
#                         "neurons": []
#                         },
#                     "sources": {
#                         "i__inf": {
#                             "voxels": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                             },
#                         "o__inf": {
#                             "voxels": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                             }
#                     },
#                 },
#                 "i__bat": {
#                     "dst_filter": {
#                         "voxels": [[0, 0, 0], [2, 0, 0]],
#                         "neurons": []
#                     },
#                     "sources": {
#                         "i__inf": {
#                             "voxels": [[1, 1, 1]],
#                             "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
#                         }
#                     }
#                 }
#             }
#     """
#
#     message = {'neuron_psp_collection_scope': message}
#     api_queue.put(item=message)
