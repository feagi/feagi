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


import tempfile
from fastapi import APIRouter, File, UploadFile
from fastapi import HTTPException
from starlette.responses import FileResponse

from ast import literal_eval
from threading import Thread

from ...commons import *
from feagi.bdu import ConnectomeManager
from feagi.core.global_objects import connectome
from feagi.core.state_manager import FeagiStateManager
from feagi.api.core.services.core_api_service import CoreAPIService


router = APIRouter()

# Helper to get state manager instance
state = FeagiStateManager.instance()

# Helper to get CoreAPIService instance
core_api_service = CoreAPIService()


# ######  Connectome Endpoints #########
# ######################################
@router.get("/cortical_areas/list/summary")
async def connectome_cortical_areas_summary():
    areas = core_api_service.get_cortical_areas()
    if not areas:
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")
    return [area["id"] for area in areas]


@router.get("/cortical_areas/list/transforming")
async def transforming_cortical_areas_summary():
    # TODO: Add transforming_areas to state manager if needed
    return getattr(state, 'transforming_areas', {})


@router.get("/cortical_areas/list/detailed")
async def connectome_cortical_areas():
    areas = core_api_service.get_cortical_areas()
    if not areas:
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")
    return areas


@router.get("/cortical_info")
async def connectome_cortical_info(cortical_area: str):
    if cortical_area in state.get_brain():
        return state.get_brain()[cortical_area]
    else:
        raise HTTPException(status_code=400, detail="Requested cortical area not found!")


# @router.get("/all")
# async def connectome_comprehensive_info(response: Response):

    # if runtime_data.brain:
    #     response.status_code = status.HTTP_200_OK
    #     return runtime_data.brain
    # else:
    #     response.status_code = status.HTTP_404_NOT_FOUND


@router.get("/plasticity")
async def connectome_plasticity_info():
    # TODO: Add plasticity_dict to state manager if needed
    return getattr(state, 'plasticity_dict', {})


@router.get("/path")
async def connectome_system_path():
    # TODO: Add connectome_path to state manager if needed
    return getattr(state, 'connectome_path', {})


# @router.post("/source")
# async def connectome_source_path(connectome_path: str):
#     feagi_thread = Thread(target=start_feagi, args=(api_queue, 'connectome', 'path', connectome_path,))
#     feagi_thread.start()


@router.post("/snapshot")
async def connectome_snapshot(connectome_storage_path: str):
    message = {'connectome_path': connectome_storage_path}
    print("Snapshot path:", message)
    api_queue.put(item=message)


@router.get("/download-cortical-area")
async def connectome_download(cortical_area: str):
    print("Downloading Connectome...")
    file_name = "connectome_" + cortical_area + datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".json"
    print(file_name)
    if state.get_brain().get(cortical_area):
        # TODO: Add connectome_path to state manager if needed
        connectome_path = getattr(state, 'connectome_path', '')
        return FileResponse(path=connectome_path + cortical_area + ".json", filename=file_name)
    else:
        raise HTTPException(status_code=400, detail="Requested cortical area not found!")


@router.post("/upload-cortical-area")
async def connectome_file_upload(file: UploadFile = File(...)):
    data = await file.read()
    connectome_str = data.decode("utf-8").split(" = ")[1]
    connectome = literal_eval(connectome_str)
    message = {"connectome": connectome}
    api_queue.put(item=message)
    return {"Connectome received as a file"}


@router.get("/properties/dimensions")
async def connectome_dimensions_report():
    # TODO: Add cortical_dimensions to state manager if needed
    return getattr(state, 'cortical_dimensions', [0, 0, 0])


@router.get("/stats/cortical/cumulative")
async def connectome_dimensions_report(cortical_area: str):
    # TODO: Add cumulative_stats to state manager if needed
    cumulative_stats = getattr(state, 'cumulative_stats', {})
    return cumulative_stats.get(cortical_area, {})


@router.get("/properties/mappings")
async def connectome_mapping_report():
    """
    Report result can be used with the following tool to visualize the connectome mapping:
    https://csacademy.com/app/graph_editor/
    """
    mappings = {}
    for neuron_id in connectome._neuron_id_to_index.keys():
        mappings[neuron_id] = connectome.get_outgoing_connections(neuron_id)
    return mappings


@router.get("/download")
async def download_connectome():
    """
    Creates a compressed file containing the entire brain data
    """
    # TODO: Implement connectome serialization
    raise NotImplementedError("Connectome download is not yet implemented.")


@router.post("/upload")
async def upload_connectome(file: UploadFile = File(...)):
    state.set_connectome_state('not_ready')
    # TODO: Implement connectome upload/restore
    raise NotImplementedError("Connectome upload is not yet implemented.")
