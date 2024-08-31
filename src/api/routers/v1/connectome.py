# Copyright 2016-2024 Neuraville Inc. Authors. All Rights Reserved.
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

from src.api.commons import *
from src.inf import runtime_data
from src.evo.synapse import cortical_mapping
from src.inf.disk_ops import preserve_brain, revive_brain
# from src.inf.feagi import start_feagi
from src.inf.initialize import deploy_genome


router = APIRouter()


# ######  Connectome Endpoints #########
# ######################################
@router.get("/cortical_areas/list/summary")
async def connectome_cortical_areas_summary():
    cortical_list = set()
    for cortical_area in runtime_data.brain:
        cortical_list.add(cortical_area)

    return cortical_list


@router.get("/cortical_areas/list/transforming")
async def transforming_cortical_areas_summary():
    return runtime_data.transforming_areas


@router.get("/cortical_areas/list/detailed")
async def connectome_cortical_areas():
    cortical_list = dict()
    for cortical_area in runtime_data.brain:
        cortical_list[cortical_area] = {}
        cortical_list[cortical_area]["name"] = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
        cortical_list[cortical_area]["type"] = runtime_data.genome["blueprint"][cortical_area]["group_id"]
        cortical_list[cortical_area]["sub_type"] = runtime_data.genome["blueprint"][cortical_area]["sub_group_id"]
        cortical_list[cortical_area]["position"] = []
    return cortical_list


@router.get("/cortical_info")
async def connectome_cortical_info(cortical_area: str):

    if cortical_area in runtime_data.brain:
        return runtime_data.brain[cortical_area]
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
    if runtime_data.plasticity_dict:
        return runtime_data.plasticity_dict
    else:
        return {}


@router.get("/path")
async def connectome_system_path():
    if runtime_data.connectome_path:
        return runtime_data.connectome_path
    else:
        return {}


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
    if runtime_data.brain[cortical_area]:
        return FileResponse(path=runtime_data.connectome_path + cortical_area + ".json", filename=file_name)
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
    if runtime_data.cortical_dimensions:
        return runtime_data.cortical_dimensions
    else:
        return [0, 0, 0]


@router.get("/stats/cortical/cumulative")
async def connectome_dimensions_report(cortical_area: str):
    if cortical_area in runtime_data.cumulative_stats:
        return runtime_data.cumulative_stats[cortical_area]
    else:
        return {}


@router.get("/properties/mappings")
async def connectome_mapping_report():
    """
    Report result can be used with the following tool to visualize the connectome mapping:

    https://csacademy.com/app/graph_editor/

    Note: Use the print out from FEAGI logs for above online editor
    """

    mappings = cortical_mapping()
    if mappings:
        return mappings
    else:
        return {}


@router.get("/download")
async def download_connectome():
    """
    Creates a compressed file containing the entire brain data
    """
    print("Downloading Genome...")

    file_name = "brain_" + datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".feagi"
    print(file_name)
    brain_data = preserve_brain()

    with tempfile.NamedTemporaryFile(delete=False, suffix='.gz') as temp_file:
        temp_file.write(brain_data)

        # Ensure all data is written
        temp_file.flush()

        # Get the size of the file
        temp_file_size = temp_file.tell()
        print(f"Size of data: {temp_file_size} bytes")

        # Seek back to the start of the file
        temp_file.seek(0)

        # Create the FileResponse inside the with block
        response = FileResponse(temp_file.name, media_type="application/gzip", filename=file_name)

        return response


@router.post("/upload")
async def upload_connectome(file: UploadFile = File(...)):

    runtime_data.brain_readiness = False
    runtime_data.genome = {}
    brain_data = await file.read()
    revive_brain(brain_data=brain_data)
    deploy_genome(genome_data=runtime_data.pending_genome)
    runtime_data.new_genome = True
    print("\n Brain successfully initialized.")
