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
import json
from enum import Enum
from datetime import datetime
import logging

from time import time
from fastapi import APIRouter, UploadFile, File, HTTPException, Depends, Query
from starlette.responses import FileResponse
from pydantic import BaseModel
from typing import Optional

from ...schemas import *
from ...commons import *
from feagi.api.dependencies import check_active_genome
from feagi.api.rest.dependencies import get_core_api_service

from feagi.evo.genome_editor import save_genome
from feagi.evo.genome_processor import genome_2_1_convertor, genome_v1_v2_converter
from feagi.bdu.brain_region import region_id_2_title, construct_genome_from_region
from feagi.evo.templates import cortical_template
from feagi.core.state_manager import FeagiStateManager, ConnectomeState
from feagi.bdu import ConnectomeManager


router = APIRouter()

# Helper to get state manager instance
state = FeagiStateManager.instance()

# Dependency for amalgamation history (can be overridden in tests)
def get_amalgamation_history_service():
    return getattr(state, 'amalgamation_history', {})


# AmalgamationRequest model for amalgamation endpoints
class AmalgamationRequest(BaseModel):
    genome_id: Optional[str] = None
    genome_title: Optional[str] = None
    genome_payload: Optional[dict] = None


# Local definition to avoid import issues
class RewiringMode(str, Enum):
    rewire_all = "all"
    rewire_system = "system"
    rewire_none = "none"


# ######  Genome Endpoints #########
# ##################################
@router.post("/upload/barebones")
async def upload_barebones_genome():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
    barebones_genome_path = os.path.join(project_root, "feagi/evo/defaults/genome/barebones_genome.json")
    with open(barebones_genome_path, "r") as genome_file:
        genome_data = json.load(genome_file)
        state.genome_file_name = "barebones_genome.json"
    state.set_connectome_state(ConnectomeState.INITIALIZING)
    core_api_service = get_core_api_service()
    result = core_api_service.load_genome(genome_data, filename="barebones_genome.json")
    result["genome_number"] = state.get_genome_counter()
    burst_engine = core_api_service.get_burst_engine()
    if burst_engine:
        burst_engine.update_with_genome()
        logger.info("Burst Engine updated with new genome", emoji="⚡ ")
    return result


@router.post("/upload/essential")
async def genome_default_upload():
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
    essential_genome_path = os.path.join(project_root, "feagi/evo/defaults/genome/essential_genome.json")
    with open(essential_genome_path, "r") as genome_file:
        genome_data = json.load(genome_file)
        state.genome_file_name = "essential_genome.json"
    state.set_connectome_state(ConnectomeState.INITIALIZING)
    core_api_service = get_core_api_service()
    result = core_api_service.load_genome(genome_data, filename="essential_genome.json")
    result["genome_number"] = state.get_genome_counter()
    burst_engine = core_api_service.get_burst_engine()
    if burst_engine:
        burst_engine.update_with_genome()
        logger.info("Burst Engine updated with new genome", emoji="⚡ ")
    return result


@router.post("/upload/file")
async def genome_file_upload(file: UploadFile = File(...)):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a python file.
    """
    data = await file.read()
    state.set_connectome_state(ConnectomeState.INITIALIZING)
    state.genome_file_name = file.filename

    genome_str = json.loads(data)

    if "genome_title" not in genome_str:
        genome_str["genome_title"] = state.genome_file_name

    if "genome_description" not in genome_str:
        genome_str["genome_description"] = ""

    # genome_str = genome_str.replace('\'', '\"')
    # genome_str = data.decode("utf-8").split(" = ")[1]
    core_api_service = get_core_api_service()
    result = core_api_service.load_genome(genome_str, filename=file.filename)
    # message = {'genome': genome_str}
    # api_queue.put(item=message)
    burst_engine = core_api_service.get_burst_engine()
    if burst_engine:
        burst_engine.update_with_genome()
        logger.info("Burst Engine updated with new genome", emoji="⚡ ")
    return {"loaded": result, "genome_counter": state.get_genome_counter()}


@router.get("/file_name")
async def genome_file_name():
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    genome_name = state.genome_file_name
    if genome_name:
        return genome_name
    else:
        return ""


@router.post("/upload/string")
async def genome_string_upload(genome: dict):

    if "genome_title" not in genome:
        genome["genome_title"] = "Unknown Genome"

    if "genome_description" not in genome:
        genome["genome_description"] = ""

    core_api_service = get_core_api_service()
    result = core_api_service.load_genome(genome)
    # message = {'genome': genome}
    # api_queue.put(item=message)
    burst_engine = core_api_service.get_burst_engine()
    if burst_engine:
        burst_engine.update_with_genome()
        logger.info("Burst Engine updated with new genome", emoji="⚡ ")
    return {"loaded": result, "genome_counter": state.get_genome_counter()}


@router.get("/download")
async def genome_download(_: str = Depends(check_active_genome)):
    logger.info("Downloading Genome...")
    genome = core_api_service.get_genome()
    save_genome(genome=genome_v1_v2_converter(genome),
                file_name=state.connectome_path + "genome.json")
    file_name = "genome-" + genome.get("genome_title", "unknown").replace(" ", "_") + ".json"
    logger.info(file_name)

    if genome and genome.get("blueprint"):
        state.changes_saved_externally = True
        file_path = state.connectome_path + "genome.json"
        headers = {"Content-Disposition": f"attachment; filename={file_name}"}
        response = FileResponse(path=file_path,
                                media_type="application/json",
                                filename=file_name,
                                headers=headers
                                )
        return response
    else:
        raise HTTPException(status_code=400, detail="No running genome found!")


@router.get("/download_region")
async def genome_download_from_region(region_id, _: str = Depends(check_active_genome)):
    logger.info(f"Downloading genome associated with {region_id} ...")

    region_title = region_id_2_title(region_id=region_id)
    genome_payload = construct_genome_from_region(region_id=region_id)

    genome_path = state.connectome_path + f"genome_{region_title}.json"
    save_genome(genome=genome_payload,
                file_name=genome_path)
    file_name = f"genome-{region_title}".replace(" ", "_") + ".json"

    if state.genome:
        state.changes_saved_externally = True
        return FileResponse(path=genome_path, media_type="application/json", filename=file_name)
    else:
        raise HTTPException(status_code=400, detail="No running genome found!")


@router.post("/upload/file/edit")
async def genome_file_upload_edit(file: UploadFile = File(...)):
    data = await file.read()
    genome_str = data.decode("utf-8")
    return {genome_str}


@router.get("/defaults/files")
async def genome_default_files():
    default_genomes_path = "./evo/defaults/genome/"
    default_genomes = os.listdir(default_genomes_path)
    genome_mappings = {}
    for genome in default_genomes:
        if genome[:2] != '__':
            with open(os.path.join(default_genomes_path, genome)) as file:
                data = file.read()
                # genome_mappings = json.loads(data)
                # data_dict = literal_eval(data.split(" = ")[1])
                genome_mappings[genome.split(".")[0]] = json.loads(data)
                # print("genome_mappings\n", genome_mappings)
    return {"genome": genome_mappings}


@router.get("/genome_number")
async def genome_number():
    """
    Return the number associated with current Genome instance.
    """
    return state.get_genome_counter()


@router.post("/reset")
async def reset_genome():
    logger.info("API call has triggered a genome reset")
    state.genome_reset_flag = True


@router.post("/amalgamation_by_payload")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest, _: str = Depends(check_active_genome)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        genome = genome_2_1_convertor(amalgamation_param.genome_payload["blueprint"])

        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        state.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        state.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        state.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        state.pending_amalgamation["initiation_time"] = time()
        state.pending_amalgamation["amalgamation_id"] = amalgamation_id
        state.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=genome["blueprint"])

        state.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/amalgamation_by_upload")
async def amalgamation_attempt(_: str = Depends(check_active_genome), file: UploadFile = File(...)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        data = await file.read()
        state.genome_file_name = file.filename

        genome_str = json.loads(data)
        if "blueprint" not in genome_str:
            raise HTTPException(status_code=400, detail="Missing 'blueprint' key in uploaded genome.")
        genome_2 = genome_2_1_convertor(genome_str["blueprint"])

        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        state.pending_amalgamation["genome_id"] = state.genome_file_name
        state.pending_amalgamation["genome_title"] = state.genome_file_name
        state.pending_amalgamation["genome_payload"] = genome_str
        state.pending_amalgamation["initiation_time"] = time()
        state.pending_amalgamation["amalgamation_id"] = amalgamation_id
        state.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=genome_2["blueprint"])

        state.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/amalgamation_by_filename")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest, _: str = Depends(check_active_genome)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        state.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        state.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        state.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        state.pending_amalgamation["initiation_time"] = time()
        state.pending_amalgamation["amalgamation_id"] = amalgamation_id
        state.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=amalgamation_param.genome_payload["blueprint"])

        state.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.get("/amalgamation_history")
async def amalgamation_history(amalgamation_history=Depends(get_amalgamation_history_service)):
    return amalgamation_history


@router.get("/cortical_template")
async def cortical_template_():
    return cortical_template


@router.post("/amalgamation_destination")
async def amalgamation_conclusion(circuit_origin_x,
                                  circuit_origin_y,
                                  circuit_origin_z,
                                  amalgamation_id,
                                  _: str = Depends(check_active_genome),
                                  brain_region_id="root",
                                  rewire_mode: RewiringMode = Query(default=RewiringMode.rewire_all)):

    if pending_amalgamation():
        payload = dict()
        payload["genome_str"] = state.pending_amalgamation["genome_payload"]
        payload["circuit_origin"] = [int(circuit_origin_x), int(circuit_origin_y), int(circuit_origin_z)]
        payload["parent_brain_region"] = brain_region_id
        payload["rewire_mode"] = rewire_mode.value
        data = {'append_circuit': payload}
        logger.info(data)
        api_queue.put(item=data)
        genome_title = state.pending_amalgamation["genome_title"]

        cancel_pending_amalgamation(amalgamation_id=amalgamation_id)
        state.amalgamation_history["amalgamation_id"] = "complete"
        return f"Amalgamation for \"{genome_title}\" is complete."
    else:
        raise HTTPException(status_code=400, detail="No pending amalgamation request found")


@router.get("/amalgamation")
async def circuit_library(amalgamation_id):
    if amalgamation_id in state.amalgamation_history:
        return state.amalgamation_history[amalgamation_id]
    else:
        raise HTTPException(status_code=400, detail="No matching amalgamation found")


@router.delete("/amalgamation_cancellation")
async def cancel_amalgamation_request(amalgamation_id):
    cancel_pending_amalgamation(amalgamation_id)


@router.get("/circuits")
async def circuit_library():
    """
    Returns the list of neuronal circuits under /evo/circuits
    """
    circuit_list = os.listdir(state.circuit_lib_path)
    return circuit_list


# @router.get("/circuit_description")
# async def cortical_area_types(circuit_name, response: Response):
#     """
#     Returns circuit aka. genome description including its size
#     """

    # with open("./evo/circuits/" + circuit_name, "r") as genome_file:
    #     genome_data = json.load(genome_file)
    #
    # genome2 = genome_2_1_convertor(flat_genome=genome_data["blueprint"])
    #
    # circuit_description = {}
    # circuit_size_ = circuit_size(blueprint=genome2["blueprint"])
    # circuit_description["size"] = circuit_size_
    # if "description" in state.genome:
    #     circuit_description["description"] = state.genome["description"]
    # else:
    #     circuit_description["description"] = ""
    # return circuit_description


@router.post("/append-file")
async def genome_append_circuit(circuit_origin_x: int,
                                circuit_origin_y: int,
                                circuit_origin_z: int,
                                file: UploadFile = File(...)):
    """
    Appends a given circuit to the running genome at a specific location.
    """
    data = await file.read()

    state.genome_file_name = file.filename

    genome_str = json.loads(data)

    payload = dict()
    payload["genome_str"] = genome_str
    payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
    data = {'append_circuit': payload}
    api_queue.put(item=data)


# @router.api_route("/append", methods=['POST'])
# async def genome_append_circuit(circuit_name: str,
#                                 circuit_origin_x: int,
#                                 circuit_origin_y: int,
#                                 circuit_origin_z: int,
#                                 response: Response):
#     """
#     Appends a given circuit to the running genome at a specific location.
#     """
#     try:
#         append_genome_from_file(circuit_name=circuit_name,
#                                 circuit_origin_x=circuit_origin_x,
#                                 circuit_origin_y=circuit_origin_y,
#                                 circuit_origin_z=circuit_origin_z)
#         response.status_code = status.HTTP_200_OK
#     except Exception as e:
#         response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
#         print("API Error:", e)


# def append_genome_from_file(circuit_name: str,
#                             circuit_origin_x: int,
#                             circuit_origin_y: int,
#                             circuit_origin_z: int):
#     circuit_list = os.listdir("./evo/circuits")
#     if circuit_name not in circuit_list:
#         raise HTTPException(status_code=404, detail="Circuit no found")
#     else:
#         with open("./evo/circuits/" + circuit_name, "r") as genome_file:
#             source_genome = json.load(genome_file)
#         payload = dict()
#         payload["genome_str"] = source_genome
#         payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
#         data = {'append_circuit': payload}
#         api_queue.put(item=data)


# def append_genome_from_payload(genome_payload: dict,
#                                circuit_origin_x: int,
#                                circuit_origin_y: int,
#                                circuit_origin_z: int):
#     payload = dict()
#     payload["genome_str"] = genome_payload
#     payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
#     data = {'append_circuit': payload}
#     api_queue.put(item=data)


def circuit_size(blueprint):
    """
    Returns the size of genome in the form of voxel count in each axis

    Returns:
        (x, y, z)
    """
    dimensions = [1, 1, 1]

    for cortical_area in blueprint:
        x_coord = blueprint[cortical_area]["block_boundaries"][0] + blueprint[cortical_area]["relative_coordinate"][0]
        y_coord = blueprint[cortical_area]["block_boundaries"][1] + blueprint[cortical_area]["relative_coordinate"][1]
        z_coord = blueprint[cortical_area]["block_boundaries"][2] + blueprint[cortical_area]["relative_coordinate"][2]

        if x_coord > dimensions[0]:
            dimensions[0] = x_coord

        if y_coord > dimensions[1]:
            dimensions[1] = y_coord

        if z_coord > dimensions[2]:
            dimensions[2] = z_coord

    return dimensions
