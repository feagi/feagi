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

from fastapi import APIRouter, UploadFile, File, HTTPException, Depends, Query
from starlette.responses import FileResponse

from ...schemas import *
from ...commons import *
from ...dependencies import check_active_genome

from src.inf import runtime_data
from src.evo.genome_editor import save_genome
from src.evo.genome_processor import genome_2_1_convertor, genome_v1_v2_converter
from src.evo.stats import circuit_size
from src.evo.region import region_id_2_title, construct_genome_from_region
from src.evo.templates import cortical_template
from src.inf.initialize import generate_cortical_dimensions_by_id


router = APIRouter()


# ######  Genome Endpoints #########
# ##################################
@router.post("/upload/barebones")
async def upload_barebones_genome():

    with open("./evo/defaults/genome/barebones_genome.json", "r") as genome_file:
        genome_data = json.load(genome_file)
        runtime_data.genome_file_name = "barebones_genome.json"
    runtime_data.brain_readiness = False
    message = {'genome': genome_data}

    api_queue.put(item=message)


@router.post("/upload/essential")
async def genome_default_upload():
    with open("./evo/defaults/genome/essential_genome.json", "r") as genome_file:
        genome_data = json.load(genome_file)
        runtime_data.genome_file_name = "essential_genome.json"
    runtime_data.brain_readiness = False
    message = {'genome': genome_data}
    api_queue.put(item=message)


@router.post("/upload/file")
async def genome_file_upload(file: UploadFile = File(...)):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a python file.
    """
    data = await file.read()
    runtime_data.brain_readiness = False
    runtime_data.genome_file_name = file.filename

    genome_str = json.loads(data)

    if "genome_title" not in genome_str:
        genome_str["genome_title"] = runtime_data.genome_file_name

    if "genome_description" not in genome_str:
        genome_str["genome_description"] = ""

    # genome_str = genome_str.replace('\'', '\"')
    # genome_str = data.decode("utf-8").split(" = ")[1]
    message = {'genome': genome_str}
    api_queue.put(item=message)


@router.get("/file_name")
async def genome_file_name():
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    genome_name = runtime_data.genome_file_name
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

    message = {'genome': genome}
    api_queue.put(item=message)


@router.get("/download")
async def genome_download(_: str = Depends(check_active_genome)):
    print("Downloading Genome...")
    save_genome(genome=genome_v1_v2_converter(runtime_data.genome),
                file_name=runtime_data.connectome_path + "genome.json")
    file_name = "genome-" + runtime_data.genome["genome_title"].replace(" ", "_") + ".json"
    print(file_name)

    if runtime_data.genome:
        runtime_data.changes_saved_externally = True
        file_path = runtime_data.connectome_path + "genome.json"
        headers = {"Content-Disposition": f"attachment; filename={file_name}"}
        response = FileResponse(path=file_path,
                                media_type="application/json",
                                filename=file_name,
                                headers=headers
                                )
        # response.headers["Content-Disposition"] = f'attachment; filename=\"{file_name}\"'
        # print("response headers", response.headers)
        return response
    else:
        raise HTTPException(status_code=400, detail="No running genome found!")


@router.get("/download_region")
async def genome_download_from_region(region_id, _: str = Depends(check_active_genome)):
    print(f"Downloading genome associated with {region_id} ...")

    region_title = region_id_2_title(region_id=region_id)
    genome_payload = construct_genome_from_region(region_id=region_id)

    genome_path = runtime_data.connectome_path + f"genome_{region_title}.json"
    save_genome(genome=genome_payload,
                file_name=genome_path)
    file_name = f"genome-{region_title}".replace(" ", "_") + ".json"

    if runtime_data.genome:
        runtime_data.changes_saved_externally = True
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
    if runtime_data.genome_counter:
        return runtime_data.genome_counter


@router.post("/reset")
async def reset_genome():
    print("API call has triggered a genome reset")
    runtime_data.genome_reset_flag = True


@router.post("/amalgamation_by_payload")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest, _: str = Depends(check_active_genome)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        genome = genome_2_1_convertor(amalgamation_param.genome_payload["blueprint"])

        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        runtime_data.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        runtime_data.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=genome["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/amalgamation_by_upload")
async def amalgamation_attempt(_: str = Depends(check_active_genome), file: UploadFile = File(...)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        data = await file.read()
        runtime_data.genome_file_name = file.filename

        genome_str = json.loads(data)
        genome_2 = genome_2_1_convertor(genome_str["blueprint"])

        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = runtime_data.genome_file_name
        runtime_data.pending_amalgamation["genome_title"] = runtime_data.genome_file_name
        runtime_data.pending_amalgamation["genome_payload"] = genome_str
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=genome_2["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/amalgamation_by_filename")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest, _: str = Depends(check_active_genome)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        runtime_data.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        runtime_data.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=amalgamation_param.genome_payload["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.get("/amalgamation_history")
async def amalgamation_history():
    return runtime_data.amalgamation_history


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
        payload["genome_str"] = runtime_data.pending_amalgamation["genome_payload"]
        payload["circuit_origin"] = [int(circuit_origin_x), int(circuit_origin_y), int(circuit_origin_z)]
        payload["parent_brain_region"] = brain_region_id
        payload["rewire_mode"] = rewire_mode.value
        data = {'append_circuit': payload}
        print(data)
        api_queue.put(item=data)
        genome_title = runtime_data.pending_amalgamation["genome_title"]

        cancel_pending_amalgamation(amalgamation_id=amalgamation_id)
        runtime_data.amalgamation_history["amalgamation_id"] = "complete"
        return f"Amalgamation for \"{genome_title}\" is complete."
    else:
        raise HTTPException(status_code=400, detail="No pending amalgamation request found")


@router.get("/amalgamation")
async def circuit_library(amalgamation_id):
    if amalgamation_id in runtime_data.amalgamation_history:
        return runtime_data.amalgamation_history[amalgamation_id]
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
    circuit_list = os.listdir(runtime_data.circuit_lib_path)
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
    # if "description" in runtime_data.genome:
    #     circuit_description["description"] = runtime_data.genome["description"]
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

    runtime_data.genome_file_name = file.filename

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
