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
from feagi.utils.logger import setup_logger
logger = setup_logger()

from time import time
from fastapi import APIRouter, UploadFile, File, HTTPException, Depends, Query
from starlette.responses import FileResponse, JSONResponse
from pydantic import BaseModel
from typing import Optional
import tempfile

from ...schemas import *
from ...commons import *
from feagi.api.dependencies import (
    check_active_genome, 
    check_connectome_ready, 
    check_burst_engine_running,
    check_genome_and_connectome,
    check_fully_operational,
    check_amalgamation_ready,
    check_deployment_ready
)
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService

from feagi.evo.genome_editor import save_genome
from feagi.evo.genome_processor import genome_2_1_convertor, genome_v1_v2_converter, process_and_load_genome
from feagi.bdu.models.brain_region import region_id_2_title, construct_genome_from_region
from feagi.evo.templates import cortical_template
from feagi.core.state_manager import FeagiStateManager, ConnectomeState, ServiceState, GenomeState
from feagi.bdu import ConnectomeManager
from feagi.api.rest.response_utils import success_response, error_response, raw_response


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
async def upload_barebones_genome(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Upload the barebones genome template"""
    try:
        # Get the path to the barebones genome
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
        barebones_genome_path = os.path.join(project_root, "feagi/evo/defaults/genome/barebones_genome.json")
        
        if not os.path.exists(barebones_genome_path):
            raise HTTPException(status_code=404, detail="Barebones genome template not found")
            
        # Load the genome data
        with open(barebones_genome_path, "r") as genome_file:
            genome_data = json.load(genome_file)
        
        # Set the genome file name in the state manager
        state_manager = FeagiStateManager.instance()
        state_manager.genome_file_name = "barebones_genome.json"
        
        # Set the connectome state
        state_manager.set_connectome_state(ConnectomeState.INITIALIZING)
        
        # Load the genome using CoreAPIService
        result = core_api_service.load_genome(genome_data, filename="barebones_genome.json")
        result["genome_number"] = state_manager.get_genome_counter()
        
        # Update the burst engine
        burst_engine = core_api_service.get_burst_engine()
        if burst_engine:
            burst_engine.update_with_genome()
            logger.info("Burst Engine updated with new genome", emoji1="⚡ ")
            
        return result
    except Exception as e:
        logger.error(f"Failed to upload barebones genome: {str(e)}", emoji1="❌")
        raise HTTPException(status_code=500, detail=f"Error uploading barebones genome: {str(e)}")


@router.post("/upload/essential", status_code=200)
async def genome_default_upload(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Upload the essential genome template"""
    try:
        essential_path = os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/essential_genome.json")
        
        if not os.path.exists(essential_path):
            return JSONResponse(
                status_code=404,
                content=error_response(message="Essential genome template not found!", error_code="RESOURCE_NOT_FOUND")
            )
            
        with open(essential_path, 'r') as f:
            genome_data = json.load(f)
        
        # Set the genome file name
        state_manager = FeagiStateManager.instance()
        state_manager.genome_file_name = "essential_genome.json"
        
        # Process and load the genome
        result = process_and_load_genome(genome_data, core_api_service)
        
        # Update burst engine with new genome
        burst_engine = core_api_service.get_burst_engine()
        if burst_engine and result["success"]:
            burst_engine.update_with_genome()
            logger.info("Burst Engine updated with new genome", emoji1="⚡")
        
        return success_response(
            data=result,
            message="Essential genome uploaded successfully"
        )
        
    except Exception as e:
        logger.error(f"Failed to upload essential genome: {str(e)}", emoji1="❌")
        return JSONResponse(
            status_code=500,
            content=error_response(message=f"Failed to upload essential genome: {str(e)}", error_code="GENOME_UPLOAD_ERROR")
        )


@router.post("/upload/file")
async def genome_file_upload(
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a JSON file.
    """
    try:
        # Read and parse the file
        data = await file.read()
        genome_str = json.loads(data)

        # Add default fields if missing
        if "genome_title" not in genome_str:
            genome_str["genome_title"] = file.filename

        if "genome_description" not in genome_str:
            genome_str["genome_description"] = ""

        # Load the genome - CoreAPIService will handle state updates internally
        result = core_api_service.load_genome(genome_str, filename=file.filename)
        
        # Update burst engine if available
        burst_engine = core_api_service.get_burst_engine()
        if burst_engine:
            burst_engine.update_with_genome()
            logger.info("Burst Engine updated with new genome", emoji1="⚡")
            
        # Return raw response for v1 compatibility
        return raw_response({
            "loaded": result, 
            "genome_counter": core_api_service.get_genome_counter()
        })
    except json.JSONDecodeError:
        logger.error("Failed to parse JSON in genome file", emoji1="❌")
        return JSONResponse(
            status_code=400,
            content=error_response(message="Invalid JSON format in genome file", error_code="INVALID_JSON")
        )
    except Exception as e:
        logger.error(f"Failed to upload genome: {str(e)}", emoji1="❌")
        return JSONResponse(
            status_code=500,
            content=error_response(message=f"Failed to upload genome: {str(e)}", error_code="GENOME_UPLOAD_ERROR")
        )


@router.get("/file_name")
async def genome_file_name(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    genome_name = core_api_service.get_genome_filename()
    return genome_name or ""


@router.post("/upload/string")
async def genome_string_upload(
    genome: dict,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Upload a genome from a JSON string/dictionary.
    
    This endpoint allows uploading a genome that's already parsed into a Python dict.
    """
    try:
        # Add defaults if not present
        if "genome_title" not in genome:
            genome["genome_title"] = "Unknown Genome"

        if "genome_description" not in genome:
            genome["genome_description"] = ""
        
        # Load the genome
        result = core_api_service.load_genome(genome)
        
        # Update burst engine if available
        burst_engine = core_api_service.get_burst_engine()
        if burst_engine:
            burst_engine.update_with_genome()
            logger.info("Burst Engine updated with new genome", emoji1="⚡")
            
        # Return the result
        return {
            "loaded": result, 
            "genome_counter": core_api_service.get_genome_counter()
        }
    except Exception as e:
        logger.error(f"Failed to upload genome string: {str(e)}", emoji1="❌")
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to upload genome string: {str(e)}"
        )


@router.get("/download")
def download_genome(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome)
):
    """Download the current genome"""
    # Get the genome from CoreAPIService
    genome = core_api_service.get_genome()
    
    # Get filename
    filename = core_api_service.get_genome_filename() or "feagi_genome.json"
    if not filename.endswith(".json"):
        filename += ".json"
        
    # Create a temporary file to store the genome
    temp_file_path = os.path.join(core_api_service.get_temp_path(), filename)
    
    # Save the genome to the file
    try:
        with open(temp_file_path, 'w') as f:
            json.dump(genome, f, indent=4)
        
        # Return the file as response
        return FileResponse(path=temp_file_path, filename=filename, media_type='application/json')
    except Exception as e:
        logger.error(f"Error downloading genome: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error downloading genome: {str(e)}")


@router.get("/download_region")
async def genome_download_from_region(
    region_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome)
):
    """
    Download a genome from a specific brain region
    
    Args:
        region_id: ID of the brain region
    """
    try:
        # Get region title and genome payload
        genome = core_api_service.get_genome_from_region(region_id)
        if not genome:
            raise HTTPException(status_code=404, detail=f"Region {region_id} not found")
        
        region_title = core_api_service.get_region_title(region_id) or region_id
        
        # Create file path
        temp_dir = core_api_service.get_temp_path()
        genome_path = os.path.join(temp_dir, f"genome_{region_title}.json")
        
        # Save genome to file
        with open(genome_path, 'w') as f:
            json.dump(genome, f, indent=4)
        
        # Create download filename
        file_name = f"genome-{region_title}".replace(" ", "_") + ".json"
        
        return FileResponse(path=genome_path, media_type="application/json", filename=file_name)
    except Exception as e:
        logger.error(f"Error downloading genome from region: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error downloading genome from region: {str(e)}")


@router.post("/upload/file/edit")
async def genome_file_upload_edit(
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Upload a genome file for editing purposes
    
    This endpoint is used to upload a genome file without loading it,
    typically for preview or editing purposes.
    
    Args:
        file: The genome file to upload
    """
    try:
        data = await file.read()
        genome_str = data.decode("utf-8")
        
        # Store the genome string in a temporary file for editing
        temp_dir = core_api_service.get_temp_path()
        edit_path = os.path.join(temp_dir, f"edit_{file.filename}")
        
        with open(edit_path, 'w') as f:
            f.write(genome_str)
            
        return {"file_path": edit_path, "content": genome_str}
    except Exception as e:
        logger.error(f"Error uploading genome for editing: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error uploading genome for editing: {str(e)}")


@router.get("/defaults/files")
async def genome_default_files(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the list of default genome files with their contents
    """
    try:
        default_genomes = core_api_service.get_default_genomes()
        return {"genome": default_genomes}
    except Exception as e:
        logger.error(f"Error retrieving default genomes: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error retrieving default genomes: {str(e)}")


@router.get("/genome_number")
async def genome_number(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome)
):
    """
    Return the number associated with current Genome instance.
    """
    return core_api_service.get_genome_counter()


@router.post("/reset")
async def reset_genome(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Reset the current genome"""
    success = core_api_service.reset_genome()
    if success:
        return {"message": "Genome reset successfully"}
    else:
        raise HTTPException(status_code=500, detail="Failed to reset genome")


@router.post("/amalgamation_by_payload")
async def amalgamation_attempt(
    amalgamation_param: AmalgamationRequest, 
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_amalgamation_ready)
):
    """
    Initiate an amalgamation using a provided genome payload.
    
    The amalgamation process allows incorporating circuit patterns into the existing genome.
    """
    if not amalgamation_param.genome_payload:
        raise HTTPException(status_code=400, detail="Genome payload is required")
    
    # Generate a unique ID for this amalgamation
    amalgamation_id = f"{datetime.now().strftime('%Y%m%d%H%M%S%f')[2:]}_AMAL"
    
    # Initialize the amalgamation
    success = core_api_service.initiate_amalgamation(
        amalgamation_id=amalgamation_id,
        genome_id=amalgamation_param.genome_id or "custom_payload",
        genome_title=amalgamation_param.genome_title or "Custom Amalgamation",
        genome_payload=amalgamation_param.genome_payload
    )
    
    if not success:
        raise HTTPException(status_code=500, detail="Failed to initialize amalgamation")
    
    return {"amalgamation_id": amalgamation_id}


@router.post("/amalgamation_by_upload")
async def amalgamation_attempt(
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_amalgamation_ready)
):
    """
    Initiate an amalgamation by uploading a genome file.
    
    This endpoint allows uploading a genome file that will be used
    for amalgamation with the current genome.
    """
    try:
        # Read and parse the uploaded file
        data = await file.read()
        genome_str = json.loads(data)
        
        # Validate the genome has a blueprint
        if "blueprint" not in genome_str:
            raise HTTPException(status_code=400, detail="Missing 'blueprint' key in uploaded genome.")
            
        # Process the genome
        genome_2 = genome_2_1_convertor(genome_str["blueprint"])
        
        # Generate amalgamation ID
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        
        # Initialize the amalgamation
        success = core_api_service.initiate_amalgamation(
            amalgamation_id=amalgamation_id,
            genome_id=file.filename,
            genome_title=file.filename,
            genome_payload=genome_str
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to initialize amalgamation")
            
        return amalgamation_id
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON in uploaded file")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing amalgamation: {str(e)}")


@router.post("/amalgamation_by_filename")
async def amalgamation_attempt(
    amalgamation_param: AmalgamationRequest, 
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_amalgamation_ready)
):
    """
    Initiate an amalgamation using an existing genome file.
    
    This endpoint takes parameters specifying the genome to use
    for amalgamation with the current genome.
    """
    try:
        # Generate amalgamation ID
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        
        # Initialize the amalgamation
        success = core_api_service.initiate_amalgamation(
            amalgamation_id=amalgamation_id,
            genome_id=amalgamation_param.genome_id or "custom_genome",
            genome_title=amalgamation_param.genome_title or "Custom Amalgamation",
            genome_payload=amalgamation_param.genome_payload
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to initialize amalgamation")
            
        return amalgamation_id
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing amalgamation: {str(e)}")


@router.get("/amalgamation_history")
async def amalgamation_history(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Get the complete history of amalgamations.
    
    Returns:
        Dict mapping amalgamation IDs to their statuses
    """
    return core_api_service.get_amalgamation_history()


@router.get("/cortical_template")
async def cortical_template_(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Get available cortical templates
    """
    return core_api_service.get_cortical_templates()


@router.post("/amalgamation_destination")
async def amalgamation_conclusion(
    circuit_origin_x,
    circuit_origin_y,
    circuit_origin_z,
    amalgamation_id,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome),
    brain_region_id="root",
    rewire_mode: RewiringMode = Query(default=RewiringMode.rewire_all)
):
    """
    Complete the amalgamation process by placing the circuit at the specified coordinates.
    
    Args:
        circuit_origin_x: X coordinate for circuit placement
        circuit_origin_y: Y coordinate for circuit placement
        circuit_origin_z: Z coordinate for circuit placement
        amalgamation_id: ID of the pending amalgamation
        brain_region_id: ID of the target brain region
        rewire_mode: Mode for rewiring the circuit
    """
    # Get the state manager for access to pending amalgamation data
    state_manager = FeagiStateManager.instance()
    
    # Check if there's a pending amalgamation
    if not state_manager.pending_amalgamation or not state_manager.pending_amalgamation.get("initiation_time"):
        raise HTTPException(status_code=400, detail="No pending amalgamation request found")
    
    # Use CoreAPIService to complete the amalgamation
    result = core_api_service.complete_amalgamation(
        amalgamation_id=amalgamation_id,
        circuit_origin=[int(circuit_origin_x), int(circuit_origin_y), int(circuit_origin_z)],
        brain_region_id=brain_region_id,
        rewire_mode=rewire_mode.value
    )
    
    if result:
        genome_title = state_manager.pending_amalgamation.get("genome_title", "Unknown")
        return f"Amalgamation for \"{genome_title}\" is complete."
    else:
        raise HTTPException(status_code=500, detail="Failed to complete amalgamation")


@router.get("/amalgamation")
async def get_amalgamation(
    amalgamation_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get the status of a specific amalgamation by ID.
    
    Args:
        amalgamation_id: ID of the amalgamation to retrieve
        
    Returns:
        Status of the amalgamation
    """
    status = core_api_service.get_amalgamation_status(amalgamation_id)
    if status is None:
        raise HTTPException(status_code=404, detail=f"Amalgamation with ID {amalgamation_id} not found")
    return status


@router.delete("/amalgamation_cancellation")
async def cancel_amalgamation_request(
    amalgamation_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Cancel a pending amalgamation request.
    
    Args:
        amalgamation_id: ID of the amalgamation to cancel
    """
    success = core_api_service.cancel_amalgamation(amalgamation_id)
    if not success:
        raise HTTPException(status_code=404, detail=f"Amalgamation with ID {amalgamation_id} not found or could not be canceled")
    return {"status": "success", "message": f"Amalgamation {amalgamation_id} canceled successfully"}


@router.get("/circuits")
async def circuit_library(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the list of neuronal circuits under /evo/circuits
    """
    return core_api_service.get_circuit_library()


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
async def genome_append_circuit(
    circuit_origin_x: int,
    circuit_origin_y: int,
    circuit_origin_z: int,
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_genome_and_connectome)
):
    """
    Appends a given circuit to the running genome at a specific location.
    
    Args:
        circuit_origin_x: X coordinate for circuit placement
        circuit_origin_y: Y coordinate for circuit placement
        circuit_origin_z: Z coordinate for circuit placement
        file: Circuit genome file to append
    """
    try:
        # Read and parse the file
        data = await file.read()
        circuit_data = json.loads(data)
        
        # Use the service to append the circuit
        result = core_api_service.append_circuit(
            circuit_origin=(circuit_origin_x, circuit_origin_y, circuit_origin_z),
            circuit_data=circuit_data,
            filename=file.filename  # Pass the filename to the service
        )
        
        if not result:
            raise HTTPException(status_code=500, detail="Failed to append circuit")
            
        return {"status": "success", "message": "Circuit appended successfully"}
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON in circuit file")
    except Exception as e:
        logger.error(f"Error appending circuit: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error appending circuit: {str(e)}")


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


async def save_upload_file(file: UploadFile) -> str:
    """
    Save an uploaded file to a temporary location and return the path.
    
    Args:
        file: The uploaded file
        
    Returns:
        Path to the saved file
    """
    # Create a temporary directory if it doesn't exist
    temp_dir = tempfile.mkdtemp(prefix="feagi_upload_")
    
    # Create the file path
    file_path = os.path.join(temp_dir, file.filename)
    
    # Read and save the file
    contents = await file.read()
    
    with open(file_path, 'wb') as f:
        f.write(contents)
        
    return file_path


@router.post("/deploy", response_model=dict)
async def deploy_genome(
    genome_file: UploadFile = File(...), 
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_deployment_ready)
):
    """
    Deploy a genome file.
    
    This is a more controlled way to load a genome with proper state transitions
    and validation.
    
    Args:
        genome_file: The genome file to deploy
    """
    try:
        # Save the uploaded file
        genome_filepath = await save_upload_file(genome_file)
        
        # Deploy the genome through CoreAPIService
        # The service will handle all state transitions internally
        result = core_api_service.deploy_genome(genome_filepath)
        
        if result:
            # Update burst engine with new genome if needed
            burst_engine = core_api_service.get_burst_engine()
            if burst_engine:
                burst_engine.update_with_genome()
                logger.info("Burst Engine updated with new genome", emoji1="⚡")
            
            return success_response(
                message="Genome deployed successfully",
                data={"filename": genome_file.filename}
            )
        else:
            return JSONResponse(
                status_code=500,
                content=error_response(
                    message="Failed to deploy genome", 
                    error_code="GENOME_DEPLOY_FAILED"
                )
            )
    except Exception as e:
        logger.error(f"Error deploying genome: {str(e)}", emoji1="❌")
        
        return JSONResponse(
            status_code=500,
            content=error_response(
                message=f"Error deploying genome: {str(e)}",
                error_code="GENOME_DEPLOY_ERROR"
            )
        )
