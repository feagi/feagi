"""Genome API endpoints for FEAGI REST API."""

import os
import json
from typing import Dict, List, Optional, Any, Union
from enum import Enum

from fastapi import APIRouter, UploadFile, File, HTTPException, Depends, Query
from starlette.responses import FileResponse
from pydantic import BaseModel
from datetime import datetime
from time import time

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Pydantic models for request/response
class AmalgamationRequest(BaseModel):
    """Request model for amalgamation."""
    genome_id: str
    genome_title: str
    genome_payload: Dict[str, Any]

class RewiringMode(str, Enum):
    """Enum for rewiring modes during amalgamation."""
    rewire_all = "rewire_all"
    rewire_sources = "rewire_sources"
    rewire_destinations = "rewire_destinations"
    no_rewiring = "no_rewiring"

# Create router
router = APIRouter(prefix="/genome", tags=["genome"])

# Helper functions
def pending_amalgamation() -> bool:
    """Check if there's a pending amalgamation request."""
    # This will need to interface with the FEAGI core to check for pending amalgamations
    core_api = get_core_api()
    return core_api.has_pending_amalgamation()

# Genome Upload Endpoints
@router.post("/upload/barebones")
async def upload_barebones_genome(core_api: CoreAPIService = Depends(get_core_api)):
    """Upload a barebones genome."""
    try:
        # The path below should be configured based on FEAGI 2.1 structure
        barebones_path = os.path.join(core_api.get_data_path(), "defaults", "genome", "barebones_genome.json")
        
        if not os.path.exists(barebones_path):
            raise HTTPException(status_code=404, detail="Barebones genome file not found")
            
        with open(barebones_path, "r") as genome_file:
            genome_data = json.load(genome_file)
        
        success = core_api.load_genome(genome_data, filename="barebones_genome.json")
        if not success:
            raise HTTPException(status_code=500, detail="Failed to load barebones genome")
            
        return {"message": "Barebones genome loaded successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error loading barebones genome: {str(e)}")

@router.post("/upload/essential")
async def genome_default_upload(core_api: CoreAPIService = Depends(get_core_api)):
    """Upload the essential genome."""
    try:
        # The path below should be configured based on FEAGI 2.1 structure
        essential_path = os.path.join(core_api.get_data_path(), "defaults", "genome", "essential_genome.json")
        
        if not os.path.exists(essential_path):
            raise HTTPException(status_code=404, detail="Essential genome file not found")
            
        with open(essential_path, "r") as genome_file:
            genome_data = json.load(genome_file)
        
        success = core_api.load_genome(genome_data, filename="essential_genome.json")
        if not success:
            raise HTTPException(status_code=500, detail="Failed to load essential genome")
            
        return {"message": "Essential genome loaded successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error loading essential genome: {str(e)}")

@router.post("/upload/file")
async def genome_file_upload(
    file: UploadFile = File(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Upload a genome file.
    
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a JSON file.
    """
    try:
        data = await file.read()
        genome_str = json.loads(data)
        
        # Set default values if not present
        if "genome_title" not in genome_str:
            genome_str["genome_title"] = file.filename
            
        if "genome_description" not in genome_str:
            genome_str["genome_description"] = ""
        
        success = core_api.load_genome(genome_str, filename=file.filename)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to load genome from file")
            
        return {"message": f"Genome {file.filename} loaded successfully"}
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON format in genome file")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error loading genome file: {str(e)}")

@router.get("/file_name")
async def genome_file_name(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Returns the name of the genome file last uploaded to FEAGI.
    """
    try:
        filename = core_api.get_genome_filename()
        return filename or ""
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving genome filename: {str(e)}")

@router.post("/upload/string")
async def genome_string_upload(
    genome: Dict[str, Any],
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Upload a genome provided as a JSON structure."""
    try:
        # Set default values if not present
        if "genome_title" not in genome:
            genome["genome_title"] = "Unknown Genome"
            
        if "genome_description" not in genome:
            genome["genome_description"] = ""
        
        success = core_api.load_genome(genome, filename=genome.get("genome_title", "Unknown Genome"))
        if not success:
            raise HTTPException(status_code=500, detail="Failed to load genome from string")
            
        return {"message": "Genome loaded successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error loading genome: {str(e)}")

@router.post("/upload/file/edit")
async def genome_file_upload_edit(file: UploadFile = File(...)):
    """Upload a genome file for editing."""
    try:
        data = await file.read()
        genome_str = data.decode("utf-8")
        return {genome_str}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing genome file: {str(e)}")

# Genome Download Endpoints
@router.get("/download")
async def genome_download(core_api: CoreAPIService = Depends(get_core_api)):
    """Download the current genome."""
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No running genome found!")
        
        # Save genome to a temporary file
        file_name = f"genome-{genome.get('genome_title', 'unknown').replace(' ', '_')}.json"
        file_path = os.path.join(core_api.get_temp_path(), file_name)
        
        with open(file_path, "w") as f:
            json.dump(genome, f)
        
        # Send file as response
        headers = {"Content-Disposition": f"attachment; filename={file_name}"}
        return FileResponse(
            path=file_path,
            media_type="application/json",
            filename=file_name,
            headers=headers
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error downloading genome: {str(e)}")

@router.get("/download_region")
async def genome_download_from_region(
    region_id: str,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Download a genome from a specific brain region."""
    try:
        # Get region title from ID
        region_title = core_api.get_region_title(region_id)
        if not region_title:
            raise HTTPException(status_code=404, detail=f"Region with ID {region_id} not found")
        
        # Get genome payload for the region
        genome_payload = core_api.get_genome_from_region(region_id)
        if not genome_payload:
            raise HTTPException(status_code=404, detail=f"Could not create genome from region {region_id}")
        
        # Save genome to a temporary file
        file_name = f"genome-{region_title.replace(' ', '_')}.json"
        file_path = os.path.join(core_api.get_temp_path(), file_name)
        
        with open(file_path, "w") as f:
            json.dump(genome_payload, f)
        
        # Send file as response
        return FileResponse(
            path=file_path,
            media_type="application/json",
            filename=file_name
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error downloading genome from region: {str(e)}")

# Genome Management Endpoints
@router.get("/defaults/files")
async def genome_default_files(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the list of default genome files."""
    try:
        default_genomes_path = os.path.join(core_api.get_data_path(), "defaults", "genome")
        
        if not os.path.exists(default_genomes_path):
            raise HTTPException(status_code=404, detail="Default genomes directory not found")
            
        default_genomes = os.listdir(default_genomes_path)
        genome_mappings = {}
        
        for genome in default_genomes:
            if genome[:2] != '__' and genome.endswith('.json'):
                with open(os.path.join(default_genomes_path, genome)) as file:
                    data = file.read()
                    genome_mappings[genome.split(".")[0]] = json.loads(data)
                    
        return {"genome": genome_mappings}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving default genomes: {str(e)}")

@router.get("/genome_number")
async def genome_number(core_api: CoreAPIService = Depends(get_core_api)):
    """Return the number associated with current Genome instance."""
    try:
        return core_api.get_genome_counter()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving genome number: {str(e)}")

@router.post("/reset")
async def reset_genome(core_api: CoreAPIService = Depends(get_core_api)):
    """Reset the current genome."""
    try:
        success = core_api.reset_genome()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to reset genome")
        return {"message": "Genome reset successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error resetting genome: {str(e)}")

# Amalgamation Endpoints
@router.post("/amalgamation_by_payload")
async def amalgamation_attempt(
    amalgamation_param: AmalgamationRequest,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Initiate amalgamation using a genome payload."""
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    
    try:
        # Convert genome format if needed
        genome = amalgamation_param.genome_payload
        
        # Generate amalgamation ID
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        
        # Initiate amalgamation
        success = core_api.initiate_amalgamation(
            amalgamation_id=amalgamation_id,
            genome_id=amalgamation_param.genome_id,
            genome_title=amalgamation_param.genome_title,
            genome_payload=genome
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to initiate amalgamation")
            
        return amalgamation_id
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error initiating amalgamation: {str(e)}")

@router.post("/amalgamation_by_upload")
async def amalgamation_attempt(
    file: UploadFile = File(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Initiate amalgamation using an uploaded genome file."""
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    
    try:
        # Read the uploaded file
        data = await file.read()
        genome_str = json.loads(data)
        
        # Generate amalgamation ID
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        
        # Initiate amalgamation
        success = core_api.initiate_amalgamation(
            amalgamation_id=amalgamation_id,
            genome_id=file.filename,
            genome_title=file.filename,
            genome_payload=genome_str
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to initiate amalgamation")
            
        return amalgamation_id
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON format in genome file")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error initiating amalgamation: {str(e)}")

@router.post("/amalgamation_by_filename")
async def amalgamation_attempt(
    amalgamation_param: AmalgamationRequest,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Initiate amalgamation using a genome filename."""
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    
    try:
        # Generate amalgamation ID
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        
        # Initiate amalgamation
        success = core_api.initiate_amalgamation_by_filename(
            amalgamation_id=amalgamation_id,
            genome_id=amalgamation_param.genome_id,
            genome_title=amalgamation_param.genome_title
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to initiate amalgamation")
            
        return amalgamation_id
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error initiating amalgamation: {str(e)}")

@router.get("/amalgamation_history")
async def amalgamation_history(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the history of amalgamation attempts."""
    try:
        return core_api.get_amalgamation_history()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving amalgamation history: {str(e)}")

@router.get("/cortical_template")
async def cortical_template_(core_api: CoreAPIService = Depends(get_core_api)):
    """Get available cortical templates."""
    try:
        return core_api.get_cortical_templates()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical templates: {str(e)}")

@router.post("/amalgamation_destination")
async def amalgamation_conclusion(
    circuit_origin_x: int,
    circuit_origin_y: int,
    circuit_origin_z: int,
    amalgamation_id: str,
    brain_region_id: str = "root",
    rewire_mode: RewiringMode = Query(default=RewiringMode.rewire_all),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Complete an amalgamation by specifying destination parameters."""
    try:
        success = core_api.complete_amalgamation(
            amalgamation_id=amalgamation_id,
            circuit_origin=(circuit_origin_x, circuit_origin_y, circuit_origin_z),
            brain_region_id=brain_region_id,
            rewire_mode=rewire_mode
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to complete amalgamation")
            
        return {"message": "Amalgamation completed successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error completing amalgamation: {str(e)}")

@router.get("/amalgamation")
async def circuit_library(
    amalgamation_id: str,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Get information about a specific amalgamation attempt."""
    try:
        amalgamation_info = core_api.get_amalgamation_info(amalgamation_id)
        if not amalgamation_info:
            raise HTTPException(status_code=404, detail=f"Amalgamation with ID {amalgamation_id} not found")
        return amalgamation_info
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving amalgamation info: {str(e)}")

@router.delete("/amalgamation_cancellation")
async def cancel_amalgamation_request(
    amalgamation_id: str,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Cancel a pending amalgamation request."""
    try:
        success = core_api.cancel_amalgamation(amalgamation_id)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to cancel amalgamation")
        return {"message": "Amalgamation cancelled successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error cancelling amalgamation: {str(e)}")

@router.get("/circuits")
async def circuit_library(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the library of available circuits."""
    try:
        return core_api.get_circuit_library()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving circuit library: {str(e)}")

@router.post("/append-file")
async def genome_append_circuit(
    circuit_origin_x: int,
    circuit_origin_y: int,
    circuit_origin_z: int,
    file: UploadFile = File(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Append a circuit to the current genome from a file."""
    try:
        data = await file.read()
        genome_str = json.loads(data)
        
        success = core_api.append_circuit(
            circuit_origin=(circuit_origin_x, circuit_origin_y, circuit_origin_z),
            circuit_data=genome_str
        )
        
        if not success:
            raise HTTPException(status_code=500, detail="Failed to append circuit")
            
        return {"message": "Circuit appended successfully"}
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON format in circuit file")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error appending circuit: {str(e)}") 