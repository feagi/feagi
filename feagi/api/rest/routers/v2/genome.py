from fastapi import APIRouter, File, UploadFile, HTTPException, Depends
from fastapi.responses import JSONResponse
import os
import json
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.evo.genome_processor import process_and_load_genome
from feagi.core.state_manager import FeagiStateManager, ConnectomeState


router = APIRouter()
state = FeagiStateManager.instance()

@router.post("/upload/essential")
async def genome_default_upload_v2(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Upload the essential genome template (v2 API with standardized response)"""
    essential_path = os.path.join(os.path.dirname(__file__), "../../../../../feagi/evo/defaults/genome/essential_genome.json")
    
    if not os.path.exists(essential_path):
        raise HTTPException(status_code=404, detail="Essential genome template not found!")
        
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
    
    # Return result directly - middleware will standardize it
    return result

@router.post("/upload/file")
async def genome_file_upload_v2(file: UploadFile = File(...)):
    """Upload a genome file to FEAGI (v2 API with standardized response)"""
    core_api_service = get_core_api_service()
    
    data = await file.read()
    state.genome_file_name = file.filename
    
    try:
        genome_data = json.loads(data)
        
        if "genome_title" not in genome_data:
            genome_data["genome_title"] = file.filename
            
        if "genome_description" not in genome_data:
            genome_data["genome_description"] = ""
            
        # Set connectome to initializing state
        state.set_connectome_state(ConnectomeState.INITIALIZING)
        
        # Process and load genome
        result = process_and_load_genome(genome_data, core_api_service)
        
        # Update burst engine
        burst_engine = core_api_service.get_burst_engine()
        if burst_engine and result["success"]:
            burst_engine.update_with_genome()
            logger.info("Burst Engine updated with new genome", emoji1="⚡")
            
        # Return result directly - middleware will standardize it
        return result
        
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON in genome file")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to upload genome: {str(e)}") 