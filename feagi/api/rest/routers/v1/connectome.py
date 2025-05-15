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
from fastapi import APIRouter, File, UploadFile, Depends, HTTPException, Body
from fastapi import HTTPException
from starlette.responses import FileResponse
from pydantic import BaseModel

from ast import literal_eval
from threading import Thread
from typing import List, Dict, Any, Tuple, Optional
from datetime import datetime

from ...commons import *
from feagi.core.state_manager import FeagiStateManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger
logger = setup_logger()
from feagi.api.rest.dependencies import get_core_api_service


router = APIRouter()

# Helper to get state manager instance
state = FeagiStateManager.instance()

# Models for batch operations
class BatchNeuronCreationRequest(BaseModel):
    area_id: str
    positions: List[Tuple[int, int, int]]
    properties: Optional[Dict[str, Any]] = None

class BatchSynapseCreationRequest(BaseModel):
    connections: List[Tuple[int, int, float]]

# ######  Connectome Endpoints #########
# ######################################
@router.get("/cortical_areas/list/summary")
async def connectome_cortical_areas_summary(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get a summary list of cortical area IDs"""
    areas = core_api_service.get_cortical_areas()
    if not areas:
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")
    return [area["id"] for area in areas]


@router.get("/cortical_areas/list/transforming")
async def transforming_cortical_areas_summary(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get a list of transforming cortical areas"""
    return core_api_service.get_transforming_areas()


@router.get("/cortical_areas/list/detailed")
async def connectome_cortical_areas(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get detailed information about all cortical areas"""
    areas = core_api_service.get_cortical_areas()
    if not areas:
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")
    return areas


@router.get("/cortical_info")
async def connectome_cortical_info(cortical_area: str, core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get detailed information about a specific cortical area"""
    area = core_api_service.get_cortical_area(cortical_area)
    if not area:
        raise HTTPException(status_code=400, detail="Requested cortical area not found!")
    return area


@router.get("/plasticity")
async def connectome_plasticity_info(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get neuroplasticity information"""
    return core_api_service.get_plasticity_info()


@router.get("/path")
async def connectome_system_path(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get connectome file system path"""
    # Use the CoreAPIService method instead of direct state access
    return core_api_service.get_temp_path()


@router.post("/snapshot")
async def connectome_snapshot(connectome_storage_path: str, core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Create a snapshot of the current connectome"""
    success = core_api_service.save_connectome_snapshot(connectome_storage_path)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to create connectome snapshot")
    return {"message": "Connectome snapshot saved successfully", "path": connectome_storage_path}


@router.get("/download-cortical-area")
async def connectome_download(cortical_area: str, core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Download a specific cortical area as a JSON file"""
    area = core_api_service.get_cortical_area(cortical_area)
    if not area:
        raise HTTPException(status_code=400, detail="Requested cortical area not found!")
        
    # Generate a filename with timestamp
    file_name = f"connectome_{cortical_area}_{datetime.now().strftime('%Y_%m_%d-%I:%M:%S_%p')}.json"
    logger.info(file_name)
    
    # Create a temporary file to store the cortical area data
    temp_dir = core_api_service.get_temp_path()
    temp_file_path = f"{temp_dir}/{file_name}"
    
    try:
        # Serialize the cortical area to the temp file
        with open(temp_file_path, 'w') as f:
            import json
            json.dump(area, f, indent=2)
            
        # Return the file as a download
        return FileResponse(path=temp_file_path, filename=file_name, media_type='application/json')
    except Exception as e:
        logger.error(f"Error creating cortical area download: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error creating download: {str(e)}")


@router.post("/upload-cortical-area")
async def connectome_file_upload(
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Upload a cortical area JSON file"""
    try:
        data = await file.read()
        connectome_str = data.decode("utf-8").split(" = ")[1]
        cortical_area_data = literal_eval(connectome_str)
        
        success = core_api_service.import_cortical_area(cortical_area_data)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to import cortical area")
            
        return {"message": "Cortical area imported successfully"}
    except Exception as e:
        logger.error(f"Error importing cortical area: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error importing cortical area: {str(e)}")


@router.get("/properties/dimensions")
async def connectome_dimensions_report(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """Get the overall dimensions of the connectome"""
    return core_api_service.get_connectome_dimensions()


@router.get("/stats/cortical/cumulative")
async def connectome_stats_report(
    cortical_area: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Get cumulative statistics for a specific cortical area"""
    stats = core_api_service.get_cortical_area_stats(cortical_area)
    if not stats:
        raise HTTPException(status_code=404, detail=f"Statistics for cortical area {cortical_area} not found")
    return stats


@router.get("/properties/mappings")
async def connectome_mapping_report(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Report result can be used with the following tool to visualize the connectome mapping:
    https://csacademy.com/app/graph_editor/
    """
    try:
        return core_api_service.get_neuron_mappings()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve neuron mappings: {str(e)}")


@router.get("/download")
async def download_connectome(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Creates a compressed file containing the entire brain data
    """
    # TODO: Implement connectome serialization in CoreAPIService
    raise NotImplementedError("Connectome download is not yet implemented.")


@router.post("/upload")
async def upload_connectome(
    file: UploadFile = File(...),
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Upload a complete connectome file"""
    # TODO: Implement connectome upload/restore in CoreAPIService
    state.set_connectome_state('not_ready')
    raise NotImplementedError("Connectome upload is not yet implemented.")


# Batch operations endpoints
@router.post("/neurons/batch")
async def batch_create_neurons(
    request: BatchNeuronCreationRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Create multiple neurons in a batch operation.
    
    Args:
        request: BatchNeuronCreationRequest containing area_id, positions, and optional properties
        
    Returns:
        List of created neuron IDs
    """
    neuron_ids = core_api_service.batch_create_neurons(
        area_id=request.area_id,
        positions=request.positions,
        properties=request.properties
    )
    
    if not neuron_ids:
        raise HTTPException(status_code=500, detail="Failed to create neurons")
        
    return {"created_neurons": neuron_ids, "count": len(neuron_ids)}


@router.post("/synapses/batch")
async def batch_create_synapses(
    request: BatchSynapseCreationRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Create multiple synapses in a batch operation.
    
    Args:
        request: BatchSynapseCreationRequest containing connections (pre_id, post_id, weight)
        
    Returns:
        Number of created synapses
    """
    success_count = core_api_service.batch_create_synapses(request.connections)
    
    return {"created_synapses": success_count}
