"""Inputs API router for FEAGI REST API."""

from typing import Dict, List, Optional, Any, Union
from fastapi import APIRouter, HTTPException, Depends, Path, Query, Body, File, UploadFile
from pydantic import BaseModel, Field
import numpy as np
import base64
from io import BytesIO
import json

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Pydantic models for request/response
class InputSourceBase(BaseModel):
    """Base model for input source properties."""
    name: str
    type: str
    properties: Dict[str, Any] = Field(default={}, description="Additional properties for the input source")
    target_area_id: str = Field(..., description="ID of the target cortical area")

class InputSourceCreate(InputSourceBase):
    """Request model for creating an input source."""
    pass

class InputSourceUpdate(BaseModel):
    """Request model for updating an input source."""
    name: Optional[str] = None
    type: Optional[str] = None
    properties: Optional[Dict[str, Any]] = None
    target_area_id: Optional[str] = None

class InputSourceResponse(InputSourceBase):
    """Response model for input source information."""
    id: str

class InputSourceList(BaseModel):
    """Response model for list of input sources."""
    sources: List[InputSourceResponse]

class VectorInputData(BaseModel):
    """Request model for vector input data."""
    data: List[float] = Field(..., description="Vector of input values")

class MatrixInputData(BaseModel):
    """Request model for matrix (2D) input data."""
    data: List[List[float]] = Field(..., description="2D matrix of input values")
    
class TensorInputData(BaseModel):
    """Request model for tensor (3D) input data."""
    data: List[List[List[float]]] = Field(..., description="3D tensor of input values")

class ImageInputData(BaseModel):
    """Request model for image input data."""
    base64_data: str = Field(..., description="Base64 encoded image data")
    format: str = Field("RGB", description="Image format (RGB, RGBA, grayscale)")

class StimulusResponse(BaseModel):
    """Response model for stimulus application."""
    success: bool
    message: str

# Stimulus models
class CorticalStimulationRequest(BaseModel):
    """Request model for stimulating a cortical area."""
    pattern: str = Field("random", description="Stimulus pattern (random, specific, etc.)")
    intensity: float = Field(1.0, description="Stimulus intensity (0.0-1.0)")
    duration: int = Field(1, description="Stimulus duration in bursts")
    coordinates: Optional[List[Dict[str, int]]] = Field(None, description="Specific coordinates to stimulate")

# Create router
router = APIRouter(prefix="/inputs", tags=["inputs"])

# Input Source Endpoints
@router.get("/sources", response_model=InputSourceList)
async def get_all_input_sources(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get all input sources.
    
    Returns a list of all input sources configured in the system.
    """
    try:
        # Get input sources from the core API
        sources = core_api.get_input_sources()
        return {"sources": sources}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving input sources: {str(e)}")

@router.get("/sources/{source_id}", response_model=InputSourceResponse)
async def get_input_source(
    source_id: str = Path(..., description="ID of the input source"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific input source by ID.
    
    Args:
        source_id: ID of the input source to retrieve.
    
    Returns:
        Detailed information about the specified input source.
    """
    try:
        # Get the input source from the core API
        source = core_api.get_input_source(source_id)
        if not source:
            raise HTTPException(status_code=404, detail=f"Input source {source_id} not found")
        return source
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving input source {source_id}: {str(e)}")

@router.post("/sources", response_model=InputSourceResponse)
async def create_input_source(
    source: InputSourceCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new input source.
    
    Args:
        source: Details of the input source to create.
    
    Returns:
        Information about the newly created input source.
    """
    try:
        # Validate that the target cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if source.target_area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Target cortical area {source.target_area_id} not found")
        
        # Validate the input source type
        valid_types = ["vector", "matrix", "tensor", "image", "audio", "text", "custom"]
        if source.type not in valid_types:
            raise HTTPException(
                status_code=400, 
                detail=f"Invalid input source type. Must be one of: {', '.join(valid_types)}"
            )
        
        # Create the input source via the core API
        source_data = {
            "name": source.name,
            "type": source.type,
            "properties": source.properties,
            "target_area_id": source.target_area_id
        }
        source_id = core_api.register_input_source(source_data)
        
        # Get the newly created source
        new_source = core_api.get_input_source(source_id)
        if not new_source:
            raise HTTPException(status_code=500, detail="Failed to retrieve newly created input source")
        
        return new_source
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating input source: {str(e)}")

@router.put("/sources/{source_id}", response_model=InputSourceResponse)
async def update_input_source(
    source_id: str = Path(..., description="ID of the input source"),
    source_update: InputSourceUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing input source.
    
    Args:
        source_id: ID of the input source to update.
        source_update: Updated details for the input source.
    
    Returns:
        Information about the updated input source.
    """
    try:
        # Check if the input source exists
        existing_source = core_api.get_input_source(source_id)
        if not existing_source:
            raise HTTPException(status_code=404, detail=f"Input source {source_id} not found")
        
        # Update the input source
        source_data = {}
        if source_update.name is not None:
            source_data["name"] = source_update.name
        if source_update.type is not None:
            source_data["type"] = source_update.type
        if source_update.properties is not None:
            source_data["properties"] = source_update.properties
        if source_update.target_area_id is not None:
            source_data["target_area_id"] = source_update.target_area_id
        
        success = core_api.update_input_source(source_id, source_data)
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to update input source {source_id}")
        
        # Return success message
        return {
            "message": f"Input source {source_id} updated successfully"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating input source {source_id}: {str(e)}")

@router.delete("/sources/{source_id}")
async def delete_input_source(
    source_id: str = Path(..., description="ID of the input source"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete an input source.
    
    Args:
        source_id: ID of the input source to delete.
    
    Returns:
        Confirmation message.
    """
    try:
        # Check if the input source exists
        existing_source = core_api.get_input_source(source_id)
        if not existing_source:
            raise HTTPException(status_code=404, detail=f"Input source {source_id} not found")
        
        # Delete the input source
        success = core_api.remove_input_source(source_id)
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to delete input source {source_id}")
        
        # Return success message
        return {
            "message": f"Input source {source_id} removed successfully"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting input source {source_id}: {str(e)}")

# Direct Input Stimulation Endpoints
@router.post("/stimulate/vector/{area_id}", response_model=StimulusResponse)
async def stimulate_with_vector(
    area_id: str = Path(..., description="ID of the target cortical area"),
    input_data: VectorInputData = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with vector data.
    
    Args:
        area_id: ID of the target cortical area.
        input_data: Vector data for stimulation.
    
    Returns:
        Confirmation of stimulus application.
    """
    try:
        # Validate that the cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # Convert the input data to the required format for the cortical area stimulation
        stimulation_pattern = {
            "type": "vector",
            "data": input_data.data
        }
        
        # Stimulate the cortical area
        success = core_api.stimulate_cortical_area(area_id, stimulation_pattern)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to stimulate cortical area {area_id}")
        
        return {
            "success": True,
            "message": f"Successfully stimulated cortical area {area_id} with vector data"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}")

@router.post("/stimulate/matrix/{area_id}", response_model=StimulusResponse)
async def stimulate_with_matrix(
    area_id: str = Path(..., description="ID of the target cortical area"),
    input_data: MatrixInputData = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with matrix (2D) data.
    
    Args:
        area_id: ID of the target cortical area.
        input_data: Matrix data for stimulation.
    
    Returns:
        Confirmation of stimulus application.
    """
    try:
        # Validate that the cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # Convert the input data to the required format for the cortical area stimulation
        stimulation_pattern = {
            "type": "matrix",
            "data": input_data.data
        }
        
        # Stimulate the cortical area
        success = core_api.stimulate_cortical_area(area_id, stimulation_pattern)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to stimulate cortical area {area_id}")
        
        return {
            "success": True,
            "message": f"Successfully stimulated cortical area {area_id} with matrix data"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}")

@router.post("/stimulate/tensor/{area_id}", response_model=StimulusResponse)
async def stimulate_with_tensor(
    area_id: str = Path(..., description="ID of the target cortical area"),
    input_data: TensorInputData = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with tensor (3D) data.
    
    Args:
        area_id: ID of the target cortical area.
        input_data: Tensor data for stimulation.
    
    Returns:
        Confirmation of stimulus application.
    """
    try:
        # Validate that the cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # Convert the input data to the required format for the cortical area stimulation
        stimulation_pattern = {
            "type": "tensor",
            "data": input_data.data
        }
        
        # Stimulate the cortical area
        success = core_api.stimulate_cortical_area(area_id, stimulation_pattern)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to stimulate cortical area {area_id}")
        
        return {
            "success": True,
            "message": f"Successfully stimulated cortical area {area_id} with tensor data"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}")

@router.post("/stimulate/image/{area_id}", response_model=StimulusResponse)
async def stimulate_with_image(
    area_id: str = Path(..., description="ID of the target cortical area"),
    input_data: ImageInputData = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with image data.
    
    Args:
        area_id: ID of the target cortical area.
        input_data: Image data (base64 encoded) for stimulation.
    
    Returns:
        Confirmation of stimulus application.
    """
    try:
        # Validate that the cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # In a real implementation, this would decode the base64 image and process it
        # For demonstration purposes, we'll just pass it through
        
        # Convert the input data to the required format for the cortical area stimulation
        stimulation_pattern = {
            "type": "image",
            "format": input_data.format,
            "data": input_data.base64_data
        }
        
        # Stimulate the cortical area
        success = core_api.stimulate_cortical_area(area_id, stimulation_pattern)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to stimulate cortical area {area_id}")
        
        return {
            "success": True,
            "message": f"Successfully stimulated cortical area {area_id} with image data"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}")

@router.post("/stimulate/custom/{area_id}", response_model=StimulusResponse)
async def stimulate_with_custom_data(
    area_id: str = Path(..., description="ID of the target cortical area"),
    input_data: Dict[str, Any] = Body(..., description="Custom input data"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with custom data.
    
    This endpoint allows for sending custom-formatted data directly to a cortical area.
    The structure of the input data is determined by the requirements of the specific cortical area.
    
    Args:
        area_id: ID of the target cortical area.
        input_data: Custom-structured data for stimulation.
    
    Returns:
        Confirmation of stimulus application.
    """
    try:
        # Validate that the cortical area exists
        areas = core_api.get_cortical_areas()
        area_ids = [area["id"] for area in areas]
        
        if area_id not in area_ids:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # Stimulate the cortical area with the custom data
        success = core_api.stimulate_cortical_area(area_id, input_data)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to stimulate cortical area {area_id}")
        
        return {
            "success": True,
            "message": f"Successfully stimulated cortical area {area_id} with custom data"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}")

# Add the stimulate_area endpoint
@router.post("/stimulate_area/{area_id}")
async def stimulate_area(
    area_id: str = Path(..., description="ID of the cortical area to stimulate"),
    stimulation: CorticalStimulationRequest = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with the specified pattern.
    
    Args:
        area_id: ID of the cortical area to stimulate.
        stimulation: Stimulation parameters.
    
    Returns:
        Information about the applied stimulation.
    """
    try:
        # Check if the cortical area exists
        area = core_api.get_cortical_area(area_id)
        if not area:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        
        # Apply stimulation
        # In a real implementation, this would apply the stimulation to the cortical area
        # via the core API. Here we just return a placeholder response.
        return {
            "stimulated_neurons": 100,
            "timestamp": 123456789
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}") 