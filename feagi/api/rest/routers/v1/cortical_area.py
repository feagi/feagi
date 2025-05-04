"""Cortical Area API endpoints for FEAGI REST API."""

import json
from typing import Dict, List, Optional, Any, Union

from fastapi import APIRouter, HTTPException, Depends, Path, Query, Body
from pydantic import BaseModel, Field

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Pydantic models for request/response
class CorticalAreaBase(BaseModel):
    """Base model for cortical area properties."""
    name: str
    coordinates: Dict[str, int] = Field(..., description="3D coordinates of the cortical area")
    dimensions: Dict[str, int] = Field(..., description="Dimensions of the cortical area")
    type: str
    parameters: Dict[str, Any] = Field(default={}, description="Additional parameters for the cortical area")

class CorticalAreaCreate(CorticalAreaBase):
    """Request model for creating a cortical area."""
    pass

class CorticalAreaUpdate(BaseModel):
    """Request model for updating a cortical area."""
    name: Optional[str] = None
    coordinates: Optional[Dict[str, int]] = None
    dimensions: Optional[Dict[str, int]] = None
    type: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None

class CorticalAreaResponse(CorticalAreaBase):
    """Response model for cortical area information."""
    id: str
    neuron_count: int = 0

class CorticalAreaList(BaseModel):
    """Response model for list of cortical areas."""
    areas: List[CorticalAreaResponse]

# Create router
router = APIRouter(prefix="/cortical_area", tags=["cortical_area"])

# Cortical Area Endpoints
@router.get("/", response_model=CorticalAreaList)
async def get_all_cortical_areas(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get all cortical areas.
    
    Returns a list of all cortical areas in the current brain.
    """
    try:
        areas = core_api.get_cortical_areas()
        return {"areas": areas}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical areas: {str(e)}")

@router.get("/{area_id}", response_model=CorticalAreaResponse)
async def get_cortical_area(
    area_id: str = Path(..., description="String representation of the cortical_idx"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a specific cortical area by ID.
    
    Args:
        area_id: String representation of the cortical_idx.
    
    Returns:
        Detailed information about the specified cortical area.
    """
    try:
        area = core_api.get_cortical_area(area_id)
        if not area:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return area
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical area {area_id}: {str(e)}")

@router.post("/", response_model=CorticalAreaResponse)
async def create_cortical_area(
    area: CorticalAreaCreate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Create a new cortical area.
    
    Args:
        area: Details of the cortical area to create.
    
    Returns:
        Information about the newly created cortical area.
    """
    try:
        new_area = core_api.create_cortical_area(
            name=area.name,
            coordinates=area.coordinates,
            dimensions=area.dimensions,
            area_type=area.type,
            parameters=area.parameters
        )
        if not new_area:
            raise HTTPException(status_code=500, detail="Failed to create cortical area")
        return new_area
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating cortical area: {str(e)}")

@router.put("/{area_id}", response_model=CorticalAreaResponse)
async def update_cortical_area(
    area_id: str = Path(..., description="String representation of the cortical_idx to update"),
    area: CorticalAreaUpdate = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update an existing cortical area.
    
    Args:
        area_id: String representation of the cortical_idx to update.
        area: Updated cortical area details.
    
    Returns:
        The updated cortical area information.
    """
    try:
        updated_area = core_api.update_cortical_area(
            area_id=area_id,
            name=area.name,
            coordinates=area.coordinates,
            dimensions=area.dimensions,
            area_type=area.type,
            parameters=area.parameters
        )
        if not updated_area:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return updated_area
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating cortical area {area_id}: {str(e)}")

@router.delete("/{area_id}", status_code=204)
async def delete_cortical_area(
    area_id: str = Path(..., description="String representation of the cortical_idx to delete"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Delete a cortical area.
    
    Args:
        area_id: String representation of the cortical_idx to delete.
    """
    try:
        success = core_api.delete_cortical_area(area_id)
        if not success:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting cortical area {area_id}: {str(e)}")

@router.get("/{area_id}/neurons")
async def get_cortical_area_neurons(
    area_id: str = Path(..., description="ID of the cortical area"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get neuron information for a specific cortical area.
    
    Args:
        area_id: ID of the cortical area.
    
    Returns:
        Information about the neurons in the specified cortical area.
    """
    try:
        neurons = core_api.get_cortical_area_neurons(area_id)
        if neurons is None:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return {"area_id": area_id, "neurons": neurons}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving neurons for cortical area {area_id}: {str(e)}")

@router.get("/{area_id}/activity")
async def get_cortical_area_activity(
    area_id: str = Path(..., description="ID of the cortical area"),
    window: int = Query(1, description="Time window for activity data (in bursts)"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get neural activity data for a specific cortical area.
    
    Args:
        area_id: ID of the cortical area.
        window: Time window for activity data (in bursts).
    
    Returns:
        Activity data for the specified cortical area.
    """
    try:
        activity = core_api.get_cortical_area_activity(area_id, window)
        if activity is None:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return {"area_id": area_id, "window": window, "activity": activity}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving activity for cortical area {area_id}: {str(e)}")

@router.get("/{area_id}/connectivity")
async def get_cortical_area_connectivity(
    area_id: str = Path(..., description="ID of the cortical area"),
    direction: str = Query("both", description="Connection direction: 'incoming', 'outgoing', or 'both'"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get connectivity information for a specific cortical area.
    
    Args:
        area_id: ID of the cortical area.
        direction: Connection direction ('incoming', 'outgoing', or 'both').
    
    Returns:
        Connectivity information for the specified cortical area.
    """
    if direction not in ["incoming", "outgoing", "both"]:
        raise HTTPException(status_code=400, detail="Direction must be 'incoming', 'outgoing', or 'both'")
    
    try:
        connectivity = core_api.get_cortical_area_connectivity(area_id, direction)
        if connectivity is None:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return {"area_id": area_id, "direction": direction, "connectivity": connectivity}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving connectivity for cortical area {area_id}: {str(e)}")

@router.get("/types")
async def get_cortical_area_types(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get available cortical area types.
    
    Returns:
        List of available cortical area types and their descriptions.
    """
    try:
        types = core_api.get_cortical_area_types()
        return {"types": types}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical area types: {str(e)}")

@router.post("/{area_id}/stimulate")
async def stimulate_cortical_area(
    area_id: str = Path(..., description="ID of the cortical area"),
    pattern: Dict[str, Any] = Body(..., description="Stimulation pattern"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Stimulate a cortical area with a specific pattern.
    
    Args:
        area_id: ID of the cortical area to stimulate.
        pattern: Stimulation pattern data.
    
    Returns:
        Confirmation message.
    """
    try:
        success = core_api.stimulate_cortical_area(area_id, pattern)
        if not success:
            raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
        return {"message": f"Cortical area {area_id} stimulated successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stimulating cortical area {area_id}: {str(e)}") 