"""Version 1 API router for FEAGI REST API."""

from typing import Dict, List, Any, Optional
from fastapi import APIRouter, HTTPException, Depends, Path
from pydantic import BaseModel

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Models for request/response
class CorticalAreaResponse(BaseModel):
    """Response model for cortical area information."""
    id: str
    name: str
    dimensions: Dict[str, int]
    position: Dict[str, int]
    type: str
    properties: Dict[str, Any]

class SimulationStatusResponse(BaseModel):
    """Response model for simulation status."""
    running: bool
    burst_count: int
    uptime: float
    performance: Dict[str, float]

class ConfigurationUpdateRequest(BaseModel):
    """Request model for configuration updates."""
    parameters: Dict[str, Any]

# Create router
router = APIRouter(tags=["v1"])

# Brain state endpoints
@router.get("/brain")
async def get_brain_state(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current brain state."""
    brain_state = core_api.get_brain_state()
    return {"brain_state": brain_state}

@router.post("/brain/save")
async def save_brain_state(path: str, core_api: CoreAPIService = Depends(get_core_api)):
    """Save the current brain state to a file."""
    success = core_api.save_brain_state(path)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to save brain state")
    return {"message": "Brain state saved successfully"}

@router.post("/brain/load")
async def load_brain_state(path: str, core_api: CoreAPIService = Depends(get_core_api)):
    """Load a brain state from a file."""
    success = core_api.load_brain_state(path)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to load brain state")
    return {"message": "Brain state loaded successfully"}

# Cortical area endpoints
@router.get("/cortical-areas", response_model=List[CorticalAreaResponse])
async def get_cortical_areas(core_api: CoreAPIService = Depends(get_core_api)):
    """Get all cortical areas."""
    areas = core_api.get_cortical_areas()
    return areas

@router.get("/cortical-areas/{area_id}", response_model=CorticalAreaResponse)
async def get_cortical_area(
    area_id: str = Path(..., description="ID of the cortical area"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Get a cortical area by ID."""
    area = core_api.get_cortical_area(area_id)
    if not area:
        raise HTTPException(status_code=404, detail=f"Cortical area {area_id} not found")
    return area

# Simulation control endpoints
@router.post("/simulation/start")
async def start_simulation(core_api: CoreAPIService = Depends(get_core_api)):
    """Start the simulation."""
    success = core_api.start_simulation()
    if not success:
        raise HTTPException(status_code=500, detail="Failed to start simulation")
    return {"message": "Simulation started successfully"}

@router.post("/simulation/stop")
async def stop_simulation(core_api: CoreAPIService = Depends(get_core_api)):
    """Stop the simulation."""
    success = core_api.stop_simulation()
    if not success:
        raise HTTPException(status_code=500, detail="Failed to stop simulation")
    return {"message": "Simulation stopped successfully"}

@router.get("/simulation/status", response_model=SimulationStatusResponse)
async def get_simulation_status(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current simulation status."""
    status = core_api.get_simulation_status()
    return status

# Configuration endpoints
@router.get("/configuration")
async def get_configuration(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current configuration."""
    config = core_api.get_configuration()
    return {"configuration": config}

@router.put("/configuration")
async def update_configuration(
    config_update: ConfigurationUpdateRequest,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Update the configuration."""
    success = core_api.update_configuration(config_update.parameters)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update configuration")
    return {"message": "Configuration updated successfully"} 