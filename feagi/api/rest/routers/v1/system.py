"""System API router for FEAGI REST API."""

from typing import Dict, Any
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Models for request/response
class ConfigurationUpdateRequest(BaseModel):
    """Request model for configuration updates."""
    parameters: Dict[str, Any]

# Main system router
router = APIRouter(prefix="/system", tags=["system"])

# Brain state endpoints
@router.get("/brain/")
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

# Configuration endpoints
@router.get("/configuration/")
async def get_configuration(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current configuration."""
    config = core_api.get_configuration()
    return {"configuration": config}

@router.put("/configuration/")
async def update_configuration(
    config_update: ConfigurationUpdateRequest,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Update the configuration."""
    success = core_api.update_configuration(config_update.parameters)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update configuration")
    return {"message": "Configuration updated successfully"} 