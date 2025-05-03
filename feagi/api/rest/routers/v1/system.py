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

# Create nested routers for better organization
brain_router = APIRouter(prefix="/brain", tags=["brain"])
config_router = APIRouter(prefix="/configuration", tags=["configuration"])

# Brain state endpoints
@brain_router.get("/")
async def get_brain_state(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current brain state."""
    brain_state = core_api.get_brain_state()
    return {"brain_state": brain_state}

@brain_router.post("/save")
async def save_brain_state(path: str, core_api: CoreAPIService = Depends(get_core_api)):
    """Save the current brain state to a file."""
    success = core_api.save_brain_state(path)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to save brain state")
    return {"message": "Brain state saved successfully"}

@brain_router.post("/load")
async def load_brain_state(path: str, core_api: CoreAPIService = Depends(get_core_api)):
    """Load a brain state from a file."""
    success = core_api.load_brain_state(path)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to load brain state")
    return {"message": "Brain state loaded successfully"}

# Configuration endpoints
@config_router.get("/")
async def get_configuration(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current configuration."""
    config = core_api.get_configuration()
    return {"configuration": config}

@config_router.put("/")
async def update_configuration(
    config_update: ConfigurationUpdateRequest,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """Update the configuration."""
    success = core_api.update_configuration(config_update.parameters)
    if not success:
        raise HTTPException(status_code=500, detail="Failed to update configuration")
    return {"message": "Configuration updated successfully"}

# Main system router that includes the subrouters
router = APIRouter()
router.include_router(brain_router)
router.include_router(config_router) 