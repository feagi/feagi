"""Simulation API router for FEAGI REST API."""

from typing import Dict, Any
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Models for request/response
class SimulationStatusResponse(BaseModel):
    """Response model for simulation status."""
    running: bool
    burst_count: int
    uptime: float
    performance: Dict[str, float]

# Create router with appropriate tag
router = APIRouter(prefix="/simulation", tags=["simulation"])

@router.post("/start")
async def start_simulation(core_api: CoreAPIService = Depends(get_core_api)):
    """Start the simulation."""
    success = core_api.start_simulation()
    if not success:
        raise HTTPException(status_code=500, detail="Failed to start simulation")
    return {"message": "Simulation started successfully"}

@router.post("/stop")
async def stop_simulation(core_api: CoreAPIService = Depends(get_core_api)):
    """Stop the simulation."""
    success = core_api.stop_simulation()
    if not success:
        raise HTTPException(status_code=500, detail="Failed to stop simulation")
    return {"message": "Simulation stopped successfully"}

@router.get("/status", response_model=SimulationStatusResponse)
async def get_simulation_status(core_api: CoreAPIService = Depends(get_core_api)):
    """Get the current simulation status."""
    status = core_api.get_simulation_status()
    return status 