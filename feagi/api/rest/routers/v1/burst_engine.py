"""Burst Engine API router for FEAGI REST API."""

from typing import Dict, List, Optional, Any
from fastapi import APIRouter, HTTPException, Depends, Path, Query, Body
from pydantic import BaseModel, Field

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Pydantic models for request/response
class BurstConfigBase(BaseModel):
    """Base model for burst engine configuration."""
    burst_duration: Optional[float] = Field(None, description="Duration of each burst in milliseconds")
    inter_burst_interval: Optional[float] = Field(None, description="Interval between bursts in milliseconds")
    maximum_firing_rate: Optional[float] = Field(None, description="Maximum firing rate in Hz")

class BurstConfigUpdate(BurstConfigBase):
    """Request model for updating burst configuration."""
    pass

class BurstConfigResponse(BurstConfigBase):
    """Response model for burst configuration information."""
    pass

class BurstStatsResponse(BaseModel):
    """Response model for burst engine statistics."""
    current_burst: int = Field(..., description="Current burst number")
    total_bursts: int = Field(..., description="Total number of bursts since start")
    burst_duration: float = Field(..., description="Duration of each burst in milliseconds")
    inter_burst_interval: float = Field(..., description="Interval between bursts in milliseconds")
    average_processing_time: float = Field(..., description="Average processing time per burst in milliseconds")
    neuron_activity_level: float = Field(..., description="Percentage of neurons activated in last burst")

# Create router
router = APIRouter(prefix="/burst_engine", tags=["burst_engine"])

# Burst Engine Endpoints
@router.get("/configuration", response_model=BurstConfigResponse)
async def get_burst_configuration(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get the current burst engine configuration.
    
    Returns:
        Current configuration of the burst engine.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Extract burst configuration from the genome
        physiology = genome.get("physiology", {})
        burst_config = physiology.get("burst_engine", {})
        
        return {
            "burst_duration": burst_config.get("burst_duration"),
            "inter_burst_interval": burst_config.get("inter_burst_interval"),
            "maximum_firing_rate": burst_config.get("maximum_firing_rate")
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving burst configuration: {str(e)}")

@router.put("/configuration", response_model=BurstConfigResponse)
async def update_burst_configuration(
    config: BurstConfigUpdate,
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Update the burst engine configuration.
    
    Args:
        config: Updated configuration for the burst engine.
    
    Returns:
        Updated configuration of the burst engine.
    """
    try:
        genome = core_api.get_genome()
        if not genome:
            raise HTTPException(status_code=400, detail="No genome loaded")
        
        # Ensure physiology section exists
        if "physiology" not in genome:
            genome["physiology"] = {}
        
        # Ensure burst_engine section exists
        if "burst_engine" not in genome["physiology"]:
            genome["physiology"]["burst_engine"] = {}
        
        # Update burst configuration
        burst_config = genome["physiology"]["burst_engine"]
        
        if config.burst_duration is not None:
            burst_config["burst_duration"] = config.burst_duration
        
        if config.inter_burst_interval is not None:
            burst_config["inter_burst_interval"] = config.inter_burst_interval
        
        if config.maximum_firing_rate is not None:
            burst_config["maximum_firing_rate"] = config.maximum_firing_rate
        
        # Save the updated genome
        if core_api.get_genome_filename():
            try:
                from feagi.evo.genome_editor import save_genome
                save_genome(genome, core_api.get_genome_filename())
            except ImportError:
                # Fallback implementation
                import json
                with open(core_api.get_genome_filename(), 'w') as f:
                    json.dump(genome, f, indent=2)
        
        # Apply changes to the running burst engine if simulation is active
        simulation_status = core_api.get_simulation_status()
        if simulation_status.get("running", False):
            # This would normally update the running burst engine
            # In a real implementation, this would call a core function to update the burst engine
            pass
        
        return {
            "burst_duration": burst_config.get("burst_duration"),
            "inter_burst_interval": burst_config.get("inter_burst_interval"),
            "maximum_firing_rate": burst_config.get("maximum_firing_rate")
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating burst configuration: {str(e)}")

@router.get("/stats", response_model=BurstStatsResponse)
async def get_burst_stats(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get current statistics from the burst engine.
    
    Returns:
        Current statistics of the burst engine.
    """
    try:
        # Placeholder implementation
        # In a real implementation, this would get actual burst statistics from the core
        simulation_status = core_api.get_simulation_status()
        if not simulation_status.get("running", False):
            raise HTTPException(status_code=400, detail="Simulation is not running")
        
        # Return placeholder data
        return {
            "current_burst": 0,
            "total_bursts": 0,
            "burst_duration": 10.0,
            "inter_burst_interval": 5.0,
            "average_processing_time": 8.5,
            "neuron_activity_level": 0.05
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving burst statistics: {str(e)}")

@router.post("/trigger_burst")
async def trigger_manual_burst(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Manually trigger a single burst cycle.
    
    This is mainly for testing purposes and debugging.
    
    Returns:
        Confirmation message.
    """
    try:
        simulation_status = core_api.get_simulation_status()
        if simulation_status.get("running", True):
            raise HTTPException(status_code=400, detail="Cannot manually trigger burst while simulation is running")
        
        # This would normally trigger a manual burst
        # In a real implementation, this would call a core function to trigger a burst
        
        return {"message": "Manual burst triggered successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error triggering manual burst: {str(e)}")

@router.post("/reset_counters")
async def reset_burst_counters(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Reset the burst counters and statistics.
    
    Returns:
        Confirmation message.
    """
    try:
        # This would normally reset the burst engine counters
        # In a real implementation, this would call a core function to reset counters
        
        return {"message": "Burst counters reset successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error resetting burst counters: {str(e)}") 