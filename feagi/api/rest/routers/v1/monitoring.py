"""
Monitoring router for system performance and health metrics.
"""
from fastapi import APIRouter, Depends, HTTPException, Query
from typing import Dict, Any, List, Optional

from feagi.api.dependencies import (
    check_monitoring_available,
    check_active_genome,
    check_connectome_ready,
    check_burst_engine_running
)
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.core.state_manager import FeagiStateManager

router = APIRouter()

@router.get("/system")
async def get_system_metrics(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_monitoring_available)
):
    """
    Get basic system metrics (CPU, memory, etc.)
    
    This endpoint only requires that monitoring capabilities are available,
    but doesn't require a genome/connectome to be ready.
    """
    try:
        # Get system metrics from the core service
        metrics = await core_api_service.get_system_metrics()
        return metrics
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get system metrics: {str(e)}"
        )

@router.get("/burst-engine")
async def get_burst_engine_stats(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_burst_engine_running)
):
    """
    Get burst engine statistics.
    
    This endpoint requires the burst engine to be running.
    """
    try:
        # Get burst engine statistics
        stats = core_api_service.get_burst_engine_stats()
        return stats
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get burst engine statistics: {str(e)}"
        )

@router.get("/connectome/stats")
async def get_connectome_stats(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready)
):
    """
    Get connectome statistics (neuron count, synapse count, etc.)
    
    This endpoint requires the connectome to be ready.
    """
    try:
        # Gather connectome statistics
        connectome_manager = core_api_service.get_connectome_manager()
        
        # Calculate neuron and synapse counts
        neuron_count = len(connectome_manager.neurons) if hasattr(connectome_manager, 'neurons') else 0
        synapse_count = connectome_manager.synapse_count() if hasattr(connectome_manager, 'synapse_count') else 0
        
        # Get cortical area count
        cortical_areas_count = len(connectome_manager.cortical_areas) if hasattr(connectome_manager, 'cortical_areas') else 0
        
        # Get connectome dimensions
        dimensions = core_api_service.get_connectome_dimensions()
        
        return {
            "neuron_count": neuron_count,
            "synapse_count": synapse_count, 
            "cortical_areas_count": cortical_areas_count,
            "dimensions": dimensions,
            "timestamp": connectome_manager.current_timestep if hasattr(connectome_manager, 'current_timestep') else 0
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get connectome statistics: {str(e)}"
        )

@router.get("/activity")
async def get_neural_activity(
    window: int = Query(1, description="Activity window in bursts"),
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready)
):
    """
    Get neural activity across the connectome.
    
    This endpoint requires the connectome to be ready.
    
    Args:
        window: The number of bursts to include in the activity window
    """
    try:
        # Get all cortical areas
        areas = core_api_service.get_cortical_areas()
        
        # Collect activity data for each area
        activity_data = {}
        
        for area in areas:
            area_id = area["id"]
            area_activity = core_api_service.get_cortical_area_activity(area_id, window)
            
            if area_activity:
                activity_data[area_id] = {
                    "name": area["name"],
                    "total_neurons": area_activity["total_neurons"],
                    "active_neurons": area_activity["active_neurons"],
                    "activity_ratio": area_activity["activity_ratio"]
                }
                
        # Calculate overall statistics
        total_neurons = sum(data["total_neurons"] for data in activity_data.values())
        active_neurons = sum(data["active_neurons"] for data in activity_data.values())
        
        return {
            "activity_by_area": activity_data,
            "overall": {
                "total_neurons": total_neurons,
                "active_neurons": active_neurons,
                "activity_ratio": active_neurons / total_neurons if total_neurons > 0 else 0
            },
            "window": window
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get neural activity: {str(e)}"
        )

@router.get("/performance")
async def get_performance_stats(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_burst_engine_running)
):
    """
    Get detailed performance statistics.
    
    This endpoint requires the burst engine to be running.
    """
    try:
        # Get performance statistics
        stats = await core_api_service.get_performance_stats()
        return stats
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get performance statistics: {str(e)}"
        ) 