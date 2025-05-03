"""Insights API router for FEAGI REST API."""

from typing import Dict, List, Optional, Any, Union
from fastapi import APIRouter, HTTPException, Depends, Path, Query, Body
from pydantic import BaseModel, Field
import numpy as np
import json
from datetime import datetime, timedelta

from feagi.api.core.services import CoreAPIService
from feagi.api.rest.app import get_core_api

# Pydantic models for request/response
class ActivityHeatmapRequest(BaseModel):
    """Request model for activity heatmap."""
    window: int = Field(1, description="Time window for activity data (in bursts)")
    include_empty: bool = Field(False, description="Include areas with no activity")
    threshold: Optional[float] = Field(None, description="Activity threshold (0.0-1.0)")

class ActivityHeatmapResponse(BaseModel):
    """Response model for activity heatmap data."""
    timestamp: str
    data: Dict[str, Dict[str, float]] = Field(..., description="Activity data by area and coordinates")

class NeuronActivityRequest(BaseModel):
    """Request model for neuron activity time series."""
    neuron_ids: List[str] = Field(..., description="List of neuron IDs to track")
    window: int = Field(10, description="Time window for activity data (in bursts)")

class NeuronActivityResponse(BaseModel):
    """Response model for neuron activity time series."""
    timestamps: List[str]
    data: Dict[str, List[float]] = Field(..., description="Activity data by neuron ID")

class NetworkAnalyticsResponse(BaseModel):
    """Response model for network analytics."""
    neuron_count: int
    synapse_count: int
    area_count: int
    average_connectivity: float
    most_active_areas: List[Dict[str, Any]]
    least_active_areas: List[Dict[str, Any]]
    activity_distribution: Dict[str, int]

class PerformanceStatsResponse(BaseModel):
    """Response model for performance statistics."""
    average_burst_duration: float
    min_burst_duration: float
    max_burst_duration: float
    average_memory_usage: float
    average_cpu_usage: float
    timestamp: str

# Create router
router = APIRouter(prefix="/insights", tags=["insights"])

# Insights Endpoints
@router.post("/activity/heatmap", response_model=ActivityHeatmapResponse)
async def get_activity_heatmap(
    request: ActivityHeatmapRequest = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a heatmap of neural activity across all cortical areas.
    
    Args:
        request: Parameters for the activity heatmap.
    
    Returns:
        Activity heatmap data.
    """
    try:
        # In a real implementation, this would get actual activity data from the core
        # For now, return a placeholder with simulated data
        
        # Get cortical areas
        areas = core_api.get_cortical_areas()
        
        # Generate simulated activity data
        data = {}
        for area in areas:
            area_id = area["id"]
            dimensions = {
                "width": area["dimensions"]["width"],
                "height": area["dimensions"]["height"],
                "depth": area["dimensions"]["depth"]
            }
            
            # If area should be included (either has activity or we include empty areas)
            if request.include_empty or np.random.random() > 0.3:  # 70% chance of having activity
                area_data = {}
                
                # Generate random activities for each position in the area
                total_coords = dimensions["width"] * dimensions["height"] * dimensions["depth"]
                # Limit to a reasonable number for demo
                coords_to_include = min(total_coords, 10)
                
                for _ in range(coords_to_include):
                    x = np.random.randint(0, dimensions["width"])
                    y = np.random.randint(0, dimensions["height"])
                    z = np.random.randint(0, dimensions["depth"])
                    activity = np.random.random()
                    
                    # Apply threshold if provided
                    if request.threshold is None or activity >= request.threshold:
                        area_data[f"{x},{y},{z}"] = float(activity)
                
                if area_data:  # Only include if there's data after filtering
                    data[area_id] = area_data
        
        return {
            "timestamp": datetime.now().isoformat(),
            "data": data
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving activity heatmap: {str(e)}")

@router.post("/activity/neurons", response_model=NeuronActivityResponse)
async def get_neuron_activity(
    request: NeuronActivityRequest = Body(...),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get time series data of activity for specific neurons.
    
    Args:
        request: Parameters for the neuron activity data.
    
    Returns:
        Neuron activity time series data.
    """
    try:
        # In a real implementation, this would get actual neuron activity data from the core
        # For now, return a placeholder with simulated data
        
        # Generate timestamps for the window
        now = datetime.now()
        timestamps = [(now - timedelta(seconds=i)).isoformat() for i in range(request.window)]
        timestamps.reverse()  # Put in chronological order
        
        # Generate simulated activity data for each neuron
        data = {}
        for neuron_id in request.neuron_ids:
            # Generate a random activity pattern
            base_activity = np.random.random() * 0.5  # Base level
            activities = []
            for _ in range(request.window):
                # Add some random variation around the base level
                activity = base_activity + (np.random.random() - 0.5) * 0.3
                activity = max(0.0, min(1.0, activity))  # Clamp between 0 and 1
                activities.append(float(activity))
            
            data[neuron_id] = activities
        
        return {
            "timestamps": timestamps,
            "data": data
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving neuron activity: {str(e)}")

@router.get("/network/analytics", response_model=NetworkAnalyticsResponse)
async def get_network_analytics(core_api: CoreAPIService = Depends(get_core_api)):
    """
    Get analytics about the neural network.
    
    Returns:
        Network analytics data.
    """
    try:
        # In a real implementation, this would get actual network data from the core
        # For now, return a placeholder with simulated data
        
        # Get cortical areas for basic statistics
        areas = core_api.get_cortical_areas()
        area_count = len(areas)
        
        # Simulated values for other metrics
        neuron_count = np.random.randint(1000, 100000)
        synapse_count = np.random.randint(neuron_count * 10, neuron_count * 100)
        average_connectivity = synapse_count / (neuron_count * neuron_count) if neuron_count > 0 else 0
        
        # Generate most/least active areas
        active_areas = []
        for area in areas:
            area_id = area["id"]
            active_areas.append({
                "id": area_id,
                "name": area["name"],
                "activity_level": np.random.random()
            })
        
        # Sort by activity level
        active_areas.sort(key=lambda x: x["activity_level"], reverse=True)
        most_active = active_areas[:min(5, len(active_areas))]
        least_active = active_areas[-min(5, len(active_areas)):]
        
        # Generate activity distribution
        distribution = {
            "high": np.random.randint(0, area_count // 3),
            "medium": np.random.randint(area_count // 3, 2 * area_count // 3),
            "low": np.random.randint(2 * area_count // 3, area_count)
        }
        
        return {
            "neuron_count": neuron_count,
            "synapse_count": synapse_count,
            "area_count": area_count,
            "average_connectivity": float(average_connectivity),
            "most_active_areas": most_active,
            "least_active_areas": least_active,
            "activity_distribution": distribution
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving network analytics: {str(e)}")

@router.get("/performance/stats", response_model=PerformanceStatsResponse)
async def get_performance_stats(
    window: int = Query(10, description="Time window for stats (in seconds)"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get performance statistics.
    
    Args:
        window: Time window for the statistics in seconds.
    
    Returns:
        Performance statistics data.
    """
    try:
        # In a real implementation, this would get actual performance data from the core
        # For now, return a placeholder with simulated data
        
        # Generate simulated performance data
        avg_burst = np.random.uniform(5.0, 20.0)
        min_burst = avg_burst * (1.0 - np.random.uniform(0.1, 0.3))
        max_burst = avg_burst * (1.0 + np.random.uniform(0.1, 0.5))
        
        avg_memory = np.random.uniform(100.0, 2000.0)  # MB
        avg_cpu = np.random.uniform(10.0, 90.0)  # Percentage
        
        return {
            "average_burst_duration": float(avg_burst),
            "min_burst_duration": float(min_burst),
            "max_burst_duration": float(max_burst),
            "average_memory_usage": float(avg_memory),
            "average_cpu_usage": float(avg_cpu),
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving performance statistics: {str(e)}")

@router.get("/activity/summary")
async def get_activity_summary(
    window: int = Query(1, description="Time window for activity summary (in bursts)"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a summary of neural activity across the brain.
    
    Args:
        window: Time window for the summary in bursts.
    
    Returns:
        Activity summary data.
    """
    try:
        # In a real implementation, this would get actual activity data from the core
        # For now, return a placeholder with simulated data
        
        # Get cortical areas
        areas = core_api.get_cortical_areas()
        
        # Generate simulated activity summary
        area_summaries = []
        for area in areas:
            area_id = area["id"]
            
            # Generate random activity metrics for each area
            area_summaries.append({
                "id": area_id,
                "name": area["name"],
                "activity_level": np.random.random(),
                "active_neurons": np.random.randint(0, 1000),
                "total_neurons": np.random.randint(1000, 10000),
                "activity_change": np.random.uniform(-0.2, 0.2)  # Change from previous window
            })
        
        # Overall brain activity metrics
        overall_activity = np.mean([a["activity_level"] for a in area_summaries]) if area_summaries else 0
        
        return {
            "timestamp": datetime.now().isoformat(),
            "window_size": window,
            "overall_activity_level": float(overall_activity),
            "area_summaries": area_summaries
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving activity summary: {str(e)}")

@router.get("/connectivity/graph")
async def get_connectivity_graph(
    include_weights: bool = Query(False, description="Include synapse weights in the graph"),
    min_weight: Optional[float] = Query(None, description="Minimum weight threshold for including connections"),
    core_api: CoreAPIService = Depends(get_core_api)
):
    """
    Get a graph representation of connectivity between cortical areas.
    
    Args:
        include_weights: Whether to include synapse weights in the graph.
        min_weight: Minimum weight threshold for including connections.
    
    Returns:
        Connectivity graph data.
    """
    try:
        # In a real implementation, this would get actual connectivity data from the core
        # For now, return a placeholder with simulated data
        
        # Get cortical areas
        areas = core_api.get_cortical_areas()
        
        # Generate nodes for the graph
        nodes = []
        for area in areas:
            nodes.append({
                "id": area["id"],
                "name": area["name"],
                "type": area["type"],
                "coordinates": area["coordinates"]
            })
        
        # Generate edges (connections) between areas
        edges = []
        for i, source in enumerate(areas):
            # Each area connects to ~30% of other areas
            for j, target in enumerate(areas):
                if i != j and np.random.random() < 0.3:
                    weight = np.random.random()
                    if min_weight is None or weight >= min_weight:
                        connection = {
                            "source": source["id"],
                            "target": target["id"]
                        }
                        if include_weights:
                            connection["weight"] = float(weight)
                        edges.append(connection)
        
        return {
            "nodes": nodes,
            "edges": edges
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving connectivity graph: {str(e)}") 