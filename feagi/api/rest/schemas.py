"""
Pydantic schemas for the REST API.
"""

from typing import Dict, List, Any, Optional, Union
from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    """Error response model."""
    detail: Union[str, List[Dict[str, Any]]] = Field(description="Error details")


class SuccessResponse(BaseModel):
    """Generic success response model."""
    success: bool = Field(True, description="Success status")
    message: Optional[str] = Field(None, description="Success message")


class BurstEngineConfig(BaseModel):
    """Burst engine configuration model."""
    burst_duration: float = Field(10.0, description="Duration of a burst in milliseconds")
    inter_burst_interval: float = Field(5.0, description="Interval between bursts in milliseconds")
    maximum_firing_rate: float = Field(100.0, description="Maximum firing rate in Hz")
    refractory_period: float = Field(5.0, description="Refractory period in milliseconds")
    threshold: float = Field(0.5, description="Activation threshold")
    decay_rate: float = Field(0.1, description="Decay rate")
    firing_threshold: float = Field(0.7, description="Firing threshold")
    membrane_potential_decay: float = Field(0.05, description="Membrane potential decay rate")


class CorticalArea(BaseModel):
    """Cortical area model."""
    id: str = Field(description="Unique identifier")
    name: str = Field(description="Name of the cortical area")
    type: str = Field(description="Type of cortical area")
    coordinates: Dict[str, int] = Field(description="Coordinates in 3D space")
    dimensions: Dict[str, int] = Field(description="Dimensions in 3D space")
    properties: Optional[Dict[str, Any]] = Field(None, description="Additional properties")


class CorticalAreaResponse(BaseModel):
    """Response model for cortical areas endpoint."""
    areas: List[CorticalArea] = Field(description="List of cortical areas")


class CorticalAreaTypes(BaseModel):
    """Cortical area types model."""
    cortical: List[str] = Field(description="Cortical area types")
    subcortical: List[str] = Field(description="Subcortical area types")


class GenomeResponse(BaseModel):
    """Response model for genome endpoints."""
    filename: str = Field(description="Genome filename")
    success: bool = Field(True, description="Success status")
    message: Optional[str] = Field(None, description="Message")


class GenomeUploadResponse(SuccessResponse):
    """Response model for genome upload endpoint."""
    filename: str = Field(description="Uploaded genome filename")


class SimulationStatus(BaseModel):
    """Simulation status model."""
    running: bool = Field(description="Whether the simulation is running")
    burst_count: int = Field(description="Number of bursts executed")
    cycle_time: float = Field(description="Time per cycle in milliseconds")
    running_time: float = Field(description="Total running time in seconds") 