"""
Pydantic schemas for the REST API.
"""

from typing import Dict, List, Any, Optional, Union, Tuple
from pydantic import BaseModel, Field
from enum import Enum


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


class FCLSamplerConsumer(int, Enum):
    visualization = 1
    motor = 2
    # Add more consumers as needed in the future


class FCLSamplerConfig(BaseModel):
    frequency: float = Field(..., description="FCLSampler frequency in Hz")
    consumer: FCLSamplerConsumer = Field(..., description="FCLSampler consumer (visualization, motor, etc.)")


class FCLSampleRateConfig(BaseModel):
    sample_rate: float = Field(..., description="FCL sample rate for this cortical area in Hz")


class AgentRegistration(BaseModel):
    agent_id: str
    agent_type: str
    agent_data_port: int
    agent_version: str
    controller_version: str
    capabilities: Optional[Dict[str, Any]] = None


class RobotController(BaseModel):
    # Add fields as needed; using a generic dict for now
    parameters: Optional[Dict[str, Any]] = None


class RobotModel(BaseModel):
    # Add fields as needed; using a generic dict for now
    parameters: Optional[Dict[str, Any]] = None


class ManualStimulation(BaseModel):
    stimulation_payload: Any 


class RewiringMode(str, Enum):
    rewire_all = "all"
    rewire_system = "system"
    rewire_none = "none"


class UserPreferences(BaseModel):
    adv_mode: bool
    ui_magnification: float = 1.0


# Registration model for agent registration endpoints
class Registration(BaseModel):
    agent_id: str
    agent_type: str
    agent_name: str
    agent_version: str
    agent_description: str
    agent_ip: str
    agent_port: int
    agent_status: str
    agent_capabilities: dict


# Logs model for log management endpoints
class Logs(BaseModel):
    log_level: str
    message: str
    timestamp: Optional[str] = None


# Subscriber model for beacon subscription endpoints
class Subscriber(BaseModel):
    subscriber_id: str
    subscriber_type: str
    subscriber_name: str
    subscriber_ip: str
    subscriber_port: int
    subscriber_status: str
    subscriber_capabilities: dict


# VizSkipRate model for cortical area visualization skip rate endpoints
class VizSkipRate(BaseModel):
    cortical_area: str
    skip_rate: int


# VizThreshold model for cortical area visualization suppression threshold endpoints
class VizThreshold(BaseModel):
    visualization_threshold: int


# BrainVisualization model for global activity visualization endpoints
class BrainVisualization(BaseModel):
    global_visualization: bool


class CorticalId(BaseModel):
    cortical_id: str


class CorticalIdList(BaseModel):
    cortical_id_list: list[str]


class CorticalName(BaseModel):
    cortical_name: str


class CorticalList(BaseModel):
    area_list: list[str]


class NewCorticalProperties(BaseModel):
    """Request model for adding a new core cortical area."""
    cortical_id: str
    cortical_type: str
    cortical_name: str
    parent_region_id: Optional[str] = None
    sub_group_id: Optional[str] = None
    cortical_dimensions: Optional[List[int]] = None
    coordinates_2d: Optional[List[int]] = None
    coordinates_3d: Optional[List[int]] = None
    per_voxel_neuron_cnt: Optional[int] = None
    dev_count: Optional[int] = None


class NewCustomCorticalProperties(BaseModel):
    """Request model for adding a new custom cortical area."""
    cortical_name: str
    parent_region_id: str
    sub_group_id: Optional[str] = None
    cortical_dimensions: Optional[List[int]] = None
    coordinates_2d: Optional[List[int]] = None
    coordinates_3d: Optional[List[int]] = None
    per_voxel_neuron_cnt: Optional[int] = None
    dev_count: Optional[int] = None
    copy_of: Optional[str] = None
    is_memory: Optional[bool] = None


class UpdateCorticalProperties(BaseModel):
    """Request model for updating a single cortical area's properties."""
    cortical_id: str
    parent_region_id: Optional[str] = None
    cortical_dimensions: Optional[List[int]] = None
    cortical_neuron_per_vox_count: Optional[int] = None
    coordinates_2d: Optional[List[int]] = None
    coordinates_3d: Optional[List[int]] = None
    per_voxel_neuron_cnt: Optional[int] = None
    dev_count: Optional[int] = None
    sub_group_id: Optional[str] = None


class UpdateMultipleCorticalProperties(BaseModel):
    """Request model for updating multiple cortical areas at once."""
    cortical_id_list: List[str]
    parent_region_id: Optional[str] = None
    cortical_dimensions: Optional[List[int]] = None
    cortical_neuron_per_vox_count: Optional[int] = None
    coordinates_2d: Optional[List[int]] = None
    coordinates_3d: Optional[List[int]] = None
    per_voxel_neuron_cnt: Optional[int] = None
    dev_count: Optional[int] = None
    sub_group_id: Optional[str] = None


class CorticalAreaSrcDst(BaseModel):
    """Model for specifying source and destination cortical area IDs for mapping queries."""
    source: str
    destination: str


class UpdateCorticalMappingProperties(BaseModel):
    """Model for updating properties of a cortical mapping between two areas."""
    source: str
    destination: str
    mapping_type: Optional[str] = None
    weight: Optional[float] = None
    delay: Optional[float] = None
    enabled: Optional[bool] = None


class SuggestedMapping(BaseModel):
    """Model for a suggested cortical mapping between two areas."""
    source: str
    destination: str
    weight: Optional[float] = None
    delay: Optional[float] = None
    mapping_type: Optional[str] = None
    enabled: Optional[bool] = None


class NewRegionProperties(BaseModel):
    """Request model for creating a new brain region."""
    region_id: str
    title: str
    parent_region_id: str
    coordinate_2d: Optional[List[int]] = None
    coordinate_3d: Optional[List[int]] = None


class UpdateRegionProperties(BaseModel):
    """Request model for updating a brain region's properties."""
    region_id: str
    title: Optional[str] = None
    region_description: Optional[str] = None
    parent_region_id: Optional[str] = None
    coordinate_2d: Optional[List[int]] = None
    coordinate_3d: Optional[List[int]] = None


class Id(BaseModel):
    """Simple model for returning or receiving an ID string."""
    id: str


class RegionAssociation(BaseModel):
    """Model for associating a cortical area with a brain region."""
    cortical_area_id: str
    region_id: str


class VisionSettings(BaseModel):
    central_vision_resolution: Tuple[Optional[int], Optional[int]]
    peripheral_vision_resolution: Tuple[Optional[int], Optional[int]]
    flicker_period: Optional[int]
    color_vision: Optional[bool]
    eccentricity: Optional[Tuple[Optional[float], Optional[float]]]
    modulation: Optional[Tuple[Optional[float], Optional[float]]]
    brightness: Optional[float]
    contrast: Optional[float]
    shadows: Optional[float]
    pixel_change_limit: Optional[float]
    horizontal_flip: Optional[bool] = None
