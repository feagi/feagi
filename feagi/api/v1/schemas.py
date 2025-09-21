"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
FEAGI v1 API Schemas

Transport-agnostic request and response schemas for FEAGI v1 API.
These schemas define the data structures used by all endpoints,
regardless of transport protocol (HTTP, ZMQ, etc.).
"""

from typing import Any, Dict, List, Optional
from enum import Enum

from pydantic import AliasChoices, BaseModel, Field, RootModel

# ===== System Schemas =====


class LoggingLevel(str, Enum):
    """Enumeration of available logging levels for global system logging control.
    
    These levels control the minimum severity of log messages that will be displayed
    across all FEAGI components. Changes take effect immediately at runtime.
    
    Levels (from most to least verbose):
    - DEBUG: All messages including detailed debugging information
    - INFO: Informational messages and above (normal operation details)
    - WARNING: Warning messages and above (potential issues, default level)
    - ERROR: Error messages and above (actual problems)
    - CRITICAL: Only critical system failures
    """
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class UserPreferencesRequest(BaseModel):
    """Request schema for updating user preferences."""

    adv_mode: bool
    ui_magnification: float
    auto_pns_area_creation: Optional[bool] = None


class UserPreferencesResponse(BaseModel):
    """Response schema for user preferences."""

    adv_mode: bool
    ui_magnification: float
    auto_pns_area_creation: bool


class VersionsResponse(BaseModel):
    """Response schema for system versions."""

    feagi_core: str
    python: str
    timestamp: str
    # Optional components
    numpy: Optional[str] = None
    torch: Optional[str] = None


class HealthCheckResponse(BaseModel):
    """Response schema for system health check."""

    burst_engine: bool
    connected_agents: Optional[int]
    influxdb_availability: bool
    neuron_count_max: int
    synapse_count_max: int
    latest_changes_saved_externally: bool
    genome_availability: bool
    genome_validity: Optional[bool]
    brain_readiness: bool
    # Optional fields when genome is loaded
    fitness: Optional[float] = None
    cortical_area_count: Optional[int] = None
    neuron_count: Optional[int] = None  # Total neurons (regular + memory)
    memory_neuron_count: Optional[int] = None  # Memory neurons only
    regular_neuron_count: Optional[int] = None  # Regular neurons only
    synapse_count: Optional[int] = None
    estimated_brain_size_in_MB: Optional[float] = None
    # Genome tracking fields for downstream clients (Bridge/Godot)
    genome_num: Optional[int] = None
    genome_timestamp: Optional[int] = None
    # Simulation timing
    simulation_timestep: Optional[float] = None  # Time between neural bursts in seconds
    # Memory area statistics (per-cortical-area breakdown)
    memory_area_stats: Optional[Dict[str, Dict[str, Any]]] = None  # Per-area memory neuron stats
    # Amalgamation status (when amalgamation is pending)
    amalgamation_pending: Optional[Dict[str, Any]] = None  # Pending amalgamation information


class ConfigurationResponse(BaseModel):
    """Response schema for system configuration."""

    # This will be expanded based on actual configuration structure
    config: Dict[str, Any]


class InfluxDBTestResponse(BaseModel):
    """Response schema for InfluxDB test."""

    status: str
    database: str
    host: str
    port: int


class CorticalAreaTypesResponse(BaseModel):
    """Response schema for cortical area types."""

    types: Dict[str, Any]


class VisualizationSkipRateRequest(BaseModel):
    """Request schema for visualization skip rate."""

    skip_rate: float


class VisualizationThresholdRequest(BaseModel):
    """Request schema for visualization threshold."""

    threshold: float


class BrainVisualizationRequest(BaseModel):
    """Request schema for brain visualization settings."""

    enabled: bool


class RegistrationRequest(BaseModel):
    """Request schema for system registration."""

    # Will be expanded based on actual registration needs
    registration_data: Dict[str, Any]


class LogsRequest(BaseModel):
    """Request schema for log management."""

    log_data: Dict[str, Any]


class SubscriberRequest(BaseModel):
    """Request schema for beacon subscription."""

    subscriber_address: str


class CircuitLibraryPathRequest(BaseModel):
    """Request schema for setting circuit library path."""

    path: str


# ===== Standard Response Wrappers =====


class SuccessResponse(BaseModel):
    """Standard success response."""

    status: str = "success"
    message: Optional[str] = None


class ErrorResponse(BaseModel):
    """Standard error response."""

    status: str = "error"
    message: str
    code: Optional[str] = None


# ===== Genome Schemas =====


class AmalgamationRequest(BaseModel):
    """Request model for genome amalgamation operations."""

    genome_id: Optional[str] = None
    genome_title: Optional[str] = None
    genome_payload: Optional[dict] = None


class GenomeUploadResponse(BaseModel):
    """Response model for genome upload operations.

    This response follows the standard FEAGI response format:
    - success: True if the overall operation succeeded (file uploaded AND genome loaded)
    - message: Human-readable description of what happened
    - genome_number: Current genome counter (incremented on successful loads)
    - details: Standard FEAGI details field containing:
        - success: Whether genome validation and brain development succeeded
        - error: Error message if genome loading failed
        - validation_errors: List of specific validation errors (if validation failed)
        - Additional metadata about the loading process
    """

    success: bool
    message: str
    genome_number: Optional[int] = None
    details: Optional[dict] = None


class GenomeFileNameResponse(BaseModel):
    """Response model for genome file name query."""

    file_name: str


class GenomeNumberResponse(BaseModel):
    """Response model for genome number query."""

    genome_number: int


class GenomeDownloadResponse(BaseModel):
    """Response model for genome download."""

    genome_data: dict
    filename: str


class AmalgamationResponse(BaseModel):
    """Response model for amalgamation operations."""

    amalgamation_id: str
    status: str
    message: str


class AmalgamationHistoryResponse(BaseModel):
    """Response model for amalgamation history."""

    history: List[dict]


class CircuitLibraryResponse(BaseModel):
    """Response model for circuit library listing."""

    circuits: List[dict]


class CorticalTemplateResponse(BaseModel):
    """Response model for cortical template."""

    template: dict


class GenomeDefaultFilesResponse(BaseModel):
    """Response model for default genome files."""

    files: List[str]


# ===== Cortical Area Schemas =====


class CorticalAreaPropertiesResponse(BaseModel):
    """Response model for cortical area properties."""

    properties: dict


class CorticalAreaIdListResponse(BaseModel):
    """Response model for cortical area ID list."""

    cortical_ids: List[str]


class CorticalAreaIndexListResponse(BaseModel):
    """Response model for cortical area index list."""

    indices: List[int]


class CorticalAreaNameListResponse(BaseModel):
    """Response model for cortical area names."""

    names: List[str]


class CorticalLocationResponse(BaseModel):
    """Response model for cortical area location."""

    x: int
    y: int
    z: int


class CorticalTypesResponse(BaseModel):
    """Response model for cortical area types."""

    types: List[str]


class CorticalIdNameMappingResponse(BaseModel):
    """Response model for cortical ID to name mapping."""

    mapping: Dict[str, str]


class CorticalGeometryResponse(BaseModel):
    """Response model for cortical area geometry."""

    geometry: dict


class NeuronCountResponse(BaseModel):
    """Response model for neuron count."""

    neuron_count: int


class CloneCorticalAreaRequest(BaseModel):
    """Request model for cloning a cortical area.

    Fields:
        source_area_id: Cortical ID of the area to clone
        clone_cortical_mapping: Whether to duplicate incoming/outgoing/recursive mappings (default: True)
        coordinates_3d: Optional override for new area's 3D coordinates [x, y, z]
        coordinates_2d: Optional override for new area's 2D coordinates [x, y]
    """

    source_area_id: str
    clone_cortical_mapping: Optional[bool] = True
    coordinates_3d: Optional[List[int]] = None
    coordinates_2d: Optional[List[int]] = None


class CloneCorticalAreaResponse(BaseModel):
    """Response model for cortical area clone operation."""

    new_area_id: str
    message: str


# ===== Common Request Schemas =====


class CorticalIdRequest(BaseModel):
    """Request model for operations requiring cortical ID."""

    cortical_id: str


class CorticalNameRequest(BaseModel):
    """Request model for operations requiring cortical name."""

    cortical_name: str


class CorticalIdListRequest(BaseModel):
    """Request model for operations on multiple cortical areas."""

    cortical_ids: List[str] = Field(
        validation_alias=AliasChoices("cortical_ids", "cortical_id_list")
    )


class CorticalPropertiesUpdateRequest(BaseModel):
    """Request model for updating cortical area properties."""

    cortical_id: str

    class Config:
        extra = "allow"  # Allow additional fields for dynamic properties


class AddCoreCorticalAreaRequest(BaseModel):
    """Request model for adding a core cortical area.

    Accepts core cortical fields and allows additional properties.
    """

    cortical_id: Optional[str] = None
    cortical_type: Optional[str] = None
    coordinates_2d: Optional[List[int]] = None
    coordinates_3d: Optional[List[int]] = None
    device_count: Optional[int] = None

    class Config:
        extra = "allow"  # Allow extensibility without breaking


class CustomCorticalAreaRequest(BaseModel):
    """Request model for creating custom cortical areas."""

    cortical_name: str
    brain_region_id: str  # Maps to parent_region_id
    cortical_group: str
    cortical_sub_group: str  # Maps to sub_group_id
    cortical_dimensions: List[int]
    coordinates_2d: List[int]
    coordinates_3d: List[int]
    copy_of: Optional[str] = None

    # Support for sub_group_id field (alternative to cortical_sub_group)
    sub_group_id: Optional[str] = None

    # Memory-specific properties for memory cortical areas
    init_lifespan: Optional[int] = None
    lifespan_growth_rate: Optional[float] = None
    longterm_mem_threshold: Optional[int] = None
    temporal_depth: Optional[int] = None

    class Config:
        extra = "allow"  # Allow additional fields for extensibility


class CoordinateUpdateRequest(BaseModel):
    """Request model for coordinate updates."""

    coordinates: dict


# ===== Connectome Schemas =====


class BatchNeuronCreationRequest(BaseModel):
    """Request model for batch neuron creation."""

    area_id: str
    positions: List[tuple]  # List of (x, y, z) positions
    properties: Optional[Dict[str, Any]] = None


class BatchSynapseCreationRequest(BaseModel):
    """Request model for batch synapse creation."""

    connections: List[tuple]  # List of (pre_id, post_id, weight)


class CorticalAreasListResponse(BaseModel):
    """Response model for cortical areas listing."""

    areas: List[Dict[str, Any]]


class CorticalAreaInfoResponse(BaseModel):
    """Response model for cortical area information."""

    area_info: Dict[str, Any]


# ===== System Debug Logging Schema =====

class DebugLoggingRequest(BaseModel):
    """Request to set live debug logging flags and global logging level via API.

    Keys mirror CLI debug flags. All fields are optional - only provided fields
    will be updated, allowing partial updates of debug configuration.
    
    The global_logging_level field controls system-wide log verbosity and takes
    effect immediately across all FEAGI components.
    """

    # Legacy aggregate API flag (enables all API subsystems when True)
    api: Optional[bool] = Field(None, description="Legacy aggregate API debug flag (enables all API subsystems)")
    # New granular API flags
    api_core: Optional[bool] = Field(None, description="Core API debug flag (not supported, ignored)")
    api_rest: Optional[bool] = Field(None, description="REST API debug flag (not supported, ignored)")
    api_zmq: Optional[bool] = Field(None, description="ZMQ API debug flag (not supported, ignored)")
    npu: Optional[bool] = Field(None, description="Neural Processing Unit debug flag")
    bdu: Optional[bool] = Field(None, description="Brain Development Unit debug flag")
    zmq_inbound: Optional[bool] = Field(None, description="Inbound ZMQ message debug flag")
    zmq_outbound: Optional[bool] = Field(None, description="Outbound ZMQ message debug flag")
    mem: Optional[bool] = Field(None, description="Memory system debug flag")
    global_logging_level: Optional[LoggingLevel] = Field(None, description="Global logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL) - controls system-wide log verbosity")

    class Config:
        extra = "forbid"


class DebugLoggingResponse(BaseModel):
    """Response with current debug logging flags and global logging level.
    
    Returns the current state of all debug flags for FEAGI subsystems and
    the active global logging level that controls system-wide log verbosity.
    """

    api: bool = Field(description="Legacy aggregate API debug flag")
    api_core: bool = Field(description="Core API debug flag (not supported, always False)")
    api_rest: bool = Field(description="REST API debug flag (not supported, always False)")
    api_zmq: bool = Field(description="ZMQ API debug flag (not supported, always False)")
    npu: bool = Field(description="Neural Processing Unit debug flag")
    bdu: bool = Field(description="Brain Development Unit debug flag")
    zmq_inbound: bool = Field(description="Inbound ZMQ message debug flag")
    zmq_outbound: bool = Field(description="Outbound ZMQ message debug flag")
    mem: bool = Field(description="Memory system debug flag")
    global_logging_level: LoggingLevel = Field(description="Current global logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)")



class FireQueueResponse(BaseModel):
    """Response schema for fire queue data."""

    fire_queue: Optional[Dict[str, Any]]


class PlasticityInfoResponse(BaseModel):
    """Response model for plasticity information."""

    plasticity_info: Dict[str, Any]


class ConnectomePathResponse(BaseModel):
    """Response model for connectome path."""

    path: str


class ConnectomeSnapshotResponse(BaseModel):
    """Response model for connectome snapshot."""

    message: str
    path: str


class ConnectomeDimensionsResponse(BaseModel):
    """Response model for connectome dimensions."""

    dimensions: Dict[str, Any]


class CorticalStatsResponse(BaseModel):
    """Response model for cortical area statistics."""

    stats: Dict[str, Any]


class MappingRestrictionsResponse(BaseModel):
    """Response model for mapping restrictions between cortical area types."""

    restrictions: List[Dict[str, Any]]
    defaults: List[Dict[str, Any]]


class MappingRestrictionsRequest(BaseModel):
    """Request model for getting mapping restrictions between specific
    types."""

    source_type: Optional[str] = None
    destination_type: Optional[str] = None


class CorticalAreaMappingRestrictionRequest(BaseModel):
    """Request model for getting mapping restrictions between two cortical
    areas."""

    source_cortical_id: str
    destination_cortical_id: str


class CorticalAreaMappingRestrictionResponse(BaseModel):
    """Response model for mapping restrictions between two cortical areas."""

    source_cortical_id: str
    destination_cortical_id: str
    source_type: str
    destination_type: str
    restriction: Optional[Dict[str, Any]]
    default: Optional[Dict[str, Any]]
    has_restricted_morphologies: bool
    get_morphologies_restricted_to: List[str]


class NeuronMappingsResponse(BaseModel):
    """Response model for neuron mappings."""

    mappings: Dict[str, Any]


class OutgoingSynapse(BaseModel):
    """Model for an outgoing synaptic connection."""

    target_neuron_id: int
    weight: float


class IncomingSynapse(BaseModel):
    """Model for an incoming synaptic connection."""

    source_neuron_id: int
    weight: float


class SynapseCounts(BaseModel):
    """Model for synapse count summary."""

    outgoing: int
    incoming: int
    total: int


class NeuronPropertiesResponse(BaseModel):
    """Response model for individual neuron properties."""

    neuron_id: int
    cortical_id: str
    cortical_idx: int
    position: List[
        int
    ]  # PERFORMANCE FIX: Keep positions as integers, no conversion needed
    threshold: float
    membrane_potential: float
    resting_potential: float
    decay_rate: float
    refractory_period: int
    refractory_counter: int
    properties: Dict[str, Any]
    outgoing_synapses: List[OutgoingSynapse]
    incoming_synapses: List[IncomingSynapse]
    synapse_counts: SynapseCounts


class BatchNeuronCreationResponse(BaseModel):
    """Response model for batch neuron creation."""

    created_neurons: List[int]
    count: int


class BatchSynapseCreationResponse(BaseModel):
    """Response model for batch synapse creation."""

    created_synapses: int


class CorticalAreaSynapsesResponse(BaseModel):
    """Response model for cortical area synapses."""

    synapses: Dict[str, List[int]] = Field(
        description="Dictionary where keys are destination cortical area IDs and values are lists of target neuron IDs"
    )


# ===== Burst Engine Schemas =====


class BurstEngineStatusResponse(BaseModel):
    """Response model for burst engine status."""

    status: str
    is_running: bool
    config: Optional[Dict[str, Any]] = None


class BurstEngineConfigRequest(BaseModel):
    """Request model for burst engine configuration."""

    config: Dict[str, Any]


class BurstEngineStatsResponse(BaseModel):
    """Response model for burst engine statistics."""

    stats: Dict[str, Any]


class FCLContentResponse(BaseModel):
    """Response schema for FCL content (Fire Queue firing history).
    
    Note: In the new architecture, FCL is a transient pre-burst collector.
    This endpoint returns Fire Queue data (neurons that actually fired) for compatibility.
    Window size refers to Fire Ledger historical storage configuration.
    """
    
    timestep: int = Field(description="Current timestep when firing data was captured")
    total_neurons: int = Field(description="Total number of neurons that fired in the timestep")
    global_fcl: List[int] = Field(description="List of all neuron IDs that fired (from Fire Queue)")
    cortical_areas: Dict[str, List[int]] = Field(
        description="Mapping of cortical area ID to list of fired neuron IDs"
    )
    default_window_size: int = Field(description="Default Fire Ledger historical window size")
    active_cortical_count: int = Field(description="Number of cortical areas with active neurons")


# ===== Region Schemas =====


class RegionInfoResponse(BaseModel):
    """Response model for brain region information."""

    region_info: Dict[str, Any]


class RegionListResponse(BaseModel):
    """Response model for brain region list."""

    regions: List[Dict[str, Any]]


class CreateRegionRequest(BaseModel):
    """Request model for creating a brain region."""

    region_data: Dict[str, Any]


class UpdateRegionRequest(BaseModel):
    """Request model for updating a brain region."""

    region_id: str
    updates: Dict[str, Any]


class RegionMemberRelocationRequest(RootModel):
    """Request model for relocating brain region members.
    
    Accepts a dictionary where keys are cortical area IDs and values contain
    coordinate information and/or parent region assignments. At least one of
    coordinate_2d or parent_region_id must be provided.
    
    Example:
    {
        "iic300": {
            "coordinate_2d": [-514, 114],
            "parent_region_id": "region_1"
        },
        "iic400": {
            "coordinate_2d": [-490, -82]
        },
        "iic500": {
            "parent_region_id": "region_2"  # coordinates optional
        }
    }
    """
    
    # Use RootModel for Pydantic v2 compatibility
    root: Dict[str, Dict[str, Any]] = Field(
        description="Dictionary mapping cortical area IDs to their new coordinates and optional parent region"
    )





# ===== Morphology Schemas =====


class MorphologyListResponse(BaseModel):
    """Response model for morphology list."""

    morphologies: List[Dict[str, Any]]


class MorphologyInfoResponse(BaseModel):
    """Response model for morphology information."""

    morphology: Dict[str, Any]


class CreateMorphologyRequest(BaseModel):
    """Request model for creating morphology."""

    morphology_data: Dict[str, Any]
    dimension_sensitive: Optional[bool] = Field(
        None,
        description="Whether this morphology is sensitive to cortical area dimensions. "
        "If not provided, will auto-detect based on morphology type: "
        "patterns/vectors=False, functions=True",
    )


class UpdateMorphologyRequest(BaseModel):
    """Request model for updating morphology."""

    morphology_id: str
    updates: Dict[str, Any]


class MorphologyNameRequest(BaseModel):
    """Request model for operations requiring morphology name."""

    morphology_name: str


class DirectMorphologyRequest(BaseModel):
    """Request model for direct morphology creation with client format."""

    morphology_name: str
    morphology_parameters: Dict[str, Any]
    morphology_type: str
    dimension_sensitive: Optional[bool] = Field(
        None,
        description="Whether this morphology is sensitive to cortical area dimensions. "
        "If not provided, will auto-detect based on morphology type: "
        "patterns/vectors=False, functions=True",
    )


class MorphologyPropertiesResponse(BaseModel):
    """Response model for morphology properties."""

    morphology_name: str
    type: str
    class_: Optional[str] = (
        None  # Using class_ since class is reserved keyword
    )
    parameters: Dict[str, Any]
    source: Optional[str] = None


class MorphologyUsageResponse(BaseModel):
    """Response model for morphology usage."""

    usage: List[
        List[str]
    ]  # List of [source_cortical_area, destination_cortical_area] pairs


# ===== Cortical Mapping Schemas =====


class CorticalMappingPropertiesRequest(BaseModel):
    """Request model for cortical mapping properties between two areas."""

    src_cortical_area: str
    dst_cortical_area: str


class CorticalMappingConnection(BaseModel):
    """Model for a single cortical mapping connection."""

    morphology_id: str
    morphology_scalar: List[int]  # Should be [x, y, z] coordinates
    postSynapticCurrent_multiplier: float
    plasticity_flag: bool
    plasticity_constant: float
    ltp_multiplier: float
    ltd_multiplier: float
    


class CorticalMappingPropertiesResponse(BaseModel):
    """Response model for cortical mapping properties."""

    connections: List[CorticalMappingConnection]


class UpdateCorticalMappingPropertiesRequest(BaseModel):
    """Request model for updating cortical mapping properties between two
    areas."""

    src_cortical_area: str
    dst_cortical_area: str
    mapping_string: List[Dict[str, Any]]  # List of connection dictionaries


class UpdateCorticalMappingPropertiesResponse(SuccessResponse):
    """Response for updating cortical mapping properties with region context.

    Extends the standard success response by including the normalized brain
    region objects for the source and destination cortical areas (when available).
    """

    src_region: Optional[Dict[str, Any]] = None
    dst_region: Optional[Dict[str, Any]] = None


class CreateCorticalMappingRequest(BaseModel):
    """Request model for creating a new cortical mapping between two areas."""

    src_cortical_area: str
    dst_cortical_area: str
    morphology_id: str
    morphology_scalar: List[int] = [1, 1, 1]  # Default [x, y, z] multipliers
    postSynapticCurrent_multiplier: float = 1.0
    plasticity_flag: bool = False
    plasticity_constant: float = 1.0
    ltp_multiplier: float = 1.0
    ltd_multiplier: float = 1.0


# ===== Monitoring Schemas =====


class MonitoringDataResponse(BaseModel):
    """Response model for monitoring data."""

    data: Dict[str, Any]


class SystemMetricsResponse(BaseModel):
    """Response model for system metrics."""

    metrics: Dict[str, Any]


class PerformanceStatsResponse(BaseModel):
    """Response model for performance statistics."""

    stats: Dict[str, Any]


# ===== Simulation Schemas =====


class SimulationStatusResponse(BaseModel):
    """Response model for simulation status."""

    status: str
    is_running: bool
    config: Optional[Dict[str, Any]] = None


class SimulationConfigRequest(BaseModel):
    """Request model for simulation configuration."""

    config: Dict[str, Any]


class SimulationStatsResponse(BaseModel):
    """Response model for simulation statistics."""

    stats: Dict[str, Any]


# ===== Agent Schemas =====


class AgentListResponse(RootModel[List[str]]):
    """Response model for FEAGI agent list."""

    root: List[str]  # Direct list of agent IDs


class AgentInfoResponse(BaseModel):
    """Response model for agent information."""

    agent_info: Dict[str, Any]


class AgentConfigRequest(BaseModel):
    """Request model for agent configuration."""

    agent_id: str
    config: Dict[str, Any]


class AgentRegistrationRequest(BaseModel):
    """Request model for agent registration."""

    agent_type: str
    agent_id: str
    agent_data_port: int
    agent_version: str
    controller_version: str
    capabilities: Dict[str, Any]
    agent_ip: Optional[str] = (
        None  # If not provided, will be extracted from request
    )
    # Optional metadata for additional agent-provided info
    metadata: Optional[Dict[str, Any]] = None


class AgentDeregistrationRequest(BaseModel):
    """Request model for agent deregistration."""

    agent_id: str


class AgentPropertiesRequest(BaseModel):
    """Request model for getting agent properties."""

    agent_id: str


class AgentPropertiesResponse(BaseModel):
    """Response model for agent properties."""

    agent_type: str
    agent_ip: str
    agent_data_port: int
    agent_router_address: str
    agent_version: str
    controller_version: str
    capabilities: Dict[str, Any]


# ===== Training Schemas =====


class TrainingStatusResponse(BaseModel):
    """Response model for training status."""

    status: str
    progress: Optional[float] = None
    config: Optional[Dict[str, Any]] = None


class TrainingConfigRequest(BaseModel):
    """Request model for training configuration."""

    config: Dict[str, Any]


class TrainingStatsResponse(BaseModel):
    """Response model for training statistics."""

    stats: Dict[str, Any]


# ===== Neuroplasticity Schemas =====


class NeuroplasticityRulesResponse(BaseModel):
    """Response model for neuroplasticity rules."""

    rules: List[Dict[str, Any]]


class NeuroplasticityStatsResponse(BaseModel):
    """Response model for neuroplasticity statistics."""

    stats: Dict[str, Any]


class UpdateNeuroplasticityRequest(BaseModel):
    """Request model for updating neuroplasticity rules."""

    rule_id: str
    updates: Dict[str, Any]


# ===== Insights Schemas =====


class InsightsDataResponse(BaseModel):
    """Response model for insights data."""

    insights: Dict[str, Any]


class AnalyticsResponse(BaseModel):
    """Response model for analytics data."""

    analytics: Dict[str, Any]


# ===== Network Schemas =====


class NetworkStatusResponse(BaseModel):
    """Response model for network status."""

    status: Dict[str, Any]


class NetworkConfigRequest(BaseModel):
    """Request model for network configuration."""

    config: Dict[str, Any]


# ===== Evolution Schemas =====


class EvolutionStatusResponse(BaseModel):
    """Response model for evolution status."""

    status: str
    generation: Optional[int] = None
    config: Optional[Dict[str, Any]] = None


class EvolutionConfigRequest(BaseModel):
    """Request model for evolution configuration."""

    config: Dict[str, Any]


# ===== Input/Output Schemas =====


class InputSourcesResponse(BaseModel):
    """Response model for input sources."""

    sources: List[Dict[str, Any]]


class OutputTargetsResponse(BaseModel):
    """Response model for output targets."""

    targets: List[Dict[str, Any]]


class IOConfigRequest(BaseModel):
    """Request model for input/output configuration."""

    config: Dict[str, Any]


# ===== Common Parameter Schemas =====


class IdRequest(BaseModel):
    """Generic request model for operations requiring an ID."""

    id: str


class FileUploadRequest(BaseModel):
    """Request model for file upload operations."""

    file_data: Dict[str, Any]


class PathRequest(BaseModel):
    """Request model for path-based operations."""

    path: str


# ===== Agent Stimulation Schemas =====


class ManualStimulationRequest(BaseModel):
    """Request model for manual neural stimulation across multiple cortical
    areas.

    Example payload:
    {
        "stimulation_payload": {
            "_power": [[1, 0, 0], [2, 4, 3]],
            "cx3212": [[1, 1, 0], [12, 24, 33], [0, 0, 0]]
        }
    }
    """

    stimulation_payload: Dict[str, List[List[int]]] = Field(
        description="Dictionary mapping cortical area IDs to lists of [x, y, z] coordinates"
    )


# ===== Memory Usage Schemas =====


class MemoryComponentInfo(BaseModel):
    """Memory information for a component (neurons or synapses)."""

    count: int = Field(description="Number of items")
    size_bytes: int = Field(description="Memory size in bytes")
    size_human: str = Field(
        description="Human-readable memory size (e.g., '1.2 KB')"
    )
    avg_bytes_per_item: float = Field(
        description="Average memory per item in bytes"
    )
    avg_human_per_item: str = Field(
        description="Human-readable average memory per item (e.g., '49 B')"
    )


class SynapseMemoryBreakdown(BaseModel):
    """Memory breakdown for synapses by type."""

    incoming: MemoryComponentInfo = Field(
        description="Synapses coming into this area from other areas"
    )
    outgoing: MemoryComponentInfo = Field(
        description="Synapses going from this area to other areas"
    )
    internal: MemoryComponentInfo = Field(
        description="Synapses within the area (recurrent connections)"
    )


class TotalMemoryInfo(BaseModel):
    """Total memory usage information."""

    size_bytes: int = Field(description="Total memory size in bytes")
    size_human: str = Field(
        description="Human-readable total memory size (e.g., '5.7 MB')"
    )


class CorticalAreaMemoryUsageResponse(BaseModel):
    """Response model for cortical area memory usage breakdown."""

    cortical_id: str = Field(description="The cortical area ID")
    neurons: MemoryComponentInfo = Field(
        description="Memory usage for all neurons in the area"
    )
    synapses: SynapseMemoryBreakdown = Field(
        description="Memory usage breakdown for synapses"
    )
    total: TotalMemoryInfo = Field(
        description="Total memory usage (neurons + all synapses)"
    )
