"""
FEAGI v1 API Schemas

Transport-agnostic request and response schemas for FEAGI v1 API.
These schemas define the data structures used by all endpoints,
regardless of transport protocol (HTTP, ZMQ, etc.).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
from pydantic import BaseModel


# ===== System Schemas =====

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
    neuron_count: Optional[int] = None
    synapse_count: Optional[int] = None
    estimated_brain_size_in_MB: Optional[float] = None


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
    """Response model for genome upload operations."""
    success: bool
    message: str
    genome_number: Optional[int] = None
    loaded: Optional[dict] = None


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


# ===== Common Request Schemas =====

class CorticalIdRequest(BaseModel):
    """Request model for operations requiring cortical ID."""
    cortical_id: str


class CorticalNameRequest(BaseModel):
    """Request model for operations requiring cortical name."""
    cortical_name: str


class CorticalIdListRequest(BaseModel):
    """Request model for operations on multiple cortical areas."""
    cortical_ids: List[str]


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


class NeuronMappingsResponse(BaseModel):
    """Response model for neuron mappings."""
    mappings: Dict[str, Any]


class BatchNeuronCreationResponse(BaseModel):
    """Response model for batch neuron creation."""
    created_neurons: List[int]
    count: int


class BatchSynapseCreationResponse(BaseModel):
    """Response model for batch synapse creation."""
    created_synapses: int


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


class UpdateMorphologyRequest(BaseModel):
    """Request model for updating morphology."""
    morphology_id: str
    updates: Dict[str, Any]


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

class AgentListResponse(BaseModel):
    """Response model for FEAGI agent list."""
    agents: List[Dict[str, Any]]


class AgentInfoResponse(BaseModel):
    """Response model for agent information."""
    agent_info: Dict[str, Any]


class AgentConfigRequest(BaseModel):
    """Request model for agent configuration."""
    agent_id: str
    config: Dict[str, Any]


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