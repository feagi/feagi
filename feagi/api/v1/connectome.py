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
FEAGI v1 Connectome API - Single Source of Truth

This module contains the ONLY definitions of connectome API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""

import json
from datetime import datetime
from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    BatchNeuronCreationRequest,
    BatchNeuronCreationResponse,
    BatchSynapseCreationRequest,
    BatchSynapseCreationResponse,
    ConnectomeDimensionsResponse,
    ConnectomePathResponse,
    CorticalAreaInfoResponse,
    CorticalAreasListResponse,
    CorticalStatsResponse,
    FileUploadRequest,
    FireQueueResponse,
    NeuronMappingsResponse,
    NeuronPropertiesResponse,
    PlasticityInfoResponse,
    SuccessResponse,
)

logger = setup_logger(__name__)


# Define the convenience decorator for connectome endpoints
def connectome_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for connectome endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="connectome",
    )


class ConnectomeAPI:
    """
    Connectome API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== Cortical Areas Information =====

    @connectome_endpoint(
        "GET", "/cortical_areas/list/summary", response_model=List[str]
    )
    async def get_cortical_areas_summary(self) -> List[str]:
        """Get a summary list of cortical area IDs."""
        try:
            areas = self.core_api_service.get_cortical_areas()
            if not areas:
                raise ValueError(
                    "No active genome found! Load a genome first."
                )
            return [area["id"] for area in areas]
        except Exception as e:
            logger.error(f"Error getting cortical areas summary: {e}")
            raise ValueError(
                f"Failed to get cortical areas summary: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET", "/cortical_areas/list/transforming", response_model=List[str]
    )
    async def get_transforming_cortical_areas(self) -> List[str]:
        """Get a list of transforming cortical areas."""
        try:
            return self.core_api_service.get_transforming_areas()
        except Exception as e:
            logger.error(f"Error getting transforming cortical areas: {e}")
            raise ValueError(
                f"Failed to get transforming cortical areas: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET",
        "/cortical_areas/list/detailed",
        response_model=CorticalAreasListResponse,
    )
    async def get_cortical_areas_detailed(self) -> CorticalAreasListResponse:
        """Get detailed information about all cortical areas."""
        try:
            areas = self.core_api_service.get_cortical_areas()
            if not areas:
                raise ValueError(
                    "No active genome found! Load a genome first."
                )
            return CorticalAreasListResponse(areas=areas)
        except Exception as e:
            logger.error(f"Error getting detailed cortical areas: {e}")
            raise ValueError(
                f"Failed to get detailed cortical areas: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET",
        "/cortical_info/{cortical_area}",
        response_model=CorticalAreaInfoResponse,
    )
    async def get_cortical_info(
        self, cortical_area: str
    ) -> CorticalAreaInfoResponse:
        """Get detailed information about a specific cortical area."""
        try:
            area = self.core_api_service.get_cortical_area(cortical_area)
            if not area:
                raise ValueError("Requested cortical area not found!")
            return CorticalAreaInfoResponse(area_info=area)
        except Exception as e:
            logger.error(f"Error getting cortical info: {e}")
            raise ValueError(f"Failed to get cortical info: {str(e)}") from e

    @connectome_endpoint(
        "GET", "/fire_queue/{cortical_area}", response_model=FireQueueResponse
    )
    async def get_fire_queue(self, cortical_area: str) -> FireQueueResponse:
        """Get fire queue data for a specific cortical area."""
        try:
            fire_queue_data = self.core_api_service.get_area_fire_queue(
                cortical_area
            )
            return FireQueueResponse(fire_queue=fire_queue_data)
        except Exception as e:
            logger.error(f"Error getting fire queue for {cortical_area}: {e}")
            raise ValueError(f"Failed to get fire queue: {str(e)}") from e

    @connectome_endpoint(
        "GET",
        "/neuron/{neuron_id}/properties",
        response_model=NeuronPropertiesResponse,
    )
    async def get_neuron_properties(
        self, neuron_id: int
    ) -> NeuronPropertiesResponse:
        """Get detailed properties of a specific neuron including refractory
        counter."""
        try:
            properties = self.core_api_service.get_neuron_properties(neuron_id)
            if not properties:
                raise ValueError(f"Neuron {neuron_id} not found!")
            return NeuronPropertiesResponse(**properties)
        except Exception as e:
            logger.error(
                f"Error getting neuron properties for {neuron_id}: {e}"
            )
            raise ValueError(
                f"Failed to get neuron properties: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET",
        "/cortical_area/{cortical_id}/neurons",
        response_model=List[Dict[str, Any]],
    )
    async def get_cortical_area_neurons(
        self, cortical_id: str
    ) -> List[Dict[str, Any]]:
        """Get all neurons in a specific cortical area with their
        properties."""
        try:
            neurons = self.core_api_service.get_cortical_area_neurons(
                cortical_id
            )
            if neurons is None:
                raise ValueError(f"Cortical area '{cortical_id}' not found!")
            return neurons
        except Exception as e:
            logger.error(
                f"Error getting neurons for cortical area {cortical_id}: {e}"
            )
            raise ValueError(
                f"Failed to get neurons for cortical area: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET", "/area_neurons", response_model=List[Dict[str, Any]]
    )
    async def get_area_neurons_by_query(
        self, cortical_id: str
    ) -> List[Dict[str, Any]]:
        """Get all neurons in a cortical area (query parameter version)."""
        try:
            logger.info(
                f"DEBUG: get_area_neurons_by_query called with cortical_id: {cortical_id}"
            )
            neurons = self.core_api_service.get_cortical_area_neurons(
                cortical_id
            )
            if neurons is None:
                raise ValueError(f"Cortical area '{cortical_id}' not found!")
            logger.info(
                f"DEBUG: Successfully retrieved {len(neurons)} neurons for {cortical_id}"
            )
            return neurons
        except Exception as e:
            logger.error(
                f"Error getting neurons for cortical area {cortical_id}: {e}"
            )
            raise ValueError(
                f"Failed to get neurons for cortical area: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET", "/neuron_properties", response_model=NeuronPropertiesResponse
    )
    async def get_neuron_properties_by_query(
        self, neuron_id: int
    ) -> NeuronPropertiesResponse:
        """Get neuron properties (query parameter version)."""
        try:
            logger.info(
                f"DEBUG: get_neuron_properties_by_query called with neuron_id: {neuron_id}"
            )
            properties = self.core_api_service.get_neuron_properties(neuron_id)
            if not properties:
                raise ValueError(f"Neuron {neuron_id} not found!")
            logger.info(
                f"DEBUG: Successfully retrieved properties for neuron {neuron_id}"
            )
            return NeuronPropertiesResponse(**properties)
        except Exception as e:
            logger.error(
                f"Error getting neuron properties for {neuron_id}: {e}"
            )
            raise ValueError(
                f"Failed to get neuron properties: {str(e)}"
            ) from e

    # ===== Plasticity and Properties =====

    @connectome_endpoint(
        "GET", "/plasticity", response_model=PlasticityInfoResponse
    )
    async def get_plasticity_info(self) -> PlasticityInfoResponse:
        """Get neuroplasticity information."""
        try:
            plasticity_info = self.core_api_service.get_plasticity_info()
            return PlasticityInfoResponse(plasticity_info=plasticity_info)
        except Exception as e:
            logger.error(f"Error getting plasticity info: {e}")
            raise ValueError(f"Failed to get plasticity info: {str(e)}") from e

    @connectome_endpoint("GET", "/path", response_model=ConnectomePathResponse)
    async def get_connectome_path(self) -> ConnectomePathResponse:
        """Get connectome file system path."""
        try:
            path = self.core_api_service.get_temp_path()
            return ConnectomePathResponse(path=path)
        except Exception as e:
            logger.error(f"Error getting connectome path: {e}")
            raise ValueError(f"Failed to get connectome path: {str(e)}") from e

    @connectome_endpoint(
        "GET",
        "/properties/dimensions",
        response_model=ConnectomeDimensionsResponse,
    )
    async def get_connectome_dimensions(self) -> ConnectomeDimensionsResponse:
        """Get the overall dimensions of the connectome."""
        try:
            dimensions = self.core_api_service.get_connectome_dimensions()
            return ConnectomeDimensionsResponse(dimensions=dimensions)
        except Exception as e:
            logger.error(f"Error getting connectome dimensions: {e}")
            raise ValueError(
                f"Failed to get connectome dimensions: {str(e)}"
            ) from e

    @connectome_endpoint(
        "GET", "/properties/mappings", response_model=NeuronMappingsResponse
    )
    async def get_connectome_mappings(self) -> NeuronMappingsResponse:
        """Get neuron mappings for connectome visualization."""
        try:
            mappings = self.core_api_service.get_neuron_mappings()
            return NeuronMappingsResponse(mappings=mappings)
        except Exception as e:
            logger.error(f"Error getting connectome mappings: {e}")
            raise ValueError(
                f"Failed to get connectome mappings: {str(e)}"
            ) from e

    # ===== Statistics =====

    @connectome_endpoint(
        "GET",
        "/stats/cortical/cumulative/{cortical_area}",
        response_model=CorticalStatsResponse,
    )
    async def get_cortical_stats(
        self, cortical_area: str
    ) -> CorticalStatsResponse:
        """Get cumulative statistics for a specific cortical area."""
        try:
            stats = self.core_api_service.get_cortical_area_stats(
                cortical_area
            )
            if not stats:
                raise ValueError(
                    f"Statistics for cortical area {cortical_area} not found"
                )
            return CorticalStatsResponse(stats=stats)
        except Exception as e:
            logger.error(f"Error getting cortical stats: {e}")
            raise ValueError(f"Failed to get cortical stats: {str(e)}") from e

    # ===== Download Operations =====

    @connectome_endpoint(
        "GET",
        "/download-cortical-area/{cortical_area}",
        response_model=Dict[str, Any],
    )
    async def download_cortical_area(
        self, cortical_area: str
    ) -> Dict[str, Any]:
        """Download a specific cortical area as a JSON file."""
        try:
            area = self.core_api_service.get_cortical_area(cortical_area)
            if not area:
                raise ValueError("Requested cortical area not found!")

            # Generate a filename with timestamp
            file_name = f"connectome_{cortical_area}_{datetime.now().strftime('%Y_%m_%d-%I:%M:%S_%p')}.json"

            # Create a temporary file to store the cortical area data
            temp_dir = self.core_api_service.get_temp_path()
            temp_file_path = f"{temp_dir}/{file_name}"

            # Serialize the cortical area to the temp file
            with open(temp_file_path, "w") as f:
                json.dump(area, f, indent=2)

            #  Return file info (transport adapters will handle actual file
            #  serving)
            return {
                "file_path": temp_file_path,
                "file_name": file_name,
                "media_type": "application/json",
                "area_data": area,
            }
        except Exception as e:
            logger.error(f"Error downloading cortical area: {e}")
            raise ValueError(
                f"Failed to download cortical area: {str(e)}"
            ) from e

    @connectome_endpoint("GET", "/download", response_model=Dict[str, Any])
    async def download_connectome(self) -> Dict[str, Any]:
        """Download the complete connectome."""
        # TODO: Implement connectome serialization in CoreAPIService
        raise NotImplementedError(
            "Connectome download is not yet implemented."
        )

    # ===== Upload Operations =====

    @connectome_endpoint(
        "POST", "/upload-cortical-area", response_model=SuccessResponse
    )
    async def upload_cortical_area(
        self, file_data: FileUploadRequest
    ) -> SuccessResponse:
        """Upload a cortical area JSON file."""
        try:
            # Extract file content
            content = file_data.file_data.get("content", "")
            if not content:
                raise ValueError("No file content provided")

            # Parse the cortical area data
            if " = " in content:
                connectome_str = content.split(" = ")[1]
                from ast import literal_eval

                cortical_area_data = literal_eval(connectome_str)
            else:
                cortical_area_data = json.loads(content)

            success = self.core_api_service.import_cortical_area(
                cortical_area_data
            )
            if not success:
                raise ValueError("Failed to import cortical area")

            return SuccessResponse(
                message="Cortical area imported successfully"
            )
        except Exception as e:
            logger.error(f"Error uploading cortical area: {e}")
            raise ValueError(
                f"Failed to upload cortical area: {str(e)}"
            ) from e

    @connectome_endpoint("POST", "/upload", response_model=SuccessResponse)
    async def upload_connectome(
        self, file_data: FileUploadRequest
    ) -> SuccessResponse:
        """Upload a complete connectome file."""
        # TODO: Implement connectome upload/restore in CoreAPIService
        raise NotImplementedError("Connectome upload is not yet implemented.")

    # ===== Batch Operations =====

    @connectome_endpoint(
        "POST",
        "/neurons/batch",
        request_model=BatchNeuronCreationRequest,
        response_model=BatchNeuronCreationResponse,
    )
    async def batch_create_neurons(
        self, request: BatchNeuronCreationRequest
    ) -> BatchNeuronCreationResponse:
        """Create multiple neurons in a batch operation."""
        try:
            neuron_ids = self.core_api_service.batch_create_neurons(
                area_id=request.area_id,
                positions=request.positions,
                properties=request.properties,
            )

            if not neuron_ids:
                raise ValueError("Failed to create neurons")

            return BatchNeuronCreationResponse(
                created_neurons=neuron_ids, count=len(neuron_ids)
            )
        except Exception as e:
            logger.error(f"Error in batch neuron creation: {e}")
            raise ValueError(f"Failed to create neurons: {str(e)}") from e

    @connectome_endpoint(
        "POST",
        "/synapses/batch",
        request_model=BatchSynapseCreationRequest,
        response_model=BatchSynapseCreationResponse,
    )
    async def batch_create_synapses(
        self, request: BatchSynapseCreationRequest
    ) -> BatchSynapseCreationResponse:
        """Create multiple synapses in a batch operation."""
        try:
            success_count = self.core_api_service.batch_create_synapses(
                request.connections
            )
            return BatchSynapseCreationResponse(created_synapses=success_count)
        except Exception as e:
            logger.error(f"Error in batch synapse creation: {e}")
            raise ValueError(f"Failed to create synapses: {str(e)}") from e


# ===== Factory Function =====


def create_connectome_api(core_api_service: CoreAPIService) -> ConnectomeAPI:
    """Factory function to create a ConnectomeAPI instance.

    This function can be used by transport adapters to get a configured
    ConnectomeAPI instance with the required dependencies.
    """
    return ConnectomeAPI(core_api_service)
