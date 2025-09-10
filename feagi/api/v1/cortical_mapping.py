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

FEAGI v1 Cortical Mapping API
"""

from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

from .decorators import endpoint
from .schemas import (
    CorticalMappingConnection,
    CorticalMappingPropertiesRequest,
    CreateCorticalMappingRequest,
    SuccessResponse,
    UpdateCorticalMappingPropertiesRequest,
    UpdateCorticalMappingPropertiesResponse,
)


def cortical_mapping_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods,
        path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="cortical_mapping",
    )


class CorticalMappingAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @cortical_mapping_endpoint(
        "GET", "/mapping", response_model=Dict[str, Any]
    )
    async def get_cortical_mapping(self) -> Dict[str, Any]:
        mapping = self.core_api_service.get_cortical_mapping()
        return mapping

    @cortical_mapping_endpoint(
        "POST",
        "/mapping",
        request_model=CreateCorticalMappingRequest,
        response_model=SuccessResponse,
    )
    async def create_cortical_mapping(
        self, request: CreateCorticalMappingRequest
    ) -> SuccessResponse:
        """Create a new cortical mapping between two cortical areas."""
        try:
            # Convert request to the format expected by the core API
            mapping_data = {
                request.src_cortical_area: {
                    request.dst_cortical_area: [
                        {
                            "morphology_id": request.morphology_id,
                            "morphology_scalar": request.morphology_scalar,
                            "postSynapticCurrent_multiplier": (
                                request.postSynapticCurrent_multiplier
                            ),
                            "plasticity_flag": request.plasticity_flag,
                            "plasticity_constant": request.plasticity_constant,
                            "ltp_multiplier": request.ltp_multiplier,
                            "ltd_multiplier": request.ltd_multiplier,
                        }
                    ]
                }
            }

            # Route through CoreAPIService to ensure proper hierarchical
            # genome -> connectome flow
            success = self.core_api_service.update_cortical_mapping(
                mapping_data
            )

            if not success:
                raise ValueError("Failed to create cortical mapping")

            # Trigger automatic I/O designation
            try:
                connectome = self.core_api_service.get_connectome()
                if connectome and hasattr(connectome, 'on_cortical_mapping_created'):
                    connectome.on_cortical_mapping_created(
                        request.src_cortical_area, 
                        request.dst_cortical_area
                    )
            except Exception as e:
                # Don't fail the mapping creation if I/O designation fails
                logger.warning(f"Failed to process automatic I/O designation: {e}")

            return SuccessResponse(
                message=(
                    f"Cortical mapping created successfully from "
                    f"{request.src_cortical_area} to "
                    f"{request.dst_cortical_area}"
                )
            )
        except Exception as e:
            raise ValueError(
                f"Failed to create cortical mapping: {str(e)}"
            ) from e

    @cortical_mapping_endpoint(
        "PUT", "/mapping", response_model=SuccessResponse
    )
    async def update_cortical_mapping(
        self, mapping: Dict[str, Any]
    ) -> SuccessResponse:
        """Update cortical mapping with new data."""
        success = self.core_api_service.update_cortical_mapping(mapping)
        if not success:
            raise ValueError("Failed to update cortical mapping") from None
        return SuccessResponse(message="Cortical mapping updated successfully")

    @cortical_mapping_endpoint(
        "POST",
        "/mapping_properties",
        request_model=CorticalMappingPropertiesRequest,
        response_model=List[CorticalMappingConnection],
    )
    async def get_mapping_properties(
        self, request: CorticalMappingPropertiesRequest
    ) -> List[CorticalMappingConnection]:
        """Get cortical mapping properties between two cortical areas."""
        try:
            properties = self.core_api_service.get_cortical_mapping_properties(
                request.src_cortical_area, request.dst_cortical_area
            )

            # Convert the response to the proper Pydantic models
            connections = []
            for prop in properties:
                connection = CorticalMappingConnection(
                    morphology_id=prop["morphology_id"],
                    morphology_scalar=prop["morphology_scalar"],
                    postSynapticCurrent_multiplier=prop[
                        "postSynapticCurrent_multiplier"
                    ],
                    plasticity_flag=prop["plasticity_flag"],
                    plasticity_constant=prop["plasticity_constant"],
                    ltp_multiplier=prop["ltp_multiplier"],
                    ltd_multiplier=prop["ltd_multiplier"],
                )
                # Attach brain region context for source and destination areas
                try:
                    regions = self.core_api_service.get_brain_regions()
                    region_by_id = {r.get("region_id"): r for r in regions}
                    # Determine area->region via hierarchy
                    cm = self.core_api_service.get_connectome_manager()
                    src_region_id = None
                    dst_region_id = None
                    if hasattr(cm, "brain_region_hierarchy"):
                        src_region_id = cm.brain_region_hierarchy.get_region_for_area(request.src_cortical_area)
                        dst_region_id = cm.brain_region_hierarchy.get_region_for_area(request.dst_cortical_area)
                    if src_region_id and src_region_id in region_by_id:
                        connection.src_region = region_by_id[src_region_id]
                    if dst_region_id and dst_region_id in region_by_id:
                        connection.dst_region = region_by_id[dst_region_id]
                except Exception as e:
                    logger.warning(f"Failed to attach brain region context: {e}")
                connections.append(connection)

            return connections
        except Exception as e:
            raise ValueError(
                f"Failed to get mapping properties: {str(e)}"
            ) from e

    @cortical_mapping_endpoint(
        "PUT",
        "/mapping_properties",
        request_model=UpdateCorticalMappingPropertiesRequest,
        response_model=UpdateCorticalMappingPropertiesResponse,
    )
    async def update_mapping_properties(
        self, request: UpdateCorticalMappingPropertiesRequest
    ) -> UpdateCorticalMappingPropertiesResponse:
        """Update cortical mapping properties between two cortical areas."""
        try:
            success = self.core_api_service.update_cortical_mapping_properties(
                request.src_cortical_area,
                request.dst_cortical_area,
                request.mapping_string,
            )

            if not success:
                raise ValueError(
                    "Failed to update cortical mapping properties"
                )

            # Build region context for response (optional enrichment)
            src_region_obj = None
            dst_region_obj = None
            try:
                regions = self.core_api_service.get_brain_regions()
                region_by_id = {r.get("region_id"): r for r in regions}
                cm = self.core_api_service.get_connectome_manager()
                src_region_id = None
                dst_region_id = None
                if hasattr(cm, "brain_region_hierarchy"):
                    src_region_id = cm.brain_region_hierarchy.get_region_for_area(request.src_cortical_area)
                    dst_region_id = cm.brain_region_hierarchy.get_region_for_area(request.dst_cortical_area)
                if src_region_id and src_region_id in region_by_id:
                    src_region_obj = region_by_id[src_region_id]
                if dst_region_id and dst_region_id in region_by_id:
                    dst_region_obj = region_by_id[dst_region_id]
            except Exception as e:
                logger.warning(f"Failed to build brain region context for update response: {e}")

            return UpdateCorticalMappingPropertiesResponse(
                message=(
                    f"Cortical mapping properties updated successfully from "
                    f"{request.src_cortical_area} to "
                    f"{request.dst_cortical_area}"
                ),
                src_region=src_region_obj,
                dst_region=dst_region_obj,
            )

        except Exception as e:
            raise ValueError(
                f"Failed to update mapping properties: {str(e)}"
            ) from e

    @cortical_mapping_endpoint(
        "DELETE",
        "/mapping",
        request_model=CorticalMappingPropertiesRequest,
        response_model=SuccessResponse,
    )
    async def delete_cortical_mapping(
        self, request: CorticalMappingPropertiesRequest
    ) -> SuccessResponse:
        """Delete cortical mapping and all associated synapses between two
        cortical areas."""
        try:
            success = self.core_api_service.delete_cortical_mapping(
                request.src_cortical_area,
                request.dst_cortical_area,
            )

            if not success:
                raise ValueError("Failed to delete cortical mapping")

            return SuccessResponse(
                message=(
                    f"Cortical mapping deleted successfully from "
                    f"{request.src_cortical_area} to "
                    f"{request.dst_cortical_area}"
                )
            )

        except Exception as e:
            raise ValueError(
                f"Failed to delete cortical mapping: {str(e)}"
            ) from e


def create_cortical_mapping_api(
    core_api_service: CoreAPIService,
) -> CorticalMappingAPI:
    return CorticalMappingAPI(core_api_service)
