"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""FEAGI v1 Cortical Mapping API"""
from typing import Any, Dict, List

from feagi.api.core.services.core_api_service import CoreAPIService

from .decorators import endpoint
from .schemas import (
    CorticalMappingConnection,
    CorticalMappingPropertiesRequest,
    SuccessResponse,
)


def cortical_mapping_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="cortical_mapping",
    )


class CorticalMappingAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @cortical_mapping_endpoint("GET", "/mapping", response_model=Dict[str, Any])
    async def get_cortical_mapping(self) -> Dict[str, Any]:
        mapping = self.core_api_service.get_cortical_mapping()
        return mapping

    @cortical_mapping_endpoint("PUT", "/mapping", response_model=SuccessResponse)
    async def update_cortical_mapping(self, mapping: Dict[str, Any]) -> SuccessResponse:
        success = self.core_api_service.update_cortical_mapping(mapping)
        if not success:
            raise ValueError("Failed to update cortical mapping")
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
                connections.append(connection)

            return connections
        except Exception as e:
            raise ValueError(f"Failed to get mapping properties: {str(e)}")


def create_cortical_mapping_api(core_api_service: CoreAPIService) -> CorticalMappingAPI:
    return CorticalMappingAPI(core_api_service)
