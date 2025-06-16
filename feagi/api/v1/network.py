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

"""FEAGI v1 Network API"""

from feagi.api.core.services.core_api_service import CoreAPIService

from .decorators import endpoint
from .schemas import NetworkConfigRequest, NetworkStatusResponse, SuccessResponse


def network_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="network",
    )


class NetworkAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @network_endpoint("GET", "/status", response_model=NetworkStatusResponse)
    async def get_network_status(self) -> NetworkStatusResponse:
        status = self.core_api_service.get_network_status()
        return NetworkStatusResponse(status=status)

    @network_endpoint(
        "POST",
        "/configure",
        request_model=NetworkConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_network(self, request: NetworkConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_network(request.config)
        if not success:
            raise ValueError("Failed to configure network")
        return SuccessResponse(message="Network configured successfully")


def create_network_api(core_api_service: CoreAPIService) -> NetworkAPI:
    return NetworkAPI(core_api_service)
