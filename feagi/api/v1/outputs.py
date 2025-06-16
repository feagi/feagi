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

"""FEAGI v1 Outputs API"""

from feagi.api.core.services.core_api_service import CoreAPIService

from .decorators import endpoint
from .schemas import IOConfigRequest, OutputTargetsResponse, SuccessResponse


def outputs_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="outputs",
    )


class OutputsAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @outputs_endpoint("GET", "/targets", response_model=OutputTargetsResponse)
    async def get_output_targets(self) -> OutputTargetsResponse:
        targets = self.core_api_service.get_output_targets()
        return OutputTargetsResponse(targets=targets)

    @outputs_endpoint(
        "POST",
        "/configure",
        request_model=IOConfigRequest,
        response_model=SuccessResponse,
    )
    async def configure_outputs(self, request: IOConfigRequest) -> SuccessResponse:
        success = self.core_api_service.configure_outputs(request.config)
        if not success:
            raise ValueError("Failed to configure outputs")
        return SuccessResponse(message="Outputs configured successfully")


def create_outputs_api(core_api_service: CoreAPIService) -> OutputsAPI:
    return OutputsAPI(core_api_service)
