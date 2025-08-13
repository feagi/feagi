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

"""FEAGI v1 Insights API"""

from feagi.api.core.services.core_api_service import CoreAPIService

from .decorators import endpoint
from .schemas import AnalyticsResponse, InsightsDataResponse


def insights_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="insights",
    )


class InsightsAPI:
    def __init__(self, core_api_service: CoreAPIService):
        self.core_api_service = core_api_service

    @insights_endpoint("GET", "/data", response_model=InsightsDataResponse)
    async def get_insights_data(self) -> InsightsDataResponse:
        insights = self.core_api_service.get_insights_data()
        return InsightsDataResponse(insights=insights)

    @insights_endpoint("GET", "/analytics", response_model=AnalyticsResponse)
    async def get_analytics(self) -> AnalyticsResponse:
        analytics = self.core_api_service.get_analytics_data()
        return AnalyticsResponse(analytics=analytics)


def create_insights_api(core_api_service: CoreAPIService) -> InsightsAPI:
    return InsightsAPI(core_api_service)
