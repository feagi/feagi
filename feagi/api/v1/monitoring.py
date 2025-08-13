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

"""
FEAGI v1 Monitoring API - Single Source of Truth

This module contains the ONLY definitions of monitoring API endpoints.
Each endpoint is decorated to automatically register for ALL transport protocols
(FastAPI, ZMQ, gRPC, etc.) ensuring perfect consistency across transports.

NO endpoint definitions should exist anywhere else - this is the single source of truth.
"""


from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.utils.logger import setup_logger

from .decorators import endpoint
from .schemas import (
    MonitoringDataResponse,
    PerformanceStatsResponse,
    SystemMetricsResponse,
)

logger = setup_logger(__name__)


# Define the convenience decorator for monitoring endpoints
def monitoring_endpoint(
    methods, path, request_model=None, response_model=None, description=None
):
    """Convenience decorator for monitoring endpoints."""
    return endpoint(
        methods=methods,
        path=path,
        request_model=request_model,
        response_model=response_model,
        description=description,
        module="monitoring",
    )


class MonitoringAPI:
    """
    Monitoring API - Single Source of Truth for ALL Transports

    Each method in this class is decorated to automatically register
    the endpoint for FastAPI, ZMQ, and any future transport protocols.

    This ensures identical behavior across all transports with zero duplication.
    """

    def __init__(self, core_api_service: CoreAPIService):
        """Initialize with core API service dependency."""
        self.core_api_service = core_api_service

    # ===== System Monitoring =====

    @monitoring_endpoint(
        "GET", "/metrics", response_model=SystemMetricsResponse
    )
    async def get_system_metrics(self) -> SystemMetricsResponse:
        """Get current system metrics."""
        try:
            metrics = self.core_api_service.get_system_metrics()
            return SystemMetricsResponse(metrics=metrics)
        except Exception as e:
            logger.error(f"Error getting system metrics: {e}")
            raise ValueError(f"Failed to get system metrics: {str(e)}") from e

    @monitoring_endpoint(
        "GET", "/performance", response_model=PerformanceStatsResponse
    )
    async def get_performance_stats(self) -> PerformanceStatsResponse:
        """Get performance statistics."""
        try:
            stats = self.core_api_service.get_performance_stats()
            return PerformanceStatsResponse(stats=stats)
        except Exception as e:
            logger.error(f"Error getting performance stats: {e}")
            raise ValueError(
                f"Failed to get performance stats: {str(e)}"
            ) from e

    @monitoring_endpoint("GET", "/data", response_model=MonitoringDataResponse)
    async def get_monitoring_data(self) -> MonitoringDataResponse:
        """Get general monitoring data."""
        try:
            data = self.core_api_service.get_monitoring_data()
            return MonitoringDataResponse(data=data)
        except Exception as e:
            logger.error(f"Error getting monitoring data: {e}")
            raise ValueError(f"Failed to get monitoring data: {str(e)}") from e


# ===== Factory Function =====


def create_monitoring_api(core_api_service: CoreAPIService) -> MonitoringAPI:
    """
    Factory function to create a MonitoringAPI instance.

    This function can be used by transport adapters to get a configured
    MonitoringAPI instance with the required dependencies.
    """
    return MonitoringAPI(core_api_service)
