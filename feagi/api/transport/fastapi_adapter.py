"""
FastAPI Transport Adapter for FEAGI v1 API

This adapter provides HTTP REST API access to FEAGI's v1 business logic.
It translates HTTP requests to v1 API calls and returns HTTP responses.

The adapter ensures that HTTP clients get identical behavior to other
transport protocols (ZMQ, etc.) by using the same underlying v1 business logic.
"""

from fastapi import APIRouter, HTTPException, Depends
from fastapi.responses import JSONResponse
from typing import Dict, Any, Union

from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.system import create_system_api, SystemAPI
from feagi.api.v1.schemas import (
    UserPreferencesRequest, UserPreferencesResponse,
    VersionsResponse, HealthCheckResponse, ConfigurationResponse,
    InfluxDBTestResponse, CorticalAreaTypesResponse,
    SuccessResponse, ErrorResponse,
    RegistrationRequest, LogsRequest, SubscriberRequest,
    CircuitLibraryPathRequest
)
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)

# Create the FastAPI router
router = APIRouter()


def _get_system_api(core_api_service: CoreAPIService = Depends(get_core_api_service)) -> SystemAPI:
    """Dependency to get a SystemAPI instance."""
    return create_system_api(core_api_service)


def _handle_api_exception(e: Exception) -> JSONResponse:
    """Convert API exceptions to HTTP error responses."""
    if isinstance(e, ValueError):
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": str(e)}
        )
    else:
        logger.error(f"Unexpected API error: {e}")
        return JSONResponse(
            status_code=500,
            content={"status": "error", "message": "Internal server error"}
        )


# ===== User Preferences Endpoints =====

@router.get("/user_preferences", response_model=UserPreferencesResponse)
async def get_user_preferences(system_api: SystemAPI = Depends(_get_system_api)):
    """Get current user preferences."""
    try:
        return system_api.get_user_preferences()
    except Exception as e:
        return _handle_api_exception(e)


@router.put("/user_preferences", response_model=SuccessResponse)
async def update_user_preferences(
    request: UserPreferencesRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """Update user preferences."""
    try:
        return system_api.update_user_preferences(request)
    except Exception as e:
        return _handle_api_exception(e)


# ===== System Information Endpoints =====

@router.get("/versions", response_model=VersionsResponse)
def get_versions(system_api: SystemAPI = Depends(_get_system_api)):
    """Get system version information."""
    try:
        return system_api.get_versions()
    except Exception as e:
        return _handle_api_exception(e)


@router.get("/health_check", response_model=HealthCheckResponse)
async def get_health_check(system_api: SystemAPI = Depends(_get_system_api)):
    """Get comprehensive system health information."""
    try:
        return await system_api.get_health_check()
    except Exception as e:
        return _handle_api_exception(e)


@router.get("/configuration", response_model=ConfigurationResponse)
async def get_configuration(system_api: SystemAPI = Depends(_get_system_api)):
    """Get system configuration."""
    try:
        return system_api.get_configuration()
    except Exception as e:
        return _handle_api_exception(e)


# ===== External Services Endpoints =====

@router.get("/db/influxdb/test", response_model=InfluxDBTestResponse)
async def test_influxdb(system_api: SystemAPI = Depends(_get_system_api)):
    """Test InfluxDB connection."""
    try:
        return system_api.test_influxdb()
    except Exception as e:
        return _handle_api_exception(e)


# ===== System Configuration Endpoints =====

@router.post("/circuit_library_path", response_model=SuccessResponse)
async def set_circuit_library_path(
    request: CircuitLibraryPathRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """Set the circuit library path."""
    try:
        return system_api.set_circuit_library_path(request.path)
    except Exception as e:
        return _handle_api_exception(e)


@router.get("/cortical_area_types", response_model=CorticalAreaTypesResponse)
async def get_cortical_area_types(system_api: SystemAPI = Depends(_get_system_api)):
    """Get available cortical area types."""
    try:
        return system_api.get_cortical_area_types()
    except Exception as e:
        return _handle_api_exception(e)


# ===== System Control Endpoints =====

@router.post("/fcl_reset", response_model=SuccessResponse)
async def reset_fcl(system_api: SystemAPI = Depends(_get_system_api)):
    """Reset the Fire Candidate List."""
    try:
        return system_api.reset_fcl()
    except Exception as e:
        return _handle_api_exception(e)


# ===== Legacy/Placeholder Endpoints =====

@router.post("/register", response_model=SuccessResponse)
async def register_system(
    request: RegistrationRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """System registration (placeholder implementation)."""
    try:
        return system_api.register_system(request.registration_data)
    except Exception as e:
        return _handle_api_exception(e)


@router.post("/logs", response_model=SuccessResponse)
async def manage_logs(
    request: LogsRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """Manage system logs."""
    try:
        return system_api.manage_logs(request.log_data)
    except Exception as e:
        return _handle_api_exception(e)


@router.get("/beacon/subscribers")
async def get_beacon_subscribers(system_api: SystemAPI = Depends(_get_system_api)):
    """Get current beacon subscribers."""
    try:
        return system_api.get_beacon_subscribers()
    except Exception as e:
        return _handle_api_exception(e)


@router.post("/beacon/subscribe", response_model=SuccessResponse)
async def subscribe_to_beacon(
    request: SubscriberRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """Subscribe to beacon notifications."""
    try:
        return system_api.subscribe_to_beacon(request.subscriber_address)
    except Exception as e:
        return _handle_api_exception(e)


@router.delete("/beacon/unsubscribe", response_model=SuccessResponse)
async def unsubscribe_from_beacon(
    request: SubscriberRequest,
    system_api: SystemAPI = Depends(_get_system_api)
):
    """Unsubscribe from beacon notifications."""
    try:
        return system_api.unsubscribe_from_beacon(request.subscriber_address)
    except Exception as e:
        return _handle_api_exception(e)


# ===== Version Compatibility =====

@router.get("/version")
async def get_version(system_api: SystemAPI = Depends(_get_system_api)):
    """Get FEAGI version (legacy endpoint)."""
    try:
        versions = system_api.get_versions()
        return {"version": versions.feagi_core}
    except Exception as e:
        return _handle_api_exception(e) 