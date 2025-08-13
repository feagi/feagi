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
Debug Control API

Provides REST endpoints for controlling FEAGI debugging features at runtime,
including ZMQ traffic debugging, performance monitoring, and log control.
"""

from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from feagi.utils.zmq_debug import (
    DebugLevel,
    MessageType,
    enable_inbound_debug,
    enable_outbound_debug,
    get_debug_status,
    get_endpoint_stats,
    reset_debug_stats,
    set_console_output,
    set_debug_level,
    set_endpoint_filters,
    set_message_filters,
    set_rate_limit,
)

router = APIRouter(prefix="/debug", tags=["debug"])


# Request/Response Models


class ZMQDebugConfig(BaseModel):
    """ZMQ debug configuration model."""

    inbound_enabled: Optional[bool] = None
    outbound_enabled: Optional[bool] = None
    debug_level: Optional[str] = None  # off, minimal, headers, summary, full
    message_filters: Optional[List[str]] = None  # Empty list = all messages
    endpoint_filters: Optional[List[str]] = None  # Empty list = all endpoints
    rate_limit_per_second: Optional[int] = None
    console_output: Optional[bool] = None  # Enable console output

    class Config:
        # Make the model more permissive for Swagger UI
        extra = "ignore"  # Ignore extra fields
        # Add example for Swagger UI
        json_schema_extra = {
            "example": {
                "console_output": True,
                "debug_level": "summary",
                "message_filters": ["visualization"],
                "rate_limit_per_second": 10,
            }
        }


class ZMQDebugResponse(BaseModel):
    """ZMQ debug status response model."""

    inbound_enabled: bool
    outbound_enabled: bool
    debug_level: str
    message_filters: List[str]
    endpoint_filters: List[str]
    rate_limit_per_second: int
    console_output: bool
    stats: Dict[str, Any]


class EndpointStatsResponse(BaseModel):
    """Endpoint statistics response model."""

    endpoints: Dict[str, Dict[str, Any]]


# ZMQ Debug Control Endpoints


@router.get("/zmq/status", response_model=ZMQDebugResponse)
async def get_zmq_debug_status():
    """
    Get current ZMQ debugging status and configuration.

    Returns current settings including:
    - Inbound/outbound debugging state
    - Debug verbosity level
    - Active filters
    - Performance statistics
    """
    try:
        status = get_debug_status()
        return ZMQDebugResponse(**status)
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error getting debug status: {str(e)}"
        )


@router.post("/zmq/configure")
async def configure_zmq_debug(config: ZMQDebugConfig):
    """
    Configure ZMQ debugging settings at runtime.

    Allows dynamic control of:
    - Enable/disable inbound or outbound debugging
    - Change verbosity level (off, minimal, headers, summary, full)
    - Set message type filters (sensory, motor, visualization, control, rest)
    - Set endpoint filters (specific IP:port combinations)
    - Configure rate limiting

    Changes take effect immediately without restart.
    """
    try:
        # Track what we're configuring for better error messages
        configured_fields = []

        if config.inbound_enabled is not None:
            enable_inbound_debug(config.inbound_enabled)
            configured_fields.append(f"inbound_enabled={config.inbound_enabled}")

        if config.outbound_enabled is not None:
            enable_outbound_debug(config.outbound_enabled)
            configured_fields.append(f"outbound_enabled={config.outbound_enabled}")

        if config.debug_level is not None:
            if not isinstance(config.debug_level, str):
                raise HTTPException(
                    status_code=400,
                    detail=f"debug_level must be a string, got {type(config.debug_level).__name__}",
                )
            try:
                level = DebugLevel[config.debug_level.upper()]
                set_debug_level(level)
                configured_fields.append(f"debug_level={config.debug_level}")
            except KeyError:
                valid_levels = [level.name.lower() for level in DebugLevel]
                raise HTTPException(
                    status_code=400,
                    detail=f"Invalid debug level: '{config.debug_level}'. Valid levels: {valid_levels}",
                )

        if config.message_filters is not None:
            if not isinstance(config.message_filters, list):
                raise HTTPException(
                    status_code=400,
                    detail=f"message_filters must be a list, got {type(config.message_filters).__name__}",
                )
            try:
                filters = [MessageType(f.lower()) for f in config.message_filters]
                set_message_filters(filters)
                configured_fields.append(f"message_filters={config.message_filters}")
            except ValueError:
                valid_types = [mt.value for mt in MessageType]
                raise HTTPException(
                    status_code=400,
                    detail=f"Invalid message type in filters: {config.message_filters}. Valid types: {valid_types}",
                )

        if config.endpoint_filters is not None:
            if not isinstance(config.endpoint_filters, list):
                raise HTTPException(
                    status_code=400,
                    detail=f"endpoint_filters must be a list, got {type(config.endpoint_filters).__name__}",
                )
            set_endpoint_filters(config.endpoint_filters)
            configured_fields.append(f"endpoint_filters={config.endpoint_filters}")

        if config.rate_limit_per_second is not None:
            if not isinstance(config.rate_limit_per_second, int):
                raise HTTPException(
                    status_code=400,
                    detail=f"rate_limit_per_second must be an integer, got {type(config.rate_limit_per_second).__name__}",
                )
            if config.rate_limit_per_second < 1 or config.rate_limit_per_second > 10000:
                raise HTTPException(
                    status_code=400,
                    detail=f"Rate limit must be between 1 and 10000 messages per second, got {config.rate_limit_per_second}",
                )
            set_rate_limit(config.rate_limit_per_second)
            configured_fields.append(
                f"rate_limit_per_second={config.rate_limit_per_second}"
            )

        if config.console_output is not None:
            if not isinstance(config.console_output, bool):
                raise HTTPException(
                    status_code=400,
                    detail=f"console_output must be a boolean, got {type(config.console_output).__name__}",
                )
            set_console_output(config.console_output)
            configured_fields.append(f"console_output={config.console_output}")

        # Return updated status
        status = get_debug_status()
        return {
            "status": "configured",
            "configured_fields": configured_fields,
            "current_config": status,
        }

    except HTTPException:
        raise
    except Exception as e:
        # More detailed error for debugging
        import traceback

        raise HTTPException(
            status_code=500,
            detail=f"Error configuring debug settings: {str(e)}\nTraceback: {traceback.format_exc()}",
        )


@router.post("/zmq/enable")
async def enable_zmq_debug(inbound: bool = True, outbound: bool = True):
    """
    Quick enable ZMQ debugging with default settings.

    Args:
        inbound: Enable inbound message debugging
        outbound: Enable outbound message debugging
    """
    try:
        enable_inbound_debug(inbound)
        enable_outbound_debug(outbound)

        return {
            "status": "enabled",
            "inbound": inbound,
            "outbound": outbound,
            "message": "ZMQ debugging enabled with summary level logging",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error enabling debug: {str(e)}")


@router.post("/zmq/disable")
async def disable_zmq_debug():
    """
    Disable all ZMQ debugging.

    This immediately stops all debug logging with zero overhead.
    """
    try:
        enable_inbound_debug(False)
        enable_outbound_debug(False)

        return {"status": "disabled", "message": "All ZMQ debugging disabled"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error disabling debug: {str(e)}")


@router.get("/zmq/endpoints", response_model=EndpointStatsResponse)
async def get_zmq_endpoint_stats():
    """
    Get per-endpoint ZMQ debugging statistics.

    Returns statistics for each ZMQ endpoint that has been debugged,
    including message counts, byte totals, and rate limiting information.
    """
    try:
        stats = get_endpoint_stats()
        return EndpointStatsResponse(endpoints=stats)
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error getting endpoint stats: {str(e)}"
        )


@router.post("/zmq/reset-stats")
async def reset_zmq_debug_stats():
    """
    Reset all ZMQ debugging statistics.

    Clears all counters and statistics, useful for measuring
    performance over a specific time period.
    """
    try:
        reset_debug_stats()
        return {"status": "reset", "message": "All ZMQ debug statistics reset"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error resetting stats: {str(e)}")


# Advanced Debug Control


@router.post("/zmq/filter/messages")
async def set_message_type_filters(message_types: List[str]):
    """
    Set message type filters for ZMQ debugging.

    Args:
        message_types: List of message types to debug.
                      Valid types: sensory, motor, visualization, control, rest, heartbeat
                      Empty list = debug all message types
    """
    try:
        if not message_types:
            # Empty list means no filtering (allow all)
            set_message_filters([])
            return {
                "status": "configured",
                "filters": "none",
                "message": "All message types will be debugged",
            }

        # Validate message types
        valid_types = [mt.value for mt in MessageType]
        invalid_types = [t for t in message_types if t.lower() not in valid_types]

        if invalid_types:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid message types: {invalid_types}. Valid types: {valid_types}",
            )

        # Set filters
        filters = [MessageType(t.lower()) for t in message_types]
        set_message_filters(filters)

        return {
            "status": "configured",
            "filters": message_types,
            "message": f"Debugging enabled for message types: {', '.join(message_types)}",
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error setting message filters: {str(e)}"
        )


@router.post("/zmq/filter/endpoints")
async def set_endpoint_filters_rest(endpoints: List[str]):
    """
    Set endpoint filters for ZMQ debugging (REST alias).

    Args:
        endpoints: List of endpoints to debug (e.g., ["tcp://localhost:5562", "tcp://*:5564"])
                  Empty list = debug all endpoints
    """
    try:
        set_endpoint_filters(endpoints)

        if not endpoints:
            message = "All endpoints will be debugged"
        else:
            message = f"Debugging enabled for endpoints: {', '.join(endpoints)}"

        return {"status": "configured", "endpoints": endpoints, "message": message}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error setting endpoint filters: {str(e)}"
        )


@router.post("/zmq/level/{level}")
async def set_zmq_debug_level(level: str):
    """
    Set ZMQ debug verbosity level.

    Args:
        level: Debug level (off, minimal, headers, summary, full)
               - off: No debugging
               - minimal: Just endpoint and message count
               - headers: Add topics, sizes, timestamps
               - summary: Add data previews (first 200 chars)
               - full: Complete data dumps (use with caution!)
    """
    try:
        valid_levels = [dl.name.lower() for dl in DebugLevel]

        if level.lower() not in valid_levels:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid debug level: {level}. Valid levels: {', '.join(valid_levels)}",
            )

        debug_level = DebugLevel[level.upper()]
        set_debug_level(debug_level)

        return {
            "status": "configured",
            "level": level.lower(),
            "message": f"Debug level set to: {level.lower()}",
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error setting debug level: {str(e)}"
        )


@router.post("/zmq/rate-limit/{limit}")
async def set_zmq_rate_limit(limit: int):
    """
    Set rate limiting for ZMQ debug messages.

    Args:
        limit: Maximum messages per second to log (1-10000)
               Higher values = more verbose, lower values = less spam
    """
    try:
        if limit < 1 or limit > 10000:
            raise HTTPException(
                status_code=400,
                detail="Rate limit must be between 1 and 10000 messages per second",
            )

        set_rate_limit(limit)

        return {
            "status": "configured",
            "rate_limit": limit,
            "message": f"Rate limit set to {limit} messages per second",
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error setting rate limit: {str(e)}"
        )


# Information and Help Endpoints


@router.get("/zmq/help")
async def get_zmq_debug_help():
    """
    Get help information for ZMQ debugging features.

    Returns comprehensive documentation about available debugging
    features, configuration options, and usage examples.
    """
    return {
        "zmq_debugging": {
            "description": "High-performance ZMQ traffic debugging with zero overhead when disabled",
            "features": [
                "Runtime enable/disable without restart",
                "Multiple verbosity levels (off to full data dumps)",
                "Message type filtering (sensory, motor, visualization, control, rest)",
                "Endpoint filtering (specific IP:port combinations)",
                "Rate limiting to prevent log spam",
                "Performance impact monitoring",
                "Per-endpoint statistics",
            ],
        },
        "debug_levels": {
            "off": "No debugging (zero overhead)",
            "minimal": "Just endpoint and message counts",
            "headers": "Add topics, sizes, timestamps",
            "summary": "Add data previews (first 200 characters)",
            "full": "Complete data dumps (use with caution!)",
        },
        "message_types": {
            "sensory": "Neural data from sensors/agents to FEAGI",
            "motor": "Motor commands from FEAGI to agents/robots",
            "visualization": "Brain activity data for visualization clients",
            "control": "Control messages for system management",
            "rest": "REST API requests over ZMQ",
            "heartbeat": "Client heartbeat messages",
        },
        "usage_examples": {
            "enable_basic": "POST /debug/zmq/enable - Enable with default settings",
            "enable_selective": "POST /debug/zmq/configure - Configure specific settings",
            "filter_by_type": "POST /debug/zmq/filter/messages - Debug only specific message types",
            "filter_by_endpoint": "POST /debug/zmq/filter/endpoints - Debug only specific endpoints",
            "set_verbosity": "POST /debug/zmq/level/{level} - Change verbosity level",
            "monitor_performance": "GET /debug/zmq/endpoints - View per-endpoint statistics",
        },
        "performance_tips": [
            "Use 'minimal' or 'headers' level for production debugging",
            "Use message type filters to reduce noise",
            "Set rate limits to prevent log flooding",
            "Monitor debug overhead with endpoint statistics",
            "Disable debugging when not needed for zero overhead",
        ],
    }


@router.get("/info")
async def get_debug_info():
    """
    Get general debug information about FEAGI systems.

    Returns information about available debugging features,
    current system state, and debugging recommendations.
    """
    try:
        zmq_status = get_debug_status()

        return {
            "system_debug_info": {
                "zmq_debugging": {
                    "available": True,
                    "inbound_enabled": zmq_status["inbound_enabled"],
                    "outbound_enabled": zmq_status["outbound_enabled"],
                    "current_level": zmq_status["debug_level"],
                    "overhead_ms": zmq_status["stats"]["debug_overhead_ms"],
                    "messages_logged": zmq_status["stats"]["messages_logged"],
                    "uptime_seconds": zmq_status["stats"]["uptime_seconds"],
                }
            },
            "available_debug_features": [
                "ZMQ traffic debugging (/debug/zmq/*)",
                "Runtime configuration changes",
                "Performance impact monitoring",
                "Message filtering and rate limiting",
            ],
            "endpoints": {
                "zmq_status": "GET /debug/zmq/status",
                "zmq_configure": "POST /debug/zmq/configure",
                "zmq_help": "GET /debug/zmq/help",
            },
        }

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error getting debug info: {str(e)}"
        )


@router.post("/zmq/console")
async def enable_console_output(enabled: bool = True):
    """
    Simple endpoint to enable/disable console output for ZMQ debugging.

    This is easier to use from Swagger UI than the full configure endpoint.

    Args:
        enabled: Whether to enable console output (default: True)
    """
    try:
        set_console_output(enabled)

        return {
            "status": "configured",
            "console_output": enabled,
            "message": f"Console output {'enabled' if enabled else 'disabled'}",
        }
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error setting console output: {str(e)}"
        )
