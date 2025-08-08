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
Common utilities and classes for the REST API.
"""

import queue

from feagi.utils.logger import setup_logger

logger = setup_logger("feagi.api.rest.commons")
from typing import Any, Dict, Optional

from fastapi import HTTPException, Request

# Queue for API requests processing
api_queue = queue.Queue()


class CustomError(Exception):
    """
    Custom error class for API-specific exceptions.

    Attributes:
        message: Error message
        status_code: HTTP status code
        details: Additional error details
    """

    def __init__(
        self,
        message: str,
        status_code: int = 500,
        details: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize the custom error.

        Args:
            message: Error message
            status_code: HTTP status code
            details: Additional error details
        """
        self.message = message
        self.status_code = status_code
        self.details = details or {}
        super().__init__(self.message)

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the error to a dictionary representation.

        Returns:
            Dictionary containing error information
        """
        result = {"message": self.message, "status_code": self.status_code}

        if self.details:
            result["details"] = self.details

        return result


async def check_brain_running(request: Request):
    """
    Dependency to check if the brain is running.
    Raises an HTTPException if the brain is not running.
    """
    from feagi.api.rest.dependencies import get_connectome
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    # Check if the connectome is available
    try:
        connectome = get_connectome()
        if (
            not connectome
            or not hasattr(connectome, "is_initialized")
            or not connectome.is_initialized
        ):
            raise HTTPException(status_code=400, detail="Brain is not running!")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Brain is not running: {str(e)}")

    # Also check the state manager
    state_manager = FeagiStateManager.instance()
    if state_manager.get_burst_engine_state() != ServiceState.READY:
        raise HTTPException(status_code=400, detail="Brain is not running!")


async def check_active_genome(request: Request):
    """
    Dependency to check if there is an active genome.
    Raises an HTTPException if no genome is loaded.
    """
    from feagi.api.rest.dependencies import get_core_api_service

    try:
        core_api = get_core_api_service()
        if not core_api or not core_api.genome_is_loaded():
            raise HTTPException(status_code=400, detail="No genome loaded!")
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Genome access error: {str(e)}")


async def check_burst_engine(request: Request):
    """
    Dependency to check if the burst engine is running.
    Raises an HTTPException if the burst engine is not ready.
    """
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    state_manager = FeagiStateManager.instance()
    if state_manager.get_burst_engine_state() != ServiceState.READY:
        raise HTTPException(status_code=400, detail="Burst engine is not running!")


async def check_burst_engine_or_allow_genome_ops(request: Request):
    """
    Similar to check_burst_engine, but also allows genome operations and burst engine control operations
    when the burst engine is not yet running.
    """
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    # Skip the check for genome loading/initial operations
    if (
        "/v1/genome/upload" in request.url.path
        or request.url.path.endswith("/v1/genome/download")
        or request.url.path.endswith("/v1/genome/genome_number")
        or request.url.path.endswith("/v1/genome/file_name")
        or request.url.path.endswith("/v1/burst_engine/start")
        or request.url.path.endswith("/v1/burst_engine/stop")
        or request.url.path.endswith("/v1/burst_engine/status")
    ):
        return

    # Otherwise perform the standard check
    state_manager = FeagiStateManager.instance()
    if state_manager.get_burst_engine_state() != ServiceState.READY:
        raise HTTPException(status_code=400, detail="Burst engine is not running!")


async def check_burst_engine_or_allow_config_ops(request: Request):
    """
    Similar to check_burst_engine, but also allows configuration operations
    like getting simulation_timestep when the burst engine is not yet running.
    """
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    # Allow configuration/read-only operations even when burst engine not READY
    config_read_endpoints = [
        "simulation_timestep",  # 1/frequency - just a configuration read
        "config",  # Burst engine configuration
        "status",  # Burst engine status
        "burst_counter",  # Current burst count
        "fcl_sampler",  # FCL sampling operations
        "neuron_fcl",  # Neuron FCL operations
    ]

    # Check if this is a config/read operation
    for endpoint in config_read_endpoints:
        if endpoint in request.url.path:
            return  # Allow config operations

    # For control operations, require burst engine to be ready
    await check_burst_engine(request)


async def check_burst_engine_for_processing(request: Request):
    """
    Check burst engine for operations that require active neural processing.
    Blocks if engine is ON_HOLD (paused).
    """
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    state_manager = FeagiStateManager.instance()
    burst_state = state_manager.get_burst_engine_state()

    if burst_state == ServiceState.ON_HOLD:
        raise HTTPException(
            status_code=400,
            detail="Burst engine is on hold (paused) - resume to perform this operation",
        )
    elif burst_state != ServiceState.READY:
        raise HTTPException(status_code=400, detail="Burst engine is not running!")
