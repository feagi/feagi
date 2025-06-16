#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from fastapi import Depends, HTTPException

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.dependencies import get_core_api_service
from feagi.core.state_manager import (
    ConnectomeState,
    FeagiStateManager,
    GenomeState,
    ServiceState,
)

# FEAGI state check
# Genome running conditions
# Agent connectivity

# Helper to get state manager instance
state = FeagiStateManager.instance()


# Get state manager instance
def get_state_manager():
    return FeagiStateManager.instance()


# Genome state checks
def check_active_genome(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
):
    """
    Dependency that verifies a genome is loaded and ready.
    Raises HTTPException if no genome is loaded.
    """
    state_manager = get_state_manager()

    # Check state manager genome state first
    if not state_manager.is_genome_loaded():
        raise HTTPException(
            status_code=400,
            detail="No active genome found. Please load a genome first.",
        )

    # Double-check with CoreAPIService
    if not core_api_service.genome_is_loaded():
        raise HTTPException(
            status_code=400,
            detail="No active genome found. Please load a genome first.",
        )

    return "OK"


# Connectome state checks
def check_connectome_ready(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
):
    """
    Dependency that verifies the connectome is initialized and ready.
    Raises HTTPException if connectome is not ready.
    """
    state_manager = get_state_manager()

    # Check if connectome is in an appropriate state
    connectome_state = state_manager.get_connectome_state()
    if connectome_state not in [ConnectomeState.READY, ConnectomeState.RUNNING]:
        raise HTTPException(
            status_code=400,
            detail=f"Connectome is not ready (current state: {connectome_state.name}). Please wait for initialization to complete.",
        )

    # Additional check: Make sure we have cortical areas
    connectome_manager = core_api_service.get_connectome_manager()
    if (
        not hasattr(connectome_manager, "cortical_areas")
        or not connectome_manager.cortical_areas
    ):
        raise HTTPException(
            status_code=400,
            detail="Connectome has no cortical areas. Please load a genome with cortical areas.",
        )

    return "OK"


# Burst engine state checks
def check_burst_engine_running(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
):
    """
    Dependency that verifies the burst engine is running.
    Raises HTTPException if burst engine is not in RUNNING state.
    """
    state_manager = get_state_manager()

    # Check if burst engine is running
    burst_engine_state = state_manager.get_burst_engine_state()
    if burst_engine_state != ServiceState.RUNNING:
        raise HTTPException(
            status_code=400,
            detail=f"Burst engine is not running (current state: {burst_engine_state.name}). Please start the burst engine.",
        )

    # Additional check: Get burst engine from service
    burst_engine = core_api_service.get_burst_engine()
    if not burst_engine:
        raise HTTPException(
            status_code=500, detail="Burst engine not available through service."
        )

    return "OK"


# Combined checks for specific requirements
def check_genome_and_connectome(
    _genome: str = Depends(check_active_genome),
    _connectome: str = Depends(check_connectome_ready),
):
    """
    Combined dependency that checks both genome and connectome are ready.
    """
    return "OK"


def check_fully_operational(
    _genome: str = Depends(check_active_genome),
    _connectome: str = Depends(check_connectome_ready),
    _burst_engine: str = Depends(check_burst_engine_running),
):
    """
    Combined dependency that checks the system is fully operational.
    """
    return "OK"


def check_brain_running(_: bool = Depends(check_active_genome)):
    if state.get_brain_readiness():
        return True
    else:
        raise HTTPException(
            status_code=400, detail="Brain not yet ready! Please try again later."
        )


# Specialized operational checks for different system capabilities
def check_plasticity_enabled(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_genome_and_connectome),
):
    """
    Verifies neuroplasticity is enabled and operational.
    Raises HTTPException if plasticity is not enabled.

    This is used for endpoints that modify neural connections or
    neuroplasticity settings.
    """
    state_manager = get_state_manager()

    # First check if the burst engine is running (plasticity requires it)
    if state_manager.get_burst_engine_state() != ServiceState.RUNNING:
        raise HTTPException(
            status_code=400, detail="Plasticity requires the burst engine to be running"
        )

    # Get plasticity info and check if it's enabled
    try:
        plasticity_info = core_api_service.get_plasticity_info()
        if not plasticity_info or not plasticity_info.get("enabled", False):
            raise HTTPException(
                status_code=400,
                detail="Neuroplasticity is not enabled in the current configuration",
            )
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to verify neuroplasticity state: {str(e)}"
        )

    return "OK"


# Input/Output system checks
def check_io_system_ready(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready),
):
    """
    Verifies the I/O system is ready for operations.
    Raises HTTPException if I/O areas are not properly configured.

    This is used for endpoints that interact with sensory inputs
    or motor outputs.
    """
    try:
        # Check for presence of IPU/OPU areas
        areas = core_api_service.get_cortical_areas()

        has_ipu = any(area["type"] == "IPU" for area in areas)
        has_opu = any(area["type"] == "OPU" for area in areas)

        if not has_ipu:
            raise HTTPException(
                status_code=400,
                detail="No Input Processing Unit (IPU) found in the current connectome",
            )

        if not has_opu:
            raise HTTPException(
                status_code=400,
                detail="No Output Processing Unit (OPU) found in the current connectome",
            )

        # You could also check for specific IPU/OPU types here
        # (e.g., visual, auditory, motor) depending on the endpoint's needs

        return "OK"
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to verify I/O system readiness: {str(e)}"
        )


# Deployment checks
def check_deployment_ready(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
):
    """
    Verifies the system is ready for deployment operations.

    This performs a minimal check without requiring an existing genome
    to be in place, since deployment typically replaces the genome.
    """
    state_manager = get_state_manager()

    # Check if currently in the middle of another operation
    genome_state = state_manager.get_genome_state()
    if genome_state in [GenomeState.LOADING, GenomeState.SAVING]:
        raise HTTPException(
            status_code=409,  # Conflict
            detail=f"System is currently {genome_state.name}. Please wait for operation to complete.",
        )

    # Check if burst engine is in a state where deployment is safe
    burst_state = state_manager.get_burst_engine_state()
    if burst_state not in [
        ServiceState.READY,
        ServiceState.STOPPED,
        ServiceState.UNAVAILABLE,
    ]:
        raise HTTPException(
            status_code=409,  # Conflict
            detail=f"Burst engine is {burst_state.name}. Please stop the burst engine before deployment.",
        )

    # Check if connectome is in a state where deployment is safe
    connectome_state = state_manager.get_connectome_state()
    if connectome_state not in [
        ConnectomeState.READY,
        ConnectomeState.INITIALIZING,
        ConnectomeState.UNAVAILABLE,
    ]:
        raise HTTPException(
            status_code=409,  # Conflict
            detail=f"Connectome is {connectome_state.name}. Please wait for it to stabilize.",
        )

    return "OK"


# Amalgamation checks
def check_amalgamation_ready(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_active_genome),
):
    """
    Verifies the system is ready for amalgamation operations.

    This checks for conditions specific to amalgamation, like
    no pending amalgamation already in progress.
    """
    state_manager = get_state_manager()

    # Check if there's already a pending amalgamation
    if state_manager.pending_amalgamation and state_manager.pending_amalgamation.get(
        "initiation_time"
    ):
        raise HTTPException(
            status_code=409,  # Conflict
            detail="An existing amalgamation attempt is already pending",
        )

    return "OK"


# Check for specific cortical area
def check_cortical_area_exists(
    cortical_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    _: str = Depends(check_connectome_ready),
):
    """
    Verify a specific cortical area exists.

    Args:
        cortical_id: The ID of the cortical area to check

    Raises HTTPException if the area doesn't exist.
    """
    try:
        # Check if cortical area exists using the proper cortical_id (string)
        area_data = core_api_service.get_cortical_area(cortical_id)
        if not area_data:
            raise HTTPException(
                status_code=404, detail=f"Cortical area with ID {cortical_id} not found"
            )
    except HTTPException:
        # Re-raise HTTPExceptions
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error checking cortical area: {str(e)}"
        )

    return "OK"


# Monitoring capability check
def check_monitoring_available(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
):
    """
    Verify monitoring capabilities are available.
    Monitoring endpoints might work even without a full genome/connectome.
    """
    # Even without a genome, we should be able to monitor system stats
    # state_manager = get_state_manager()  # Unused variable removed

    # Check if basic monitoring is available
    try:
        # This could be expanded to check for specific monitoring capabilities
        if not hasattr(core_api_service, "get_system_metrics"):
            raise HTTPException(
                status_code=503,  # Service Unavailable
                detail="Monitoring capabilities are not available",
            )
        return "OK"
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(
            status_code=500, detail=f"Error checking monitoring availability: {str(e)}"
        )
