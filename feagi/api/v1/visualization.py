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
"""

"""
FEAGI Visualization API endpoints.

Provides REST endpoints for managing visualization clients and controlling
the visualization stream and FQ sampler.
"""

import uuid
from typing import Any, Dict, Optional

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.v1.schemas import SuccessResponse
from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
router = APIRouter()

# Lightweight per-client heartbeat counters
_heartbeat_counts: Dict[str, int] = {}
_heartbeat_window_start: Dict[str, float] = {}
_HEARTBEAT_WINDOW_SEC = 1.0
_HEARTBEAT_WARN_THRESHOLD = 20


class VisualizationClientRequest(BaseModel):
    """Request model for visualization client registration."""

    client_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


class VisualizationClientResponse(BaseModel):
    """Response model for visualization client registration."""

    client_id: str
    success: bool
    message: str


class VisualizationHeartbeatRequest(BaseModel):
    """Request model for visualization client heartbeat."""

    client_id: str


class VisualizationStatusResponse(BaseModel):
    """Response model for visualization status."""

    enabled: bool
    active_clients: int
    fq_sampler_enabled: bool
    message: str


@router.post("/register_client", response_model=VisualizationClientResponse)
async def register_visualization_client(
    request: VisualizationClientRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
) -> VisualizationClientResponse:
    """Register a visualization client.

    This triggers the FQ sampler to start sampling for visualization data.
    """
    try:
        # Generate client ID if not provided
        client_id = request.client_id or str(uuid.uuid4())

        logger.info(f"🔌 Registering visualization client: {client_id}")

        # Get the process manager to access the visualization stream
        from feagi.process_manager import get_process_manager

        pm = get_process_manager()

        if pm and hasattr(pm, "_processes") and "zmq_server" in pm._processes:
            #  Get ZMQ server from _processes dictionary where it's actually
            #  stored
            zmq_server = pm._processes["zmq_server"]
            if zmq_server:
                # Get visualization stream from ZMQ server
                viz_stream = zmq_server.get_visualization_stream()
                if viz_stream:
                    viz_stream.register_visualization_client(client_id)
                    logger.info(
                        f"[OK] Visualization client registered: {client_id}"
                    )

                    return VisualizationClientResponse(
                        client_id=client_id,
                        success=True,
                        message=f"Visualization client {client_id} registered successfully",
                    )
                else:
                    logger.error("[ERR] Visualization stream not available")
                    raise HTTPException(
                        status_code=503,
                        detail="Visualization stream not available",
                    )
            else:
                logger.error("[ERR] ZMQ server not available in processes")
                raise HTTPException(
                    status_code=503, detail="ZMQ server not available"
                )
        else:
            logger.error("[ERR] Process manager or ZMQ server not available")
            raise HTTPException(
                status_code=503, detail="FEAGI services not available"
            )

    except Exception as e:
        logger.error(f"[ERR] Error registering visualization client: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Registration failed: {str(e)}"
        ) from e


@router.post("/unregister_client", response_model=SuccessResponse)
async def unregister_visualization_client(
    request: VisualizationClientRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
) -> SuccessResponse:
    """Unregister a visualization client.

    If this is the last client, the FQ sampler will stop sampling.
    """
    try:
        client_id = request.client_id
        if not client_id:
            raise HTTPException(
                status_code=400, detail="Client ID is required"
            )

        logger.info(f"🔌 Unregistering visualization client: {client_id}")

        # Get the process manager to access the visualization stream
        from feagi.process_manager import get_process_manager

        pm = get_process_manager()

        if pm and hasattr(pm, "_processes") and "zmq_server" in pm._processes:
            #  Get ZMQ server from _processes dictionary where it's actually
            #  stored
            zmq_server = pm._processes["zmq_server"]
            if zmq_server:
                # Get visualization stream from ZMQ server
                viz_stream = zmq_server.get_visualization_stream()
                if viz_stream:
                    viz_stream.unregister_visualization_client(client_id)
                    logger.info(
                        f"[OK] Visualization client unregistered: {client_id}"
                    )

                    return SuccessResponse(
                        message=f"Visualization client {client_id} unregistered successfully"
                    )
                else:
                    logger.error("[ERR] Visualization stream not available")
                    raise HTTPException(
                        status_code=503,
                        detail="Visualization stream not available",
                    )
            else:
                logger.error("[ERR] ZMQ server not available in processes")
                raise HTTPException(
                    status_code=503, detail="ZMQ server not available"
                )
        else:
            logger.error("[ERR] Process manager or ZMQ server not available")
            raise HTTPException(
                status_code=503, detail="FEAGI services not available"
            )

    except Exception as e:
        logger.error(
            f"[ERR] Error unregistering visualization client: {str(e)}"
        )
        raise HTTPException(
            status_code=500, detail=f"Unregistration failed: {str(e)}"
        ) from e


@router.post("/heartbeat", response_model=SuccessResponse)
async def visualization_client_heartbeat(
    request: VisualizationHeartbeatRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service),
    http_req: Request = None,
) -> SuccessResponse:
    """Send a heartbeat from a visualization client.

    This keeps the client active and prevents timeout.
    """
    try:
        client_id = request.client_id
        client_ip = http_req.client.host if http_req and http_req.client else "unknown"
        ua = http_req.headers.get("user-agent", "<none>") if http_req else "<none>"

        # Per-client heartbeat rate warning
        import time as _time
        now = _time.time()
        ws = _heartbeat_window_start.get(client_id, 0.0)
        if now - ws > _HEARTBEAT_WINDOW_SEC:
            _heartbeat_window_start[client_id] = now
            _heartbeat_counts[client_id] = 1
        else:
            _heartbeat_counts[client_id] = _heartbeat_counts.get(client_id, 0) + 1
            if _heartbeat_counts[client_id] >= _HEARTBEAT_WARN_THRESHOLD:
                logger.warning(
                    f"[VIZ-HB] High heartbeat rate from client {client_id} ({client_ip}, UA={ua}): {_heartbeat_counts[client_id]} in {_HEARTBEAT_WINDOW_SEC:.1f}s"
                )
                # reset window to avoid spamming
                _heartbeat_window_start[client_id] = now
                _heartbeat_counts[client_id] = 0

        logger.debug(f"💗 Heartbeat from visualization client: {client_id} ({client_ip}, UA={ua})")

        # Get the process manager to access the visualization stream
        from feagi.process_manager import get_process_manager

        pm = get_process_manager()

        if pm and hasattr(pm, "_processes") and "zmq_server" in pm._processes:
            #  Get ZMQ server from _processes dictionary where it's actually
            #  stored
            zmq_server = pm._processes["zmq_server"]
            if zmq_server:
                # Get visualization stream from ZMQ server
                viz_stream = zmq_server.get_visualization_stream()
                if viz_stream:
                    viz_stream.heartbeat_visualization_client(client_id)

                    return SuccessResponse(
                        message=f"Heartbeat received from client {client_id}"
                    )
                else:
                    logger.error("[ERR] Visualization stream not available")
                    raise HTTPException(
                        status_code=503,
                        detail="Visualization stream not available",
                    )
            else:
                logger.error("[ERR] ZMQ server not available in processes")
                raise HTTPException(
                    status_code=503, detail="ZMQ server not available"
                )
        else:
            logger.error("[ERR] Process manager or ZMQ server not available")
            raise HTTPException(
                status_code=503, detail="FEAGI services not available"
            )

    except Exception as e:
        logger.error(
            f"[ERR] Error processing visualization heartbeat: {str(e)}"
        )
        raise HTTPException(
            status_code=500, detail=f"Heartbeat failed: {str(e)}"
        ) from e


@router.get("/status", response_model=VisualizationStatusResponse)
async def get_visualization_status(
    core_api_service: CoreAPIService = Depends(get_core_api_service),
) -> VisualizationStatusResponse:
    """Get the current visualization system status.

    Returns information about active clients and FQ sampler status.
    """
    try:
        logger.debug("[STATS] Getting visualization status")

        # Get the process manager to access the visualization stream
        from feagi.process_manager import get_process_manager

        pm = get_process_manager()

        if pm and hasattr(pm, "_processes") and "zmq_server" in pm._processes:
            #  Get ZMQ server from _processes dictionary where it's actually
            #  stored
            zmq_server = pm._processes["zmq_server"]
            if zmq_server:
                # Get visualization stream from ZMQ server
                viz_stream = zmq_server.get_visualization_stream()
                if viz_stream:
                    # Get status from visualization stream - USE CORRECT METHOD
                    active_clients = viz_stream.get_connected_client_count()

                    # Check FQ sampler status using correct attributes
                    fq_sampler_enabled = getattr(
                        viz_stream, "_fq_sampler_enabled", False
                    )

                    return VisualizationStatusResponse(
                        enabled=True,
                        active_clients=active_clients,
                        fq_sampler_enabled=fq_sampler_enabled,
                        message=f"Visualization system active with {active_clients} clients",
                    )
                else:
                    return VisualizationStatusResponse(
                        enabled=False,
                        active_clients=0,
                        fq_sampler_enabled=False,
                        message="Visualization stream not available",
                    )
            else:
                return VisualizationStatusResponse(
                    enabled=False,
                    active_clients=0,
                    fq_sampler_enabled=False,
                    message="ZMQ server not available",
                )
        else:
            return VisualizationStatusResponse(
                enabled=False,
                active_clients=0,
                fq_sampler_enabled=False,
                message="FEAGI services not available",
            )

    except Exception as e:
        logger.error(f"[ERR] Error getting visualization status: {str(e)}")
        raise HTTPException(
            status_code=500, detail=f"Status check failed: {str(e)}"
        ) from e
