"""
FEAGI Visualization API endpoints.

Provides REST endpoints for managing visualization clients and controlling
the visualization stream and FQ sampler.
"""

import uuid
from typing import Dict, Any, Optional
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.v1.schemas import SuccessResponse, ErrorResponse
from feagi.utils.logger import setup_logger

logger = setup_logger()
router = APIRouter()


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
    core_api_service: CoreAPIService = Depends(get_core_api_service)
) -> VisualizationClientResponse:
    """
    Register a visualization client.
    
    This triggers the FQ sampler to start sampling for visualization data.
    """
    try:
        # Generate client ID if not provided
        client_id = request.client_id or str(uuid.uuid4())
        
        logger.info(f"🔌 Registering visualization client: {client_id}")
        
        # Get the process manager to access the visualization stream
        from feagi.process_manager import ProcessManager
        pm = ProcessManager.get_instance()
        
        if pm and hasattr(pm, '_zmq_server') and pm._zmq_server:
            # Get visualization stream from ZMQ server
            viz_stream = pm._zmq_server.get_visualization_stream()
            if viz_stream:
                await viz_stream.register_visualization_client(client_id)
                logger.info(f"✅ Visualization client registered: {client_id}")
                
                return VisualizationClientResponse(
                    client_id=client_id,
                    success=True,
                    message=f"Visualization client {client_id} registered successfully"
                )
            else:
                logger.error("❌ Visualization stream not available")
                raise HTTPException(status_code=503, detail="Visualization stream not available")
        else:
            logger.error("❌ Process manager or ZMQ server not available")
            raise HTTPException(status_code=503, detail="FEAGI services not available")
            
    except Exception as e:
        logger.error(f"❌ Error registering visualization client: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")


@router.post("/unregister_client", response_model=SuccessResponse)
async def unregister_visualization_client(
    request: VisualizationClientRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
) -> SuccessResponse:
    """
    Unregister a visualization client.
    
    If this is the last client, the FQ sampler will stop sampling.
    """
    try:
        client_id = request.client_id
        if not client_id:
            raise HTTPException(status_code=400, detail="Client ID is required")
        
        logger.info(f"🔌 Unregistering visualization client: {client_id}")
        
        # Get the process manager to access the visualization stream
        from feagi.process_manager import ProcessManager
        pm = ProcessManager.get_instance()
        
        if pm and hasattr(pm, '_zmq_server') and pm._zmq_server:
            # Get visualization stream from ZMQ server
            viz_stream = pm._zmq_server.get_visualization_stream()
            if viz_stream:
                await viz_stream.unregister_visualization_client(client_id)
                logger.info(f"✅ Visualization client unregistered: {client_id}")
                
                return SuccessResponse(
                    message=f"Visualization client {client_id} unregistered successfully"
                )
            else:
                logger.error("❌ Visualization stream not available")
                raise HTTPException(status_code=503, detail="Visualization stream not available")
        else:
            logger.error("❌ Process manager or ZMQ server not available")
            raise HTTPException(status_code=503, detail="FEAGI services not available")
            
    except Exception as e:
        logger.error(f"❌ Error unregistering visualization client: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Unregistration failed: {str(e)}")


@router.post("/heartbeat", response_model=SuccessResponse)
async def visualization_client_heartbeat(
    request: VisualizationHeartbeatRequest,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
) -> SuccessResponse:
    """
    Send a heartbeat from a visualization client.
    
    This keeps the client active and prevents timeout.
    """
    try:
        client_id = request.client_id
        
        logger.debug(f"💗 Heartbeat from visualization client: {client_id}")
        
        # Get the process manager to access the visualization stream
        from feagi.process_manager import ProcessManager
        pm = ProcessManager.get_instance()
        
        if pm and hasattr(pm, '_zmq_server') and pm._zmq_server:
            # Get visualization stream from ZMQ server
            viz_stream = pm._zmq_server.get_visualization_stream()
            if viz_stream:
                await viz_stream.heartbeat_visualization_client(client_id)
                
                return SuccessResponse(
                    message=f"Heartbeat received from client {client_id}"
                )
            else:
                logger.error("❌ Visualization stream not available")
                raise HTTPException(status_code=503, detail="Visualization stream not available")
        else:
            logger.error("❌ Process manager or ZMQ server not available")
            raise HTTPException(status_code=503, detail="FEAGI services not available")
            
    except Exception as e:
        logger.error(f"❌ Error processing visualization heartbeat: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Heartbeat failed: {str(e)}")


@router.get("/status", response_model=VisualizationStatusResponse)
async def get_visualization_status(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
) -> VisualizationStatusResponse:
    """
    Get the current visualization system status.
    
    Returns information about active clients and FQ sampler status.
    """
    try:
        logger.debug("📊 Getting visualization status")
        
        # Get the process manager to access the visualization stream
        from feagi.process_manager import ProcessManager
        pm = ProcessManager.get_instance()
        
        if pm and hasattr(pm, '_zmq_server') and pm._zmq_server:
            # Get visualization stream from ZMQ server
            viz_stream = pm._zmq_server.get_visualization_stream()
            if viz_stream:
                # Get status from visualization stream
                active_clients = len(getattr(viz_stream, '_active_clients', {}))
                
                # Check FQ sampler status
                fq_sampler_enabled = False
                if hasattr(pm, '_fq_sampler') and pm._fq_sampler:
                    fq_sampler_enabled = getattr(pm._fq_sampler, '_has_visualization_subscribers', False)
                
                return VisualizationStatusResponse(
                    enabled=True,
                    active_clients=active_clients,
                    fq_sampler_enabled=fq_sampler_enabled,
                    message=f"Visualization system active with {active_clients} clients"
                )
            else:
                return VisualizationStatusResponse(
                    enabled=False,
                    active_clients=0,
                    fq_sampler_enabled=False,
                    message="Visualization stream not available"
                )
        else:
            return VisualizationStatusResponse(
                enabled=False,
                active_clients=0,
                fq_sampler_enabled=False,
                message="FEAGI services not available"
            )
            
    except Exception as e:
        logger.error(f"❌ Error getting visualization status: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Status check failed: {str(e)}") 