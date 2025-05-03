"""REST API application for FEAGI."""

from typing import Dict, List, Optional, Any
from fastapi import FastAPI, HTTPException, Depends, Request
import time
import logging

from feagi.api.core.services import CoreAPIService
from feagi.api.gateway import APIGateway

logger = logging.getLogger(__name__)

def get_api_gateway() -> APIGateway:
    """
    Get the API Gateway instance as a FastAPI dependency.
    
    Returns:
        APIGateway instance.
    """
    # Ensure we have a singleton instance of the gateway
    if not hasattr(get_api_gateway, "instance"):
        get_api_gateway.instance = APIGateway()
    return get_api_gateway.instance

def get_core_api() -> CoreAPIService:
    """
    Get the Core API service as a FastAPI dependency.
    
    Returns:
        CoreAPIService instance.
    """
    return get_api_gateway().core_api

async def log_request_middleware(request: Request, call_next):
    """
    Middleware to log requests and measure response time.
    
    Args:
        request: FastAPI request.
        call_next: Next middleware in the chain.
        
    Returns:
        FastAPI response.
    """
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    
    logger.info(
        f"Method: {request.method}, "
        f"Path: {request.url.path}, "
        f"Status: {response.status_code}, "
        f"Duration: {process_time:.4f}s"
    )
    
    return response

def create_rest_app() -> FastAPI:
    """
    Create a FastAPI application for FEAGI's REST API.
    
    Returns:
        FastAPI application.
    """
    app = FastAPI(
        title="FEAGI REST API",
        description="REST API for the Framework for Evolutionary Artificial General Intelligence",
        version="1.0.0",
    )
    
    # Add middleware
    app.middleware("http")(log_request_middleware)
    
    # Root endpoint
    @app.get("/")
    async def root():
        """Root endpoint."""
        return {"message": "Welcome to FEAGI REST API"}
    
    # Health check endpoint
    @app.get("/health")
    async def health_check():
        """Health check endpoint."""
        return {"status": "ok"}
    
    # API version endpoint
    @app.get("/version")
    async def version():
        """Get API version."""
        return {"version": "1.0.0"}
    
    # Include versioned API routers
    from feagi.api.rest.routers.v1 import (
        cortical_area_router as v1_cortical_area_router,
        genome_router as v1_genome_router,
        simulation_router as v1_simulation_router,
        system_router as v1_system_router,
        morphology_router as v1_morphology_router,
        neuroplasticity_router as v1_neuroplasticity_router,
        connectome_router as v1_connectome_router,
        burst_engine_router as v1_burst_engine_router,
        inputs_router as v1_inputs_router,
        region_router as v1_region_router,
        insights_router as v1_insights_router,
        cortical_mapping_router as v1_cortical_mapping_router
    )
    
    # Include all domain-specific routers
    app.include_router(v1_cortical_area_router, prefix="/api/v1")
    app.include_router(v1_genome_router, prefix="/api/v1")
    app.include_router(v1_simulation_router, prefix="/api/v1")
    app.include_router(v1_system_router, prefix="/api/v1")
    app.include_router(v1_morphology_router, prefix="/api/v1")
    app.include_router(v1_neuroplasticity_router, prefix="/api/v1")
    app.include_router(v1_connectome_router, prefix="/api/v1")
    app.include_router(v1_burst_engine_router, prefix="/api/v1")
    app.include_router(v1_inputs_router, prefix="/api/v1")
    app.include_router(v1_region_router, prefix="/api/v1")
    app.include_router(v1_insights_router, prefix="/api/v1")
    app.include_router(v1_cortical_mapping_router, prefix="/api/v1")
    
    return app 