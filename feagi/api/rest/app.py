"""REST API application for FEAGI."""

from typing import Dict, List, Optional, Any
import time
import logging
from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from feagi.api.core.services import CoreAPIService
from feagi.api.gateway import APIGateway

logger = logging.getLogger(__name__)

# Standard error response models
class GeneralErrorResponse(BaseModel):
    """General error response model."""
    error_code: int
    error_message: str
    
class InternalServerErrorResponse(BaseModel):
    """Internal server error response model."""
    error_code: int
    error_message: str

# Define standard responses
standard_response = {
    400: {
        "model": GeneralErrorResponse,
        "description": "All Handled Errors",
        "content": {
            "application/json": {
                "example": {"error_code": 400, "error_message": "Request failed"}
            }
        }
    },
    500: {
        "model": InternalServerErrorResponse,
        "description": "Internal Server Error",
        "content": {
            "application/json": {
                "example": {"error_code": 500, "error_message": "Internal error"}
            }
        }
    },
}

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

# Define dependency check functions
def check_active_genome(core_api: CoreAPIService = Depends(get_core_api)):
    """Dependency to check if an active genome is loaded."""
    if not core_api.get_genome():
        raise HTTPException(status_code=400, detail="No active genome loaded")
    return core_api

def check_brain_running(core_api: CoreAPIService = Depends(get_core_api)):
    """Dependency to check if brain is running."""
    if not core_api.get_simulation_status().get("running", False):
        raise HTTPException(status_code=400, detail="Brain is not running")
    return core_api

def check_burst_engine(core_api: CoreAPIService = Depends(get_core_api)):
    """Dependency to check if burst engine is available."""
    # This is just a placeholder for now, as there's no direct way to check
    # for burst engine status in the current implementation
    return core_api

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
        swagger_ui_parameters={
            "defaultModelsExpandDepth": -1,
            "filter": True,
        }
    )
    
    # Add middleware
    app.middleware("http")(log_request_middleware)
    
    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # Can be restricted to specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
        expose_headers=["Content-Disposition"],
    )
    
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
    
    # Include all domain-specific routers with proper prefixes, tags, and dependencies
    app.include_router(
        v1_cortical_area_router, 
        prefix="/api/v1",
        tags=["CORTICAL AREAS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    app.include_router(
        v1_genome_router, 
        prefix="/api/v1",
        tags=["GENOME"],
        dependencies=[Depends(check_burst_engine)],
        responses=standard_response
    )
    
    app.include_router(
        v1_simulation_router, 
        prefix="/api/v1",
        tags=["SIMULATION"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response
    )
    
    app.include_router(
        v1_system_router, 
        prefix="/api/v1",
        tags=["SYSTEM"],
        responses=standard_response
    )
    
    app.include_router(
        v1_morphology_router, 
        prefix="/api/v1",
        tags=["NEURON MORPHOLOGIES"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    app.include_router(
        v1_neuroplasticity_router, 
        prefix="/api/v1",
        tags=["NEUROPLASTICITY"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    app.include_router(
        v1_connectome_router, 
        prefix="/api/v1",
        tags=["CONNECTOME"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response
    )
    
    app.include_router(
        v1_burst_engine_router, 
        prefix="/api/v1",
        tags=["BURST ENGINE"],
        dependencies=[Depends(check_burst_engine)],
        responses=standard_response
    )
    
    app.include_router(
        v1_inputs_router, 
        prefix="/api/v1",
        tags=["INPUT MANAGEMENT"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    app.include_router(
        v1_region_router, 
        prefix="/api/v1",
        tags=["BRAIN REGIONS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    app.include_router(
        v1_insights_router, 
        prefix="/api/v1",
        tags=["INSIGHTS"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response
    )
    
    app.include_router(
        v1_cortical_mapping_router, 
        prefix="/api/v1",
        tags=["CORTICAL MAPPINGS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    return app 