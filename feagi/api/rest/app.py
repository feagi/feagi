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
    return APIGateway()

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
    from feagi.api.rest.v1.router import router as v1_router
    app.include_router(v1_router, prefix="/api/v1")
    
    return app 