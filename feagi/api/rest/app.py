"""REST API server implementation for FEAGI."""

import os
import time
import logging
import importlib
import traceback
from typing import Dict, Any, List, Optional, Callable, Awaitable

from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from ..core.services.core_api_service import CoreAPIService

logger = logging.getLogger(__name__)


# Global variable to store the CoreApiService instance
core_api = None


def create_rest_app() -> FastAPI:
    """
    Create and configure the FastAPI application.
    
    This is a factory function that creates a new FastAPI application
    instance each time it's called. It's used with the --factory flag
    in uvicorn to ensure clean startup/shutdown.
    
    Returns:
        Configured FastAPI application instance
    """
    # Create the FastAPI app
    app = FastAPI(
        title="FEAGI API",
        description="FEAGI (Flexible Extensible Artificial General Intelligence) REST API",
        version="2.1.0",
        docs_url="/docs",
        redoc_url="/redoc",
    )
    
    # Configure CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # Allow all origins
        allow_credentials=True,
        allow_methods=["*"],  # Allow all methods
        allow_headers=["*"],  # Allow all headers
    )
    
    # Initialize Core API Service if enabled
    global core_api
    core_api_available = os.environ.get("FEAGI_CORE_API_AVAILABLE", "0") == "1"
    
    if core_api_available:
        try:
            core_api = CoreAPIService()
            logger.info("Core API service initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Core API service: {e}")
            core_api = None
    
    # Connect to ZMQ if enabled
    zmq_enabled = os.environ.get("FEAGI_ZMQ_ENABLED", "0") == "1"
    if zmq_enabled:
        try:
            # We don't need to create a ZMQ client here since
            # the process manager will have started the ZMQ server
            logger.info("ZMQ connection enabled")
        except Exception as e:
            logger.error(f"Failed to connect to ZMQ server: {e}")
    
    # Register middleware
    @app.middleware("http")
    async def log_request_middleware(request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """Log details about each request and response."""
        start_time = time.time()
        method = request.method
        path = request.url.path
        
        try:
            response = await call_next(request)
            duration = time.time() - start_time
            logger.info(f"Method: {method}, Path: {path}, Status: {response.status_code}, Duration: {duration:.4f}s")
            return response
        except Exception as e:
            duration = time.time() - start_time
            logger.error(f"Method: {method}, Path: {path}, Error: {str(e)}, Duration: {duration:.4f}s")
            logger.error(traceback.format_exc())
            return JSONResponse(
                status_code=500,
                content={"detail": "Internal server error"}
            )
    
    # Register exception handler
    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException) -> JSONResponse:
        """Handle HTTP exceptions."""
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail}
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception) -> JSONResponse:
        """Handle general exceptions."""
        logger.error(f"Unhandled exception: {str(exc)}")
        logger.error(traceback.format_exc())
        return JSONResponse(
            status_code=500,
            content={"detail": "Internal server error"}
        )
    
    # Import and include API routers
    try:
        from .v1.router import router as router_v1
        app.include_router(router_v1, prefix="/api/v1")
    except ImportError as e:
        logger.error(f"Failed to import API v1 router: {e}")
    
    @app.get("/")
    async def root() -> Dict[str, Any]:
        """Root endpoint."""
        return {
            "name": "FEAGI API",
            "version": "2.1.0",
            "docs": "/docs",
            "status": "running"
        }
    
    @app.get("/status")
    async def status() -> Dict[str, Any]:
        """Server status endpoint."""
        return {
            "status": "running",
            "timestamp": time.time(),
            "core_api_available": core_api is not None,
            "zmq_enabled": zmq_enabled
        }
    
    return app 