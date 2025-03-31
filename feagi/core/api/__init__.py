"""Core API module for FEAGI."""
from fastapi import FastAPI

def create_api() -> FastAPI:
    """
    Create and configure the FastAPI application.
    
    Returns:
        Configured FastAPI application
    """
    from .routes import register_routes
    
    app = FastAPI(
        title="FEAGI API",
        description="API for the Framework for Evolutionary Artificial General Intelligence",
        version="0.1.0",
    )
    
    # Register all routes
    register_routes(app)
    
    return app
