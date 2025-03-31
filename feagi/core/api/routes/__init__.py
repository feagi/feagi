"""Routes module for FEAGI API."""
from fastapi import FastAPI

def register_routes(app: FastAPI) -> None:
    """
    Register all API routes to the FastAPI application.
    
    Args:
        app: FastAPI application
    """
    # Import the version-specific route registration functions
    from .v1 import register_routes as register_v1_routes
    from .v2 import register_routes as register_v2_routes
    
    # Register v1 routes
    register_v1_routes(app)
    
    # Register v2 routes
    register_v2_routes(app)
