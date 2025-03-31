"""V2 routes for FEAGI API."""
from fastapi import FastAPI, APIRouter

def register_routes(app: FastAPI) -> None:
    """
    Register all v2 routes to the FastAPI application.
    
    Args:
        app: FastAPI application
    """
    # Create a v2 router (placeholder for future development)
    v2_router = APIRouter(prefix="/v2")
    
    # Add the v2 router to the app
    app.include_router(v2_router)
