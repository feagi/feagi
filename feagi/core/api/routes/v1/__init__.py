"""V1 routes for FEAGI API."""
from fastapi import FastAPI, APIRouter

def register_routes(app: FastAPI) -> None:
    """
    Register all v1 routes to the FastAPI application.
    
    Args:
        app: FastAPI application
    """
    # Create a v1 router
    v1_router = APIRouter(prefix="/v1")
    
    # Import and register functional area routers
    from .core import router as core_router
    from .connectome import router as connectome_router
    from .burst_engine import router as burst_engine_router
    
    # Add all functional area routers to the v1 router
    v1_router.include_router(core_router)
    v1_router.include_router(connectome_router)
    v1_router.include_router(burst_engine_router)
    
    # Add the v1 router to the app
    app.include_router(v1_router)
