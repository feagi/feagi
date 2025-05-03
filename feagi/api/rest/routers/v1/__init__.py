"""FEAGI REST API v1 routers package."""

from feagi.api.rest.routers.v1.cortical_area import router as cortical_area_router
from feagi.api.rest.routers.v1.genome import router as genome_router
from feagi.api.rest.routers.v1.simulation import router as simulation_router
from feagi.api.rest.routers.v1.system import router as system_router
from feagi.api.rest.routers.v1.morphology import router as morphology_router
from feagi.api.rest.routers.v1.neuroplasticity import router as neuroplasticity_router
from feagi.api.rest.routers.v1.connectome import router as connectome_router
from feagi.api.rest.routers.v1.burst_engine import router as burst_engine_router
from feagi.api.rest.routers.v1.inputs import router as inputs_router
from feagi.api.rest.routers.v1.region import router as region_router
from feagi.api.rest.routers.v1.insights import router as insights_router
from feagi.api.rest.routers.v1.cortical_mapping import router as cortical_mapping_router

__all__ = [
    "cortical_area_router", 
    "genome_router", 
    "simulation_router", 
    "system_router", 
    "morphology_router", 
    "neuroplasticity_router",
    "connectome_router",
    "burst_engine_router",
    "inputs_router",
    "region_router",
    "insights_router",
    "cortical_mapping_router"
] 