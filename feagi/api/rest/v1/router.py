"""Main router for FEAGI REST API v1."""

from fastapi import APIRouter

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

# Create the main API router
router = APIRouter()

# Include all the sub-routers without duplicate prefixes
router.include_router(cortical_area_router)
router.include_router(genome_router)
router.include_router(simulation_router)
router.include_router(system_router)
router.include_router(morphology_router)
router.include_router(neuroplasticity_router)
router.include_router(connectome_router)
router.include_router(burst_engine_router)
router.include_router(inputs_router)
router.include_router(region_router)
router.include_router(insights_router)
router.include_router(cortical_mapping_router) 