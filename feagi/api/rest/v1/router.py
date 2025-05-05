"""Main router for FEAGI REST API v0."""

from fastapi import APIRouter, Depends, Request, Body, HTTPException
from fastapi.responses import JSONResponse, Response
from fastapi.routing import APIRoute

from feagi.api.rest.routers.v0.cortical_area import router as cortical_area_router, CorticalAreaCreate, CorticalId
from feagi.api.rest.routers.v0.genome import router as genome_router
from feagi.api.rest.routers.v0.simulation import router as simulation_router
from feagi.api.rest.routers.v0.system import router as system_router
from feagi.api.rest.routers.v0.morphology import router as morphology_router
from feagi.api.rest.routers.v0.neuroplasticity import router as neuroplasticity_router
from feagi.api.rest.routers.v0.connectome import router as connectome_router
from feagi.api.rest.routers.v0.burst_engine import router as burst_engine_router
from feagi.api.rest.routers.v0.inputs import router as inputs_router
from feagi.api.rest.routers.v0.region import router as region_router
from feagi.api.rest.routers.v0.insights import router as insights_router
from feagi.api.rest.routers.v0.cortical_mapping import router as cortical_mapping_router
from feagi.api.rest.app import get_core_api
from feagi.api.core.services import CoreAPIService

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

# Create legacy API aliases for backward compatibility
# Legacy FEAGI uses cortical-areas (with hyphen) instead of cortical_area
legacy_cortical_areas_router = APIRouter(prefix="/cortical_areas", tags=["CORTICAL AREAS"])

# Add the legacy routers to the main router
router.include_router(legacy_cortical_areas_router)
router.include_router(legacy_cortical_area_types_router)
router.include_router(legacy_genome_router)
router.include_router(legacy_burst_engine_router)

app.include_router(
    genome.router,
    prefix="/v0/genome",
    tags=["GENOME"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    connectome.router,
    prefix="/v0/connectome",
    tags=["CONNECTOME"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    burst_engine.router,
    prefix="/v0/burst_engine",
    tags=["BURST ENGINE"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    evolution.router,
    prefix="/v0/evolution",
    tags=["EVOLUTIONARY"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    feagi_agent.router,
    prefix="/v0/agent",
    tags=["FEAGI AGENT"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    insights.router,
    prefix="/v0/insight",
    tags=["INSIGHTS"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    morphology.router,
    prefix="/v0/morphology",
    tags=["NEURON MORPHOLOGIES"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    cortical_area.router,
    prefix="/v0/cortical_area",
    tags=["CORTICAL AREAS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    region.router,
    prefix="/v0/region",
    tags=["BRAIN REGIONS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    cortical_mapping.router,
    prefix="/v0/cortical_mapping",
    tags=["CORTICAL MAPPINGS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    neuroplasticity.router,
    prefix="/v0/neuroplasticity",
    tags=["NEUROPLASTICITY"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    inputs.router,
    prefix="/v0/input",
    tags=["INPUT MANAGEMENT"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    network.router,
    prefix="/v0/network",
    tags=["NETWORK"],
    dependencies=[],
    responses=standard_response
)

app.include_router(
    simulation.router,
    prefix="/v0/simulation",
    tags=["SIMULATION"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    system.router,
    prefix="/v0/system",
    tags=["SYSTEM"],
    dependencies=[],
    responses=standard_response
)
app.include_router(
    training.router,
    prefix="/v0/training",
    tags=["TRAINING"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

