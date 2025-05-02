"""FEAGI REST API v1 package."""

from feagi.api.rest.v1.router import router
from feagi.api.rest.v1.genome import router as genome_router
from feagi.api.rest.v1.cortical_area import router as cortical_area_router

__all__ = ["router", "genome_router", "cortical_area_router"] 