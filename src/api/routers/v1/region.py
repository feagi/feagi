
from fastapi import APIRouter, HTTPException, status
from fastapi.responses import JSONResponse

from ...schemas import *
from ...commons import *

from src.inf import runtime_data
from src.api.error_handling import generate_response
from src.inf.initialize import generate_cortical_dimensions_by_id
from src.evo.genome_properties import genome_properties
from src.evo.x_genesis import add_core_cortical_area, add_custom_cortical_area
from src.evo.neuroembryogenesis import cortical_name_list, cortical_name_to_id
from src.evo.templates import cortical_types


"""
[POST] create a new brain region: /v1/region/region
[PUT] Update brain region properties (title, coordinates) /v1/region/region
[GET] Update brain region properties (title, coordinates) /v1/region/region
[DELETE] Deletes a brain region /v1/region/region

[GET] list all brain regions (summary): /v1/TBD
[GET] list all brain regions and their members (comprehensive): /v1/TBD
[GET] list members of a given brain region: /v1/TBD
[PUT] associate a cortical area or brain region to another brain region: /v1/TBD


"""

router = APIRouter()


@router.post("/region")
async def create_brain_region(region_data: NewRegionProperties):
    pass


@router.put("/region")
async def update_region_properties(region_id: Id):
    pass


@router.get("/region")
async def view_region_properties(region_id: Id):
    pass


@router.delete("/region")
async def delete_region(region_id: Id):
    pass


@router.delete("/region_and_members")
async def delete_region_and_members(region_id: Id):
    pass


@router.get("/regions")
async def list_all_regions():
    pass


@router.get("/regions_members")
async def list_all_regions_and_members():
    pass


@router.put("/cortical_area_region")
async def update_cortical_area_region_association(association_data: RegionAssociation):
    pass
