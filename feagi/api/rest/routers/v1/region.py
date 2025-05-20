#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from fastapi import APIRouter, HTTPException, status, Depends
from fastapi.responses import JSONResponse
from feagi.utils.logger import setup_logger
logger = setup_logger()

from ...schemas import *
from ...commons import *

from feagi.api.response_templates import generate_response
from feagi.evo.genome_properties import genome_properties
from feagi.evo.templates import cortical_types
from feagi.core.state_manager import FeagiStateManager
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService

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
async def create_brain_region(
    region_data: NewRegionProperties,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Create a new brain region.
    """
    try:
        region_id = core_api_service.create_brain_region(region_data.dict())
        if region_id:
            return {"region_id": region_id}
        raise HTTPException(status_code=400, detail=f"Failed to create brain region")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/region")
async def update_region_properties(
    region_data: UpdateRegionProperties,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Update properties of a brain region.
    """
    try:
        success = core_api_service.update_brain_region(region_data.dict())
        if not success:
            raise HTTPException(status_code=400, detail=f"Failed to update brain region")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/region")
async def view_region_properties(
    region_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get properties of a brain region.
    """
    region = core_api_service.get_brain_region(region_id)
    if not region:
        raise HTTPException(status_code=400, detail=f"{region_id} is not a valid region id")
    return region


@router.delete("/region")
async def delete_region(
    region_id: Id,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Delete a brain region and reassign its members to its parent.
    """
    try:
        success = core_api_service.delete_brain_region(region_id.id)
        if not success:
            raise HTTPException(status_code=400, detail=f"Failed to delete region {region_id.id}")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/region_and_members")
async def delete_region_and_members(
    region_id: Id,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Delete a brain region and all its members.
    """
    try:
        success = core_api_service.delete_brain_region_with_members(region_id.id)
        if not success:
            raise HTTPException(status_code=400, detail=f"Failed to delete region {region_id.id} and members")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/regions")
async def list_all_regions(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    List all brain regions with summary information.
    """
    return core_api_service.list_brain_regions(include_members=False)


@router.get("/regions_members")
async def list_all_regions_and_members(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    List all brain regions with detailed member information.
    """
    return core_api_service.list_brain_regions(include_members=True)


@router.get("/region_titles")
async def list_all_region_titles(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    List all brain region titles.
    """
    return core_api_service.get_region_titles()


@router.put("/change_cortical_area_region")
async def update_cortical_area_region_association(
    association_data: RegionAssociation,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Change the brain region a cortical area belongs to.
    """
    try:
        success = core_api_service.update_region_association(
            association_data.cortical_area_id,
            association_data.region_id
        )
        if not success:
            raise HTTPException(status_code=400, detail="Failed to update cortical area association")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/change_region_parent")
async def update_brain_region_parent(
    association_data: RegionAssociation,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Change the parent brain region of another brain region.
    """
    try:
        # In the schema, it's using id and new_region_id for brain regions
        success = core_api_service.update_region_parent(
            association_data.id,
            association_data.new_region_id
        )
        if not success:
            raise HTTPException(status_code=400, detail="Failed to update brain region parent")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/relocate_members")
async def brain_region_member_relocation(
    relocation_data: dict,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Relocate brain region members (update coordinates and/or parent region).
    
    Input format:
    {
        "region_id_1": {
            "coordinate_2d": [10, 9],
            "parent_region_id": "fhafsihwfiuhr23r_b",
        },
        "region_id_2": {
            "coordinate_2d": [4, 93],
            "parent_region_id": "dhdfsihwfiuhr23r_b",
        },
        "cortical_area_id": {
            "coordinate_2d": [30, 29],
            "parent_region_id": "gdfsihwfiuhr23r_b",
        }
    }
    """
    try:
        success = core_api_service.relocate_region_members(relocation_data)
        if not success:
            raise HTTPException(status_code=400, detail="Failed to relocate region members")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
