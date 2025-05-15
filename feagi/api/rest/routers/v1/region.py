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

from feagi.bdu.models.brain_region import create_region, update_region, delete_region_with_members, change_cortical_area_parent, change_brain_region_parent, relocate_region_members
from feagi.api.response_templates import generate_response
from feagi.evo.genome_properties import genome_properties
from feagi.evo.templates import cortical_types
from feagi.core.state_manager import FeagiStateManager


"""
[POST] create a new brain region: /v0/region/region
[PUT] Update brain region properties (title, coordinates) /v0/region/region
[GET] Update brain region properties (title, coordinates) /v0/region/region
[DELETE] Deletes a brain region /v0/region/region

[GET] list all brain regions (summary): /v0/TBD
[GET] list all brain regions and their members (comprehensive): /v0/TBD
[GET] list members of a given brain region: /v0/TBD
[PUT] associate a cortical area or brain region to another brain region: /v0/TBD


"""

router = APIRouter()
state = FeagiStateManager.instance()


@router.post("/region")
async def create_brain_region(region_data: NewRegionProperties):
    if region_data.parent_region_id not in state.genome["brain_regions"]:
        raise HTTPException(status_code=400, detail=f"{region_data.parent_region_id} is not a valid region id")
    else:
        region_id = create_region(region_data)
        return {"region_id": region_id}


@router.put("/region")
async def update_region_properties(region_data: UpdateRegionProperties):
    region_dict = region_data.__dict__
    unacceptable_root_fields = ["parent_region_id", "coordinates_2d", "coordinates_3d"]
    if region_data.region_id == "root":
        for field in unacceptable_root_fields:
            if field in region_dict:
                raise HTTPException(status_code=400, detail=f"{field} cannot be modified for root region")
    if "parent_region_id" in region_dict:
        region_dict.pop("parent_region_id")

    update_region(region_data=region_dict)


@router.get("/region")
async def view_region_properties(region_id):
    if region_id in state.genome["brain_regions"]:
        return state.genome["brain_regions"][region_id]
    else:
        raise HTTPException(status_code=400, detail=f"{region_id} is not a valid region id")


@router.delete("/region")
async def delete_region(region_id: Id):
    if region_id.id == "root":
        raise HTTPException(status_code=400, detail="Root region cannot be deleted")
    elif region_id.id in state.genome["brain_regions"]:
        region_parent = state.genome["brain_regions"][region_id.id]["parent_region_id"]
        for area_id in state.genome["brain_regions"][region_id.id].get("areas", []):
            change_cortical_area_parent(cortical_area_id=area_id,
                                        new_parent_id=region_parent)
            state.cortical_area_region_association[area_id] = region_parent
        for region_id_ in state.genome["brain_regions"][region_id.id].get("regions", []):
            change_brain_region_parent(region_id=region_id_,
                                       new_parent_id=region_parent)
        state.genome["brain_regions"].pop(region_id.id)
        state.genome["brain_regions"][region_parent]["regions"].remove(region_id.id)

    else:
        raise HTTPException(status_code=400, detail=f"{region_id.id} is not a valid region id")


@router.delete("/region_and_members")
async def delete_region_and_members(region_id: Id):
    if region_id.id == "root":
        raise HTTPException(status_code=400, detail="Root region cannot be deleted")
    elif region_id.id in state.genome["brain_regions"]:
        delete_region_with_members(region_id=region_id)
    else:
        raise HTTPException(status_code=400, detail=f"{region_id.id} is not a valid region id")


@router.get("/regions")
async def list_all_regions():
    region_summary = dict()
    region_list = state.genome["brain_regions"].keys()
    for region in region_list:
        region_summary[region] = {}
        region_summary[region]["coordinate_2d"] = state.genome["brain_regions"][region].get("coordinate_2d")
        region_summary[region]["coordinate_3d"] = state.genome["brain_regions"][region].get("coordinate_3d")

    return region_summary


@router.get("/regions_members")
async def list_all_regions_and_members():
    return state.genome["brain_regions"]


@router.get("/region_titles")
async def list_all_region_titles():
    title_list = []
    for region_id in state.genome["brain_regions"]:
        title_list.append((region_id, region_id_2_title(region_id=region_id)))
    return title_list


@router.put("/change_cortical_area_region")
async def update_cortical_area_region_association(association_data: RegionAssociation):
    if association_data.id not in state.genome["blueprint"]:
        raise HTTPException(status_code=400, detail=f"{association_data.id} is not a valid cortical area id")
    elif state.genome["blueprint"][association_data.id]["group_id"] in ["IPU", "OPU", "CORE"]:
        raise HTTPException(status_code=400, detail=f"{association_data.id} is not custom area and "
                                                    f"restricted to move")
    elif association_data.new_region_id not in state.genome["regions"]:
        raise HTTPException(status_code=400, detail=f"{association_data.new_region_id} is not a valid brain region  id")
    else:
        change_cortical_area_parent(cortical_area_id=association_data.id,
                                    new_parent_id=association_data.new_region_id)


@router.put("/change_region_parent")
async def update_brain_region_parent(association_data: RegionAssociation):
    if association_data.id not in state.genome["regions"]:
        raise HTTPException(status_code=400, detail=f"{association_data.id} is not a valid brain region id")
    elif association_data.new_region_id not in state.genome["regions"]:
        raise HTTPException(status_code=400, detail=f"{association_data.new_region_id} is not a valid brain region  id")
    else:
        change_brain_region_parent(region_id=association_data.id,
                                   new_parent_id=association_data.new_region_id)


@router.put("/relocate_members")
async def brain_region_member_relocation(relocation_data: dict):
    """
    Accepts a dictionary of 2D coordinates of one or more cortical areas and update them in genome.

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
    relocate_region_members(relocation_data=relocation_data)
