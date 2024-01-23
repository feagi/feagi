# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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


from fastapi import APIRouter, HTTPException

from ...commons import *
from ...schemas import *

from src.inf import runtime_data
from src.evo.genome_properties import genome_properties
from src.evo.x_genesis import neighboring_cortical_areas


router = APIRouter()


# @router.post("/v1/feagi/genome/cortical_mappings")
# async def add_cortical_mapping(cortical_area):
#     """
#     Returns the list of cortical areas downstream to the given cortical areas
#     """
#     return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
#


@router.post("/efferents")
async def fetch_cortical_mappings(cortical_id: CorticalId):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    cortical_area = cortical_id.cortical_id
    if len(cortical_area) == genome_properties["structure"]["cortical_id_length"]:
        cortical_mappings = set()
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            cortical_mappings.add(destination)
        return cortical_mappings
    else:
        raise HTTPException(status_code=400, detail="Wrong cortical id format!")


@router.post("/afferents")
async def fetch_cortical_mappings(cortical_id: CorticalId):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    cortical_area = cortical_id.cortical_id
    if len(cortical_area) == genome_properties["structure"]["cortical_id_length"]:
        upstream_cortical_areas, downstream_cortical_areas = \
            neighboring_cortical_areas(cortical_area, blueprint=runtime_data.genome["blueprint"])
        return upstream_cortical_areas
    else:
        raise HTTPException(status_code=400, detail="Wrong cortical id format!")


@router.post("/cortical_mappings_by_name")
async def fetch_cortical_mappings(cortical_id: CorticalId):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    cortical_area = cortical_id.cortical_id
    cortical_mappings = set()
    for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        cortical_mappings.add(runtime_data.genome['blueprint'][destination]['cortical_name'])

    return cortical_mappings


@router.post("/cortical_mappings_detailed")
async def fetch_cortical_mappings(cortical_id: CorticalId):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    cortical_area = cortical_id.cortical_id
    if runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
    else:
        raise HTTPException(status_code=400, detail=f"Cortical area with id={cortical_area} not found!")


@router.get("/mapping_properties")
async def fetch_cortical_mapping_properties(source_destination: CorticalAreaSrcDst):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    src_cortical_area = source_destination.src_cortical_area
    dst_cortical_area = source_destination.dst_cortical_area
    if dst_cortical_area in runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst']:
        return runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]
    else:
        raise HTTPException(status_code=400,
                            detail=f"{dst_cortical_area} is not a cortical destination of {src_cortical_area}!")


@router.put("/mapping_properties")
async def update_cortical_mapping_properties(cortical_mapping_properties: UpdateCorticalMappingProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    src_cortical_area = cortical_mapping_properties.src_cortical_area
    dst_cortical_area = cortical_mapping_properties.dst_cortical_area
    mapping_string = cortical_mapping_properties.mapping_string
    data = dict()
    data["mapping_data"] = mapping_string
    data["src_cortical_area"] = src_cortical_area
    data["dst_cortical_area"] = dst_cortical_area
    data = {'update_cortical_mappings': data}
    api_queue.put(item=data)


@router.get("/cortical_map")
async def connectome_cortical_map():
    cortical_map = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_map[cortical_area] = dict()
        for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            cortical_map[cortical_area][dst] = 0
            for _ in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst] += 1

    return cortical_map
