# Copyright 2016-2024 Neuraville Inc. Authors. All Rights Reserved.
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


from fastapi import APIRouter, HTTPException, Depends

from ...commons import *
from ...schemas import *
from ...dependencies import check_brain_running

from src.inf import runtime_data
from src.evo.genome_properties import genome_properties
from src.evo.synapse import neighboring_cortical_areas


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


@router.post("/mapping_properties")
async def fetch_cortical_mapping_properties(source_destination: CorticalAreaSrcDst):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    src_cortical_area = source_destination.src_cortical_area
    dst_cortical_area = source_destination.dst_cortical_area
    if dst_cortical_area in runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst']:
        return runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]
    else:
        return []


@router.put("/mapping_properties")
async def update_cortical_mapping_properties(cortical_mapping_properties: UpdateCorticalMappingProperties,
                                             _: str = Depends(check_brain_running)):
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


@router.delete("/delete_suggested_mappings")
async def delete_suggested_mapping(mapping_data: SuggestedMapping):
    """
    Deletes suggested mapping hint associated with a brain region
    """
    region_id = mapping_data.brain_region_id
    mapping_type = mapping_data.mapping_type

    if region_id in runtime_data.genome["brain_regions"]:
        if mapping_type in ["inputs", "outputs"]:
            for mapping_definition in mapping_data.mapping_definitions:
                if mapping_definition in runtime_data.genome["brain_regions"][region_id][mapping_type]:
                    runtime_data.genome["brain_regions"][region_id][mapping_type].remove(mapping_definition)
                else:
                    raise HTTPException(status_code=400, detail=f"Mapping definition not found!")
        else:
            raise HTTPException(status_code=400, detail=f"Mapping type can be defined as either 'inputs' or 'outputs' ")
    else:
        raise HTTPException(status_code=400, detail=f"Brain region id {region_id} is not valid.")
