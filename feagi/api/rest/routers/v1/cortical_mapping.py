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


from fastapi import APIRouter, HTTPException, Depends

from ...commons import *
from ...schemas import *

from feagi.evo.genome_properties import genome_properties
from feagi.api.rest.dependencies import get_core_api_service
from feagi.api.core.services.core_api_service import CoreAPIService


router = APIRouter()

# Helper to get state manager instance
# state = FeagiStateManager.instance()


# @router.post("/v1/feagi/genome/cortical_mappings")
# async def add_cortical_mapping(cortical_area):
#     """
#     Returns the list of cortical areas downstream to the given cortical areas
#     """
#     return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
#


@router.post("/efferents")
async def fetch_cortical_mappings(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    return core_api_service.get_efferent_mappings(cortical_id.cortical_id)


@router.post("/afferents")
async def fetch_cortical_mappings(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the list of cortical areas upstream to the given cortical areas
    """
    return core_api_service.get_afferent_mappings(cortical_id.cortical_id)


@router.post("/cortical_mappings_by_name")
async def fetch_cortical_mappings(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    return core_api_service.get_cortical_mappings_by_name(cortical_id.cortical_id)


@router.post("/cortical_mappings_detailed")
async def fetch_cortical_mappings(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        return core_api_service.get_detailed_mapping_targets(cortical_id.cortical_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/mapping_properties")
async def fetch_cortical_mapping_properties(
    source_destination: CorticalAreaSrcDst, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the properties of mapping between two cortical areas
    """
    return core_api_service.get_cortical_mapping_properties(
        source_destination.src_cortical_area, 
        source_destination.dst_cortical_area
    )


@router.put("/mapping_properties")
async def update_cortical_mapping_properties(
    cortical_mapping_properties: UpdateCorticalMappingProperties,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Updates the properties of mapping between two cortical areas
    """
    success = core_api_service.update_cortical_mapping_properties(
        cortical_mapping_properties.src_cortical_area,
        cortical_mapping_properties.dst_cortical_area,
        cortical_mapping_properties.mapping_string
    )
    if not success:
        raise HTTPException(status_code=400, detail="Failed to update mapping properties")
    return {"success": True}


@router.get("/cortical_map")
async def connectome_cortical_map(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the full cortical map with connection counts between areas
    """
    return core_api_service.get_cortical_map()


@router.delete("/delete_suggested_mappings")
async def delete_suggested_mapping(
    mapping_data: SuggestedMapping,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Deletes suggested mapping hint associated with a brain region
    """
    try:
        success = core_api_service.modify_region_suggested_mappings(
            region_id=mapping_data.brain_region_id,
            mapping_type=mapping_data.mapping_type,
            mapping_definitions=mapping_data.mapping_definitions,
            operation="delete"
        )
        if not success:
            raise HTTPException(status_code=400, detail="Failed to delete suggested mappings")
        return {"success": True}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
