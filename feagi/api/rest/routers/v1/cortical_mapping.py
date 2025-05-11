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
from feagi.bdu import ConnectomeManager
from feagi.api.rest.dependencies import get_connectome


router = APIRouter()

# Helper to get state manager instance
# state = FeagiStateManager.instance()


# @router.post("/v0/feagi/genome/cortical_mappings")
# async def add_cortical_mapping(cortical_area):
#     """
#     Returns the list of cortical areas downstream to the given cortical areas
#     """
#     return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
#


@router.post("/efferents")
async def fetch_cortical_mappings(cortical_id: CorticalId, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    neuron_id = cortical_id.cortical_id
    return connectome.get_outgoing_connections(neuron_id)


@router.post("/afferents")
async def fetch_cortical_mappings(cortical_id: CorticalId, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    neuron_id = cortical_id.cortical_id
    return connectome.get_incoming_connections(neuron_id)


@router.post("/cortical_mappings_by_name")
async def fetch_cortical_mappings(cortical_id: CorticalId, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    neuron_id = cortical_id.cortical_id
    mappings = set()
    for dst_id, _ in connectome.get_outgoing_connections(neuron_id):
        # Look up the name from the area if available
        area = connectome._areas.get(dst_id)
        if area:
            mappings.add(area.name)
        else:
            mappings.add(str(dst_id))
    return list(mappings)


@router.post("/cortical_mappings_detailed")
async def fetch_cortical_mappings(cortical_id: CorticalId, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    neuron_id = cortical_id.cortical_id
    connections = connectome.get_outgoing_connections(neuron_id)
    if connections:
        return [dst_id for dst_id, _ in connections]
    else:
        raise HTTPException(status_code=400, detail=f"Cortical area with id={neuron_id} not found!")


@router.post("/mapping_properties")
async def fetch_cortical_mapping_properties(source_destination: CorticalAreaSrcDst, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    src = source_destination.src_cortical_area
    dst = source_destination.dst_cortical_area
    return connectome.synapse_manager.get_synapse_info(src, dst)


@router.put("/mapping_properties")
async def update_cortical_mapping_properties(cortical_mapping_properties: UpdateCorticalMappingProperties):
    if not state.is_connectome_ready():
        raise HTTPException(status_code=400, detail="Connectome is not ready!")
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
    for neuron_id in connectome._neuron_id_to_index.keys():
        cortical_map[neuron_id] = dict()
        for dst_id, _ in connectome.get_outgoing_connections(neuron_id):
            if dst_id not in cortical_map[neuron_id]:
                cortical_map[neuron_id][dst_id] = 0
            cortical_map[neuron_id][dst_id] += 1
    return cortical_map


@router.delete("/delete_suggested_mappings")
async def delete_suggested_mapping(mapping_data: SuggestedMapping):
    """
    Deletes suggested mapping hint associated with a brain region
    """
    region_id = mapping_data.brain_region_id
    mapping_type = mapping_data.mapping_type

    if region_id in state.genome["brain_regions"]:
        if mapping_type in ["inputs", "outputs"]:
            for mapping_definition in mapping_data.mapping_definitions:
                if mapping_definition in state.genome["brain_regions"][region_id][mapping_type]:
                    state.genome["brain_regions"][region_id][mapping_type].remove(mapping_definition)
                else:
                    raise HTTPException(status_code=400, detail=f"Mapping definition not found!")
        else:
            raise HTTPException(status_code=400, detail=f"Mapping type can be defined as either 'inputs' or 'outputs' ")
    else:
        raise HTTPException(status_code=400, detail=f"Brain region id {region_id} is not valid.")
