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

from feagi.api.response_templates import generate_response
from feagi.evo.genome_properties import genome_properties
from feagi.evo.templates import cortical_types
from feagi.bdu.models.brain_region import change_cortical_area_parent, create_region, update_region, delete_region_with_members, relocate_region_members
from feagi.bdu.connectivity.mapping_utils import get_detailed_cortical_map
from feagi.bdu import ConnectomeManager
from feagi.api.rest.dependencies import get_connectome, get_core_api_service, get_core_api
from feagi.core.state_manager import get_state_manager, ServiceState
from feagi.core.state_manager import FeagiStateManager
from feagi.api.core.services import CoreAPIService
from feagi.api.rest.common import raw_response

from ...schemas import CorticalId, CorticalIdList, NewCorticalProperties, NewCustomCorticalProperties, UpdateCorticalProperties, UpdateMultipleCorticalProperties, CorticalName, CorticalList
from ...commons import *


router = APIRouter()


@router.post("/cortical_area_properties")
async def fetch_cortical_properties(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the properties of a cortical area
    """
    try:
        return core_api_service.get_cortical_area_properties(cortical_id.cortical_id)
    except ValueError:
        return generate_response("CORTICAL_AREA_INVALID_ID_LENGTH")
    except KeyError:
        return generate_response("CORTICAL_AREA_NOT_FOUND")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical properties: {str(e)}")


@router.put("/cortical_area")
async def update_cortical_properties(
    message: UpdateCorticalProperties, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Update properties of a cortical area.
    """
    try:
        # Convert the Pydantic model to a dictionary, excluding None values
        properties = message.dict(exclude_none=True)
        cortical_id = properties.pop("cortical_id")
        
        # Use the CoreAPIService to update the properties
        success = core_api_service.update_cortical_area_properties(
            cortical_id=cortical_id,
            properties=properties
        )
        
        if not success:
            raise HTTPException(status_code=400, detail="Failed to update cortical area properties")
            
        return {"status": "success", "message": f"Cortical area {cortical_id} update request submitted"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating cortical area: {str(e)}")


@router.post("/cortical_area")
async def add_cortical_area(new_cortical_properties: NewCorticalProperties, connectome: ConnectomeManager = Depends(get_connectome)):
    if not connectome.is_connectome_ready():
        raise HTTPException(status_code=400, detail="Connectome is not ready!")
    print("Adding core cortical area:\n", new_cortical_properties)
    message = new_cortical_properties.dict()
    message = {'add_core_cortical_area': message}
    print("*" * 50 + "\n", message)
    connectome.add_core_cortical_area(message)
    return JSONResponse(status_code=200, content={'cortical_id': new_cortical_properties.cortical_id})


@router.post("/custom_cortical_area")
async def add_cortical_area_custom(new_custom_cortical_properties: NewCustomCorticalProperties, connectome: ConnectomeManager = Depends(get_connectome)):
    if not connectome.is_connectome_ready():
        raise HTTPException(status_code=400, detail="Connectome is not ready!")
    message = new_custom_cortical_properties.dict(exclude_none=True)

    # Generate Cortical ID
    # todo: instead of hard coding the length have the genome properties captured and reference instead
    cortical_name = new_custom_cortical_properties.cortical_name
    temp_name = cortical_name
    if len(cortical_name) < 3:
        temp_name = cortical_name + "000"

    parent_region_id = new_custom_cortical_properties.parent_region_id
    if parent_region_id not in connectome.genome["brain_regions"]:
        return JSONResponse(status_code=400, content={'message': f"{parent_region_id} does not exist!"})

    sub_group_id = new_custom_cortical_properties.sub_group_id
    copy_of = new_custom_cortical_properties.copy_of
    if "MEMORY" in sub_group_id:
        is_memory = True
        cortical_dimensions = [1, 1, 1]
    else:
        is_memory = False
        cortical_dimensions = new_custom_cortical_properties.cortical_dimensions

    message["is_memory"] = is_memory
    message["cortical_dimensions"] = cortical_dimensions

    cortical_id = connectome.generate_cortical_id(temp_name[:3], is_memory=is_memory)

    neuron_density = 1
    if copy_of:
        neuron_density = connectome.genome["blueprint"][copy_of]["per_voxel_neuron_cnt"]

    message["copy_of"] = copy_of
    message["cortical_id"] = cortical_id

    neuron_count = neuron_density * cortical_dimensions[0] * cortical_dimensions[1] * cortical_dimensions[2]
    max_allowable_neuron_count = int(connectome.parameters["Limits"]["max_neuron_count"])
    if neuron_count + connectome.brain_stats["neuron_count"] > max_allowable_neuron_count:
        return JSONResponse(status_code=400, content={'message': f"Cannot create new cortical area as neuron count will"
                                                         f" exceed {max_allowable_neuron_count} threshold"})

    message = {'add_custom_cortical_area': message}
    print("*-----* " * 200 + "\n", message)
    connectome.add_custom_cortical_area(message)

    return JSONResponse(status_code=200, content={'cortical_id': cortical_id})


@router.delete("/cortical_area")
async def delete_cortical_area(
    cortical_id: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Deletes a single cortical area
    """
    try:
        # Use the CoreAPIService to delete the cortical area
        result = core_api_service.delete_cortical_area(cortical_id.cortical_id)
        
        if not result:
            raise HTTPException(status_code=404, detail=f"Cortical area {cortical_id.cortical_id} not found or could not be deleted")
        
        return {"status": "success", "message": f"Cortical area {cortical_id.cortical_id} deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting cortical area: {str(e)}")


@router.get("/cortical_area_id_list")
async def cortical_area_id_list(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Return the list of cortical area IDs (6-letter strings) present in the current genome"""
    return core_api_service.get_cortical_area_id_list()


@router.get("/cortical_area_index_list")
async def cortical_area_index_list(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Return the list of cortical area indices (integers) used by the FCL"""
    return core_api_service.get_cortical_area_index_list()


@router.post("/cortical_name_location")
async def genome_cortical_location_by_name(cortical_name: CorticalName, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns a comprehensive list of all cortical area names.
    """
    cortical_name = cortical_name.cortical_name
    cortical_area = connectome.get_cortical_name_to_id(cortical_name)
    return connectome.genome["blueprint"][cortical_area]["relative_coordinate"]


@router.get("/cortical_area_name_list")
async def genome_cortical_names(connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns a comprehensive list of all cortical area names.
    """
    name_list = connectome.get_cortical_name_list()
    if name_list:
        return sorted(name_list)


@router.get("/cortical_types")
async def cortical_area_types(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the list of supported cortical types
    """
    return core_api_service.get_cortical_area_types()


@router.post("/cortical_type_options")
async def cortical_area_types(cortical_type: CorticalId, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the list of supported cortical area for a given type
    """

    if cortical_type in cortical_types:
        cortical_list = set()
        for item in cortical_types[cortical_type]['supported_devices']:
            if cortical_types[cortical_type]['supported_devices'][item]['enabled']:
                cortical_list.add(item)
        return cortical_list
    else:
        return []


@router.get("/cortical_id_name_mapping")
async def connectome_cortical_id_name_mapping_table(connectome: ConnectomeManager = Depends(get_connectome)):
    """Return a mapping of cortical IDs to their corresponding names"""
    try:
        mapping_table = {}
        
        # Directly use the cortical_id and name attributes from CorticalArea objects
        if hasattr(connectome, '_areas') and connectome._areas:
            for area in connectome._areas.values():
                if hasattr(area, 'cortical_id') and hasattr(area, 'name'):
                    mapping_table[area.cortical_id] = area.name
            
            if mapping_table:
                return mapping_table
        
        # Fallback: Use the _cortical_id_to_idx mapping if available
        if (hasattr(connectome, '_cortical_id_to_idx') and connectome._cortical_id_to_idx and 
            hasattr(connectome, '_areas') and connectome._areas):
            for cortical_id, idx in connectome._cortical_id_to_idx.items():
                if idx in connectome._areas:
                    mapping_table[cortical_id] = connectome._areas[idx].name
            
            if mapping_table:
                return mapping_table
        
        # Fallback: Try the genome blueprint
        if hasattr(connectome, 'genome') and connectome.genome and 'blueprint' in connectome.genome:
            # Extract unique cortical IDs from the blueprint
            cortical_ids = set()
            blueprint = connectome.genome['blueprint']
            
            # First identify all cortical IDs
            for gene_key in blueprint:
                if isinstance(gene_key, str) and '-' in gene_key:
                    parts = gene_key.split('-')
                    if len(parts) >= 2:
                        cortical_ids.add(parts[1])
            
            # Then find names for each ID
            for cortical_id in cortical_ids:
                for gene_key in blueprint:
                    if isinstance(gene_key, str) and f"-{cortical_id}-" in gene_key:
                        parts = gene_key.split("-")
                        if len(parts) >= 4 and parts[3] == "__name":
                            mapping_table[cortical_id] = blueprint[gene_key]
                            break
                # If no name found, use the ID as the name
                if cortical_id not in mapping_table:
                    mapping_table[cortical_id] = cortical_id
            
            return mapping_table

        # Return empty dict if we couldn't find any mappings
        return {}
    except Exception as e:
        logger.error(f"Error getting cortical ID-name mapping: {str(e)}")
        return {}


@router.get("/cortical_locations_2d")
async def cortical_2d_locations(connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Enables changes against various Burst Engine parameters.
    """
    report = dict()
    for area in connectome.genome["blueprint"]:
        if area not in report:
            report[area] = list()
        if "2d_coordinate" in connectome.genome['blueprint'][area]:
            report[area] = connectome.genome['blueprint'][area]["2d_coordinate"]
        else:
            report[area].append([None, None])
    return report


@router.get("/cortical_area/geometry")
async def cortical_area_geometry(connectome: ConnectomeManager = Depends(get_connectome)):
    if connectome.cortical_dimensions_by_id:
        return connectome.cortical_dimensions_by_id
    else:
        return {}


@router.put("/coord_2d")
async def update_coord_2d(new_2d_coordinates: dict, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Accepts a dictionary of 2D coordinates of one or more cortical areas and update them in genome.
    """

    for cortical_area in new_2d_coordinates:
        if cortical_area in connectome.genome["blueprint"]:
            connectome.genome["blueprint"][cortical_area]["2d_coordinate"][0] = \
                new_2d_coordinates[cortical_area][0]
            connectome.genome["blueprint"][cortical_area]["2d_coordinate"][1] = \
                new_2d_coordinates[cortical_area][1]

    connectome.cortical_dimensions_by_id = None


@router.put("/coord_3d")
async def update_coord_3d(new_3d_coordinates: dict, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Accepts a dictionary of 3D coordinates of one or more cortical areas and update them in genome.
    """
    for cortical_area in new_3d_coordinates:
        if cortical_area in connectome.genome["blueprint"]:
            connectome.genome["blueprint"][cortical_area]["relative_coordinate"][0] = \
                new_3d_coordinates[cortical_area][0]
            connectome.genome["blueprint"][cortical_area]["relative_coordinate"][1] = \
                new_3d_coordinates[cortical_area][1]
            connectome.genome["blueprint"][cortical_area]["relative_coordinate"][2] = \
                new_3d_coordinates[cortical_area][2]

    connectome.cortical_dimensions_by_id = None


@router.get("/ipu")
async def current_ipu_list(connectome: ConnectomeManager = Depends(get_connectome)):
    if connectome.ipu_list:
        return connectome.ipu_list
    else:
        return {}


@router.get("/opu")
async def current_opu_list(connectome: ConnectomeManager = Depends(get_connectome)):
    if connectome.opu_list:
        return connectome.opu_list
    else:
        return {}


@router.get("/cortical_map_detailed")
async def connectome_detailed_cortical_map(connectome: ConnectomeManager = Depends(get_connectome)):
    cortical_map = get_detailed_cortical_map(connectome)
    return cortical_map


@router.get("/cortical_visibility")
async def fetch_visualized_cortical_list(connectome: ConnectomeManager = Depends(get_connectome)):
    return connectome.cortical_viz_list


@router.put("/suppress_cortical_visibility")
async def suppress_cortical_activity_visualization(cortical_id_list: list, connectome: ConnectomeManager = Depends(get_connectome)):
    unprocessed_list = set()
    connectome.cortical_viz_list = set()
    for cortical_id in cortical_id_list:
        if cortical_id in connectome.cortical_list:
            connectome.cortical_viz_list.add(cortical_id)
        else:
            unprocessed_list.add(cortical_id)
            connectome.genome["blueprint"][cortical_id]["visualization"] = False

    if unprocessed_list:
        return JSONResponse(status_code=400, content={'message': f"Following cortical ids were not found!\n "
                                                                 f"{unprocessed_list}"})


@router.post("/multi/cortical_area_properties")
async def fetch_multiple_cortical_properties(cortical_id_list: CorticalIdList, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Returns the properties of multiple cortical areas
    """
    results = list()
    for cortical_area in cortical_id_list.cortical_id_list:
        if len(cortical_area) == genome_properties["structure"]["cortical_id_length"]:
            if cortical_area in connectome.genome['blueprint']:
                cortical_data = connectome.genome['blueprint'][cortical_area]
                brain_region_id = connectome.cortical_area_region_association[cortical_area]
                brain_region_title = connectome.genome["brain_regions"][brain_region_id]["title"]

                if 'mp_charge_accumulation' not in cortical_data:
                    cortical_data['mp_charge_accumulation'] = False

                if 'mp_driven_psp' not in cortical_data:
                    cortical_data['mp_driven_psp'] = False

                if '2d_coordinate' not in cortical_data:
                    cortical_data['2d_coordinate'] = list()
                    cortical_data['2d_coordinate'].append(None)
                    cortical_data['2d_coordinate'].append(None)

                leak_variability = cortical_data.get('leak_variability', 0)
                if not leak_variability:
                    leak_variability = 0

                cortical_properties = {
                    "cortical_id": cortical_area,
                    "cortical_name": cortical_data['cortical_name'],
                    "parent_region_id": brain_region_id,
                    "parent_region_title": brain_region_title,
                    "cortical_group": cortical_data['group_id'],
                    "cortical_sub_group": cortical_data['sub_group_id'],
                    "cortical_neuron_per_vox_count": cortical_data['per_voxel_neuron_cnt'],
                    "visualization": not (cortical_area in connectome.cortical_viz_list),
                    "cortical_synaptic_attractivity": cortical_data['synapse_attractivity'],
                    "coordinates_3d": [
                        cortical_data["relative_coordinate"][0],
                        cortical_data["relative_coordinate"][1],
                        cortical_data["relative_coordinate"][2]
                    ],
                    "coordinates_2d": [
                        cortical_data["2d_coordinate"][0],
                        cortical_data["2d_coordinate"][1]
                    ],
                    "cortical_dimensions": [
                        cortical_data["block_boundaries"][0],
                        cortical_data["block_boundaries"][1],
                        cortical_data["block_boundaries"][2]
                    ],
                    "cortical_destinations": connectome.get_outgoing_connections(cortical_area),
                    "neuron_post_synaptic_potential": cortical_data['postsynaptic_current'],
                    "neuron_post_synaptic_potential_max": cortical_data['postsynaptic_current_max'],
                    "neuron_fire_threshold": cortical_data['firing_threshold'],
                    "neuron_fire_threshold_increment": [
                        cortical_data['firing_threshold_increment_x'],
                        cortical_data['firing_threshold_increment_y'],
                        cortical_data['firing_threshold_increment_z']
                    ],
                    "neuron_firing_threshold_limit": cortical_data['firing_threshold_limit'],
                    "neuron_refractory_period": cortical_data['refractory_period'],
                    "neuron_leak_coefficient": cortical_data['leak_coefficient'],
                    "neuron_leak_variability": leak_variability,
                    "neuron_consecutive_fire_count": cortical_data['consecutive_fire_cnt_max'],
                    "neuron_snooze_period": cortical_data['snooze_length'],
                    "neuron_degeneracy_coefficient": cortical_data['degeneration'],
                    "neuron_psp_uniform_distribution": cortical_data['psp_uniform_distribution'],
                    "neuron_mp_charge_accumulation": cortical_data['mp_charge_accumulation'],
                    "neuron_mp_driven_psp": cortical_data['mp_driven_psp'],
                    "neuron_longterm_mem_threshold": cortical_data['longterm_mem_threshold'],
                    "neuron_lifespan_growth_rate": cortical_data['lifespan_growth_rate'],
                    "neuron_init_lifespan": cortical_data['init_lifespan'],
                    "temporal_depth": cortical_data['temporal_depth'],
                    "neuron_excitability": cortical_data['neuron_excitability'],
                    "transforming": False
                }

                cortical_type = connectome.get_cortical_area_type(cortical_area)
                if cortical_type in ["IPU", "OPU"]:
                    dev_count = connectome.genome["blueprint"][cortical_area]["dev_count"]
                    unit_dim_x = cortical_types[cortical_type]["supported_devices"][cortical_area]["resolution"][0]
                    unit_dim_y = cortical_types[cortical_type]["supported_devices"][cortical_area]["resolution"][1]
                    unit_dim_z = cortical_types[cortical_type]["supported_devices"][cortical_area]["resolution"][2]

                    cortical_properties["dev_count"] = dev_count
                    cortical_properties["cortical_dimensions_per_device"] = [unit_dim_x, unit_dim_y, unit_dim_z]

                if cortical_area in connectome.transforming_areas:
                    cortical_properties["transforming"] = True
                else:
                    cortical_properties["transforming"] = False
                results.append(cortical_properties)
            else:
                return generate_response("CORTICAL_AREA_NOT_FOUND")
        else:
            return generate_response("CORTICAL_AREA_INVALID_ID_LENGTH")
    return results


@router.put("/multi/cortical_area")
async def update_multiple_cortical_properties(message: UpdateMultipleCorticalProperties, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Updates properties for multiple cortical areas at the same time
    """

    cortical_id_list = message.cortical_id_list

    # Check to ensure all selected areas are of same type
    message_dict = message.dict(exclude_none=True)
    message_dict.pop("cortical_id_list")

    type_list = set()
    transforming = False
    for cortical_id in message.cortical_id_list:
        type_list.add(connectome.genome["blueprint"][cortical_id]["is_mem_type"])
        if "transforming" in connectome.genome["blueprint"][cortical_id]:
            if connectome.genome["blueprint"][cortical_id]["transforming"]:
                transforming = True
    if len(type_list) > 1:
        return JSONResponse(status_code=400, content={'message': f"Memory and non-memory type cortical areas cannot"
                                                         f"be edited at the same time"})

    if transforming:
        return generate_response("CORTICAL_AREA_UNDERGOING_TRANSFORMATION")

    multi_edit_payload = []

    # Proceed with updates
    for cortical_id in cortical_id_list:
        current_cortical_size = connectome.genome["blueprint"][cortical_id]["block_boundaries"][0] * \
                                connectome.genome["blueprint"][cortical_id]["block_boundaries"][1] * \
                                connectome.genome["blueprint"][cortical_id]["block_boundaries"][2]
        updated_cortical_size = current_cortical_size

        if message_dict.get("cortical_dimensions"):
            updated_cortical_size = message.cortical_dimensions[0] * \
                                message.cortical_dimensions[1] * \
                                message.cortical_dimensions[2]

        current_neuron_density = connectome.genome["blueprint"][cortical_id]["per_voxel_neuron_cnt"]
        updated_neuron_density = current_neuron_density

        if message_dict.get("cortical_neuron_per_vox_count"):
            updated_neuron_density = message.cortical_neuron_per_vox_count

        if message_dict.get("parent_region_id"):
            change_cortical_area_parent(cortical_area_id=cortical_id, new_parent_id=message.parent_region_id, connectome=connectome)

        current_neuron_count = current_cortical_size * current_neuron_density
        updated_neuron_count = updated_cortical_size * updated_neuron_density

        max_allowable_neuron_count = int(connectome.parameters["Limits"]["max_neuron_count"])

        if connectome.brain_stats["neuron_count"] - current_neuron_count + updated_neuron_count > \
                max_allowable_neuron_count:
            return JSONResponse(status_code=400, content={'message': f"Cannot create new cortical area as neuron count"
                                                             f" will exceed {max_allowable_neuron_count} "
                                                             f"threshold"})

        cortical_payload = message_dict.copy()
        cortical_payload["cortical_id"] = cortical_id
        multi_edit_payload.append(cortical_payload)

    message_ = {'update_multiple_cortical_properties': multi_edit_payload}
    api_queue.put(item=message_)


@router.delete("/multi/cortical_area")
async def delete_multiple_cortical_areas(cortical_id_list: CorticalIdList, connectome: ConnectomeManager = Depends(get_connectome)):
    """
    Deletes multiple cortical areas at the same time
    """
    for cortical_id in cortical_id_list.cortical_id_list:
        if cortical_id in connectome.genome["blueprint"]:

            message = {'delete_cortical_area': cortical_id}
            api_queue.put(item=message)
        else:
            return generate_response("CORTICAL_AREA_NOT_FOUND")


@router.get("/neuron_count")
async def area_neuron_count(cortical_id: str, connectome: ConnectomeManager = Depends(get_connectome)):
    if cortical_id in connectome.brain:
        return len(connectome.brain[cortical_id])


@router.put("/reset")
async def reset_cortical_area(cortical_list: CorticalList, connectome: ConnectomeManager = Depends(get_connectome)):
    for cortical_id in cortical_list.area_list:
        if cortical_id in connectome.cortical_list:
            message = {'reset': cortical_id}
            api_queue.put(item=message)
