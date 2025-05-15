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
async def genome_cortical_location_by_name(
    cortical_name: CorticalName,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the 3D location of a cortical area by name.
    """
    try:
        name = cortical_name.cortical_name
        return core_api_service.get_cortical_location_by_name(name)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Cortical area with name '{name}' not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical location: {str(e)}")


@router.get("/cortical_area_name_list")
async def genome_cortical_names(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        name_list = core_api_service.get_cortical_area_name_list()
        if name_list:
            return sorted(name_list)
        return []
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical area names: {str(e)}")


@router.get("/cortical_types")
async def cortical_area_types(core_api_service: CoreAPIService = Depends(get_core_api_service)):
    """
    Returns the list of supported cortical types
    """
    return core_api_service.get_cortical_area_types()


@router.post("/cortical_type_options")
async def cortical_area_types(
    cortical_type: CorticalId, 
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the list of supported cortical area for a given type
    """
    try:
        # Get cortical types from service
        cortical_types_dict = core_api_service.get_cortical_area_types()
        type_id = cortical_type.cortical_id
        
        if type_id in cortical_types_dict:
            cortical_list = set()
            for item in cortical_types_dict[type_id]['supported_devices']:
                if cortical_types_dict[type_id]['supported_devices'][item]['enabled']:
                    cortical_list.add(item)
            return cortical_list
        else:
            return []
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical type options: {str(e)}")


@router.get("/cortical_id_name_mapping")
async def connectome_cortical_id_name_mapping_table(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """Return a mapping of cortical IDs to their corresponding names"""
    try:
        # Get all cortical areas from the service
        areas = core_api_service.get_cortical_areas()
        
        # Create ID to name mapping
        mapping_table = {}
        for area in areas:
            area_id = area.get("id")
            name = area.get("name")
            if area_id and name:
                mapping_table[area_id] = name
                
        return mapping_table
    except Exception as e:
        logger.error(f"Error getting cortical ID-name mapping: {str(e)}")
        return {}


@router.get("/cortical_locations_2d")
async def cortical_2d_locations(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get 2D coordinates for all cortical areas for visualization
    """
    try:
        return core_api_service.get_cortical_2d_locations()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving 2D locations: {str(e)}")


@router.get("/cortical_area/geometry")
async def cortical_area_geometry(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get geometry information for all cortical areas
    """
    try:
        geometry = core_api_service.get_cortical_area_geometry()
        if geometry:
            return geometry
        else:
            return {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical geometry: {str(e)}")


@router.put("/coord_2d")
async def update_coord_2d(
    new_2d_coordinates: dict,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Updates 2D coordinates of one or more cortical areas
    
    Args:
        new_2d_coordinates: Dictionary mapping cortical area IDs to their new 2D coordinates
    """
    try:
        success = core_api_service.update_2d_coordinates(new_2d_coordinates)
        if not success:
            raise HTTPException(status_code=400, detail="Failed to update 2D coordinates")
        return {"status": "success", "message": "2D coordinates updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating 2D coordinates: {str(e)}")


@router.put("/coord_3d")
async def update_coord_3d(
    new_3d_coordinates: dict,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Updates 3D coordinates of one or more cortical areas
    
    Args:
        new_3d_coordinates: Dictionary mapping cortical area IDs to their new 3D coordinates
    """
    try:
        success = core_api_service.update_3d_coordinates(new_3d_coordinates)
        if not success:
            raise HTTPException(status_code=400, detail="Failed to update 3D coordinates")
        return {"status": "success", "message": "3D coordinates updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating 3D coordinates: {str(e)}")


@router.get("/ipu")
async def current_ipu_list(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get a list of all input processing units (IPUs)
    """
    try:
        ipu_list = core_api_service.get_ipu_list()
        return ipu_list if ipu_list else {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving IPU list: {str(e)}")


@router.get("/opu")
async def current_opu_list(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get a list of all output processing units (OPUs)
    """
    try:
        opu_list = core_api_service.get_opu_list()
        return opu_list if opu_list else {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving OPU list: {str(e)}")


@router.get("/cortical_map_detailed")
async def connectome_detailed_cortical_map(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get a detailed map of all cortical areas and their connections
    """
    try:
        return core_api_service.get_detailed_cortical_map()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving detailed cortical map: {str(e)}")


@router.get("/cortical_visibility")
async def fetch_visualized_cortical_list(
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get list of cortical areas that are currently being visualized
    """
    try:
        return core_api_service.get_cortical_visualization_list()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical visualization list: {str(e)}")


@router.put("/suppress_cortical_visibility")
async def suppress_cortical_activity_visualization(
    cortical_id_list: list,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Update which cortical areas should be visualized
    
    Args:
        cortical_id_list: List of cortical area IDs to visualize
    """
    try:
        result = core_api_service.update_cortical_visualization(cortical_id_list)
        if isinstance(result, set) and result:
            # Some cortical areas were not found
            return JSONResponse(
                status_code=400, 
                content={
                    'message': f"Following cortical ids were not found!\n {result}"
                }
            )
        return {"status": "success", "message": "Cortical visualization updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating cortical visualization: {str(e)}")


@router.post("/multi/cortical_area_properties")
async def fetch_multiple_cortical_properties(
    cortical_id_list: CorticalIdList,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Returns the properties of multiple cortical areas at once
    
    Args:
        cortical_id_list: List of cortical area IDs to fetch properties for
    """
    try:
        results = core_api_service.get_multiple_cortical_properties(cortical_id_list.cortical_id_list)
        if not results:
            raise HTTPException(status_code=404, detail="One or more cortical areas not found")
        return results
    except ValueError as e:
        return generate_response("CORTICAL_AREA_INVALID_ID_LENGTH")
    except KeyError as e:
        return generate_response("CORTICAL_AREA_NOT_FOUND")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving cortical properties: {str(e)}")


@router.put("/multi/cortical_area")
async def update_multiple_cortical_properties(
    message: UpdateMultipleCorticalProperties,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Updates properties for multiple cortical areas at the same time
    
    Args:
        message: Properties to update and list of cortical areas to update
    """
    try:
        success = core_api_service.update_multiple_cortical_properties(message)
        if not success:
            return JSONResponse(
                status_code=400, 
                content={
                    'message': "Failed to update cortical areas. Check logs for details."
                }
            )
        return {"status": "success", "message": "Update request submitted for the specified cortical areas"}
    except ValueError as e:
        # Handle specific errors
        return JSONResponse(
            status_code=400,
            content={'message': str(e)}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating cortical areas: {str(e)}")


@router.delete("/multi/cortical_area")
async def delete_multiple_cortical_areas(
    cortical_id_list: CorticalIdList,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Deletes multiple cortical areas at the same time
    
    Args:
        cortical_id_list: List of cortical area IDs to delete
    """
    try:
        result = core_api_service.delete_multiple_cortical_areas(cortical_id_list.cortical_id_list)
        if isinstance(result, list) and result:
            # Some cortical areas were not found
            return generate_response("CORTICAL_AREA_NOT_FOUND")
        return {"status": "success", "message": "Delete request submitted for the specified cortical areas"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting cortical areas: {str(e)}")


@router.get("/neuron_count")
async def area_neuron_count(
    cortical_id: str,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Get the number of neurons in a cortical area
    
    Args:
        cortical_id: ID of the cortical area
    """
    try:
        count = core_api_service.get_cortical_area_neuron_count(cortical_id)
        if count is None:
            raise HTTPException(status_code=404, detail=f"Cortical area {cortical_id} not found")
        return count
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving neuron count: {str(e)}")


@router.put("/reset")
async def reset_cortical_area(
    cortical_list: CorticalList,
    core_api_service: CoreAPIService = Depends(get_core_api_service)
):
    """
    Reset the specified cortical areas
    
    Args:
        cortical_list: List of cortical area IDs to reset
    """
    try:
        success = core_api_service.reset_cortical_areas(cortical_list.area_list)
        if not success:
            raise HTTPException(status_code=400, detail="Failed to reset one or more cortical areas")
        return {"status": "success", "message": "Reset request submitted for the specified cortical areas"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error resetting cortical areas: {str(e)}")
