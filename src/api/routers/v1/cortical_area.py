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


from fastapi import APIRouter, HTTPException, status
from fastapi.responses import JSONResponse

from ...schemas import *
from ...commons import *

from src.inf import runtime_data
from src.api.error_handling import generate_response
from src.evo.voxels import generate_cortical_dimensions_by_id
from src.evo.genome_properties import genome_properties
from src.evo.x_genesis import add_core_cortical_area, add_custom_cortical_area
from src.evo.neuroembryogenesis import cortical_name_list, cortical_name_to_id
from src.evo.templates import cortical_types
from src.evo.region import change_cortical_area_parent


router = APIRouter()


@router.post("/cortical_area_properties")
async def fetch_cortical_properties(cortical_id: CorticalId):
    """
    Returns the properties of cortical areas
    """
    cortical_area = cortical_id.cortical_id
    if len(cortical_area) == genome_properties["structure"]["cortical_id_length"]:
        if cortical_area in runtime_data.genome['blueprint']:
            cortical_data = runtime_data.genome['blueprint'][cortical_area]
            brain_region_id = runtime_data.cortical_area_region_association[cortical_area]
            brain_region_title = runtime_data.genome["brain_regions"][brain_region_id]["title"]

            if 'mp_charge_accumulation' not in cortical_data:
                cortical_data['mp_charge_accumulation'] = True

            if 'mp_driven_psp' not in cortical_data:
                cortical_data['mp_driven_psp'] = False

            if '2d_coordinate' not in cortical_data:
                cortical_data['2d_coordinate'] = list()
                cortical_data['2d_coordinate'].append(None)
                cortical_data['2d_coordinate'].append(None)

            cortical_properties = {
                "cortical_id": cortical_area,
                "cortical_name": cortical_data['cortical_name'],
                "parent_region_id": brain_region_id,
                "parent_region_title": brain_region_title,
                "cortical_group": cortical_data['group_id'],
                "cortical_sub_group": cortical_data['sub_group_id'],
                "cortical_neuron_per_vox_count": cortical_data['per_voxel_neuron_cnt'],
                "cortical_visibility": cortical_area in runtime_data.cortical_viz_list,
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
                "cortical_destinations": cortical_data['cortical_mapping_dst'],
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
                "neuron_leak_variability": cortical_data['leak_variability'],
                "neuron_consecutive_fire_count": cortical_data['consecutive_fire_cnt_max'],
                "neuron_snooze_period": cortical_data['snooze_length'],
                "neuron_degeneracy_coefficient": cortical_data['degeneration'],
                "neuron_psp_uniform_distribution": cortical_data['psp_uniform_distribution'],
                "neuron_mp_charge_accumulation": cortical_data['mp_charge_accumulation'],
                "neuron_mp_driven_psp": cortical_data['mp_driven_psp'],
                "neuron_longterm_mem_threshold": cortical_data['longterm_mem_threshold'],
                "neuron_lifespan_growth_rate": cortical_data['lifespan_growth_rate'],
                "neuron_init_lifespan": cortical_data['init_lifespan'],
                "neuron_excitability": cortical_data['neuron_excitability'],
                "transforming": False
            }
            if cortical_area in runtime_data.transforming_areas:
                cortical_properties["transforming"] = True
            else:
                cortical_properties["transforming"] = False
            return cortical_properties
        else:
            return generate_response("CORTICAL_AREA_NOT_FOUND")
    else:
        return generate_response("CORTICAL_AREA_INVALID_ID_LENGTH")


@router.put("/cortical_area")
async def update_cortical_properties(message: UpdateCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """

    current_cortical_size = runtime_data.genome["blueprint"][message.cortical_id]["block_boundaries"][0] * \
                            runtime_data.genome["blueprint"][message.cortical_id]["block_boundaries"][1] * \
                            runtime_data.genome["blueprint"][message.cortical_id]["block_boundaries"][2]
    updated_cortical_size = current_cortical_size

    if message.cortical_dimensions:
        updated_cortical_size = message.cortical_dimensions[0] * \
                            message.cortical_dimensions[1] * \
                            message.cortical_dimensions[2]

    current_neuron_density = runtime_data.genome["blueprint"][message.cortical_id]["per_voxel_neuron_cnt"]
    updated_neuron_density = current_neuron_density

    if message.cortical_neuron_per_vox_count:
        updated_neuron_density = message.cortical_neuron_per_vox_count

    if message.parent_region_id:
        change_cortical_area_parent(cortical_area_id=message.cortical_id, new_parent_id=message.parent_region_id)

    current_neuron_count = current_cortical_size * current_neuron_density
    updated_neuron_count = updated_cortical_size * updated_neuron_density

    max_allowable_neuron_count = int(runtime_data.parameters["Limits"]["max_neuron_count"])

    if runtime_data.brain_stats["neuron_count"] - current_neuron_count + updated_neuron_count > \
            max_allowable_neuron_count:
        return JSONResponse(status_code=400, content={'message': f"Cannot create new cortical area as neuron count will"
                                                                 f" exceed {max_allowable_neuron_count} threshold"})

    if message.cortical_id in runtime_data.transforming_areas:
        return generate_response("CORTICAL_AREA_UNDERGOING_TRANSFORMATION")
    else:
        runtime_data.transforming_areas.add(message.cortical_id)
        message = message.dict()
        message = {'update_cortical_properties': message}
        print("*-----* " * 200 + "\n", message)
        api_queue.put(item=message)


@router.post("/cortical_area")
async def add_cortical_area(new_cortical_properties: NewCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """

    print("Adding core cortical area:\n", new_cortical_properties)
    # message = message.dict()
    # message = {'add_core_cortical_area': message}
    # print("*" * 50 + "\n", message)
    # api_queue.put(item=message)
    new_cortical_properties = dict(new_cortical_properties)
    cortical_id = add_core_cortical_area(cortical_properties=new_cortical_properties)
    # response.status_code = status.HTTP_200_OK
    return JSONResponse(status_code=200, content={'cortical_id': cortical_id})


@router.post("/custom_cortical_area")
async def add_cortical_area_custom(new_custom_cortical_properties: NewCustomCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    cortical_name = new_custom_cortical_properties.cortical_name
    parent_region_id = new_custom_cortical_properties.parent_region_id
    coordinates_3d = new_custom_cortical_properties.coordinates_3d.copy()
    coordinates_2d = new_custom_cortical_properties.coordinates_2d.copy()
    sub_group_id = new_custom_cortical_properties.sub_group_id
    copy_of = new_custom_cortical_properties.copy_of
    if "MEMORY" in sub_group_id:
        is_memory = True
        cortical_dimensions = [1, 1, 1]
    else:
        is_memory = False
        cortical_dimensions = new_custom_cortical_properties.cortical_dimensions

    neuron_density = 1
    if copy_of:
        neuron_density = runtime_data.genome["blueprint"][copy_of]["per_voxel_neuron_cnt"]

    neuron_count = neuron_density * cortical_dimensions[0] * cortical_dimensions[1] * cortical_dimensions[2]
    max_allowable_neuron_count = int(runtime_data.parameters["Limits"]["max_neuron_count"])
    if neuron_count + runtime_data.brain_stats["neuron_count"] > max_allowable_neuron_count:
        return JSONResponse(status_code=400, content={'message': f"Cannot create new cortical area as neuron count will"
                                                                 f" exceed {max_allowable_neuron_count} threshold"})

    if parent_region_id not in runtime_data.genome["brain_regions"]:
        return JSONResponse(status_code=400, content={'message': f"{parent_region_id} does not exist!"})

    cortical_id = add_custom_cortical_area(cortical_name=cortical_name,
                                           coordinates_3d=coordinates_3d,
                                           coordinates_2d=coordinates_2d,
                                           cortical_dimensions=cortical_dimensions,
                                           parent_region_id=parent_region_id,
                                           is_memory=is_memory,
                                           copy_of=copy_of)
    return JSONResponse(status_code=200, content={'cortical_id': cortical_id})


@router.delete("/cortical_area")
async def delete_cortical_area(cortical_id: CorticalId):
    """
    Enables changes against various Burst Engine parameters.
    """
    cortical_id = cortical_id.cortical_id
    if cortical_id in runtime_data.genome["blueprint"]:

        message = {'delete_cortical_area': cortical_id}
        api_queue.put(item=message)


@router.get("/cortical_area_id_list")
async def genome_cortical_ids():
    """
    Returns a comprehensive list of all cortical area names.
    """
    if runtime_data.cortical_list:
        return sorted(runtime_data.cortical_list)
    else:
        return []


@router.post("/cortical_name_location")
async def genome_cortical_location_by_name(cortical_name: CorticalName):
    """
    Returns a comprehensive list of all cortical area names.
    """
    cortical_name = cortical_name.cortical_name
    cortical_area = cortical_name_to_id(cortical_name=cortical_name)
    return runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"]


@router.get("/cortical_area_name_list")
async def genome_cortical_names():
    """
    Returns a comprehensive list of all cortical area names.
    """
    if cortical_name_list:
        return sorted(cortical_name_list())


@router.get("/cortical_types")
async def cortical_area_types():
    """
    Returns the list of supported cortical types
    """
    if runtime_data.cortical_defaults:
        return runtime_data.cortical_defaults


@router.post("/cortical_type_options")
async def cortical_area_types(cortical_type: CorticalId):
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
async def connectome_cortical_id_name_mapping_table():
    mapping_table = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        mapping_table[cortical_area] = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
    return mapping_table


@router.get("/cortical_locations_2d")
async def cortical_2d_locations():
    """
    Enables changes against various Burst Engine parameters.
    """
    report = dict()
    for area in runtime_data.genome["blueprint"]:
        if area not in report:
            report[area] = list()
        if "2d_coordinate" in runtime_data.genome['blueprint'][area]:
            report[area] = runtime_data.genome['blueprint'][area]["2d_coordinate"]
        else:
            report[area].append([None, None])
    return report


@router.get("/cortical_area/geometry")
async def cortical_area_geometry():
    if runtime_data.cortical_dimensions_by_id:
        return runtime_data.cortical_dimensions_by_id
    else:
        return {}


@router.put("/coord_2d")
async def update_coord_2d(new_2d_coordinates: dict):
    """
    Accepts a dictionary of 2D coordinates of one or more cortical areas and update them in genome.
    """

    for cortical_area in new_2d_coordinates:
        if cortical_area in runtime_data.genome["blueprint"]:
            runtime_data.genome["blueprint"][cortical_area]["2d_coordinate"][0] = \
                new_2d_coordinates[cortical_area][0]
            runtime_data.genome["blueprint"][cortical_area]["2d_coordinate"][1] = \
                new_2d_coordinates[cortical_area][1]

    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()


@router.put("/coord_3d")
async def update_coord_3d(new_3d_coordinates: dict):
    """
    Accepts a dictionary of 3D coordinates of one or more cortical areas and update them in genome.
    """
    for cortical_area in new_3d_coordinates:
        if cortical_area in runtime_data.genome["blueprint"]:
            runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][0] = \
                new_3d_coordinates[cortical_area][0]
            runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][1] = \
                new_3d_coordinates[cortical_area][1]
            runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][2] = \
                new_3d_coordinates[cortical_area][2]

    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()


@router.get("/ipu")
async def current_ipu_list():
    if runtime_data.ipu_list:
        return runtime_data.ipu_list
    else:
        return {}


@router.get("/opu")
async def current_opu_list():
    if runtime_data.opu_list:
        return runtime_data.opu_list
    else:
        return {}


@router.get("/cortical_map_detailed")
async def connectome_detailed_cortical_map():
    cortical_map = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_map[cortical_area] = dict()
        for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            cortical_map[cortical_area][dst] = list()
            for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst].append(mapping)

    return cortical_map


@router.get("/cortical_visibility")
async def fetch_visualized_cortical_list():
    return runtime_data.cortical_viz_list


@router.put("/cortical_visibility")
async def update_visualized_cortical_list(cortical_id_list: list):
    unprocessed_list = set()
    runtime_data.cortical_viz_list = set()
    for cortical_id in cortical_id_list:
        if cortical_id in runtime_data.cortical_list:
            runtime_data.cortical_viz_list.add(cortical_id)
        else:
            unprocessed_list.add(cortical_id)

    if unprocessed_list:
        return JSONResponse(status_code=400, content={'message': f"Following cortical ids were not found!\n "
                                                                 f"{unprocessed_list}"})
