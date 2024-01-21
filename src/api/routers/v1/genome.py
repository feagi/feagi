import os
import json
from datetime import datetime
from fastapi import APIRouter, UploadFile, File, HTTPException
from fastapi.responses import JSONResponse
from starlette.responses import FileResponse

from ...schemas import *
from ...commons import *
from ....inf import runtime_data
from ....inf.initialize import generate_cortical_dimensions_by_id
from ....evo.genome_properties import genome_properties
from ....evo.x_genesis import neighboring_cortical_areas, add_core_cortical_area, add_custom_cortical_area
from ....evo.neuroembryogenesis import cortical_name_list, cortical_name_to_id
from ....evo.genome_processor import genome_2_1_convertor
from ....evo import synaptogenesis_rules
from ....evo.synapse import morphology_usage_list
from ....evo.templates import cortical_types
from ....evo.stats import circuit_size


router = APIRouter()


# ######  Genome Endpoints #########
# ##################################
@router.post("/v1/feagi/genome/upload/blank")
async def upload_blank_genome():

    with open("./evo/defaults/genome/blank_genome.json", "r") as genome_file:
        genome_data = json.load(genome_file)
        runtime_data.genome_file_name = "blank_genome.json"
    runtime_data.brain_readiness = False
    message = {'genome': genome_data}

    api_queue.put(item=message)


@router.post("/v1/feagi/genome/upload/default")
async def genome_default_upload():
    with open("./evo/static_genome.json", "r") as genome_file:
        genome_data = json.load(genome_file)
        runtime_data.genome_file_name = "static_genome.json"
    runtime_data.brain_readiness = False
    message = {'genome': genome_data}
    api_queue.put(item=message)


@router.post("/v1/feagi/genome/upload/file", tags=["Genome"])
async def genome_file_upload(file: UploadFile = File(...)):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a python file.
    """
    data = await file.read()
    runtime_data.brain_readiness = False
    runtime_data.genome_file_name = file.filename

    genome_str = json.loads(data)

    # genome_str = genome_str.replace('\'', '\"')
    # genome_str = data.decode("utf-8").split(" = ")[1]
    message = {'genome': genome_str}
    api_queue.put(item=message)


@router.get("/v1/feagi/genome/file_name")
async def genome_file_name():
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    genome_name = runtime_data.genome_file_name
    if genome_name:
        return genome_name
    else:
        return ""


@router.post("/v1/feagi/genome/upload/string")
async def genome_string_upload(genome: dict):
    message = {'genome': genome}
    api_queue.put(item=message)


@router.get("/v1/feagi/genome/download", tags=["Genome"])
async def genome_download():
    print("Downloading Genome...")

    file_name = "genome_" + datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".json"
    print(file_name)
    if runtime_data.genome:
        return FileResponse(path=runtime_data.connectome_path + "genome.json", filename=file_name)
    else:
        raise HTTPException(status_code=404, detail="No running genome found!")


@router.post("/v1/feagi/genome/upload/file/edit")
async def genome_file_upload_edit(file: UploadFile = File(...)):
    data = await file.read()
    genome_str = data.decode("utf-8")
    return {genome_str}


@router.get("/v1/feagi/genome/defaults/files", tags=["Genome"])
async def genome_default_files():
    default_genomes_path = "./evo/defaults/genome/"
    default_genomes = os.listdir(default_genomes_path)
    genome_mappings = {}
    for genome in default_genomes:
        if genome[:2] != '__':
            with open(os.path.join(default_genomes_path, genome)) as file:
                data = file.read()
                # genome_mappings = json.loads(data)
                # data_dict = literal_eval(data.split(" = ")[1])
                genome_mappings[genome.split(".")[0]] = json.loads(data)
                # print("genome_mappings\n", genome_mappings)
    return {"genome": genome_mappings}


@router.get("/v1/feagi/genome/genome_number")
async def genome_number():
    """
    Return the number associated with current Genome instance.
    """
    if runtime_data.genome_counter:
        return runtime_data.genome_counter


@router.post("/v1/feagi/genome/reset")
async def reset_genome():
    print("API call has triggered a genome reset")
    runtime_data.genome_reset_flag = True


@router.get("/v1/feagi/genome/cortical_area")
async def fetch_cortical_properties(cortical_area):
    """
    Returns the properties of cortical areas
    """

    if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
        if cortical_area in runtime_data.genome['blueprint']:
            cortical_data = runtime_data.genome['blueprint'][cortical_area]

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
                "cortical_group": cortical_data['group_id'],
                "cortical_sub_group": cortical_data['sub_group_id'],
                "cortical_neuron_per_vox_count": cortical_data['per_voxel_neuron_cnt'],
                "cortical_visibility": cortical_data['visualization'],
                "cortical_synaptic_attractivity": cortical_data['synapse_attractivity'],
                "cortical_coordinates": [
                    cortical_data["relative_coordinate"][0],
                    cortical_data["relative_coordinate"][1],
                    cortical_data["relative_coordinate"][2]
                ],
                "cortical_coordinates_2d": [
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
                "transforming": False
            }
            if cortical_area in runtime_data.transforming_areas:
                cortical_properties["transforming"] = True
            return cortical_properties
        else:
            raise HTTPException(status_code=400, detail="Bad reqeust!")
    else:
        raise HTTPException(status_code=400, detail=f"{cortical_area} is not meeting standard length constraints")


@router.put("/v1/feagi/genome/cortical_area")
async def update_cortical_properties(message: UpdateCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """

    if message.cortical_id in runtime_data.transforming_areas:
        return JSONResponse(status_code=409, content={'message': "Operation rejected as the target cortical area is"
                                                                 "currently undergoing transformation."})
    else:
        runtime_data.transforming_areas.add(message.cortical_id)
        message = message.dict()
        message = {'update_cortical_properties': message}
        print("*-----* " * 200 + "\n", message)
        api_queue.put(item=message)


@router.post("/v1/feagi/genome/cortical_area")
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


@router.api_route("/v1/feagi/genome/custom_cortical_area", methods=['POST'], tags=["Genome"])
async def add_cortical_area_custom(new_custom_cortical_properties: NewCustomCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    cortical_name = new_custom_cortical_properties.cortical_name
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
    cortical_id = add_custom_cortical_area(cortical_name=cortical_name,
                                           coordinates_3d=coordinates_3d,
                                           coordinates_2d=coordinates_2d,
                                           cortical_dimensions=cortical_dimensions,
                                           is_memory=is_memory,
                                           copy_of=copy_of)
    return JSONResponse(status_code=200, content={'cortical_id': cortical_id})


@router.delete("/v1/feagi/genome/cortical_area")
async def delete_cortical_area(cortical_area_name):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = {'delete_cortical_area': cortical_area_name}
    api_queue.put(item=message)


@router.get("/v1/feagi/genome/cortical_area_id_list")
async def genome_cortical_ids():
    """
    Returns a comprehensive list of all cortical area names.
    """
    if runtime_data.cortical_list:
        return sorted(runtime_data.cortical_list)
    else:
        return []


@router.get("/v1/feagi/genome/cortical_name_location")
async def genome_cortical_location_by_name(cortical_name):
    """
    Returns a comprehensive list of all cortical area names.
    """

    cortical_area = cortical_name_to_id(cortical_name=cortical_name)
    return runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"]


@router.get("/v1/feagi/genome/cortical_area_name_list")
async def genome_cortical_names():
    """
    Returns a comprehensive list of all cortical area names.
    """
    if cortical_name_list:
        return sorted(cortical_name_list())


@router.get("/v1/feagi/genome/morphology_list")
async def genome_neuron_morphologies():
    """
    Returns a comprehensive list of all neuron morphologies.
    """
    morphology_names = set()

    for morphology in runtime_data.genome['neuron_morphologies']:
        morphology_names.add(morphology)
    return sorted(morphology_names)


@router.get("/v1/feagi/genome/morphology_types")
async def genome_neuron_morphology_types():
    """
    Returns the properties of a neuron morphology.
    """
    return {"vectors", "patterns", "composite", "functions"}


@router.get("/v1/feagi/morphologies/list/types")
async def genome_neuron_morphology_type_list():
    """
    Returns the properties of a neuron morphology.
    """
    report = {}
    for morphology in runtime_data.genome["neuron_morphologies"]:
        if morphology not in report:
            report[morphology] = runtime_data.genome["neuron_morphologies"][morphology]["type"]
    return report


@router.get("/v1/feagi/genome/morphology_functions")
async def genome_neuron_morphology_functions():
    """
    Returns the list of morphology function names.
    """
    morphology_list = set()
    for entry in dir(synaptogenesis_rules):
        if str(entry)[:4] == "syn_":
            morphology_list.add(str(entry))
    return morphology_list


@router.get("/v1/feagi/genome/morphology")
async def genome_neuron_morphology_properties(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    if morphology_name in runtime_data.genome['neuron_morphologies']:
        results = runtime_data.genome['neuron_morphologies'][morphology_name]
        results["morphology_name"] = morphology_name
        return results
    else:
        raise HTTPException(status_code=404, detail=f"Morphology named {morphology_name} not found!")


@router.get("/v1/feagi/genome/morphology_usage")
async def genome_neuron_morphology_usage_report(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    if morphology_name in runtime_data.genome["neuron_morphologies"]:
        usage_list = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
        if usage_list:
            return usage_list
        else:
            return JSONResponse(status_code=200, content=[])
    else:
        return JSONResponse(status_code=404, content="Morphology not found")


@router.put("/v1/feagi/genome/morphology")
async def genome_update_neuron_morphology(morphology_name: str,
                                          morphology_type: str,
                                          morphology_parameters: dict):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = dict()
    message["name"] = morphology_name
    message["type"] = morphology_type
    message["parameters"] = morphology_parameters

    message = {'update_morphology_properties': message}
    print("*" * 50 + "\n", message)
    api_queue.put(item=message)


@router.post("/v1/feagi/genome/morphology")
async def genome_add_neuron_morphology(morphology_name: str,
                                       morphology_type: str,
                                       morphology_parameters: dict):
    """
    Enables changes against various Burst Engine parameters.
    """

    if morphology_name not in runtime_data.genome['neuron_morphologies']:
        runtime_data.genome['neuron_morphologies'][morphology_name] = {}
        runtime_data.genome['neuron_morphologies'][morphology_name]["type"] = morphology_type
        runtime_data.genome['neuron_morphologies'][morphology_name]["class"] = "custom"
        runtime_data.genome['neuron_morphologies'][morphology_name]["parameters"] = morphology_parameters
    else:
        pass


@router.delete("/v1/feagi/genome/morphology")
async def genome_delete_neuron_morphology(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    # todo: Needs to be rewritten
    if morphology_name in runtime_data.genome['neuron_morphologies']:
        if "class" in runtime_data.genome['neuron_morphologies'][morphology_name]:
            if runtime_data.genome['neuron_morphologies'][morphology_name]["class"] == "custom":
                usage = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
                if not usage:
                    runtime_data.genome['neuron_morphologies'].pop(morphology_name)
                else:
                    raise HTTPException(status_code=403, detail="In use morphology cannot be deleted!")
            elif runtime_data.genome['neuron_morphologies'][morphology_name]["class"] == "core":
                raise HTTPException(status_code=403, detail="Core morphology cannot be deleted!")
        else:
            pass
    else:
        raise HTTPException(status_code=404, detail=f"Morphology wth name {morphology_name} not found!")


# @router.post("/v1/feagi/genome/cortical_mappings")
# async def add_cortical_mapping(cortical_area):
#     """
#     Returns the list of cortical areas downstream to the given cortical areas
#     """
    # return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
#


@router.get("/v1/feagi/genome/cortical_mappings/efferents")
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """

    if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
        cortical_mappings = set()
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            cortical_mappings.add(destination)
        return cortical_mappings
    else:
        raise HTTPException(status_code=400, detail="Wrong cortical id format!")


@router.get("/v1/feagi/genome/cortical_mappings/afferents")
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """

    if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
        upstream_cortical_areas, downstream_cortical_areas = \
            neighboring_cortical_areas(cortical_area, blueprint=runtime_data.genome["blueprint"])
        return upstream_cortical_areas
    else:
        raise HTTPException(status_code=400, detail="Wrong cortical id format!")


@router.get("/v1/feagi/genome/cortical_mappings_by_name")
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    cortical_mappings = set()
    for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        cortical_mappings.add(runtime_data.genome['blueprint'][destination]['cortical_name'])

    return cortical_mappings


@router.get("/v1/feagi/genome/cortical_mappings_detailed")
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """

    if runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
        return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
    else:
        raise HTTPException(status_code=404, detail=f"Cortical area with id={cortical_area} not found!")


@router.get("/v1/feagi/genome/mapping_properties")
async def fetch_cortical_mapping_properties(src_cortical_area, dst_cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    if dst_cortical_area in runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst']:
        return runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]
    else:
        raise HTTPException(status_code=404, 
                            detail=f"{dst_cortical_area} is not a cortical destination of {src_cortical_area}!")


@router.put("/v1/feagi/genome/mapping_properties")
async def update_cortical_mapping_properties(src_cortical_area, dst_cortical_area,
                                             mapping_string: list):
    """
    Enables changes against various Burst Engine parameters.
    """

    data = dict()
    data["mapping_data"] = mapping_string
    data["src_cortical_area"] = src_cortical_area
    data["dst_cortical_area"] = dst_cortical_area
    data = {'update_cortical_mappings': data}
    api_queue.put(item=data)


@router.get("/v1/feagi/genome/cortical_types")
async def cortical_area_types():
    """
    Returns the list of supported cortical types
    """
    if runtime_data.cortical_defaults:
        return runtime_data.cortical_defaults


@router.get("/v1/feagi/genome/cortical_type_options")
async def cortical_area_types(cortical_type):
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


@router.post("/v1/feagi/genome/amalgamation_by_payload")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        runtime_data.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        runtime_data.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=amalgamation_param.genome_payload["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/v1/feagi/genome/amalgamation_by_upload")
async def amalgamation_attempt(file: UploadFile = File(...)):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        data = await file.read()
        runtime_data.genome_file_name = file.filename

        genome_str = json.loads(data)
        genome_2 = genome_2_1_convertor(genome_str["blueprint"])

        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = runtime_data.genome_file_name
        runtime_data.pending_amalgamation["genome_title"] = runtime_data.genome_file_name
        runtime_data.pending_amalgamation["genome_payload"] = genome_str
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=genome_2["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.post("/v1/feagi/genome/amalgamation_by_filename")
async def amalgamation_attempt(amalgamation_param: AmalgamationRequest):
    if pending_amalgamation():
        raise HTTPException(status_code=409, detail="An existing amalgamation attempt is pending")
    else:
        now = datetime.now()
        amalgamation_id = str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_A'
        runtime_data.pending_amalgamation["genome_id"] = amalgamation_param.genome_id
        runtime_data.pending_amalgamation["genome_title"] = amalgamation_param.genome_title
        runtime_data.pending_amalgamation["genome_payload"] = amalgamation_param.genome_payload
        runtime_data.pending_amalgamation["initiation_time"] = datetime.now()
        runtime_data.pending_amalgamation["amalgamation_id"] = amalgamation_id
        runtime_data.pending_amalgamation["circuit_size"] = \
            circuit_size(blueprint=amalgamation_param.genome_payload["blueprint"])

        runtime_data.amalgamation_history[amalgamation_id] = "pending"
        return amalgamation_id


@router.get("/v1/feagi/genome/amalgamation_history")
async def amalgamation_history():
    return runtime_data.amalgamation_history


@router.post("/v1/feagi/genome/amalgamation_destination")
async def amalgamation_conclusion(circuit_origin_x: int,
                                  circuit_origin_y: int,
                                  circuit_origin_z: int,
                                  amalgamation_id):
    if pending_amalgamation():
        payload = dict()
        payload["genome_str"] = runtime_data.pending_amalgamation["genome_payload"]
        payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
        data = {'append_circuit': payload}
        api_queue.put(item=data)
        genome_title = runtime_data.pending_amalgamation["genome_title"]
        cancel_pending_amalgamation(amalgamation_id=amalgamation_id)
        runtime_data.amalgamation_history["amalgamation_id"] = "complete"
        return f"Amalgamation for \"{genome_title}\" is complete."
    else:
        raise HTTPException(status_code=404, detail="No pending amalgamation request found")


@router.get("/v1/feagi/genome/amalgamation")
async def circuit_library(amalgamation_id):
    if amalgamation_id in runtime_data.amalgamation_history:
        return runtime_data.amalgamation_history[amalgamation_id]
    else:
        raise HTTPException(status_code=404, detail="No matching amalgamation found")


@router.delete("/v1/feagi/genome/amalgamation_cancellation")
async def cancel_amalgamation_request(amalgamation_id):
    cancel_pending_amalgamation(amalgamation_id)


@router.get("/v1/feagi/genome/circuits")
async def circuit_library():
    """
    Returns the list of neuronal circuits under /evo/circuits
    """
    circuit_list = os.listdir(runtime_data.circuit_lib_path)
    return circuit_list


# @router.get("/v1/feagi/genome/circuit_description")
# async def cortical_area_types(circuit_name, response: Response):
#     """
#     Returns circuit aka. genome description including its size
#     """

    # with open("./evo/circuits/" + circuit_name, "r") as genome_file:
    #     genome_data = json.load(genome_file)
    #
    # genome2 = genome_2_1_convertor(flat_genome=genome_data["blueprint"])
    #
    # circuit_description = {}
    # circuit_size_ = circuit_size(blueprint=genome2["blueprint"])
    # circuit_description["size"] = circuit_size_
    # if "description" in runtime_data.genome:
    #     circuit_description["description"] = runtime_data.genome["description"]
    # else:
    #     circuit_description["description"] = ""
    # return circuit_description


@router.post("/v1/feagi/genome/append-file")
async def genome_append_circuit(circuit_origin_x: int,
                                circuit_origin_y: int,
                                circuit_origin_z: int,
                                file: UploadFile = File(...)):
    """
    Appends a given circuit to the running genome at a specific location.
    """
    data = await file.read()

    runtime_data.genome_file_name = file.filename

    genome_str = json.loads(data)

    payload = dict()
    payload["genome_str"] = genome_str
    payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
    data = {'append_circuit': payload}
    api_queue.put(item=data)


# @router.api_route("/v1/feagi/genome/append", methods=['POST'], tags=["Genome"])
# async def genome_append_circuit(circuit_name: str,
#                                 circuit_origin_x: int,
#                                 circuit_origin_y: int,
#                                 circuit_origin_z: int,
#                                 response: Response):
#     """
#     Appends a given circuit to the running genome at a specific location.
#     """
#     try:
#         append_genome_from_file(circuit_name=circuit_name,
#                                 circuit_origin_x=circuit_origin_x,
#                                 circuit_origin_y=circuit_origin_y,
#                                 circuit_origin_z=circuit_origin_z)
#         response.status_code = status.HTTP_200_OK
#     except Exception as e:
#         response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
#         print("API Error:", e)


# def append_genome_from_file(circuit_name: str,
#                             circuit_origin_x: int,
#                             circuit_origin_y: int,
#                             circuit_origin_z: int):
#     circuit_list = os.listdir("./evo/circuits")
#     if circuit_name not in circuit_list:
#         raise HTTPException(status_code=404, detail="Circuit no found")
#     else:
#         with open("./evo/circuits/" + circuit_name, "r") as genome_file:
#             source_genome = json.load(genome_file)
#         payload = dict()
#         payload["genome_str"] = source_genome
#         payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
#         data = {'append_circuit': payload}
#         api_queue.put(item=data)


# def append_genome_from_payload(genome_payload: dict,
#                                circuit_origin_x: int,
#                                circuit_origin_y: int,
#                                circuit_origin_z: int):
#     payload = dict()
#     payload["genome_str"] = genome_payload
#     payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
#     data = {'append_circuit': payload}
#     api_queue.put(item=data)


@router.get("/v1/feagi/genome/cortical_map_detailed")
async def connectome_detailed_cortical_map():
    cortical_map = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_map[cortical_area] = dict()
        for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            cortical_map[cortical_area][dst] = list()
            for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst].append(mapping)

    return cortical_map


@router.get("/v1/feagi/genome/cortical_map")
async def connectome_cortical_map():
    cortical_map = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        cortical_map[cortical_area] = dict()
        for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            cortical_map[cortical_area][dst] = 0
            for _ in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst] += 1

    return cortical_map


@router.get("/v1/feagi/genome/cortical_id_name_mapping")
async def connectome_cortical_id_name_mapping_table():
    mapping_table = dict()
    for cortical_area in runtime_data.genome["blueprint"]:
        mapping_table[cortical_area] = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
    return mapping_table


@router.get("/v1/feagi/genome/plasticity_queue_depth")
async def show_plasticity_queue_depth():
    """
    Returns the current plasticity queue depth value
    """
    return runtime_data.genome["plasticity_queue_depth"]


@router.put("/v1/feagi/genome/plasticity_queue_depth")
async def update_plasticity_queue_depth(queue_depth: int):
    """
    Enables changes against various Burst Engine parameters.
    """
    runtime_data.genome["plasticity_queue_depth"] = queue_depth


@router.get("/v1/feagi/genome/cortical_locations_2d")
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


@router.get("/v1/feagi/genome/cortical_area/geometry")
async def cortical_area_geometry():
    if runtime_data.cortical_dimensions_by_id:
        return runtime_data.cortical_dimensions_by_id
    else:
        return {}


@router.put("/v1/feagi/genome/coord_2d")
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


@router.put("/v1/feagi/genome/coord_3d")
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
