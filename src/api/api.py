# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
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
import datetime
import json
import os
import traceback
import time
import string
import logging
import random
import tempfile
from version import __version__
import io

from fastapi import FastAPI, File, UploadFile, Response, status, Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from starlette.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field, conint
from typing import Optional, Literal
from ast import literal_eval
from threading import Thread
from queue import Queue
from io import StringIO, BytesIO

from inf import feagi
from inf import runtime_data
from inf.baseline import gui_baseline
from evo import autopilot
from evo.synapse import cortical_mapping, morphology_usage_list
from evo.templates import cortical_types
from evo.neuroembryogenesis import cortical_name_list, cortical_name_to_id
from evo import synaptogenesis_rules
from evo.stats import circuit_size
from evo.genome_properties import genome_properties
from evo.x_genesis import neighboring_cortical_areas, add_core_cortical_area, add_custom_cortical_area
from evo.genome_processor import genome_2_1_convertor
from inf.disk_ops import preserve_brain, revive_brain
from .config import settings
from inf.messenger import Sub
from inf.initialize import deploy_genome, generate_cortical_dimensions_by_id


logger = logging.getLogger(__name__)


description = """FEAGI REST API will help you integrate FEAGI into other applications and 
provides a programmatic method to interact with FEAGI. 

"""

app = FastAPI(
    title=settings.title,
    description=settings.description,
    version=settings.version,
    terms_of_service=settings.terms_of_service,
    contact=settings.contact,
    license_info=settings.license_info,
)


favicon_path = settings.favicon_path

api_queue = Queue()

ORIGINS = settings.origins

app.add_middleware(
    CORSMiddleware,
    allow_origins=ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)


def kickstart_feagi_thread():
    print("<>--" * 20)
    print("Starting FEAGI thread..")
    runtime_data.feagi_thread = Thread(target=feagi.start_feagi, args=(api_queue,))
    runtime_data.feagi_thread.start()


kickstart_feagi_thread()


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_cortical_activity_counters: Optional[bool]
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]
    burst_duration = 1


class MorphologyProperties(BaseModel):
    name: str
    type: str
    parameters: dict


class NewCorticalProperties(BaseModel):
    cortical_type: str
    cortical_id: str
    coordinates_2d: list
    coordinates_3d: list
    channel_count: Optional[int]


class NewCustomCorticalProperties(BaseModel):
    cortical_name: str = Field(None, max_length=20, min_length=1)
    coordinates_2d: Optional[list] = [0, 0]
    coordinates_3d: list
    cortical_dimensions: list


# class NewCorticalProperties_old(BaseModel):
#     cortical_id: str = Field(None, max_length=6, min_length=6)
#     cortical_name: str
#     cortical_group: str
#     cortical_neuron_per_vox_count: int
#     cortical_visibility: bool
#     cortical_coordinates: dict = {
#         'x': 0,
#         'y': 0,
#         'z': 0,
#     }
#     cortical_dimensions: dict = {
#         'x': 1,
#         'y': 1,
#         'z': 1,
#     }
#     cortical_destinations: dict = {
#     }
#     cortical_synaptic_attractivity: int
#     neuron_post_synaptic_potential: float
#     neuron_post_synaptic_potential_max: float
#     neuron_fire_threshold: float
#     neuron_refractory_period: int
#     neuron_leak_coefficient: float
#     neuron_leak_variability: int
#     neuron_consecutive_fire_count: int
#     neuron_snooze_period: int
#     neuron_degeneracy_coefficient: float
#     neuron_psp_uniform_distribution: bool


class UpdateCorticalProperties(BaseModel):
    cortical_id: str = Field(None, max_length=6, min_length=6)
    cortical_name: Optional[str]
    cortical_group: Optional[str]
    cortical_neuron_per_vox_count: Optional[int]
    cortical_visibility: Optional[bool]
    cortical_coordinates: Optional[list]
    cortical_coordinates_2d: Optional[list]
    cortical_dimensions: Optional[list]
    cortical_synaptic_attractivity: Optional[int]
    neuron_post_synaptic_potential: Optional[float]
    neuron_post_synaptic_potential_max: Optional[float]
    neuron_fire_threshold: Optional[float]
    neuron_fire_threshold_increment: Optional[list]
    neuron_firing_threshold_limit: Optional[float]
    neuron_refractory_period: Optional[int]
    neuron_leak_coefficient: Optional[float]
    neuron_leak_variability: Optional[float]
    neuron_consecutive_fire_count: Optional[int]
    neuron_snooze_period: Optional[int]
    neuron_degeneracy_coefficient: Optional[float]
    neuron_psp_uniform_distribution: Optional[bool]
    neuron_mp_charge_accumulation: Optional[bool]
    neuron_mp_driven_psp: Optional[bool]


# class Network(BaseModel):
#     godot_host: Optional[str] = runtime_data.parameters['Sockets']['godot_host_name']
#     godot_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_godot']
#     godot_web_port: Optional[int] = 6081
#     gazebo_host: Optional[str] = runtime_data.parameters['Sockets']['gazebo_host_name']
#     gazebo_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo']
#     gazebo_web_port: Optional[int] = 6080


class ConnectomePath(BaseModel):
    connectome_path: str


class Connectome(BaseModel):
    cortical_area: str


class Registration(BaseModel):
    source: str
    host: str
    capabilities: dict


class Stats(BaseModel):
    neuron_stat_collection: Optional[bool] = False
    synapse_stat_collection: Optional[bool] = False


class Stimulation(BaseModel):
    stimulation_script: dict


class Shock(BaseModel):
    shock: tuple


class Intensity(BaseModel):
    intensity: conint(ge=0, le=9)


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope):
        response = await super().get_response(path, scope)
        print("<><><><><><>")
        if response.status_code == 404:
            print("-=-=-=-=-=-=-=-=-=-=")
            response = await super().get_response('.', scope)
        return response


class Subscriber(BaseModel):
    subscriber_address: str


class RobotController(BaseModel):
    motor_power_coefficient: float
    motor_power_coefficient = 0.5
    robot_starting_position: dict
    robot_starting_position = {
        0: [0, 0, 0],
        1: [0, 1, 1],
        2: [0, 0, 1],
        3: [0, 2, 1],
        4: [0, 1, 2]
    }


class RobotModel(BaseModel):
    robot_sdf_file_name: str
    robot_sdf_file_name = ""

    robot_sdf_file_name_path: str
    robot_sdf_file_name_path = "./lib/gazebo/robots/4WD_smart_car/"

    gazebo_floor_img_file: str
    gazebo_floor_img_file = ""

    gazebo_floor_img_file_path: str
    gazebo_floor_img_file_path = "./lib/gazebo/gazebo_maps/"

    mu: float
    mu = 1.0

    mu2: float
    mu2 = 50.0

    fdir: list
    fdir = [1, 1, 1]

    slip1: float
    slip1 = 1.0

    slip2: float
    slip2 = 1.0


@app.middleware("http")
async def log_requests(request: Request, call_next):
    """
    Credit: Phil Girard
    """
    idem = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    logger.info(f"rid={idem} start request path={request.url.path}")
    start_time = time.time()

    response = await call_next(request)

    process_time = (time.time() - start_time) * 1000
    formatted_process_time = '{0:.2f}'.format(process_time)
    logger.info(f"rid={idem} completed_in={formatted_process_time}ms status_code={response.status_code}")

    # print(response.status_code, ":", request.method, ":", request.url.path)
    return response


# todo: To add the ability of updating allowable cors list on the fly
# # Append to the CORS origin
# @app.middleware("http")
# async def update_cors_origin(request, call_next):
#     response = await call_next(request)
#     origin = response.headers.get("Access-Control-Allow-Origin", "")
#     new_origin = ""
#     response.headers["Access-Control-Allow-Origin"] = f"{origin},{new_origin}"
#     return response


# ######  Genome Endpoints #########
# ##################################
@app.api_route("/v1/feagi/genome/upload/blank", methods=['POST'], tags=["Genome"])
async def upload_blank_genome(response: Response):
    try:

        with open("./evo/defaults/genome/blank_genome.json", "r") as genome_file:
            genome_data = json.load(genome_file)
            runtime_data.genome_file_name = "blank_genome.json"
        runtime_data.brain_readiness = False
        message = {'genome': genome_data}

        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/upload/default", methods=['POST'], tags=["Genome"])
async def genome_default_upload(response: Response):
    try:

        with open("./evo/static_genome.json", "r") as genome_file:
            genome_data = json.load(genome_file)
            runtime_data.genome_file_name = "static_genome.json"
        runtime_data.brain_readiness = False
        message = {'genome': genome_data}

        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/genome/upload/file", tags=["Genome"])
async def genome_file_upload(response: Response, file: UploadFile = File(...)):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a python file.
    """
    try:

        data = await file.read()
        runtime_data.brain_readiness = False
        runtime_data.genome_file_name = file.filename

        genome_str = json.loads(data)

        # genome_str = genome_str.replace('\'', '\"')
        # genome_str = data.decode("utf-8").split(" = ")[1]
        message = {'genome': genome_str}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_400_BAD_REQUEST
        print("API ERROR during genome file upload:\n", e, traceback.print_exc())


@app.get("/v1/feagi/genome/file_name", tags=["Genome"])
async def genome_file_name(response: Response):
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    try:
        genome_name = runtime_data.genome_file_name
        if genome_name:
            response.status_code = status.HTTP_200_OK
            return genome_name
        else:
            response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/upload/string", methods=['POST'], tags=["Genome"])
async def genome_string_upload(genome: dict, response: Response):
    try:
        message = {'genome': genome}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/genome/download", tags=["Genome"])
async def genome_download(response: Response):
    print("Downloading Genome...")
    try:
        file_name = "genome_" + datetime.datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".json"
        print(file_name)
        if runtime_data.genome:
            response.status_code = status.HTTP_200_OK
            return FileResponse(path=runtime_data.connectome_path + "genome.json", filename=file_name)
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        print("API Error:", e)


@app.post("/v1/feagi/genome/upload/file/edit", tags=["Genome"])
async def genome_file_upload_edit(response: Response, file: UploadFile = File(...)):
    try:
        data = await file.read()
        genome_str = data.decode("utf-8")
        response.status_code = status.HTTP_200_OK
        return {genome_str}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/genome/defaults/files", tags=["Genome"])
async def genome_default_files(response: Response):
    try:
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
        response.status_code = status.HTTP_200_OK
        return {"genome": genome_mappings}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/genome_number", methods=['GET'], tags=["Genome"])
async def genome_number(response: Response):
    """
    Return the number associated with current Genome instance.
    """
    try:
        if runtime_data.genome_counter:
            response.status_code = status.HTTP_200_OK
            return runtime_data.genome_counter
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/genome/reset", tags=["Genome"])
async def reset_genome(response: Response):
    try:
        print("API call has triggered a genome reset")
        runtime_data.genome_reset_flag = True
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_area", methods=['GET'], tags=["Genome"])
async def fetch_cortical_properties(cortical_area, response: Response):
    """
    Returns the properties of cortical areas
    """
    try:
        if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
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
                "transforming": False
            }
            if cortical_area in runtime_data.transforming_areas:
                cortical_properties["transforming"] = True
            response.status_code = status.HTTP_200_OK
            return cortical_properties
        else:
            response.status_code = status.HTTP_400_BAD_REQUEST
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", traceback.print_exc(), e)


@app.api_route("/v1/feagi/genome/cortical_area", methods=['PUT'], tags=["Genome"])
async def update_cortical_properties(message: UpdateCorticalProperties, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        if message.cortical_id in runtime_data.transforming_areas:
            return JSONResponse(status_code=409, content={'message': "Operation rejected as the target cortical area is"
                                                                     "currently undergoing transformation."})
        else:
            runtime_data.transforming_areas.add(message.cortical_id)
            message = message.dict()
            message = {'update_cortical_properties': message}
            print("*-----* " * 200 + "\n", message)
            api_queue.put(item=message)
            response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_400_BAD_REQUEST
        print("API Error:", message, e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_area", methods=['POST'], tags=["Genome"])
async def add_cortical_area(new_cortical_properties: NewCorticalProperties, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:

        # message = message.dict()
        # message = {'add_core_cortical_area': message}
        # print("*" * 50 + "\n", message)
        # api_queue.put(item=message)
        new_cortical_properties = dict(new_cortical_properties)
        cortical_id = add_core_cortical_area(cortical_properties=new_cortical_properties)
        # response.status_code = status.HTTP_200_OK
        return JSONResponse(status_code=200, content={'cortical_id': cortical_id})
    except Exception as e:
        response.status_code = status.HTTP_400_BAD_REQUEST
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/custom_cortical_area", methods=['POST'], tags=["Genome"])
async def add_cortical_area_custom(new_custom_cortical_properties: NewCustomCorticalProperties, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    print("--------")
    print(new_custom_cortical_properties)
    try:
        cortical_name = new_custom_cortical_properties.cortical_name
        coordinates_3d = new_custom_cortical_properties.coordinates_3d
        coordinates_2d = new_custom_cortical_properties.coordinates_2d
        cortical_dimensions = new_custom_cortical_properties.cortical_dimensions
        cortical_id = add_custom_cortical_area(cortical_name=cortical_name,
                                               coordinates_3d=coordinates_3d,
                                               coordinates_2d=coordinates_2d,
                                               cortical_dimensions=cortical_dimensions)
        return JSONResponse(status_code=200, content={'cortical_id': cortical_id})
        # message = {'add_custom_cortical_area': message}
        # print("*" * 50 + "\n", message)
        # api_queue.put(item=message)
        # response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc(), new_custom_cortical_properties)


@app.api_route("/v1/feagi/genome/cortical_area", methods=['DELETE'], tags=["Genome"])
async def delete_cortical_area(cortical_area_name, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = {'delete_cortical_area': cortical_area_name}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_area_id_list", methods=['GET'], tags=["Genome"])
async def genome_cortical_ids(response: Response):
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        if runtime_data.cortical_list:
            response.status_code = status.HTTP_200_OK
            return sorted(runtime_data.cortical_list)
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_name_location", methods=['GET'], tags=["Genome"])
async def genome_cortical_location_by_name(cortical_name, response: Response):
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        cortical_area = cortical_name_to_id(cortical_name=cortical_name)
        response.status_code = status.HTTP_200_OK
        return runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"]
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_area_name_list", methods=['GET'], tags=["Genome"])
async def genome_cortical_names(response: Response):
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        if cortical_name_list:
            response.status_code = status.HTTP_200_OK
            return sorted(cortical_name_list())
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology_list", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphologies(response: Response):
    """
    Returns a comprehensive list of all neuron morphologies.
    """
    morphology_names = set()
    try:
        for morphology in runtime_data.genome['neuron_morphologies']:
            morphology_names.add(morphology)
        response.status_code = status.HTTP_200_OK
        return sorted(morphology_names)
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology_types", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_types(response: Response):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        response.status_code = status.HTTP_200_OK
        return {"vectors", "patterns", "composite", "functions"}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/morphologies/list/types", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_type_list(response: Response):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        response.status_code = status.HTTP_200_OK
        report = {}
        for morphology in runtime_data.genome["neuron_morphologies"]:
            if morphology not in report:
                report[morphology] = runtime_data.genome["neuron_morphologies"][morphology]["type"]
        return report
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology_functions", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_functions(response: Response):
    """
    Returns the list of morphology function names.
    """
    try:
        morphology_list = set()
        for entry in dir(synaptogenesis_rules):
            if str(entry)[:4] == "syn_":
                morphology_list.add(str(entry))
        response.status_code = status.HTTP_200_OK
        return morphology_list
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/morphology", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_properties(morphology_name, response: Response):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        if morphology_name in runtime_data.genome['neuron_morphologies']:
            response.status_code = status.HTTP_200_OK
            results = runtime_data.genome['neuron_morphologies'][morphology_name]
            results["morphology_name"] = morphology_name
            return results
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology_usage", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_usage_report(morphology_name, response: Response):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        if morphology_name in runtime_data.genome["neuron_morphologies"]:
            usage_list = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
            if usage_list:
                response.status_code = status.HTTP_200_OK
                return usage_list
            else:
                return JSONResponse(status_code=200, content=[])
        else:
            return JSONResponse(status_code=404, content="Morphology not found")
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/morphology", methods=['PUT'], tags=["Genome"])
async def genome_update_neuron_morphology(morphology_name: str,
                                          morphology_type: str,
                                          morphology_parameters: dict,
                                          response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = {}
        message["name"] = morphology_name
        message["type"] = morphology_type
        message["parameters"] = morphology_parameters

        message = {'update_morphology_properties': message}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology", methods=['POST'], tags=["Genome"])
async def genome_add_neuron_morphology(morphology_name: str,
                                       morphology_type: str,
                                       morphology_parameters: dict,
                                       response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        if morphology_name not in runtime_data.genome['neuron_morphologies']:
            runtime_data.genome['neuron_morphologies'][morphology_name] = {}
            runtime_data.genome['neuron_morphologies'][morphology_name]["type"] = morphology_type
            runtime_data.genome['neuron_morphologies'][morphology_name]["parameters"] = morphology_parameters
        else:
            pass
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/morphology", methods=['DELETE'], tags=["Genome"])
async def genome_delete_neuron_morphology(morphology_name, response: Response):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        if morphology_name in runtime_data.genome['neuron_morphologies']:
            usage = morphology_usage_list(morphology_name=morphology_name, genome=runtime_data.genome)
            if not usage:
                runtime_data.genome['neuron_morphologies'].pop(morphology_name)
                response.status_code = status.HTTP_200_OK
            else:
                response.status_code = status.HTTP_404_NOT_FOUND
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


#
# @app.api_route("/v1/feagi/genome/cortical_mappings", methods=['POST'], tags=["Genome"])
# async def add_cortical_mapping(cortical_area):
#     """
#     Returns the list of cortical areas downstream to the given cortical areas
#     """
#     try:
#         return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
#
#     except Exception as e:
#         print("API Error:", e)
#         


@app.api_route("/v1/feagi/genome/cortical_mappings/efferents", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area, response: Response):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
            cortical_mappings = set()
            for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
                cortical_mappings.add(destination)
            response.status_code = status.HTTP_200_OK
            return cortical_mappings
        else:
            response.status_code = status.HTTP_400_BAD_REQUEST
    except Exception as e:
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_mappings/afferents", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area, response: Response):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
            upstream_cortical_areas, downstream_cortical_areas = \
                neighboring_cortical_areas(cortical_area, blueprint=runtime_data.genome["blueprint"])
            response.status_code = status.HTTP_200_OK
            return upstream_cortical_areas
        else:
            response.status_code = status.HTTP_400_BAD_REQUEST
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_mappings_by_name", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area, response: Response):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    try:
        cortical_mappings = set()
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            cortical_mappings.add(runtime_data.genome['blueprint'][destination]['cortical_name'])
        response.status_code = status.HTTP_200_OK
        return cortical_mappings
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_mappings_detailed", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area, response: Response):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            response.status_code = status.HTTP_200_OK
            return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/mapping_properties", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mapping_properties(src_cortical_area, dst_cortical_area, response: Response):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if dst_cortical_area in runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst']:
            response.status_code = status.HTTP_200_OK
            return runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/mapping_properties", methods=['PUT'], tags=["Genome"])
async def update_cortical_mapping_properties(src_cortical_area, dst_cortical_area,
                                             mapping_string: list, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        data = dict()
        data["mapping_data"] = mapping_string
        data["src_cortical_area"] = src_cortical_area
        data["dst_cortical_area"] = dst_cortical_area
        data = {'update_cortical_mappings': data}
        api_queue.put(item=data)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        logger.error(traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_types", methods=['GET'], tags=["Genome"])
async def cortical_area_types(response: Response):
    """
    Returns the list of supported cortical types
    """
    try:
        if runtime_data.cortical_defaults:
            response.status_code = status.HTTP_200_OK
            return runtime_data.cortical_defaults
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_type_options", methods=['GET'], tags=["Genome"])
async def cortical_area_types(cortical_type, response: Response):
    """
    Returns the list of supported cortical area for a given type
    """
    try:
        if cortical_type in cortical_types:
            cortical_list = set()
            for item in cortical_types[cortical_type]['supported_devices']:
                if cortical_types[cortical_type]['supported_devices'][item]['enabled']:
                    cortical_list.add(item)
            response.status_code = status.HTTP_200_OK
            return cortical_list
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return None

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/circuits", methods=['GET'], tags=["Genome"])
async def circuit_library(response: Response):
    """
    Returns the list of neuronal circuits under /evo/circuits
    """
    try:
        circuit_list = os.listdir(runtime_data.circuit_lib_path)
        response.status_code = status.HTTP_200_OK
        return circuit_list

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/circuit_description", methods=['GET'], tags=["Genome"])
async def cortical_area_types(circuit_name, response: Response):
    """
    Returns circuit aka. genome description including its size
    """
    try:
        with open("./evo/circuits/" + circuit_name, "r") as genome_file:
            genome_data = json.load(genome_file)

        genome2 = genome_2_1_convertor(flat_genome=genome_data["blueprint"])

        circuit_description = {}
        circuit_size_ = circuit_size(blueprint=genome2["blueprint"])
        circuit_description["size"] = circuit_size_
        if "description" in runtime_data.genome:
            circuit_description["description"] = runtime_data.genome["description"]
        else:
            circuit_description["description"] = ""
        response.status_code = status.HTTP_200_OK
        return circuit_description

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())


@app.api_route("/v1/feagi/genome/append-file", methods=['POST'], tags=["Genome"])
async def genome_append_circuit(circuit_origin_x: int,
                                circuit_origin_y: int,
                                circuit_origin_z: int,
                                response: Response, file: UploadFile = File(...)):
    """
    Appends a given circuit to the running genome at a specific location.
    """
    try:
        data = await file.read()

        runtime_data.genome_file_name = file.filename

        genome_str = json.loads(data)

        payload = dict()
        payload["genome_str"] = genome_str
        payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
        data = {'append_circuit': payload}
        api_queue.put(item=data)

        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/append", methods=['POST'], tags=["Genome"])
async def genome_append_circuit(circuit_name: str,
                                circuit_origin_x: int,
                                circuit_origin_y: int,
                                circuit_origin_z: int,
                                response: Response):
    """
    Appends a given circuit to the running genome at a specific location.
    """
    try:
        circuit_list = os.listdir("./evo/circuits")
        if circuit_name not in circuit_list:
            response.status_code = status.HTTP_404_NOT_FOUND
        else:
            with open("./evo/circuits/" + circuit_name, "r") as genome_file:
                source_genome = json.load(genome_file)
            payload = dict()
            payload["genome_str"] = source_genome
            payload["circuit_origin"] = [circuit_origin_x, circuit_origin_y, circuit_origin_z]
            data = {'append_circuit': payload}
            api_queue.put(item=data)

            response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_map", methods=['GET'], tags=["Genome"])
async def connectome_cortical_map(response: Response):
    try:
        cortical_map = dict()
        for cortical_area in runtime_data.genome["blueprint"]:
            cortical_map[cortical_area] = dict()
            for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
                cortical_map[cortical_area][dst] = 0
                for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                    cortical_map[cortical_area][dst] += 1

        response.status_code = status.HTTP_200_OK
        return cortical_map

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/cortical_id_name_mapping", methods=['GET'], tags=["Genome"])
async def connectome_cortical_id_name_mapping_table(response: Response):
    try:
        mapping_table = dict()
        for cortical_area in runtime_data.genome["blueprint"]:
            mapping_table[cortical_area] = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
        response.status_code = status.HTTP_200_OK
        return mapping_table

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/plasticity_queue_depth", methods=['GET'], tags=["Genome"])
async def show_plasticity_queue_depth(response: Response):
    """
    Returns the current plasticity queue depth value
    """
    try:
        response.status_code = status.HTTP_200_OK
        return runtime_data.genome["plasticity_queue_depth"]
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        logger.error(traceback.print_exc())


@app.api_route("/v1/feagi/genome/plasticity_queue_depth", methods=['PUT'], tags=["Genome"])
async def update_plasticity_queue_depth(queue_depth: int, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        runtime_data.genome["plasticity_queue_depth"] = queue_depth
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        logger.error(traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_locations_2d", methods=['GET'], tags=["Genome"])
async def cortical_2d_locations(response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:

        report = dict()
        for area in runtime_data.genome["blueprint"]:
            if area not in report:
                report[area] = list()
            if "2d_coordinate" in runtime_data.genome['blueprint'][area]:
                report[area] = runtime_data.genome['blueprint'][area]["2d_coordinate"]
            else:
                report[area].append([None, None])

        return report
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        logger.error(traceback.print_exc())


@app.api_route("/v1/feagi/genome/cortical_area/geometry", methods=['GET'], tags=["Genome"])
async def cortical_area_geometry(response: Response):
    try:
        if runtime_data.cortical_dimensions_by_id:
            response.status_code = status.HTTP_200_OK
            return runtime_data.cortical_dimensions_by_id
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/genome/coord_2d", methods=['PUT'], tags=["Genome"])
async def update_coord_2d(new_2d_coordinates: dict, response: Response):
    """
    Accepts a dictionary of 2D coordinates of one or more cortical areas and update them in genome.
    """
    try:
        for cortical_area in new_2d_coordinates:
            if cortical_area in runtime_data.genome["blueprint"]:
                runtime_data.genome["blueprint"][cortical_area]["2d_coordinate"][0] = \
                    new_2d_coordinates[cortical_area][0]
                runtime_data.genome["blueprint"][cortical_area]["2d_coordinate"][1] = \
                    new_2d_coordinates[cortical_area][1]

        runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        print("ensure the provided data structure is a dict with cortical area id as keys and the value as [x, y]")
        logger.error(traceback.print_exc())


@app.api_route("/v1/feagi/genome/coord_3d", methods=['PUT'], tags=["Genome"])
async def update_coord_3d(new_3d_coordinates: dict, response: Response):
    """
    Accepts a dictionary of 3D coordinates of one or more cortical areas and update them in genome.
    """
    try:
        for cortical_area in new_3d_coordinates:
            if cortical_area in runtime_data.genome["blueprint"]:
                runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][0] = \
                    new_3d_coordinates[cortical_area][0]
                runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][1] = \
                    new_3d_coordinates[cortical_area][1]
                runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"][2] = \
                    new_3d_coordinates[cortical_area][2]

        runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc())
        print("ensure the provided data structure is a dict with cortical area id as keys and the value as [x, y, z]")
        logger.error(traceback.print_exc())


# ######  Evolution #########
# #############################

@app.api_route("/v1/feagi/evolution/autopilot/status", methods=['GET'], tags=["Evolution"])
async def retrun_autopilot_status(response: Response):
    """
    Returns the status of genome autopilot system.
    """
    try:
        if runtime_data.autopilot:
            response.status_code = status.HTTP_200_OK
            return runtime_data.autopilot
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/evolution/autopilot/on", tags=["Evolution"])
async def turn_autopilot_on(response: Response):
    try:
        if not runtime_data.autopilot:
            autopilot.init_generation_dict()
            if runtime_data.brain_run_id:
                autopilot.update_generation_dict()
            runtime_data.autopilot = True
            print("<" * 30, "  Autopilot has been turned on  ", ">" * 30)
        response.status_code = status.HTTP_200_OK
        return
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/evolution/autopilot/off", tags=["Evolution"])
async def turn_autopilot_off(response: Response):
    try:
        runtime_data.autopilot = False
        response.status_code = status.HTTP_200_OK
        return
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/evolution/generations", methods=['GET'], tags=["Evolution"])
async def list_generations(response: Response):
    """
    Return details about all generations.
    """
    try:
        if runtime_data.generation_dict:
            response.status_code = status.HTTP_200_OK
            return runtime_data.generation_dict
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/evolution/change_register", methods=['GET'], tags=["Evolution"])
async def list_generations(response: Response):
    """
    Return details about all generations.
    """
    try:
        if runtime_data.evo_change_register:
            response.status_code = status.HTTP_200_OK
            return runtime_data.evo_change_register
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# ######  Stimulation #########
# #############################

@app.api_route("/v1/feagi/stimulation/upload/string", methods=['POST'], tags=["Stimulation"])
async def stimulation_string_upload(stimulation_script: Stimulation, response: Response):
    """
    stimulation_script = {
    "IR_pain": {
        "repeat": 10,
        "definition": [
            [{"i__pro": ["0-0-3"], "o__mot": ["2-0-7"]}, 10],
            [{"i__pro": ["0-0-8"]}, 5],
            [{"i__bat": ["0-0-7"]}, 1],
            [{}, 50]
            ]
    },
    "exploration": {
        "definition": []
    },
    "move_forward": {
        "definition": []
    },
    "charge_batteries": {
        "repeat": 1000,
        "definition": [
            [{"i__inf": ["2-0-0"]}, 2]
        ]
    }
    """
    try:
        message = stimulation_script.dict()
        message = {'stimulation_script': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/stimulation/reset", methods=['POST'], tags=["Stimulation"])
async def stimulation_string_upload(response: Response):
    try:
        message = {"stimulation_script": {}}
        message = {'stimulation_script': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc)


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@app.get("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area, response: Response):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_mp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return True
        else:
            response.status_code = status.HTTP_200_OK
            return False

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.post("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area, state: bool, response: Response):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
        if runtime_data.influxdb:
            influx_readiness = runtime_data.influxdb.test_influxdb()
            if influx_readiness:
                if cortical_area in runtime_data.genome['blueprint']:
                    if state and cortical_area not in runtime_data.neuron_mp_collection_scope:
                        runtime_data.neuron_mp_collection_scope[cortical_area] = {}
                    elif not state and cortical_area in runtime_data.neuron_mp_collection_scope:
                        runtime_data.neuron_mp_collection_scope.pop(cortical_area)
                    else:
                        pass
                response.status_code = status.HTTP_200_OK
                return True
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            print("Error: InfluxDb is not setup to collect timeseries data!")
            return "Error: Timeseries database is not setup!"
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.get("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area, response: Response):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_psp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return True
        else:
            response.status_code = status.HTTP_200_OK
            return False
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.post("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area, state: bool, response: Response):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    try:
        if runtime_data.influxdb:
            if runtime_data.influxdb.test_influxdb():
                if cortical_area in runtime_data.genome['blueprint']:
                    if state and cortical_area not in runtime_data.neuron_psp_collection_scope:
                        runtime_data.neuron_psp_collection_scope[cortical_area] = {}
                    elif not state and cortical_area in runtime_data.neuron_psp_collection_scope:
                        runtime_data.neuron_psp_collection_scope.pop(cortical_area)
                    else:
                        pass
                response.status_code = status.HTTP_200_OK
                return True
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return "Error: Timeseries database is not setup!"
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_membrane_potential_collection_filters(response: Response):
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_mp_collection_scope)
    try:
        if runtime_data.neuron_mp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return runtime_data.neuron_mp_collection_scope
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_postsynaptic_potential_collection_filters(response: Response):
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_psp_collection_scope)
    try:
        if runtime_data.neuron_psp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return runtime_data.neuron_psp_collection_scope
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.api_route("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting",
               methods=['POST'], tags=["Insights"])
async def neuron_membrane_potential_monitoring_scope(message: dict, response: Response):
    """
    Monitor the membrane potential of select cortical areas and voxels in Grafana.
    Message Template:
            {
                "o__mot": {
                    "voxels": [[0, 0, 0], [2, 0, 0]],
                    "neurons": []
                },
                "i__inf": {
                    "voxels": [[1, 1, 1]],
                    "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                },
                ...
            }
    """

    try:
        message = {'neuron_mp_collection_scope': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring", methods=['POST'], tags=["Insights"])
async def neuron_postsynaptic_potential_monitoring_scope(message: dict, response: Response):
    """
    Monitor the post synaptic potentials of select cortical areas and voxels in Grafana.

    Message Template:
            {
                "o__mot": {
                    "dst_filter": {
                        "voxels": [[0, 0, 0], [2, 0, 0]],
                        "neurons": []
                        },
                    "sources": {
                        "i__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                            },
                        "o__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                            }
                    },
                },
                "i__bat": {
                    "dst_filter": {
                        "voxels": [[0, 0, 0], [2, 0, 0]],
                        "neurons": []
                    },
                    "sources": {
                        "i__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                        }
                    }
                }
            }
    """

    try:
        message = {'neuron_psp_collection_scope': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# ######  Training Endpoints #######
# ##################################


@app.api_route("/v1/feagi/training/reset_game_stats", methods=['DELETE'], tags=["Training"])
async def delete_game_stats_from_db(response: Response):
    """
    Erases the game statistics from the database.
    """
    try:
        runtime_data.influxdb.drop_game_activity()
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/shock/options", methods=['Get'], tags=["Training"])
async def list_available_shock_scenarios(response: Response):
    """
    Get a list of available shock scenarios.
    """
    try:
        if runtime_data.shock_scenarios_options:
            response.status_code = status.HTTP_200_OK
            return runtime_data.shock_scenarios_options
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.api_route("/v1/feagi/training/shock/status", methods=['Get'], tags=["Training"])
async def list_activated_shock_scenarios(response: Response):
    try:
        if runtime_data.shock_scenarios:
            response.status_code = status.HTTP_200_OK
            return runtime_data.shock_scenarios
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/shock/activate", methods=['POST'], tags=["Training"])
async def activate_shock_scenarios(shock: Shock, response: Response):
    """
    Enables shock for given scenarios. One or many shock scenario could coexist. e.g.

    {
      "shock": [
        "shock_scenario_1",
        "shock_scenario_2"
      ]
    }

    """
    print("----Shock API----")
    try:
        message = shock.dict()
        print(message)
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/reward", methods=['POST'], tags=["Training"])
async def reward_intensity(intensity: Intensity, response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'reward': intensity.intensity}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/punishment", methods=['POST'], tags=["Training"])
async def punishment_intensity(intensity: Intensity, response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'punishment': intensity.intensity}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/gameover", methods=['POST'], tags=["Training"])
async def gameover_signal(response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'gameover': True}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/training_report", methods=['GET'], tags=["Training"])
async def training_report():
    """
    Returns stats associated with training
    """
    return runtime_data.training_stats


# #########  Robot   ###########
# ##############################

@app.api_route("/v1/robot/parameters", methods=['POST'], tags=["Robot"])
async def robot_controller_tunner(message: RobotController, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'robot_controller': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/robot/model", methods=['POST'], tags=["Robot"])
async def robot_model_modification(message: RobotModel, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'robot_model': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/robot/gazebo/files", tags=["Robot"])
async def gazebo_robot_default_files(response: Response):
    try:
        default_robots_path = "./evo/defaults/robot/"
        default_robots = os.listdir(default_robots_path)
        response.status_code = status.HTTP_200_OK
        return {"robots": default_robots}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        return {"Request failed..."}


# ######  Connectome Endpoints #########
# ######################################

@app.api_route("/v1/feagi/connectome/cortical_areas/list/summary", methods=['GET'], tags=["Connectome"])
async def connectome_cortical_areas_summary(response: Response):
    try:
        cortical_list = set()
        for cortical_area in runtime_data.brain:
            cortical_list.add(cortical_area)
        response.status_code = status.HTTP_200_OK
        return cortical_list

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/cortical_areas/list/transforming", methods=['GET'], tags=["Connectome"])
async def transforming_cortical_areas_summary(response: Response):
    try:
        return runtime_data.transforming_areas

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/cortical_areas/list/detailed", methods=['GET'], tags=["Connectome"])
async def connectome_cortical_areas(response: Response):
    try:
        cortical_list = dict()
        for cortical_area in runtime_data.brain:
            cortical_list[cortical_area] = {}
            cortical_list[cortical_area]["name"] = runtime_data.genome["blueprint"][cortical_area]["cortical_name"]
            cortical_list[cortical_area]["type"] = runtime_data.genome["blueprint"][cortical_area]["group_id"]
            cortical_list[cortical_area]["position"] = []

        response.status_code = status.HTTP_200_OK
        return cortical_list

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/cortical_info", methods=['GET'], tags=["Connectome"])
async def connectome_cortical_info(cortical_area: str, response: Response):
    try:
        if cortical_area in runtime_data.brain:
            response.status_code = status.HTTP_200_OK
            return runtime_data.brain[cortical_area]
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return {"Requested cortical area not found!"}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# @app.api_route("/v1/feagi/connectome/all", methods=['Get'], tags=["Connectome"])
# async def connectome_comprehensive_info(response: Response):
#     try:
#         if runtime_data.brain:
#             response.status_code = status.HTTP_200_OK
#             return runtime_data.brain
#         else:
#             response.status_code = status.HTTP_404_NOT_FOUND
#     except Exception as e:
#         response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
#         print("API Error:", e)


@app.api_route("/v1/feagi/connectome/plasticity", methods=['Get'], tags=["Connectome"])
async def connectome_plasticity_info(response: Response):
    try:
        if runtime_data.plasticity_dict:
            response.status_code = status.HTTP_200_OK
            return runtime_data.plasticity_dict
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return {}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/path", methods=['Get'], tags=["Connectome"])
async def connectome_system_path(response: Response):
    try:
        if runtime_data.connectome_path:
            response.status_code = status.HTTP_200_OK
            return runtime_data.connectome_path
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return {}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/source", methods=['POST'], tags=["Connectome"])
async def connectome_source_path(connectome_path: str, response: Response):
    try:
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'connectome', 'path', connectome_path,))
        feagi_thread.start()
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/snapshot", methods=['POST'], tags=["Connectome"])
async def connectome_snapshot(connectome_storage_path: str, response: Response):
    try:
        message = {'connectome_path': connectome_storage_path}
        print("Snapshot path:", message)
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.get("/v1/feagi/connectome/download-cortical-area", tags=["Connectome"])
async def connectome_download(cortical_area: str, response: Response):
    print("Downloading Connectome...")
    try:
        file_name = "connectome_" + cortical_area + datetime.datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".json"
        print(file_name)
        if runtime_data.brain[cortical_area]:
            response.status_code = status.HTTP_200_OK
            return FileResponse(path=runtime_data.connectome_path+cortical_area + ".json", filename=file_name)
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        print("API Error:", e)


@app.post("/v1/feagi/connectome/upload-cortical-area", tags=["Connectome"])
async def connectome_file_upload(response: Response, file: UploadFile = File(...)):
    try:
        data = await file.read()
        connectome_str = data.decode("utf-8").split(" = ")[1]
        connectome = literal_eval(connectome_str)
        message = {"connectome": connectome}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        return {"Connectome received as a file"}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/properties/dimensions", methods=['GET'], tags=["Connectome"])
async def connectome_dimensions_report(response: Response):
    try:
        if runtime_data.cortical_dimensions:
            response.status_code = status.HTTP_200_OK
            return runtime_data.cortical_dimensions
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)



@app.api_route("/v1/feagi/connectome/stats/cortical/cumulative", methods=['GET'], tags=["Connectome"])
async def connectome_dimensions_report(response: Response, cortical_area: str):
    try:
        if runtime_data.cumulative_stats[cortical_area]:
            response.status_code = status.HTTP_200_OK
            return runtime_data.cumulative_stats[cortical_area]
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/connectome/properties/mappings", methods=['GET'], tags=["Connectome"])
async def connectome_mapping_report(response: Response):
    """
    Report result can be used with the following tool to visualize the connectome mapping:

    https://csacademy.com/app/graph_editor/

    Note: Use the print out from FEAGI logs for above online editor
    """
    try:
        mappings = cortical_mapping()
        if mappings:
            response.status_code = status.HTTP_200_OK
            return mappings
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/connectome/download", tags=["Connectome"])
async def download_connectome(response: Response):
    """
    Creates a compressed file containing the entire brain data
    """
    print("Downloading Genome...")
    try:
        file_name = "brain_" + datetime.datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".feagi"
        print(file_name)
        brain_data = preserve_brain()

        with tempfile.NamedTemporaryFile(delete=False, suffix='.gz') as temp_file:
            temp_file.write(brain_data)

            # Ensure all data is written
            temp_file.flush()

            # Get the size of the file
            temp_file_size = temp_file.tell()
            print(f"Size of data: {temp_file_size} bytes")

            # Seek back to the start of the file
            temp_file.seek(0)

            # Create the FileResponse inside the with block
            response = FileResponse(temp_file.name, media_type="application/gzip", filename=file_name)

            return response

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/connectome/upload", tags=["Connectome"])
async def upload_connectome(response: Response, file: UploadFile = File(...)):
    try:
        runtime_data.brain_readiness = False
        runtime_data.genome = {}
        brain_data = await file.read()
        revive_brain(brain_data=brain_data)
        deploy_genome(genome_data=runtime_data.pending_genome)
        runtime_data.new_genome = True
        print("\n Brain successfully initialized.")
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# ######  Burst-Engine Endpoints #########
# ########################################

@app.api_route("/v1/feagi/feagi/burst_engine/burst_counter", methods=['GET'], tags=["Burst Engine"])
async def burst_engine_params(response: Response):
    """
    Return the number associated with current FEAGI burst instance.
    """
    try:
        if runtime_data.burst_count:
            response.status_code = status.HTTP_200_OK
            return runtime_data.burst_count
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/burst_engine/stimulation_period", methods=['GET'], tags=["Burst Engine"])
async def burst_engine_params(response: Response):
    """
    Returns the time it takes for each burst to execute in seconds.
    """
    try:
        if runtime_data.burst_timer:
            response.status_code = status.HTTP_200_OK
            return runtime_data.burst_timer
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.api_route("/v1/feagi/feagi/burst_engine", methods=['POST'], tags=["Burst Engine"])
async def burst_management(message: BurstEngine, response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'burst_management': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

# ######  Networking Endpoints #########
# ##################################

@app.api_route("/v1/feagi/feagi/network", methods=['GET'], tags=["Networking"])
async def network_management(response: Response):
    try:
        if runtime_data.parameters['Sockets']:
            response.status_code = status.HTTP_200_OK
            return runtime_data.parameters['Sockets']
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# @app.api_route("/v1/feagi/feagi/network", methods=['POST'], tags=["Networking"])
# async def network_management(message: Network):
#     try:
#         message = message.dict()
#         message = {'network_management': message}
#         api_queue.put(item=message)
#         return runtime_data.parameters['Sockets']
#     except Exception as e:
#         print("API Error:", e)
#         


# ######  Peripheral Nervous System Endpoints #########
# #####################################################

@app.api_route("/v1/feagi/feagi/pns/current/ipu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_ipu_list(response: Response):
    try:
        if runtime_data.ipu_list:
            response.status_code = status.HTTP_200_OK
            return runtime_data.ipu_list
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/pns/current/opu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_opu_list(response: Response):
    try:
        if runtime_data.opu_list:
            response.status_code = status.HTTP_200_OK
            return runtime_data.opu_list
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc)
        

@app.api_route("/v1/agent/list", methods=['GET'], tags=["Peripheral Nervous System"])
async def agent_list(response: Response):
    try:
        agents = set(runtime_data.agent_registry.keys())
        if agents:
            response.status_code = status.HTTP_200_OK
            return agents
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
        

@app.api_route("/v1/agent/properties", methods=['GET'], tags=["Peripheral Nervous System"])
async def agent_properties(agent_id: str, response: Response):
    try:
        print("agent_id", agent_id)
        print("agent_registry", runtime_data.agent_registry)
        agent_info = {}
        if agent_id in runtime_data.agent_registry:
            agent_info["agent_type"] = runtime_data.agent_registry[agent_id]["agent_type"]
            agent_info["agent_ip"] = runtime_data.agent_registry[agent_id]["agent_ip"]
            agent_info["agent_data_port"] = runtime_data.agent_registry[agent_id]["agent_data_port"]
            agent_info["agent_router_address"] = runtime_data.agent_registry[agent_id]["agent_router_address"]
            agent_info["agent_version"] = runtime_data.agent_registry[agent_id]["agent_version"]
            agent_info["controller_version"] = runtime_data.agent_registry[agent_id]["controller_version"]
            response.status_code = status.HTTP_200_OK
            return agent_info
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc)


def assign_available_port():
    ports_used = []
    PORT_RANGES = (40001, 40050)
    for agent_id, agent_info in runtime_data.agent_registry.items():
        print(agent_id, agent_info, agent_info['agent_type'], type(agent_info['agent_type']))
        if agent_info['agent_type'] != 'monitor':
            ports_used.append(agent_info['agent_data_port'])
    print("ports_used", ports_used)
    for port in range(PORT_RANGES[0], PORT_RANGES[1]):
        if port not in ports_used:
            return port
    return None


@app.api_route("/v1/agent/register", methods=['POST'], tags=["Peripheral Nervous System"])
async def agent_registration(request: Request, agent_type: str, agent_id: str, agent_ip: str, agent_data_port: int,
                             agent_version: str, controller_version: str, response: Response):

    try:
        if agent_id in runtime_data.agent_registry:
            agent_info = runtime_data.agent_registry[agent_id]
        else:
            agent_info = {}
            agent_info["agent_id"] = agent_id
            agent_info["agent_type"] = agent_type
            # runtime_data.agent_registry[agent_id]["agent_ip"] = agent_ip
            agent_info["agent_ip"] = request.client.host
            if agent_type == 'monitor':
                agent_router_address = f"tcp://{request.client.host}:{agent_data_port}"
                agent_info["listener"] = Sub(address=agent_router_address, bind=False)
                print("Publication of brain activity turned on!")
                runtime_data.brain_activity_pub = True
            else:
                agent_data_port = assign_available_port()
                agent_router_address = f"tcp://*:{agent_data_port}"
                agent_info["listener"] = Sub(address=agent_router_address, bind=True)

            agent_info["agent_data_port"] = agent_data_port
            agent_info["agent_router_address"] = agent_router_address
            agent_info["agent_version"] = agent_version
            agent_info["controller_version"] = controller_version

        print(f"AGENT Details -- {agent_info}")
        runtime_data.agent_registry[agent_id] = agent_info

        print("New agent has been successfully registered:", runtime_data.agent_registry[agent_id])
        agent_info = runtime_data.agent_registry[agent_id].copy()
        agent_info.pop('listener')
        response.status_code = status.HTTP_200_OK
        return agent_info
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        print("Error during agent registration.:", agent_id)
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        return False


@app.api_route("/v1/agent/deregister", methods=['DELETE'], tags=["Peripheral Nervous System"])
async def agent_deregisteration(agent_id: str, response: Response):
    try:
        if agent_id in runtime_data.agent_registry:
            agent_info = runtime_data.agent_registry.pop(agent_id)
            agent_info['listener'].terminate()
            response.status_code = status.HTTP_200_OK
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


# ######   System Endpoints #########
# ###################################

def linux_to_human_time(linux_time):
    # The provided Linux time seems to be in microseconds (as it's significantly larger than typical Unix timestamps)
    # Dividing by 1,000,000 to convert to seconds
    human_readable_time = datetime.datetime.utcfromtimestamp(int(linux_time) / 1_000_000)
    return human_readable_time.strftime("%Y-%m-%d %H:%M:%S UTC")


@app.get("/v1/feagi/versions", tags=["System"])
def get_versions():
    try:
        all_versions = dict()
        all_versions["feagi"] = linux_to_human_time(__version__)
        for agent_id in runtime_data.agent_registry:
            if agent_id not in all_versions:
                all_versions[agent_id] = {}
            all_versions[agent_id]["agent_version"] = \
                linux_to_human_time(runtime_data.agent_registry[agent_id]["agent_version"])
            all_versions[agent_id]["controller_version"] = \
                linux_to_human_time(runtime_data.agent_registry[agent_id]["controller_version"])
        return all_versions
    except Exception as e:
        print(f"Error during version collection {e}")


@app.get("/v1/feagi/health_check", tags=["System"])
async def feagi_health_check(response: Response):
    response.status_code = status.HTTP_200_OK
    health = dict()
    health["burst_engine"] = not runtime_data.exit_condition
    if runtime_data.genome:
        health["genome_availability"] = True
    else:
        health["genome_availability"] = False
    health["genome_validity"] = runtime_data.genome_validity
    health["brain_readiness"] = runtime_data.brain_readiness
    return health


@app.get("/v1/feagi/unique_logs", tags=["System"])
async def unique_log_entries():
    return runtime_data.logs


@app.api_route("/v1/feagi/register", methods=['POST'], tags=["System"])
async def feagi_registration(message: Registration, response: Response):
    try:
        message = message.dict()
        source = message['source']

        host = message['host']
        capabilities = message['capabilities']
        print("########## ###### >>>>>> >>>> ", source, host, capabilities)
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/logs", methods=['POST'], tags=["System"])
async def log_management(message: Logs, response: Response):
    try:
        message = message.dict()
        message = {"log_management": message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/configuration", methods=['Get'], tags=["System"])
async def configuration_parameters(response: Response):
    try:
        response.status_code = status.HTTP_200_OK
        return runtime_data.parameters
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/beacon/subscribers", methods=['GET'], tags=["System"])
async def beacon_query(response: Response):
    try:
        if runtime_data.beacon_sub:
            print("A")
            response.status_code = status.HTTP_200_OK
            return tuple(runtime_data.beacon_sub)
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            print("B")
            return {}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/beacon/subscribe", methods=['POST'], tags=["System"])
async def beacon_subscribe(message: Subscriber, response: Response):
    try:
        message = {'beacon_sub': message.subscriber_address}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/beacon/unsubscribe", methods=['DELETE'], tags=["System"])
async def beacon_unsubscribe(message: Subscriber, response: Response):
    try:
        message = {"beacon_unsub": message.subscriber_address}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK
        
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/db/influxdb/test", methods=['GET'], tags=["System"])
async def test_influxdb(response: Response):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        influx_status = runtime_data.influxdb.test_influxdb()
        if influx_status:
            response.status_code = status.HTTP_200_OK
            return influx_status
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return influx_status
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/circuit_library_path", methods=['POST'], tags=["System"])
async def change_circuit_library_path(circuit_library_path: str, response: Response):
    try:
        if os.path.exists(circuit_library_path):
            runtime_data.circuit_lib_path = circuit_library_path
            print(f"{circuit_library_path} is the new circuit library path.")
            response.status_code = status.HTTP_200_OK
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            print(f"{circuit_library_path} is not a valid path.")
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)

