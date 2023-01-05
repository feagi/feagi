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

from time import sleep

from fastapi import FastAPI, File, UploadFile, Response, status, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from starlette.responses import FileResponse
from pydantic import BaseModel, Field
from typing import Optional, Literal
from ast import literal_eval
from threading import Thread
from queue import Queue
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
from evo.x_genesis import neighboring_cortical_areas
from evo.genome_processor import genome_2_1_convertor
from .config import settings
from inf.messenger import Pub, Sub


logger = logging.getLogger(__name__)


description = """
FEAGI REST API will help you integrate FEAGI into other applications and provides a programmatic method to interact with 
FEAGI.

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
    feagi_thread = Thread(target=feagi.start_feagi, args=(api_queue,))
    feagi_thread.start()


kickstart_feagi_thread()


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]
    burst_duration = 1


class MorphologyProperties(BaseModel):
    name: str
    type: Literal['vectors', 'patterns', 'functions']
    morphology: list


class NewCorticalProperties(BaseModel):
    cortical_type: str
    cortical_name: str
    cortical_coordinates: dict = {
        'x': 0,
        'y': 0,
        'z': 0,
    }
    channel_count: Optional[int]


class NewCustomCorticalProperties(BaseModel):
    cortical_name: str = Field(None, max_length=20, min_length=1)
    cortical_coordinates: dict = {
        'x': 0,
        'y': 0,
        'z': 0,
    }
    cortical_dimensions: dict = {
        'x': 1,
        'y': 1,
        'z': 1,
    }


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
#     neuron_plasticity_constant: float
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
    cortical_coordinates: Optional[dict] = {
        'x': 0,
        'y': 0,
        'z': 0,
    }
    cortical_dimensions: Optional[dict] = {
        'x': 1,
        'y': 1,
        'z': 1,
    }
    cortical_synaptic_attractivity: Optional[int]
    neuron_post_synaptic_potential: Optional[float]
    neuron_post_synaptic_potential_max: Optional[float]
    neuron_plasticity_constant: Optional[float]
    neuron_fire_threshold: Optional[float]
    neuron_refractory_period: Optional[int]
    neuron_leak_coefficient: Optional[float]
    neuron_leak_variability: Optional[int]
    neuron_consecutive_fire_count: Optional[int]
    neuron_snooze_period: Optional[int]
    neuron_degeneracy_coefficient: Optional[float]
    neuron_psp_uniform_distribution: Optional[bool]


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


class Genome(BaseModel):
    genome: dict


class Stimulation(BaseModel):
    stimulation_script: dict


class Training(BaseModel):
    shock: tuple


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


app.mount("/home", SPAStaticFiles(directory="gui", html=True), name="static")


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


# Append to the CORS origin
@app.middleware("http")
async def update_cors_origin(request, call_next):
    response = await call_next(request)
    origin = response.headers.get("Access-Control-Allow-Origin", "")
    new_origin = ""
    response.headers["Access-Control-Allow-Origin"] = f"{origin},{new_origin}"
    return response


# ######  Genome Endpoints #########
# ##################################

@app.api_route("/v1/feagi/genome/upload/default", methods=['POST'], tags=["Genome"])
async def genome_default_upload():
    try:

        with open("./evo/static_genome.json", "r") as genome_file:
            genome_data = json.load(genome_file)
            runtime_data.genome_file_name = "static_genome.json"
        message = {'genome': genome_data}

        api_queue.put(item=message)
        return {"FEAGI started using a static genome.", message}
    except Exception as e:
        print("API Error:", e)
        return {"FEAGI start using genome string failed ...", e}


@app.post("/v1/feagi/genome/upload/file", tags=["Genome"])
async def genome_file_upload(file: UploadFile = File(...)):
    """
    This API allows you to browse files from your computer and upload a genome to FEAGI.
    The genome must be in the form of a python file.
    """
    try:
        data = await file.read()

        runtime_data.genome_file_name = file.filename

        genome_str = json.loads(data)

        # genome_str = genome_str.replace('\'', '\"')
        # genome_str = data.decode("utf-8").split(" = ")[1]
        message = {'genome': genome_str}
        api_queue.put(item=message)

        return {"Genome received as a file"}
    except Exception as e:
        print("API ERROR during genome file upload:\n", e, traceback.print_exc())
        return {"Request failed..."}


@app.get("/v1/feagi/genome/file_name", tags=["Genome"])
async def genome_file_name():
    """
    Returns the name of the genome file last uploaded to FEAGI
    """
    try:
        return runtime_data.genome_file_name
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed..."}


@app.api_route("/v1/feagi/genome/upload/string", methods=['POST'], tags=["Genome"])
async def genome_string_upload(str_genome: Genome):
    try:
        genome = str_genome.genome
        print("@@@@@@@@@   @@@@@@@\n", genome)
        message = {'genome': genome}
        api_queue.put(item=message)

        return {"FEAGI started using a genome string."}
    except Exception as e:
        print("API Error:", e)
        return {"FEAGI start using genome string failed ...", e}


@app.get("/v1/feagi/genome/download", tags=["Genome"])
async def genome_download(response:Response):
    print("Downloading Genome...")
    try:
        file_name = "genome_" + datetime.datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".json"
        print(file_name)
        if runtime_data.genome:
            response.status_code = status.HTTP_200_OK
            return FileResponse(path="../runtime_genome.json", filename=file_name)
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/genome/upload/file/edit", tags=["Genome"])
async def genome_file_upload_edit(file: UploadFile = File(...)):
    try:
        data = await file.read()
        genome_str = data.decode("utf-8")
        return {genome_str}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.get("/v1/feagi/genome/defaults/files", tags=["Genome"])
async def genome_default_files():
    try:
        default_genomes_path = "./evo/defaults/genome/"
        default_genomes = os.listdir(default_genomes_path)
        genome_mappings = {}
        for genome in default_genomes:
            if genome[:2] != '__':
                with open(os.path.join(default_genomes_path, genome)) as file:
                    data = file.read()
                    data_dict = literal_eval(data.split(" = ")[1])
                    genome_mappings[genome.split(".")[0]] = json.dumps(data_dict)
        return {"genomes": genome_mappings}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed..."}


@app.api_route("/v1/feagi/genome/genome_number", methods=['GET'], tags=["Genome"])
async def genome_number():
    """
    Return the number associated with current Genome instance.
    """
    try:
        return runtime_data.genome_counter
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/genome/reset", tags=["Genome"])
async def reset_genome():
    try:
        print("API call has triggered a genome reset")
        runtime_data.genome_reset_flag = True
        return
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area", methods=['GET'], tags=["Genome"])
async def fetch_cortical_properties(cortical_area, response: Response):
    """
    Returns the properties of cortical areas
    """
    try:
        if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
            cortical_data = runtime_data.genome['blueprint'][cortical_area]

            cortical_properties = {
                "cortical_id": cortical_area,
                "cortical_name": cortical_data['cortical_name'],
                "cortical_group": cortical_data['group_id'],
                "cortical_neuron_per_vox_count": cortical_data['per_voxel_neuron_cnt'],
                "cortical_visibility": cortical_data['visualization'],
                "cortical_synaptic_attractivity": cortical_data['synapse_attractivity'],
                "cortical_coordinates": {
                    'x': cortical_data["relative_coordinate"][0],
                    'y': cortical_data["relative_coordinate"][1],
                    'z': cortical_data["relative_coordinate"][2]
                },
                "cortical_dimensions": {
                    'x': cortical_data["block_boundaries"][0],
                    'y': cortical_data["block_boundaries"][1],
                    'z': cortical_data["block_boundaries"][2]
                },
                "cortical_destinations": cortical_data['cortical_mapping_dst'],
                "neuron_post_synaptic_potential": cortical_data['postsynaptic_current'],
                "neuron_post_synaptic_potential_max": cortical_data['postsynaptic_current_max'],
                "neuron_plasticity_constant": cortical_data['plasticity_constant'],
                "neuron_fire_threshold": cortical_data['firing_threshold'],
                "neuron_refractory_period": cortical_data['refractory_period'],
                "neuron_leak_coefficient": cortical_data['leak_coefficient'],
                "neuron_leak_variability": cortical_data['leak_variability'],
                "neuron_consecutive_fire_count": cortical_data['consecutive_fire_cnt_max'],
                "neuron_snooze_period": cortical_data['snooze_length'],
                "neuron_degeneracy_coefficient": cortical_data['degeneration'],
                "neuron_psp_uniform_distribution": cortical_data['psp_uniform_distribution']
            }
            response.status_code = status.HTTP_200_OK
            return cortical_properties
        else:
            response.status_code = status.HTTP_400_BAD_REQUEST
            return {"message": "Error! Cortical area id should be only 6 characters long"}
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area", methods=['PUT'], tags=["Genome"])
async def update_cortical_properties(message: UpdateCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'update_cortical_properties': message}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area", methods=['POST'], tags=["Genome"])
async def add_cortical_area(message: NewCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'add_core_cortical_area': message}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/custom_cortical_area", methods=['POST'], tags=["Genome"])
async def add_cortical_area(message: NewCustomCorticalProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'add_custom_cortical_area': message}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area", methods=['DELETE'], tags=["Genome"])
async def delete_cortical_area(cortical_area_name):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = {'delete_cortical_area': cortical_area_name}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area_id_list", methods=['GET'], tags=["Genome"])
async def genome_cortical_ids():
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        return sorted(runtime_data.cortical_list)
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_name_location", methods=['GET'], tags=["Genome"])
async def genome_cortical_location_by_name(cortical_name):
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        cortical_area = cortical_name_to_id(cortical_name=cortical_name)
        return runtime_data.genome["blueprint"][cortical_area]["relative_coordinate"]
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_area_name_list", methods=['GET'], tags=["Genome"])
async def genome_cortical_names():
    """
    Returns a comprehensive list of all cortical area names.
    """
    try:
        return sorted(cortical_name_list())
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology_list", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphologies():
    """
    Returns a comprehensive list of all neuron morphologies.
    """
    morphology_names = set()
    try:
        for morphology in runtime_data.genome['neuron_morphologies']:
            morphology_names.add(morphology)
        return sorted(morphology_names)
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology_types", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_types():
    """
    Returns the properties of a neuron morphology.
    """
    try:
        return {"vectors", "patterns", "functions"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology_functions", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_functions():
    """
    Returns the list of morphology function names.
    """
    try:
        morphology_list = set()
        for entry in dir(synaptogenesis_rules):
            if str(entry)[:4] == "syn_":
                morphology_list.add(str(entry))
        return morphology_list
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_properties(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        if morphology_name in runtime_data.genome['neuron_morphologies']:
            return runtime_data.genome['neuron_morphologies'][morphology_name]
        else:
            return {}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology_usage", methods=['GET'], tags=["Genome"])
async def genome_neuron_morphology_usage_report(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        return morphology_usage_list(morphology_name=morphology_name)
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology", methods=['PUT'], tags=["Genome"])
async def genome_update_neuron_morphology(message: MorphologyProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'update_morphology_properties': message}
        print("*" * 50 + "\n", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology", methods=['POST'], tags=["Genome"])
async def genome_add_neuron_morphology(message: MorphologyProperties):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        if message.name not in runtime_data.genome['neuron_morphologies']:
            runtime_data.genome['neuron_morphologies'][message.name] = {}
            runtime_data.genome['neuron_morphologies'][message.name][message.type] = list()
            runtime_data.genome['neuron_morphologies'][message.name][message.type].append(message.morphology)
        else:
            return "Morphology already exists! Nothing was added."
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/morphology", methods=['DELETE'], tags=["Genome"])
async def genome_delete_neuron_morphology(morphology_name):
    """
    Returns the properties of a neuron morphology.
    """
    try:
        if morphology_name in runtime_data.genome['neuron_morphologies']:
            usage = morphology_usage_list(morphology_name=morphology_name)
            if not usage:
                runtime_data.genome['neuron_morphologies'].pop(morphology_name)
                return {"message": "Morphology has been successfully deleted."}
            else:
                return {"message": "Morphology could not be removed due to existing mappings",
                        "data": usage}
        else:
            return {"message": "Morphology not found!"}
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed...", e}


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
#         return {"Request failed...", e}


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
            return {"message": "Error! Cortical area id should be only 6 characters long"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_mappings/afferents", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area, response: Response):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if len(cortical_area) == genome_properties["structure"]["cortical_name_length"]:
            upstream_cortical_areas, downstream_cortical_areas = neighboring_cortical_areas(cortical_area)

            return upstream_cortical_areas
        else:
            response.status_code = status.HTTP_400_BAD_REQUEST
            return {"message": "Error! Cortical area id should be only 6 characters long"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_mappings_by_name", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical names being downstream to the given cortical areas
    """
    try:
        cortical_mappings = set()
        for destination in runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']:
            cortical_mappings.add(runtime_data.genome['blueprint'][destination]['cortical_name'])
        return cortical_mappings
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_mappings_detailed", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mappings(cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        return runtime_data.genome['blueprint'][cortical_area]['cortical_mapping_dst']

    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/mapping_properties", methods=['GET'], tags=["Genome"])
async def fetch_cortical_mapping_properties(src_cortical_area, dst_cortical_area):
    """
    Returns the list of cortical areas downstream to the given cortical areas
    """
    try:
        if dst_cortical_area in runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst']:
            print("get mapping data:", runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'])
            return runtime_data.genome['blueprint'][src_cortical_area]['cortical_mapping_dst'][dst_cortical_area]
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/mapping_properties", methods=['PUT'], tags=["Genome"])
async def update_cortical_mapping_properties(src_cortical_area, dst_cortical_area, mapping_string: list):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        print("$$ $$ " * 30)
        print(mapping_string)
        data = dict()
        data["mapping_data"] = mapping_string
        data["src_cortical_area"] = src_cortical_area
        data["dst_cortical_area"] = dst_cortical_area

        data = {'update_cortical_mappings': data}
        print("*" * 50 + "\n", data)
        api_queue.put(item=data)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        logger.error(traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_types", methods=['GET'], tags=["Genome"])
async def cortical_area_types():
    """
    Returns the list of supported cortical types
    """
    try:
        return runtime_data.cortical_types

    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/cortical_type_options", methods=['GET'], tags=["Genome"])
async def cortical_area_types(cortical_type):
    """
    Returns the list of supported cortical area for a given type
    """
    try:
        if cortical_type in cortical_types:
            cortical_list = set()
            for item in cortical_types[cortical_type]['supported_devices']:
                if cortical_types[cortical_type]['supported_devices'][item]['enabled']:
                    cortical_list.add(item)
            return cortical_list
        else:
            return None

    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/circuits", methods=['GET'], tags=["Genome"])
async def cortical_area_types():
    """
    Returns the list of neuronal circuits under /evo/circuits
    """
    try:
        circuit_list = os.listdir("./evo/circuits")
        return circuit_list

    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/circuit_size", methods=['GET'], tags=["Genome"])
async def cortical_area_types(circuit_name):
    """
    Returns the overall size of a circuit
    """
    try:
        with open("./evo/circuits/" + circuit_name, "r") as genome_file:
            genome_data = json.load(genome_file)

        genome2 = genome_2_1_convertor(flat_genome=genome_data["blueprint"])
        circuit_size_ = circuit_size(blueprint=genome2["blueprint"])

        return circuit_size_

    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/append", methods=['POST'], tags=["Genome"])
async def genome_add_neuron_morphology(circuit_name: str, location: list):
    """
    Appends a given circuit to the running genome at a specific location.
    """
    try:
        print("Placeholder")
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# ######  Evolution #########
# #############################

@app.api_route("/v1/feagi/evolution/autopilot/status", methods=['GET'], tags=["Evolution"])
async def retrun_autopilot_status():
    """
    Returns the status of genome autopilot system.
    """
    try:
        return runtime_data.autopilot
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/evolution/autopilot/on", tags=["Evolution"])
async def turn_autopilot_on():
    try:
        if not runtime_data.autopilot:
            autopilot.init_generation_dict()
            if runtime_data.brain_run_id:
                autopilot.update_generation_dict()
            runtime_data.autopilot = True
            print("<" * 30, "  Autopilot has been turned on  ", ">" * 30)
        return
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/evolution/autopilot/off", tags=["Evolution"])
async def turn_autopilot_off():
    try:
        runtime_data.autopilot = False
        return
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/evolution/generations", methods=['GET'], tags=["Evolution"])
async def list_generations():
    """
    Return details about all generations.
    """
    try:
        return runtime_data.generation_dict
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# ######  Stimulation #########
# #############################

@app.api_route("/v1/feagi/stimulation/upload/string", methods=['POST'], tags=["Stimulation"])
async def stimulation_string_upload(stimulation_script: Stimulation):
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
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/stimulation/reset", methods=['POST'], tags=["Stimulation"])
async def stimulation_string_upload():
    try:
        message = {"stimulation_script": {}}
        message = {'stimulation_script': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@app.get("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_mp_collection_scope:
            return True
        else:
            return False
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area, state: bool):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
        print("influx:", runtime_data.influxdb)
        if runtime_data.influxdb:
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope.pop(cortical_area)
                else:
                    pass
        else:
            print("Error: InfluxDb is not setup to collect timeseries data!")
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.get("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_mp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_mp_collection_scope:
            return True
        else:
            return False
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area, state: bool):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_mp_collection_scope)
    try:
        if runtime_data.influxdb:
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope.pop(cortical_area)
                else:
                    pass
        else:
            print("Error: InfluxDb is not setup to collect timeseries data!")
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.get("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_membrane_potential_collection_filters():
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_mp_collection_scope)
    try:
        return runtime_data.neuron_mp_collection_scope
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.get("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_postsynaptic_potential_collection_filters():
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_psp_collection_scope)
    try:
        return runtime_data.neuron_psp_collection_scope
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting", methods=['POST'], tags=["Insights"])
async def neuron_membrane_potential_monitoring_scope(message: dict):
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
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring", methods=['POST'], tags=["Insights"])
async def neuron_postsynaptic_potential_monitoring_scope(message: dict):
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
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# ######  Training Endpoints #######
# ##################################

@app.api_route("/v1/feagi/training/shock/options", methods=['Get'], tags=["Training"])
async def list_available_shock_scenarios():
    """
    Get a list of available shock scenarios.
    """
    try:
        return runtime_data.shock_scenarios_options
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/training/shock/status", methods=['Get'], tags=["Training"])
async def list_activated_shock_scenarios():
    try:
        return runtime_data.shock_scenarios
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/training/shock/activate", methods=['POST'], tags=["Training"])
async def activate_shock_scenarios(training: Training):
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
        message = training.dict()
        print(message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# #########  Robot   ###########
# ##############################

@app.api_route("/v1/robot/parameters", methods=['POST'], tags=["Robot"])
async def robot_controller_tunner(message: RobotController):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'robot_controller': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/robot/model", methods=['POST'], tags=["Robot"])
async def robot_model_modification(message: RobotModel):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'robot_model': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.get("/v1/feagi/robot/gazebo/files", tags=["Robot"])
async def gazebo_robot_default_files():
    try:
        default_robots_path = "./evo/defaults/robot/"
        default_robots = os.listdir(default_robots_path)
        return {"robots": default_robots}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed..."}


# ######  Connectome Endpoints #########
# ######################################

@app.api_route("/v1/feagi/connectome/cortical_areas", methods=['Get'], tags=["Connectome"])
async def connectome_cortical_areas():
    try:
        return runtime_data.cortical_list
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/cortical_info", methods=['POST'], tags=["Connectome"])
async def connectome_cortical_info(connectome: Connectome):
    try:
        if connectome.cortical_area in runtime_data.brain:
            return runtime_data.brain[connectome.cortical_area]
        else:
            return {"Requested cortical area not found!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/all", methods=['Get'], tags=["Connectome"])
async def connectome_comprehensive_info():
    try:
        return runtime_data.brain
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/plasticity", methods=['Get'], tags=["Connectome"])
async def connectome_plasticity_info():
    try:
        return runtime_data.plasticity_dict
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.post("/v1/feagi/connectome/upload", tags=["Connectome"])
async def connectome_file_upload(file: UploadFile = File(...)):
    try:
        data = await file.read()
        connectome_str = data.decode("utf-8").split(" = ")[1]
        connectome = literal_eval(connectome_str)
        message = {"connectome": connectome}
        api_queue.put(item=message)
        return {"Connectome received as a file"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/source", methods=['POST'], tags=["Connectome"])
async def connectome_source_path(connectome_path: ConnectomePath):
    try:
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'connectome', 'path',  connectome_path,))
        feagi_thread.start()

        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/snapshot", methods=['POST'], tags=["Connectome"])
async def connectome_snapshot(message: ConnectomePath):
    try:
        message = message.dict()
        message = {'connectome_snapshot': message}
        print("Snapshot path:", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/properties/dimensions", methods=['GET'], tags=["Connectome"])
async def connectome_dimensions_report():
    try:
        return runtime_data.cortical_dimensions
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/properties/mappings", methods=['GET'], tags=["Connectome"])
async def connectome_mapping_report():
    """
    Report result can be used with the following tool to visualize the connectome mapping:

    https://csacademy.com/app/graph_editor/

    Note: Use the print out from FEAGI logs for above online editor
    """
    try:
        return cortical_mapping()
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# ######  Burst-Engine Endpoints #########
# ########################################

@app.api_route("/v1/feagi/feagi/burst_engine/burst_counter", methods=['GET'], tags=["Burst Engine"])
async def burst_engine_params():
    """
    Return the number associated with current FEAGI burst instance.
    """
    try:
        return runtime_data.burst_count
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/burst_engine/stimulation_period", methods=['GET'], tags=["Burst Engine"])
async def burst_engine_params():
    """
    Returns the time it takes for each burst to execute in seconds.
    """
    try:
        return runtime_data.burst_timer
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/burst_engine", methods=['POST'], tags=["Burst Engine"])
async def burst_management(message: BurstEngine):
    """
    Enables changes against various Burst Engine parameters.
    """
    try:
        message = message.dict()
        message = {'burst_management': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# ######  Networking Endpoints #########
# ##################################

@app.api_route("/v1/feagi/feagi/network", methods=['GET'], tags=["Networking"])
async def network_management():
    try:
        return runtime_data.parameters['Sockets']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


# @app.api_route("/v1/feagi/feagi/network", methods=['POST'], tags=["Networking"])
# async def network_management(message: Network):
#     try:
#         message = message.dict()
#         message = {'network_management': message}
#         api_queue.put(item=message)
#         return runtime_data.parameters['Sockets']
#     except Exception as e:
#         print("API Error:", e)
#         return {"Request failed...", e}


# ######  Peripheral Nervous System Endpoints #########
# #####################################################

@app.api_route("/v1/feagi/feagi/pns/current/ipu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_ipu_list():
    try:
        return runtime_data.ipu_list
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/pns/current/opu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_opu_list():
    try:
        return runtime_data.opu_list
    except Exception as e:
        return {"Request failed...", e}


# ######   System Endpoints #########
# ###################################

@app.api_route("/v1/feagi/register", methods=['POST'], tags=["System"])
async def feagi_registration(message: Registration):
    try:
        message = message.dict()
        source = message['source']

        host = message['host']
        capabilities = message['capabilities']
        print("########## ###### >>>>>> >>>> ", source, host, capabilities)

        return {"Registration was successful"}
    except Exception as e:
        print("API Error:", e)
        return {"FEAGI start failed ... error details to be provided here", e}


@app.api_route("/v1/feagi/feagi/logs", methods=['POST'], tags=["System"])
async def log_management(message: Logs):
    try:
        message = message.dict()
        message = {"log_management": message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/configuration", methods=['Get'], tags=["System"])
async def configuration_parameters():
    try:
        return runtime_data.parameters
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/beacon/subscribers", methods=['GET'], tags=["System"])
async def beacon_query():
    try:
        if runtime_data.beacon_sub:
            print("A")
            return tuple(runtime_data.beacon_sub)
        else:
            print("B")
            return {}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/beacon/subscribe", methods=['POST'], tags=["System"])
async def beacon_subscribe(message: Subscriber):
    try:
        message = {'beacon_sub': message.subscriber_address}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/beacon/unsubscribe", methods=['DELETE'], tags=["System"])
async def beacon_unsubscribe(message:Subscriber):
    try:
        message = {"beacon_unsub": message.subscriber_address}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/agent/register", methods=['POST'], tags=["System"])
async def agent_registration(agent_type: str, agent_id: str, agent_ip: str, agent_data_port: int):
    try:
        if agent_id not in runtime_data.agent_registry:
            # Add new agent to the registry
            runtime_data.agent_registry[agent_id] = {}
            runtime_data.agent_registry[agent_id]["agent_type"] = agent_type
            runtime_data.agent_registry[agent_id]["agent_ip"] = agent_ip
            runtime_data.agent_registry[agent_id]["agent_data_port"] = agent_data_port

            # Create the needed ZMQ listener for new agent
            agent_router_address = "tcp://" + agent_ip + ':' + str(agent_data_port)
            runtime_data.agent_registry[agent_id]["listener"] = Sub(address=agent_router_address)

            print("New agent has been successfully registered:", runtime_data.agent_registry[agent_id])
        else:
            print("Error during agent registration. Agent with the same id is currently registered:", agent_id)

    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}



# ######   GUI  Endpoints #########
# ###################################

@app.api_route("/v1/feagi/feagi/gui_baseline/ipu", methods=['GET'], tags=["GUI"])
async def supported_ipu_list():
    try:
        return gui_baseline['ipu']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/opu", methods=['GET'], tags=["GUI"])
async def supported_opu_list():
    try:
        return gui_baseline['opu']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/morphology", methods=['GET'], tags=["GUI"])
async def supported_morphology_list():
    try:
        return gui_baseline['morphology']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/cortical-genes", methods=['GET'], tags=["GUI"])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['cortical_genes']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/morphology-scalar", methods=['GET'], tags=["GUI"])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['morphology_scalar']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/psc-multiplier", methods=['GET'], tags=["GUI"])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['postSynapticCurrent_multiplier']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/plasticity-flag", methods=['GET'], tags=["GUI"])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['plasticity_flag']
    except Exception as e:
        print("API Error:", e)
        return {"Request failed...", e}

