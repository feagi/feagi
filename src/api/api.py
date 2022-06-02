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
from fastapi import FastAPI, File, UploadFile
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from starlette.responses import FileResponse
from pydantic import BaseModel
from typing import Optional
from ast import literal_eval
from threading import Thread
from queue import Queue
from inf.feagi import *
from inf import disk_ops, runtime_data
from inf.baseline import gui_baseline
from inf.initialize import init_parameters
from evo import static_genome


init_parameters()

app = FastAPI()
favicon_path = 'favicon.svg'

api_queue = Queue()

ORIGINS = [
    "http://localhost:6080",
    "http://localhost:6081",
    "http://localhost:3000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]


class Network(BaseModel):
    godot_host: Optional[str] = runtime_data.parameters['Sockets']['godot_host_name']
    godot_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_godot']
    godot_web_port: Optional[int] = 6081
    gazebo_host: Optional[str] = runtime_data.parameters['Sockets']['gazebo_host_name']
    gazebo_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo']
    gazebo_web_port: Optional[int] = 6080


class ConnectomePath(BaseModel):
    connectome_path: str


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


class StatsCollectionScope(BaseModel):
    collection_scope: dict


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope):
        response = await super().get_response(path, scope)
        print("<><><><><><>")
        if response.status_code == 404:
            print("-=-=-=-=-=-=-=-=-=-=")
            response = await super().get_response('.', scope)
        return response


app.mount("/home", SPAStaticFiles(directory="gui", html=True), name="static")


# @app.api_route("/v1/feagi/feagi/launch", methods=['POST'])
# async def feagi_management():
#     try:
#         print("message:", message)
#         connectome_overwrite_path = message.existing_connectome
#         feagi_thread = Thread(target=start_feagi, args=(api_queue, connectome_overwrite_path,))
#         feagi_thread.start()
#
#         if message.existing_connectome:
#             return {"FEAGI started using an existing connectome."}
#         else:
#             return {"FEAGI started using a genome."}
#     except Exception as e:
#         return {"FEAGI start failed ... error details to be provided here", e}


@app.api_route("/v1/feagi/feagi/register", methods=['POST'])
async def feagi_registration(message: Registration):
    try:
        message = message.dict()
        source = message['source']
        host = message['host']
        capabilities = message['capabilities']
        print("########## ###### >>>>>> >>>> ", source, host, capabilities)

        return {"Registration was successful"}
    except Exception as e:
        return {"FEAGI start failed ... error details to be provided here", e}


@app.api_route("/v1/feagi/feagi/logs", methods=['POST'])
async def log_management(message: Logs):
    try:
        message = message.dict()
        message = {"log_management": message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/burst_engine/stimulation_period", methods=['GET'])
async def burst_engine_params():
    try:
        return runtime_data.burst_timer
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/ipu", methods=['GET'])
async def supported_ipu_list():
    try:
        return gui_baseline['ipu']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/opu", methods=['GET'])
async def supported_opu_list():
    try:
        return gui_baseline['opu']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/morphology", methods=['GET'])
async def supported_morphology_list():
    try:
        return gui_baseline['morphology']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/cortical-genes", methods=['GET'])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['cortical_genes']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/morphology-scalar", methods=['GET'])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['morphology_scalar']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/psc-multiplier", methods=['GET'])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['postSynapticCurrent_multiplier']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/gui_baseline/plasticity-flag", methods=['GET'])
async def supported_cortical_genes_list():
    try:
        return gui_baseline['plasticity_flag']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/pns/ipu", methods=['GET'])
async def ipu_list():
    try:
        return runtime_data.ipu_list
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/pns/opu", methods=['GET'])
async def ipu_list():
    try:
        return runtime_data.opu_list
    except Exception as e:
        return {"Request failed...", e}


# ######  Burst-Engine Endpoints #########
# ##################################

@app.api_route("/v1/feagi/feagi/burst_engine/burst_counter", methods=['GET'])
async def burst_engine_params():
    try:
        return runtime_data.burst_count
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/burst_engine", methods=['POST'])
async def burst_management(message: BurstEngine):
    try:
        message = message.dict()
        message = {'burst_management': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


# ######  Networking Endpoints #########
# ##################################

@app.api_route("/v1/feagi/feagi/network", methods=['GET'])
async def network_management():
    try:
        return runtime_data.parameters['Sockets']
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/feagi/network", methods=['POST'])
async def network_management(message: Network):
    try:
        message = message.dict()
        message = {'network_management': message}
        api_queue.put(item=message)
        return runtime_data.parameters['Sockets']
    except Exception as e:
        return {"Request failed...", e}


# ######  Connectome Endpoints #########
# ##################################


@app.post("/v1/feagi/connectome/upload")
async def connectome_file_upload(file: UploadFile = File(...)):
    try:
        data = await file.read()
        connectome_str = data.decode("utf-8").split(" = ")[1]
        connectome = literal_eval(connectome_str)
        message = {"connectome": connectome}
        api_queue.put(item=message)
        return {"Connectome received as a file"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/source", methods=['POST'])
async def connectome_source_path(connectome_path: ConnectomePath):
    try:
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'connectome', 'path',  connectome_path,))
        feagi_thread.start()

        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/snapshot", methods=['POST'])
async def connectome_snapshot(message: ConnectomePath):
    try:
        message = message.dict()
        message = {'connectome_snapshot': message}
        print("Snapshot path:", message)
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/properties/dimensions", methods=['GET'])
async def connectome_report():
    print("cortical_dimensions", runtime_data.cortical_dimensions)
    try:
        return runtime_data.cortical_dimensions
    except Exception as e:
        return {"Request failed...", e}


# ######  Genome Endpoints #########
# ##################################

@app.post("/v1/feagi/genome/upload/file")
async def genome_file_upload(file: UploadFile = File(...)):
    try:
        data = await file.read()

        genome_str = data.decode("utf-8").split(" = ")[1]
        runtime_data.genome = literal_eval(genome_str)

        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'genome', '',  runtime_data.genome,))
        feagi_thread.start()

        return {"Genome received as a file"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/upload/default", methods=['POST'])
async def genome_default_upload():
    try:
        runtime_data.genome = static_genome.genome

        print("default_genome", runtime_data.genome)
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'genome', '',  runtime_data.genome,))
        feagi_thread.start()

        return {"FEAGI started using a genome string."}
    except Exception as e:
        return {"FEAGI start using genome string failed ...", e}


@app.api_route("/v1/feagi/genome/upload/string", methods=['POST'])
async def genome_string_upload(genome: Genome):
    try:
        runtime_data.genome = genome.genome
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'genome', '',  runtime_data.genome,))
        feagi_thread.start()

        return {"FEAGI started using a genome string."}
    except Exception as e:
        return {"FEAGI start using genome string failed ...", e}


@app.get("/v1/feagi/genome/download/python")
async def genome_download():
    print("Downloading Genome...")
    try:
        file_name = "genome_" + datetime.datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p") + ".py"
        print(file_name)
        return FileResponse(path="../runtime_genome.py", filename=file_name)
    except Exception as e:
        return {"Request failed...", e}


# ######  Stimulation #########
# ##################################

@app.api_route("/v1/feagi/stimulation/upload/string", methods=['POST'])
async def stimulation_string_upload(stimulation_script: Stimulation):
    """
    stimulation_script = {
    "IR_pain": {
        "start_burst": 10,
        "end_burst": 1000,
        "definition": [
            {"i__pro": ["0-0-3"], "o__mot": ["2-0-7"]},
            {"i__pro": ["0-0-8"]},
            {"i__bat": ["0-0-7"]},
            {"i__bat": ["0-0-6"]},
            {"i__bat": ["0-0-5"]},
            {"i__bat": ["0-0-4"]},
            {"i__bat": ["0-0-3"]},
            {},
            {"i__bat": ["0-0-2"]},
            {"i__bat": ["0-0-1"]},
            {},
            {}
            ]
    },
    "exploration": {
        "definition": []
    },
    "move_forward": {
        "end_burst": 500,
        "definition": []
    },
    "charge_batteries": {
        "start_burst": 1000,
        "definition": [
            {"i__inf": ["2-0-0"]}
        ]
    }
}

    """
    try:
        message = stimulation_script.dict()
        message = {'stimulation_script': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


# ######  Statistics and Reporting Endpoints #########
# ##################################

@app.get("/v1/feagi/neuron/physiology/stats")
async def neuron_physiological_stat_collection_report():
    print("Physiological stat collections:", runtime_data.neuron_physiological_stat_collection)
    try:
        return runtime_data.neuron_physiological_stat_collection
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/neuron/physiology/stats", methods=['POST'])
async def neuron_physiological_stat_collection(message: StatsCollectionScope):
    """
    Message Template:
    {
        "collection_scope":
            {
                "o__mot": {
                    "voxels": [[0, 0, 0], [2, 0, 0]],
                    "neurons": [],
                    "area_wide": false,
                    "afferent": true,
                    "efferent": true
                },
                "i__inf": {
                    "voxels": [[1, 1, 1]],
                    "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3'],
                    "area_wide": true
                    "afferent": false,
                    "efferent": true
                },
                ...
            }
    }
    """

    try:
        message = message.dict()
        message = {'neuron_physiological_stat_collection': message}
        api_queue.put(item=message)
        return {"Request sent!", message}
    except Exception as e:
        return {"Request failed...", e}


################################################################################################################

def api_message_processor(api_message):
        """
        Processes the incoming API calls to FEAGI
        """

        if 'burst_management' in api_message:
            if 'burst_duration' in api_message['burst_management']:
                if api_message['burst_management']['burst_duration'] is not None:
                    runtime_data.burst_timer = api_message['burst_management']['burst_duration']

        if 'stimulation_script' in api_message:
            runtime_data.stimulation_script = api_message['stimulation_script']['stimulation_script']

        if 'log_management' in api_message:
            if 'print_burst_info' in api_message['log_management']:
                runtime_data.parameters['Logs']['print_burst_info'] \
                    = api_message['log_management']['print_burst_info']
            if 'print_messenger_logs' in api_message['log_management']:
                runtime_data.parameters['Logs']['print_messenger_logs'] \
                    = api_message['log_management']['print_messenger_logs']

        if 'connectome_snapshot' in api_message:
            if 'connectome_path' in api_message['connectome_snapshot']:
                if api_message['connectome_snapshot']['connectome_path']:
                    print("Taking a snapshot of the brain... ... ...")
                    disk_ops.save_brain_to_disk(connectome_path=api_message['connectome_snapshot']['connectome_path'],
                                                type='snapshot')
                else:
                    disk_ops.save_brain_to_disk()

        if 'neuron_physiological_stat_collection' in api_message:
            if api_message['neuron_physiological_stat_collection'] is not None:
                payload = api_message['neuron_physiological_stat_collection']['collection_scope']
                runtime_data.neuron_physiological_stat_collection = payload
                print('Membrane Potential state collection scope has been updated.')
            else:
                print('Membrane Potential state collection scope did not change.')

        if 'network_management' in api_message:
            print("api_message", api_message)
            if 'godot_host' in api_message['network_management']:
                runtime_data.parameters['Sockets']['godot_host_name'] = api_message['network_management']['godot_host']
            if 'godot_port' in api_message['network_management']:
                runtime_data.parameters['Sockets']['feagi_inbound_port_godot'] = \
                    api_message['network_management']['godot_port']

            if 'gazebo_host' in api_message['network_management']:
                runtime_data.parameters['Sockets']['gazebo_host_name'] = api_message['network_management']['gazebo_host']
            if 'gazebo_port' in api_message['network_management']:
                runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo'] = \
                    api_message['network_management']['gazebo_port']



            # todo: Handle web port assignments
