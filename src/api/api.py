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

from fastapi import FastAPI, File, UploadFile
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from ast import literal_eval
from threading import Thread
from queue import Queue
from inf.feagi import *
from inf import disk_ops, runtime_data
from inf.baseline import gui_baseline
from inf.initialize import init_parameters


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
    virtual_stimulator_host: Optional[str] = runtime_data.parameters['Sockets']['virtual_host_name']
    virtual_stimulator_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_virtual']


class ConnectomeSnapshot(BaseModel):
    save_to_path: str


class Registration(BaseModel):
    source: str
    host: str
    capabilities: dict


class Stats(BaseModel):
    neuron_stat_collection: Optional[bool] = False
    synapse_stat_collection: Optional[bool] = False


class Genome(BaseModel):
    genome: dict


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope):
        response = await super().get_response(path, scope)
        print("<><><><><><>")
        if response.status_code == 404:
            print("-=-=-=-=-=-=-=-=-=-=")
            response = await super().get_response('.', scope)
        return response


app.mount("/home", SPAStaticFiles(directory="gui", html=True), name="static")


@app.api_route("/v1/feagi/feagi/launch", methods=['POST'])
async def feagi_management(message: Launch):
    try:
        print("message:", message)
        connectome_overwrite_path = message.existing_connectome
        feagi_thread = Thread(target=start_feagi, args=(api_queue, connectome_overwrite_path,))
        feagi_thread.start()

        if message.existing_connectome:
            return {"FEAGI started using an existing connectome."}
        else:
            return {"FEAGI started using a genome."}
    except Exception as e:
        return {"FEAGI start failed ... error details to be provided here", e}


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


@app.api_route("/v1/feagi/connectome/path", methods=['POST'])
async def brain_management(message: ConnectomeSnapshot):
    try:
        message = message.dict()
        message = {'connectome_snapshot': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/connectome/snapshot", methods=['POST'])
async def brain_management(message: ConnectomeSnapshot):
    try:
        message = message.dict()
        message = {'connectome_snapshot': message}
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
        genome = literal_eval(genome_str)

        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'genome', '',  genome,))
        feagi_thread.start()

        return {"Genome received as a file"}
    except Exception as e:
        return {"Request failed...", e}


@app.api_route("/v1/feagi/genome/upload/string", methods=['POST'])
async def genome_string_upload(genome: Genome):
    try:
        feagi_thread = Thread(target=start_feagi, args=(api_queue, 'genome', '',  genome.genome,))
        feagi_thread.start()

        return {"FEAGI started using a genome string."}
    except Exception as e:
        return {"FEAGI start using genome string failed ...", e}


# ######  Statistics and Reporting Endpoints #########
# ##################################

@app.api_route("/v1/feagi/stats", methods=['POST'])
async def stat_management(message: Stats):
    try:
        message = message.dict()
        message = {'stats': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}


def api_message_processor(api_message):
        """
        Processes the incoming API calls to FEAGI
        """

        if 'burst_management' in api_message:
            if 'burst_duration' in api_message['burst_management']:
                if api_message['burst_management']['burst_duration'] is not None:
                    runtime_data.burst_timer = api_message['burst_management']['burst_duration']

        if 'log_management' in api_message:
            if 'print_burst_info' in api_message['log_management']:
                runtime_data.parameters['Logs']['print_burst_info'] \
                    = api_message['log_management']['print_burst_info']
            if 'print_messenger_logs' in api_message['log_management']:
                runtime_data.parameters['Logs']['print_messenger_logs'] \
                    = api_message['log_management']['print_messenger_logs']

        if 'connectome_snapshot' in api_message:
            if 'save_to_path' in api_message['connectome_snapshot']:
                if api_message['connectome_snapshot']['save_to_path']:
                    disk_ops.save_brain_to_disk(connectome_path=api_message['connectome_snapshot']['save_to_path'],
                                                type='snapshot')
                else:
                    disk_ops.save_brain_to_disk()

        if 'stats' in api_message:
            if 'neuron_stat_collection' in api_message['stats'] and \
                    api_message['stats']['neuron_stat_collection'] is not None:
                if api_message['stats']['neuron_stat_collection']:
                    runtime_data.collect_neuron_stats = True
                    print("Starting to capture neuronal activity stats into database...")
                else:
                    runtime_data.collect_neuron_stats = False
                    print("Stopping the capture of neuronal activity stats into database.")

            if 'synapse_stat_collection' in api_message['stats'] and \
                    api_message['stats']['synapse_stat_collection'] is not None:
                if api_message['stats']['synapse_stat_collection']:
                    runtime_data.collect_synapse_stats = True
                    print("Starting to capture synaptic activity stats into database...")
                else:
                    runtime_data.collect_synapse_stats = False
                    print("Stopping the capture of synaptic activity stats into database.")

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

            if 'virtual_host' in api_message['network_management']:
                runtime_data.parameters['Sockets']['virtual_host_name'] = api_message['network_management'][
                    'virtual_host']
            if 'virtual_port' in api_message['network_management']:
                runtime_data.parameters['Sockets']['feagi_inbound_port_virtual'] = \
                    api_message['network_management']['virtual_port']

        if 'genome' in api_message:
            pass

            # todo: Handle web port assignments
