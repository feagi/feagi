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

from fastapi import FastAPI
from pydantic import BaseModel
from typing import Optional
from fastapi.staticfiles import StaticFiles
from threading import Thread
from queue import Queue
from inf.feagi import *
from inf import disk_ops, runtime_data


app = FastAPI()
favicon_path = 'favicon.svg'

api_queue = Queue()


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]


class Network(BaseModel):
    godot_host: Optional[str] = 'godot'
    godot_data_port: Optional[int] = 30001
    godot_web_port: Optional[int] = 6081
    gazebo_host: Optional[str] = 'gazebo'
    gazebo_data_port: Optional[int] = 30002
    gazebo_web_port: Optional[int] = 6080
    virtual_stimulator_host: Optional[str] = 'virtual'
    virtual_stimulator_data_port: Optional[int] = 30003


class ConnectomeSnapshot(BaseModel):
    save_to_path: str


class Stats(BaseModel):
    neuron_stat_collection: Optional[bool] = False
    synapse_stat_collection: Optional[bool] = False


app.mount("/home", StaticFiles(directory="api/static", html=True), name="static")


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


@app.api_route("/v1/feagi/feagi/logs", methods=['POST'])
async def log_management(message: Logs):
    try:
        message = message.dict()
        message = {"log_management": message}
        api_queue.put(item=message)
        return {"Request sent!"}
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


@app.api_route("/v1/feagi/feagi/network", methods=['POST'])
async def network_management(message: Network):
    try:
        message = message.dict()
        message = {'network_management': message}
        api_queue.put(item=message)
        return runtime_data.parameters['Sockets']
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

            # todo: Handle web port assignments
