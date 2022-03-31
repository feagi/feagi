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
from threading import Thread
from queue import Queue
from inf.feagi import *

app = FastAPI()

api_queue = Queue()


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]


class ConnectomeSnapshot(BaseModel):
    save_to_path: str


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


@app.api_route("/v1/feagi/connectome/snapshot", methods=['POST'])
async def brain_management(message: ConnectomeSnapshot):
    try:
        message = message.dict()
        message = {'connectome_snapshot': message}
        api_queue.put(item=message)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}
