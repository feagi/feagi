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
feagi_thread = Thread(target=start_feagi, args=(api_queue,))


class FeagiAdmin(BaseModel):
    online: Optional[bool]


class FeagiLogs(BaseModel):
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


@app.api_route("/v1/feagi/feagi/start", methods=['POST'])
async def feagi_management(message: FeagiAdmin):
    try:
        # obj = 'feagi'
        # end_pnt = 'start'
        # message_to_feagi = {obj, end_pnt, message}
        # api_queue.put(item=message_to_feagi)

        if message.online:
            feagi_thread.start()
            return {"FEAGI Started!"}
        else:
            feagi_thread.join()
            return {"FEAGI Stopped!"}
    except Exception as e:
        return {"FEAGI start failed ... error details to be provided here", e}


@app.api_route("/v1/feagi/feagi/logs", methods=['POST'])
async def log_management(message: FeagiLogs):
    try:
        obj = 'feagi'
        end_pnt = 'logs'
        message_to_feagi = [obj, end_pnt, message]
        api_queue.put(item=message_to_feagi)
        return {"Request sent!"}
    except Exception as e:
        return {"Request failed...", e}
