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


from fastapi import APIRouter, HTTPException

from ...commons import *

router = APIRouter()


# ######  Networking Endpoints #########
# ##################################

@router.get("/v1/feagi/feagi/network")
async def network_management():
    if runtime_data.parameters['Sockets']:
        return runtime_data.parameters['Sockets']
    else:
        raise HTTPException(status_code=400, detail=f"Networking data not available!")


# @router.api_route("/v1/feagi/feagi/network", methods=['POST'], tags=["Networking"])
# async def network_management(message: Network):
#     try:
#         message = message.dict()
#         message = {'network_management': message}
#         api_queue.put(item=message)
#         return runtime_data.parameters['Sockets']
#     except Exception as e:
#         print("API Error:", e)
#
