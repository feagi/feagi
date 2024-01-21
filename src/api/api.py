# Copyright 2016-2023 The FEAGI Authors. All Rights Reserved.
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

import traceback
import time
import string
import logging
import random

import io

from fastapi import FastAPI, Depends, File, UploadFile, Response, status, Request, HTTPException


from fastapi.middleware.cors import CORSMiddleware
from starlette.responses import FileResponse, StreamingResponse


from threading import Thread

from io import StringIO, BytesIO

from inf import feagi
from inf import runtime_data
from inf.baseline import gui_baseline

from .config import settings


from .dependencies import *
from .error_handling import *
from .commons import *
from .models import *
from .routers.v1 import burst_engine, connectome, embodiment, evolution, feagi_agent, genome, insights, morphology, \
    network, simulation, system, training

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
    license_info=settings.license_info
)


favicon_path = settings.favicon_path


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


@app.middleware("http")
async def catch_exceptions_middleware(request: Request, call_next):
    try:
        return await call_next(request)
    except CustomError as e:
        # Handle CustomError
        print(f"Exception:\n {e}", traceback.print_exc())
        return JSONResponse(
            status_code=e.status_code,
            content={"message": f"A custom error occurred: {str(e.message)}"},
        )
    except Exception as e:
        print(f"Exception:\n {e}", traceback.print_exc())
        return JSONResponse(
            status_code=500,
            content={"message": f"An error occurred: {str(e)}"},
        )

standard_response = {
        401: {
            "model": UnauthorizedResponse,
            "description": "Unauthorized: Invalid or expired token",
            "content": {
                "application/json": {
                    "example": {"error_code": 401, "error_message": "Invalid token"}
                }
            }
        },
        404: {"description": "Item Not found"},
        500: {
            "model": InternalServerErrorResponse,
            "description": "Internal Server Error",
            "content": {
                "application/json": {
                    "example": {"error_code": 500, "error_message": "Internal error"}
                }
            }
        },
    }


# todo: To add the ability of updating allowable cors list on the fly
# # Append to the CORS origin
# @app.middleware("http")
# async def update_cors_origin(request, call_next):
#     response = await call_next(request)
#     origin = response.headers.get("Access-Control-Allow-Origin", "")
#     new_origin = ""
#     response.headers["Access-Control-Allow-Origin"] = f"{origin},{new_origin}"
#     return response



app.include_router(
    burst_engine.router,
    prefix="/v1/genomes",
    tags=["GENOME MANAGEMENT"],
    dependencies=[Depends(tbd)],
    responses=standard_response
)
