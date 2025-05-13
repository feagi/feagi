#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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
import random
from feagi.utils.logger import setup_logger
logger = setup_logger(name="api__server")
logger.info("...")
import json

from fastapi import FastAPI, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from threading import Thread
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException

from .config import settings

from .commons import CustomError, api_queue, check_burst_engine_or_allow_genome_ops

from .routers.v1 import burst_engine, connectome, evolution, feagi_agent, genome, insights, morphology, \
    network, simulation, system, training, cortical_area, neuroplasticity, cortical_mapping, region
from .routers.v1 import inputs
from feagi.api.dependencies import *
from feagi.api.models import *
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.dependencies import get_connectome
from .response_utils import success_response, error_response

# Import the v2 routers
from .routers.v2 import genome as genome_v2
# ... other v2 imports




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
    swagger_ui_parameters={
        "defaultModelsExpandDepth": -1,
        "filter": True,  # Enable filtering
        # "jsonEditor": True
        }
    )


favicon_path = settings.favicon_path


ORIGINS = settings.origins

app.add_middleware(
    CORSMiddleware,
    allow_origins=ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["Content-Disposition"],
)


@app.middleware("http")
async def log_requests(request: Request, call_next):
    """
    Credit: Phil Girard
    """
    idem = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    logger.info(f"rid={idem} start request path={request.url.path}", emoji1="🌐")
    start_time = time.time()

    response = await call_next(request)

    process_time = (time.time() - start_time) * 1000
    formatted_process_time = '{0:.2f}'.format(process_time)
    logger.info(f"rid={idem} completed_in={formatted_process_time}ms status_code={response.status_code}", emoji1="✅")

    # print(response.status_code, ":", request.method, ":", request.url.path)
    return response


@app.middleware("http")
async def catch_exceptions_middleware(request: Request, call_next):
    try:
        return await call_next(request)
    except CustomError as e:
        # Handle CustomError
        logger.error(f"❌ Exception:\n {e}\n{traceback.format_exc()}")
        return JSONResponse(
            status_code=e.status_code,
            content={"message": f"A custom error occurred: {str(e.message)}"},
        )
    except Exception as e:
        logger.error(f"❌ Exception:\n {e}\n{traceback.format_exc()}")
        return JSONResponse(
            status_code=500,
            content={
                "type": "error",
                "code": "UNHANDLED_EXCEPTION",
                "message": f"An error occurred: {str(e)}"
            },
        )

standard_response = {
        400: {
            "model": GeneralErrorResponse,
            "description": "All Handled Errors",
            "content": {
                "application/json": {
                    "example": {"error_code": 400, "error_message": "Request failed"}
                }
            }
        },
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
    genome.router,
    prefix="/v1/genome",
    tags=["GENOME"],
    dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
    responses=standard_response
)

app.include_router(
    connectome.router,
    prefix="/v1/connectome",
    tags=["CONNECTOME"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    burst_engine.router,
    prefix="/v1/burst_engine",
    tags=["BURST ENGINE"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    evolution.router,
    prefix="/v1/evolution",
    tags=["EVOLUTIONARY"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    feagi_agent.router,
    prefix="/v1/agent",
    tags=["FEAGI AGENT"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    insights.router,
    prefix="/v1/insight",
    tags=["INSIGHTS"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    morphology.router,
    prefix="/v1/morphology",
    tags=["NEURON MORPHOLOGIES"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    cortical_area.router,
    prefix="/v1/cortical_area",
    tags=["CORTICAL AREAS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    region.router,
    prefix="/v1/region",
    tags=["BRAIN REGIONS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    cortical_mapping.router,
    prefix="/v1/cortical_mapping",
    tags=["CORTICAL MAPPINGS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    neuroplasticity.router,
    prefix="/v1/neuroplasticity",
    tags=["NEUROPLASTICITY"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    inputs.router,
    prefix="/v1/input",
    tags=["INPUT MANAGEMENT"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    network.router,
    prefix="/v1/network",
    tags=["NETWORK"],
    dependencies=[],
    responses=standard_response
)

app.include_router(
    simulation.router,
    prefix="/v1/simulation",
    tags=["SIMULATION"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    system.router,
    prefix="/v1/system",
    tags=["SYSTEM"],
    dependencies=[],
    responses=standard_response
)
app.include_router(
    training.router,
    prefix="/v1/training",
    tags=["TRAINING"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

# Register v2 routes
app.include_router(
    genome_v2.router,
    prefix="/v2/genome",
    tags=["GENOME V2"],
    dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
    responses=standard_response
)

@app.on_event("startup")
async def set_api_state_ready():
    state = FeagiStateManager.instance()
    state.set_api_state(ServiceState.READY)

def create_rest_app(connectome: ConnectomeManager = None):
    """Factory function to return the FastAPI app instance, with connectome dependency injection."""
    
    # If no connectome was provided, create a new one
    if connectome is None:
        connectome = ConnectomeManager()
        connectome.initialize_arrays()
    # If connectome was provided but isn't initialized, initialize it
    elif not hasattr(connectome, 'fcl_manager') or connectome.fcl_manager is None:
        connectome.initialize_arrays()
        
    # Set the connectome in the dependencies module, not in a local variable
    from feagi.api.rest.dependencies import set_connectome_instance, set_core_api_service
    set_connectome_instance(connectome)
    
    # Create the core API service with the initialized connectome
    core_api_service = CoreAPIService(connectome)
    
    # Make it available to routers
    set_core_api_service(core_api_service)
    
    return app

def get_core_api():
    """Dependency placeholder for the core API service. Should be overridden in tests."""
    raise NotImplementedError("get_core_api must be overridden in tests with a mock implementation.")

@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    return JSONResponse(
        status_code=exc.status_code,
        content=error_response(message=exc.detail, error_code=f"HTTP_{exc.status_code}")
    )

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    return JSONResponse(
        status_code=422,
        content=error_response(
            message="Validation error",
            error_code="VALIDATION_ERROR",
            metadata={"errors": exc.errors()}
        )
    )

@app.exception_handler(Exception)
async def generic_exception_handler(request, exc):
    # Log the exception here
    return JSONResponse(
        status_code=500,
        content=error_response(message=str(exc), error_code="INTERNAL_ERROR")
    )

@app.middleware("http")
async def standardize_response_format(request, call_next):
    """
    Middleware that standardizes API responses.
    - Skips standardization for v1 routes
    - Applies standardization to v2+ routes
    - Honors raw_response() markers 
    """
    response = await call_next(request)
    
    # Skip if response is already in our format or for non-JSON responses
    if response.headers.get("content-type") != "application/json":
        return response
        
    # First check if this is a raw response that should bypass standardization
    try:
        body = await response.body()
        content = json.loads(body)
        
        # Check for raw response marker
        if isinstance(content, dict) and content.get("__raw_response__"):
            # Remove the marker and return raw response
            if "__raw_response__" in content:
                del content["__raw_response__"]
            return JSONResponse(content=content, status_code=response.status_code)
    except:
        # If we can't parse JSON or other issues, just return original response
        return response
        
    # Skip standardization for v1 routes to maintain backward compatibility
    if "/v1/" in request.url.path:
        # Need to rebuild the response since we've consumed the body
        return Response(
            content=body,
            status_code=response.status_code,
            headers=dict(response.headers),
            media_type=response.media_type
        )
    
    # For v2+ routes, apply standardization for success responses
    if response.status_code < 400:
        try:
            content = json.loads(body)
            return JSONResponse(
                content=success_response(data=content),
                status_code=response.status_code
            )
        except:
            # If we can't standardize, return original response rebuilt
            return Response(
                content=body,
                status_code=response.status_code,
                headers=dict(response.headers),
                media_type=response.media_type
            )
    
    # For any other case, rebuild the original response
    return Response(
        content=body,
        status_code=response.status_code,
        headers=dict(response.headers),
        media_type=response.media_type
    )

