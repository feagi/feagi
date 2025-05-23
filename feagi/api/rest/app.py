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
import os
from pathlib import Path

from fastapi import FastAPI, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response, HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.openapi.utils import get_openapi
from threading import Thread
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException

from .config import settings

from .commons import CustomError, api_queue, check_burst_engine_or_allow_genome_ops, check_burst_engine, check_brain_running, check_active_genome

# Remove the old router imports - no longer needed since we use universal wrapper directly
from feagi.api.dependencies import *
from feagi.api.models import *
from feagi.core.state_manager import FeagiStateManager, ServiceState
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.rest.dependencies import get_connectome
from .response_utils import success_response, error_response

# Import the universal FastAPI wrapper directly instead of individual router files
from feagi.api.transport.universal_fastapi import (
    get_system_router, get_genome_router, get_cortical_area_router,
    get_connectome_router, get_burst_engine_router, get_neuroplasticity_router,
    get_region_router, get_morphology_router, get_monitoring_router,
    get_simulation_router, get_feagi_agent_router, get_insights_router,
    get_training_router, get_cortical_mapping_router, get_network_router,
    get_inputs_router, get_outputs_router, get_evolution_router
)

# Note: v2 routers have been removed since we now use the universal wrapper directly for all routes
# The v2 functionality can be added in the future if needed via the universal wrapper pattern




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
    docs_url=None,
    swagger_ui_parameters={
        "defaultModelsExpandDepth": -1,
        "filter": True,  # Enable filtering
        # "jsonEditor": True
        "syntaxHighlight.theme": "monokai",
        "docExpansion": "none",
        "deepLinking": True,
        "persistAuthorization": True,
        "displayOperationId": False,
        "tryItOutEnabled": True,
        "theme": "dark",
        "defaultModelRendering": "model",
        "showExtensions": True,
        "showCommonExtensions": True,
        "layout": "BaseLayout",
        "displayRequestDuration": True,
        "withCredentials": True,
        "requestSnippetsEnabled": True,
        "requestSnippets": {
            "generators": {
                "curl_bash": {"title": "cURL (bash)", "syntax": "bash"},
                "curl_powershell": {"title": "cURL (PowerShell)", "syntax": "powershell"},
                "python_requests": {"title": "Python (requests)", "syntax": "python"}
            },
            "defaultExpanded": True,
            "languages": ["curl_bash", "curl_powershell", "python_requests"]
        }
    }
)

# Get the directory of the current file
current_dir = Path(__file__).parent
static_dir = current_dir / "static"

# Mount static files directory
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

# Custom Swagger UI with dark theme
def custom_swagger_ui_html():
    # Read the custom HTML template directly
    template_path = Path(__file__).parent / "static" / "custom-swagger-ui.html"
    print(f"DEBUG: Loading custom Swagger UI template from {template_path}")
    try:
        with open(template_path, "r") as f:
            html_content = f.read()
        
        # Replace the OpenAPI URL placeholder with the actual URL
        html_content = html_content.replace("{{ openapi_url }}", str(app.openapi_url))
        
        print("DEBUG: Custom Swagger UI template loaded successfully")
        return HTMLResponse(content=html_content)
    except Exception as e:
        print(f"ERROR: Failed to load custom Swagger UI template: {e}")
        # Fall back to default Swagger UI
        return get_swagger_ui_html(
            openapi_url=app.openapi_url,
            title=app.title + " - API Documentation",
            swagger_js_url="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5.9.0/swagger-ui-bundle.js",
            swagger_css_url="https://cdn.jsdelivr.net/npm/swagger-ui-dist@5.9.0/swagger-ui.css",
        )


@app.get("/docs", include_in_schema=False)
async def swagger_ui_html_route():
    # Use the same custom UI for the default /docs route
    return custom_swagger_ui_html()


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
    Conditional API request logging middleware.
    Only logs when FEAGI_DEBUG_API environment variable is set to '1'.
    When enabled, provides detailed request/response information for debugging.
    
    Credit: Phil Girard (original middleware)
    """
    # Check if debug API logging is enabled
    debug_api_enabled = os.environ.get("FEAGI_DEBUG_API", "0") == "1"
    
    if not debug_api_enabled:
        # If debug is not enabled, just pass through without logging
        return await call_next(request)
    
    # Generate unique request ID for tracking
    idem = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    
    # Log request start with detailed information
    logger.info(f"rid={idem} start request method={request.method} path={request.url.path}", emoji1="🌐")
    logger.debug(f"rid={idem} url={str(request.url)}")
    logger.debug(f"rid={idem} headers={dict(request.headers)}")
    logger.debug(f"rid={idem} query_params={dict(request.query_params)}")
    
    # Log path parameters if available
    if hasattr(request, 'path_params') and request.path_params:
        logger.debug(f"rid={idem} path_params={dict(request.path_params)}")
    
    # Note: We avoid reading request.body() here to prevent stream consumption issues
    # Request body logging would require more complex caching mechanisms
    
    start_time = time.time()
    response = await call_next(request)
    process_time = (time.time() - start_time) * 1000
    formatted_process_time = '{0:.2f}'.format(process_time)
    
    # Log response information
    logger.info(f"rid={idem} completed_in={formatted_process_time}ms status_code={response.status_code}", emoji1="✅")
    logger.debug(f"rid={idem} response_headers={dict(response.headers)}")
    
    # Note: Response body logging is also complex due to streaming responses
    # For debugging purposes, the status code and headers are usually sufficient
    
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
    get_genome_router(),
    prefix="/v1/genome",
    tags=["GENOME"],
    dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
    responses=standard_response
)

app.include_router(
    get_connectome_router(),
    prefix="/v1/connectome",
    tags=["CONNECTOME"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    get_burst_engine_router(),
    prefix="/v1/burst_engine",
    tags=["BURST ENGINE"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    get_evolution_router(),
    prefix="/v1/evolution",
    tags=["EVOLUTIONARY"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    get_feagi_agent_router(),
    prefix="/v1/agent",
    tags=["FEAGI AGENT"],
    dependencies=[Depends(check_burst_engine)],
    responses=standard_response
)

app.include_router(
    get_insights_router(),
    prefix="/v1/insight",
    tags=["INSIGHTS"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    get_morphology_router(),
    prefix="/v1/morphology",
    tags=["NEURON MORPHOLOGIES"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_cortical_area_router(),
    prefix="/v1/cortical_area",
    tags=["CORTICAL AREAS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_region_router(),
    prefix="/v1/region",
    tags=["BRAIN REGIONS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_cortical_mapping_router(),
    prefix="/v1/cortical_mapping",
    tags=["CORTICAL MAPPINGS"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_neuroplasticity_router(),
    prefix="/v1/neuroplasticity",
    tags=["NEUROPLASTICITY"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_inputs_router(),
    prefix="/v1/input",
    tags=["INPUT MANAGEMENT"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

app.include_router(
    get_network_router(),
    prefix="/v1/network",
    tags=["NETWORK"],
    dependencies=[],
    responses=standard_response
)

app.include_router(
    get_simulation_router(),
    prefix="/v1/simulation",
    tags=["SIMULATION"],
    dependencies=[Depends(check_brain_running)],
    responses=standard_response
)

app.include_router(
    get_system_router(),
    prefix="/v1/system",
    tags=["SYSTEM"],
    dependencies=[],
    responses=standard_response
)

app.include_router(
    get_training_router(),
    prefix="/v1/training",
    tags=["TRAINING"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

# Add the missing outputs router
app.include_router(
    get_outputs_router(),
    prefix="/v1/output",
    tags=["OUTPUT MANAGEMENT"],
    dependencies=[Depends(check_active_genome)],
    responses=standard_response
)

# Add the missing monitoring router
app.include_router(
    get_monitoring_router(),
    prefix="/v1/monitoring",
    tags=["MONITORING"],
    dependencies=[],
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

