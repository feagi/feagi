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
import os
from feagi.utils.logger import setup_logger
logger = setup_logger(name="api__server")
logger.info("...")
import json
from pathlib import Path
from typing import Dict, Any

from fastapi import FastAPI, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.openapi.utils import get_openapi
from threading import Thread
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException

from .config import settings

from .commons import CustomError, api_queue, check_burst_engine_or_allow_genome_ops, check_burst_engine, check_brain_running, check_active_genome, check_burst_engine_or_allow_config_ops

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

# Import the visualization router
from feagi.api.v1.visualization import router as visualization_router

# Note: v2 routers have been removed since we now use the universal wrapper directly for all routes
# The v2 functionality can be added in the future if needed via the universal wrapper pattern




description = """FEAGI REST API will help you integrate FEAGI into other applications and 
provides a programmatic method to interact with FEAGI. 

"""

# Create the FastAPI application
app = FastAPI(
    title="FEAGI REST API",
    description="Framework for Evolutionary Artificial General Intelligence",
    version="2.0.0",
    docs_url=None,  # Disable default docs
    redoc_url=None,  # Disable default redoc
    openapi_url="/openapi.json"
)

# Get the directory of the current file
current_dir = Path(__file__).parent
static_dir = current_dir / "static"

# Mount static files directory
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

# Custom Swagger UI with dark theme
def custom_swagger_ui_html():
    """
    Custom Swagger UI with Windows compatibility and better error handling.
    This fixes the white screen issue that can occur on Windows systems.
    """
    # Read the custom HTML template directly with Windows-compatible path handling
    template_path = Path(__file__).parent / "static" / "custom-swagger-ui.html"
    logger.info(f"Loading custom Swagger UI template from {template_path}", status="[CONFIG]")
    
    try:
        # Use pathlib and explicit encoding for Windows compatibility
        with open(template_path, "r", encoding="utf-8") as f:
            html_content = f.read()
        
        # Replace the OpenAPI URL placeholder with the actual URL
        # Ensure the URL is properly formatted for all platforms
        openapi_url = str(app.openapi_url)
        if not openapi_url.startswith('/'):
            openapi_url = '/' + openapi_url
            
        html_content = html_content.replace("{{ openapi_url }}", openapi_url)
        
        # Add Windows-specific error handling and debugging
        debug_script = """
        <script>
        // Windows compatibility debugging
        console.log('FEAGI Swagger UI Debug: Starting initialization...');
        console.log('OpenAPI URL:', '""" + openapi_url + """');
        
        // Add error handling for failed OpenAPI spec loading
        window.addEventListener('error', function(e) {
            console.error('FEAGI Swagger UI Error:', e);
            document.getElementById('swagger-ui').innerHTML = 
                '<div style="padding: 20px; background: #fff3cd; border: 1px solid #ffeaa7; color: #856404; border-radius: 5px; margin: 20px;">' +
                '<h3>API Documentation Loading Issue</h3>' +
                '<p>The Swagger UI failed to load. This can happen due to:</p>' +
                '<ul>' +
                '<li>Network connectivity issues</li>' +
                '<li>CORS restrictions</li>' +
                '<li>OpenAPI specification errors</li>' +
                '<li>JavaScript errors in browser console</li>' +
                '</ul>' +
                '<p><strong>Solutions:</strong></p>' +
                '<ul>' +
                '<li>Check browser console for detailed errors</li>' +
                '<li>Try accessing <a href="' + '""" + openapi_url + """' + '" target="_blank">OpenAPI spec directly</a></li>' +
                '<li>Refresh the page or try a different browser</li>' +
                '<li>Contact support if the issue persists</li>' +
                '</ul>' +
                '</div>';
        });
        </script>
        """
        
        # Insert debug script before the closing body tag
        html_content = html_content.replace("</body>", debug_script + "</body>")
        
        logger.info("Custom Swagger UI template loaded successfully", status="[OK]")
        return HTMLResponse(content=html_content)
        
    except FileNotFoundError:
        logger.error(f"Custom Swagger UI template not found at {template_path}", status="[ERR]")
        return _fallback_swagger_ui()
    except UnicodeDecodeError as e:
        logger.error(f"Encoding error reading Swagger UI template: {e}", status="[ERR]")
        return _fallback_swagger_ui()
    except Exception as e:
        logger.error(f"Failed to load custom Swagger UI template: {e}", status="[ERR]")
        return _fallback_swagger_ui()


def _fallback_swagger_ui():
    """
    Fallback to default FastAPI Swagger UI if custom template fails.
    """
    logger.warning("Using fallback default Swagger UI", status="[WARN]")
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


@app.get("/", include_in_schema=False)
async def root_redirect():
    """
    Automatically redirect root URL to API documentation.
    When users visit http://127.0.0.1:8000/, they'll be redirected to /docs
    """
    return RedirectResponse(url="/docs", status_code=302)


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
    Enhanced to capture request body and response details for comprehensive debugging.
    """
    # Check if debug API logging is enabled
    debug_api_enabled = os.environ.get("FEAGI_DEBUG_API", "0") == "1"
    
    if not debug_api_enabled:
        # If debug is not enabled, just pass through without logging
        return await call_next(request)
    
    # Show debug enabled message on first request (if not already shown)
    if not hasattr(log_requests, '_debug_shown'):
        logger.info("🐛 API Debug logging is ENABLED - detailed request/response logging active")
        log_requests._debug_shown = True
    
    # Generate unique request ID for tracking
    idem = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    
    # Log request start with detailed information
    logger.info(f"rid={idem} [OK] start request method={request.method} path={request.url.path}")
    logger.info(f"rid={idem} [NET] url={str(request.url)}")
    logger.info(f"rid={idem} headers={dict(request.headers)}")
    logger.info(f"rid={idem} [SEARCH] query_params={dict(request.query_params)}")
    
    # Log path parameters if available
    if hasattr(request, 'path_params') and request.path_params:
        logger.info(f"rid={idem} [PATH]  path_params={dict(request.path_params)}")
    
    # Capture request body for debugging
    request_body = None
    try:
        body_bytes = await request.body()
        if body_bytes:
            request_body = body_bytes.decode('utf-8')
            logger.info(f"rid={idem} [LOG] request_body={request_body}")
        else:
            logger.info(f"rid={idem} [LOG] request_body=<empty>")
    except Exception as e:
        logger.warning(f"rid={idem} [WARN] failed to read request body: {e}")
    
    # Store original body for downstream handlers (since we consumed the stream)
    async def receive():
        return {"type": "http.request", "body": body_bytes if 'body_bytes' in locals() else b""}
    
    # Patch the request's receive method
    if 'body_bytes' in locals():
        request._receive = receive
    
    start_time = time.time()
    response = await call_next(request)
    process_time = (time.time() - start_time) * 1000
    formatted_process_time = '{0:.2f}'.format(process_time)
    
    # Log response details
    logger.info(f"rid={idem} [OK] completed method={request.method} path={request.url.path} status={response.status_code} duration={formatted_process_time}ms")
    
    # Try to capture response body if it's JSON
    try:
        if hasattr(response, 'body') and response.body:
            response_body = response.body.decode('utf-8')
            # Truncate very long responses to avoid log spam
            if len(response_body) > 1000:
                response_body = response_body[:1000] + "... (truncated)"
            logger.info(f"rid={idem} response_body={response_body}")
        else:
            logger.info(f"rid={idem} response_body=<empty or not accessible>")
    except Exception as e:
        logger.info(f"rid={idem} response_body=<could not read: {e}>")
    
    return response


@app.middleware("http")
async def catch_exceptions_middleware(request: Request, call_next):
    try:
        return await call_next(request)
    except CustomError as e:
        # Handle CustomError
        logger.error(f"[ERR] Exception:\n {e}\n{traceback.format_exc()}")
        return JSONResponse(
            status_code=e.status_code,
            content={"message": f"A custom error occurred: {str(e.message)}"},
        )
    except Exception as e:
        logger.error(f"[ERR] Exception:\n {e}\n{traceback.format_exc()}")
        return JSONResponse(
            status_code=500,
            content={
                "type": "error",
                "code": "UNHANDLED_EXCEPTION",
                "message": f"An error occurred: {str(e)}"
            },
        )

# Standard response format for all endpoints
standard_response = {
    200: {"description": "Success"},
    400: {"description": "Bad Request"},
    404: {"description": "Not Found"},
    500: {"description": "Internal Server Error"}
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

@app.on_event("startup")
async def set_api_state_ready():
    state = FeagiStateManager.instance()
    state.set_api_state(ServiceState.READY)

def create_rest_app_direct(config: Dict[str, Any]):
    """
    RUST/RTOS COMPATIBLE: Factory function for REST app with direct dependency injection.
    
    This eliminates subprocess boundaries and environment variable dependencies,
    making the code much easier to port to Rust where all services run as async tasks
    in the same process space.
    
    Args:
        config: Direct configuration with all dependencies:
            - core_api: CoreAPIService instance
            - state_manager: FeagiStateManager instance  
            - connectome_manager: ConnectomeManager instance
            - host: API server host
            - port: API server port
            - debug: Debug mode flag
    """
    logger.info("[LINK] Creating REST app with direct dependency injection (Rust/RTOS compatible)", status="[LINK]")
    
    # RUST/RTOS COMPATIBLE: Direct dependency injection instead of environment lookup
    core_api_service = config['core_api']
    state_manager = config['state_manager']
    connectome_manager = config['connectome_manager']
    
    if not core_api_service:
        raise RuntimeError("CoreAPIService is required for direct REST app creation")
    
    logger.info("[OK] Using directly injected CoreAPIService and ConnectomeManager", status="[OK]")
    logger.info("[TARGET] All dependencies injected directly - no environment variables needed", status="[TARGET]")
    
    # Set the connectome instance for FastAPI dependency injection  
    from feagi.api.rest.dependencies import set_connectome_instance
    set_connectome_instance(connectome_manager)
    
    # Set core API service for dependency injection
    from feagi.api.rest.dependencies import set_core_api_service_instance
    set_core_api_service_instance(core_api_service)
    
    # For now, return the existing app instance (already configured with all routes)
    # In future iterations, we can create a fresh app instance here
    global app
    
    # Set state in FeagiStateManager
    state_manager.set_api_state(ServiceState.READY)
    logger.info("REST API state changed: UNAVAILABLE → READY", status="[FAST]")
    
    return app

def create_rest_app(connectome: ConnectomeManager = None):
    """Factory function to return the FastAPI app instance, with connectome dependency injection."""
    
    # CRITICAL FIX: Ensure true singleton pattern for mission-critical reliability
    core_api_service = None
    
    # Check if we're running as part of the main FEAGI process (singleton mode)
    if os.environ.get("FEAGI_INITIALIZED") == "1":
        logger.info("[LINK] Running in FEAGI subprocess, using singleton ConnectomeManager", status="[LINK]")
        
        # CRITICAL FIX: In subprocess mode, we can't access parent's ProcessManager
        # Instead, use the singleton ConnectomeManager directly
        from feagi.bdu.connectome_manager import ConnectomeManager
        connectome = ConnectomeManager.instance()
        logger.info("[TARGET] Created singleton ConnectomeManager instance", status="[TARGET]")
        
        # Create CoreAPIService with singleton instances
        from feagi.core.state_manager import FeagiStateManager
        from feagi.core import create_core_api
        
        state_manager = FeagiStateManager.instance()
        core_api_service = create_core_api(connectome, {})
        
        if core_api_service:
            logger.info("[OK] Successfully created CoreAPIService with singleton ConnectomeManager", status="[OK]")
        else:
            logger.error("[ERR] Failed to create CoreAPIService", status="[ERR]")
            raise RuntimeError("Failed to create CoreAPIService")
            
        logger.info("[TARGET] FastAPI app configured with subprocess singleton services", status="[TARGET]")
        
    else:
        # Standalone mode (development/testing)
        logger.info("Running in standalone mode, using singleton ConnectomeManager", status="[CONFIG]")
        
        if connectome is None:
            from feagi.bdu.connectome_manager import ConnectomeManager
            connectome = ConnectomeManager.instance()  # Use singleton pattern instead of direct instantiation
            
        # Create a basic CoreAPIService for standalone operation
        from feagi.core import create_core_api
        core_api_service = create_core_api(connectome, {})
        
        if not core_api_service:
            logger.error("[ERR] Failed to create CoreAPIService in standalone mode", status="[ERR]")
            raise RuntimeError("Failed to create CoreAPIService in standalone mode")
    
    # Set the connectome instance for FastAPI dependency injection  
    from feagi.api.rest.dependencies import set_connectome_instance
    set_connectome_instance(connectome)
    
    # Set core API service for dependency injection
    from feagi.api.rest.dependencies import set_core_api_service_instance
    set_core_api_service_instance(core_api_service)
    
    # Get the global app instance
    global app
    
    # CRITICAL: Include all routers here instead of at module level
    # This prevents FastAPI router creation during module import in embedded mode
    logger.info("[LINK] Including FastAPI routers for REST endpoints")
    
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
        dependencies=[Depends(check_active_genome)],
        responses=standard_response
    )
    
    # Add the visualization router
    app.include_router(
        visualization_router,
        prefix="/v1/visualization",
        tags=["VISUALIZATION"],
        dependencies=[],
        responses=standard_response
    )
    
    # Add the debug control router
    from feagi.api.rest.v1.debug import router as debug_router
    app.include_router(
        debug_router,
        prefix="/v1",
        tags=["DEBUG"],
        dependencies=[],
        responses=standard_response
    )
    
    logger.info("[OK] All FastAPI routers included successfully")
    
    # Set state in FeagiStateManager
    from feagi.core.state_manager import FeagiStateManager, ServiceState
    state_manager = FeagiStateManager.instance()
    state_manager.set_api_state(ServiceState.READY)
    
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

