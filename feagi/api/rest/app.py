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
#  ==============================================================================

import os
import random
import string
import time
import traceback

from feagi.utils.logger import setup_logger
import threading

logger = setup_logger(name="feagi.api.rest.app")
logger.info("...")
import json
from pathlib import Path
from typing import Any, Dict, Tuple

from fastapi import Depends, FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    RedirectResponse,
    Response,
)
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException

#  Remove the old router imports - no longer needed since we use universal
#  wrapper directly
from feagi.api.dependencies import *
from feagi.api.models import *

#  Import the universal FastAPI wrapper directly instead of individual router
#  files
from feagi.api.transport.universal_fastapi import (
    get_burst_engine_router,
    get_connectome_router,
    get_cortical_area_router,
    get_cortical_mapping_router,
    get_evolution_router,
    get_feagi_agent_router,
    get_genome_router,
    get_inputs_router,
    get_insights_router,
    get_monitoring_router,
    get_morphology_router,
    get_network_router,
    get_neuroplasticity_router,
    get_outputs_router,
    get_region_router,
    get_simulation_router,
    get_system_router,
    get_training_router,
)

# Import the visualization router
from feagi.api.v1.visualization import router as visualization_router
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager, ServiceState

from .commons import (
    CustomError,
    check_active_genome,
    check_brain_running,
    check_burst_engine,
    check_burst_engine_or_allow_genome_ops,
)
from .config import settings
from .response_utils import error_response, success_response

#  Note: v2 routers have been removed since we now use the universal wrapper
#  directly for all routes
#  The v2 functionality can be added in the future if needed via the universal
#  wrapper pattern


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
    openapi_url="/openapi.json",
)

# Get the directory of the current file
current_dir = Path(__file__).parent
static_dir = current_dir / "static"

# Mount static files directory
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


# Custom Swagger UI with dark theme
def custom_swagger_ui_html():
    """Custom Swagger UI with Windows compatibility and better error handling.

    This fixes the white screen issue that can occur on Windows systems.
    """
    #  Read the custom HTML template directly with Windows-compatible path
    #  handling
    template_path = Path(__file__).parent / "static" / "custom-swagger-ui.html"
    logger.info(
        f"Loading custom Swagger UI template from {template_path}",
        status="[CONFIG]",
    )

    try:
        # Use pathlib and explicit encoding for Windows compatibility
        with open(template_path, "r", encoding="utf-8") as f:
            html_content = f.read()

        # Replace the OpenAPI URL placeholder with the actual URL
        # Ensure the URL is properly formatted for all platforms
        openapi_url = str(app.openapi_url)
        if not openapi_url.startswith("/"):
            openapi_url = "/" + openapi_url

        html_content = html_content.replace("{{ openapi_url }}", openapi_url)

        # Add Windows-specific error handling and debugging
        debug_script = (
            """
        <script>
        // Windows compatibility debugging
        console.log('FEAGI Swagger UI Debug: Starting initialization...');
        console.log('OpenAPI URL:', '"""
            + openapi_url
            + """');

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
                '<li>Try accessing <a href="' + '"""
            + openapi_url
            + """' + '" target="_blank">OpenAPI spec directly</a></li>' +
                '<li>Refresh the page or try a different browser</li>' +
                '<li>Contact support if the issue persists</li>' +
                '</ul>' +
                '</div>';
        });
        </script>
        """
        )

        # Insert debug script before the closing body tag
        html_content = html_content.replace(
            "</body>", debug_script + "</body>"
        )

        logger.info(
            "Custom Swagger UI template loaded successfully", status="[OK]"
        )
        return HTMLResponse(content=html_content)

    except FileNotFoundError:
        logger.error(
            f"Custom Swagger UI template not found at {template_path}",
            status="[ERR]",
        )
        return _fallback_swagger_ui()
    except UnicodeDecodeError as e:
        logger.error(
            f"Encoding error reading Swagger UI template: {e}", status="[ERR]"
        )
        return _fallback_swagger_ui()
    except Exception as e:
        logger.error(
            f"Failed to load custom Swagger UI template: {e}", status="[ERR]"
        )
        return _fallback_swagger_ui()


def _fallback_swagger_ui():
    """Fallback to default FastAPI Swagger UI if custom template fails."""
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
    """Automatically redirect root URL to API documentation.

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


# --- Lightweight rate/concurrency tracker (always on) ---
_RATE_WINDOW_SEC: float = 1.0
_RATE_WARN_THRESHOLD: int = 20
_RATE_SUPPRESS_SEC: float = 10.0
_rate_track: Dict[Tuple[str, str], Dict[str, float]] = {}
_INFLIGHT_WARN_THRESHOLD: int = 25
_inflight_by_ip: Dict[str, int] = {}
_inflight_lock = threading.Lock()
_fd_hot_until: float = 0.0


def _fd_monitor_thread() -> None:
    """Background FD usage monitor; logs WARN when nearing soft limit."""
    try:
        import resource as _resource  # type: ignore
        soft, hard = _resource.getrlimit(_resource.RLIMIT_NOFILE)
    except Exception:
        soft, hard = (256, 256)
    warn_ratio = 0.8
    last_detail = 0.0
    while True:
        try:
            import os as _os
            fd_count = 0
            try:
                # macOS/Linux
                fd_count = len(_os.listdir("/dev/fd"))
            except Exception:
                fd_count = 0
            if soft and fd_count > int(soft * warn_ratio):
                logger.warning(
                    f"[FD] Open file descriptors near limit: {fd_count}/{soft} (hard {hard})."
                )
                # Detailed breakdown no more than every 5 seconds
                now = time.time()
                if now - last_detail > 5.0:
                    last_detail = now
                    sockets = 0
                    pipes = 0
                    files = 0
                    shm = 0
                    path_counts: Dict[str, int] = {}
                    try:
                        for name in _os.listdir("/dev/fd"):
                            p = f"/dev/fd/{name}"
                            try:
                                target = _os.readlink(p)
                            except Exception:
                                continue
                            t = str(target)
                            if "socket:" in t:
                                sockets += 1
                            elif "pipe:" in t:
                                pipes += 1
                            else:
                                files += 1
                                if "feagi-shm" in t or "feagi-shared-mem" in t:
                                    shm += 1
                                # Tally top paths (trim long)
                                key = t
                                if len(key) > 80:
                                    key = key[:77] + "..."
                                path_counts[key] = path_counts.get(key, 0) + 1
                    except Exception:
                        pass
                    logger.warning(
                        f"[FD-DETAIL] sockets={sockets}, pipes={pipes}, files={files}, shm_like={shm}"
                    )
                    # Enable hot request sampling for next 5 seconds
                    try:
                        globals()["_fd_hot_until"] = now + 5.0
                    except Exception:
                        pass
                    # Log top file targets
                    try:
                        top = sorted(path_counts.items(), key=lambda kv: kv[1], reverse=True)[:5]
                        if top:
                            for k, v in top:
                                logger.warning(f"[FD-DETAIL] {v} × {k}")
                    except Exception:
                        pass
        except Exception:
            pass
        time.sleep(2.0)


@app.middleware("http")
async def log_requests(request: Request, call_next):
    """Enhanced API debug logging middleware for comprehensive request/response
    tracking.

    When --debug-api is enabled, this logs:
    - Complete request details (method, URL, headers, query params, body)
    - Response details (status, headers, body)
    - Timing information
    - Request/response correlation via unique ID

    Enhanced to capture request body and response details for comprehensive debugging.
    """
    # === ALWAYS-ON: request-rate tracking to detect abusive callers ===
    try:
        client_ip = request.client.host if request.client else "unknown"
        path = request.url.path
        now = time.time()
        key = (client_ip, path)
        st = _rate_track.get(key)
        if not st or (now - st.get("window_start", 0.0)) > _RATE_WINDOW_SEC:
            _rate_track[key] = {"window_start": now, "count": 1.0, "last_warn": st.get("last_warn", 0.0) if st else 0.0}
        else:
            st["count"] = st.get("count", 0.0) + 1.0
            # Warn when rate crosses threshold, suppress repeated warnings briefly
            if st["count"] >= _RATE_WARN_THRESHOLD and (now - st.get("last_warn", 0.0)) > _RATE_SUPPRESS_SEC:
                ua = request.headers.get("user-agent", "<none>")
                logger.warning(
                    f"[API-RATE] High call rate: {client_ip} → {path}: {int(st['count'])} req in {_RATE_WINDOW_SEC:.1f}s; UA={ua}"
                )
                st["last_warn"] = now
    except Exception:
        # Never fail request due to diagnostics
        pass

    # Track in-flight concurrently per client
    client_ip = request.client.host if request.client else "unknown"
    try:
        with _inflight_lock:
            _inflight_by_ip[client_ip] = _inflight_by_ip.get(client_ip, 0) + 1
            if _inflight_by_ip[client_ip] >= _INFLIGHT_WARN_THRESHOLD:
                ua = request.headers.get("user-agent", "<none>")
                logger.warning(
                    f"[API-CONC] High concurrent requests from {client_ip}: {_inflight_by_ip[client_ip]} in-flight; UA={ua}"
                )
    except Exception:
        pass

    # If FD pressure is hot, sample request sources for quick identification
    try:
        import time as _t
        hot_until = globals().get('_fd_hot_until', 0.0)
        if _t.time() < hot_until:
            ua = request.headers.get("user-agent", "<none>")
            logger.warning(f"[FD-HOT] {client_ip} → {path} UA={ua}")
    except Exception:
        pass

    # Check if debug API logging is enabled - try multiple methods to detect it
    state_manager = FeagiStateManager.instance()
    debug_api_enabled = False

    # Method 1: Check state manager
    try:
        debug_api_enabled = state_manager.is_debug_api_enabled()
    except Exception:
        pass

    # Method 2: Check environment variable as fallback
    if not debug_api_enabled:
        import os

        debug_api_enabled = os.environ.get("FEAGI_DEBUG_API", "0") == "1"

    # Method 3: Check for debug flag in command line (ultimate fallback)
    if not debug_api_enabled:
        import sys

        debug_api_enabled = "--debug-api" in sys.argv

    # Minimize diagnostics unless API debug is enabled
    if debug_api_enabled and not hasattr(log_requests, "_diagnostic_shown"):
        try:
            logger.debug("[API-DEBUG] Diagnostic status check initialized")
        except Exception:
            pass
        log_requests._diagnostic_shown = True

    if not debug_api_enabled:
        # If debug is not enabled, still ensure inflight tracking is decremented
        try:
            response = await call_next(request)
            return response
        finally:
            try:
                with _inflight_lock:
                    if client_ip in _inflight_by_ip:
                        _inflight_by_ip[client_ip] = max(0, _inflight_by_ip[client_ip] - 1)
            except Exception:
                pass

    # Show debug enabled message on first request (if not already shown)
    if not hasattr(log_requests, "_debug_shown"):
        logger.info(
            "🐛 API Debug logging is ENABLED - detailed request/response logging active"
        )
        log_requests._debug_shown = True

    # Generate unique request ID for tracking
    idem = "".join(random.choices(string.ascii_uppercase + string.digits, k=8))

    # Start timing
    start_time = time.time()

    # ===== ENHANCED REQUEST LOGGING =====
    logger.info("🔵 [API-DEBUG] ═══════════════════════════════════════")
    logger.info(f"🔵 [API-DEBUG] REQUEST START [ID: {idem}]")
    logger.info("🔵 [API-DEBUG] ═══════════════════════════════════════")

    # Basic request info
    logger.info(f"🔵 [API-DEBUG] Method: {request.method}")
    logger.info(f"🔵 [API-DEBUG] URL: {str(request.url)}")
    logger.info(f"🔵 [API-DEBUG] Path: {request.url.path}")
    logger.info(
        f"🔵 [API-DEBUG] Client: {request.client.host if request.client else 'unknown'}:{request.client.port if request.client else 'unknown'}"
    )

    # Headers (formatted nicely)
    logger.info("🔵 [API-DEBUG] Headers:")
    for name, value in request.headers.items():
        # Mask sensitive headers
        if name.lower() in ["authorization", "cookie", "x-api-key"]:
            value = "***MASKED***"
        logger.info(f"🔵 [API-DEBUG]   {name}: {value}")

    # Query parameters
    if request.query_params:
        logger.info("🔵 [API-DEBUG] Query Parameters:")
        for name, value in request.query_params.items():
            logger.info(f"🔵 [API-DEBUG]   {name}: {value}")
    else:
        logger.info("🔵 [API-DEBUG] Query Parameters: <none>")

    # Path parameters
    if hasattr(request, "path_params") and request.path_params:
        logger.info("🔵 [API-DEBUG] Path Parameters:")
        for name, value in request.path_params.items():
            logger.info(f"🔵 [API-DEBUG]   {name}: {value}")
    else:
        logger.info("🔵 [API-DEBUG] Path Parameters: <none>")

    # Capture and log request body
    request_body = None
    body_bytes = b""
    try:
        body_bytes = await request.body()
        if body_bytes:
            request_body = body_bytes.decode("utf-8")
            # Pretty print JSON if possible
            try:
                import json

                parsed_json = json.loads(request_body)
                formatted_json = json.dumps(parsed_json, indent=2)
                logger.info("🔵 [API-DEBUG] Request Body (JSON):")
                for line in formatted_json.split("\n"):
                    logger.info(f"🔵 [API-DEBUG]   {line}")
            except (json.JSONDecodeError, ValueError):
                # Not JSON, log as plain text (truncate if too long)
                if len(request_body) > 2000:
                    truncated_body = request_body[:2000] + "... (truncated)"
                    logger.info(
                        f"🔵 [API-DEBUG] Request Body (truncated): {truncated_body}"
                    )
                else:
                    logger.info(f"🔵 [API-DEBUG] Request Body: {request_body}")
        else:
            logger.info("🔵 [API-DEBUG] Request Body: <empty>")
    except Exception as e:
        logger.warning(f"🔵 [API-DEBUG] Failed to read request body: {e}")

    #  Store original body for downstream handlers (since we consumed the
    #  stream)
    async def receive():
        return {
            "type": "http.request",
            "body": body_bytes,
        }

    # Patch the request's receive method
    request._receive = receive

    # Process the request
    try:
        response = await call_next(request)
        process_time = (time.time() - start_time) * 1000

        # ===== ENHANCED RESPONSE LOGGING =====
        logger.info("🟢 [API-DEBUG] ═══════════════════════════════════════")
        logger.info(f"🟢 [API-DEBUG] RESPONSE [ID: {idem}]")
        logger.info("🟢 [API-DEBUG] ═══════════════════════════════════════")

        # Response status and timing
        status_emoji = (
            "✅"
            if 200 <= response.status_code < 300
            else "❌" if response.status_code >= 400 else "⚠️"
        )
        logger.info(
            f"🟢 [API-DEBUG] Status: {response.status_code} {status_emoji}"
        )
        logger.info(f"🟢 [API-DEBUG] Duration: {process_time:.2f}ms")

        # Response headers
        logger.info("🟢 [API-DEBUG] Response Headers:")
        for name, value in response.headers.items():
            logger.info(f"🟢 [API-DEBUG]   {name}: {value}")

        # Try to capture and log response body (route-scoped capture for JSON)
        try:
            should_capture = request.url.path in (
                "/v1/cortical_mapping/mapping_properties",
                "/v1/cortical_mapping/mapping",
            )
            content_type = response.headers.get("content-type", "")
            if should_capture and content_type.startswith("application/json"):
                # Safely materialize the response body for both standard and streaming responses
                body_bytes = b""
                if hasattr(response, "body_iterator") and response.body_iterator is not None:
                    try:
                        async for chunk in response.body_iterator:
                            if chunk:
                                body_bytes += chunk
                    except Exception:
                        body_bytes = b""
                else:
                    # Starlette Response exposes raw body as bytes
                    try:
                        body_bytes = response.body  # type: ignore[attr-defined]
                    except Exception:
                        body_bytes = b""

                if body_bytes:
                    try:
                        parsed = json.loads(body_bytes)
                        pretty = json.dumps(parsed, indent=2)
                        logger.info("🟢 [API-DEBUG] Response Body (JSON):")
                        for line in pretty.split("\n"):
                            logger.info(f"🟢 [API-DEBUG]   {line}")
                    except Exception:
                        # Not JSON or parse failed, log as text (truncate)
                        text = body_bytes.decode("utf-8", errors="replace")
                        if len(text) > 2000:
                            text = text[:2000] + "... (truncated)"
                        logger.info(f"🟢 [API-DEBUG] Response Body: {text}")

                    # Rebuild response so downstream can still read the body
                    response = Response(
                        content=body_bytes,
                        status_code=response.status_code,
                        headers=dict(response.headers),
                        media_type=content_type,
                    )
                else:
                    logger.info("🟢 [API-DEBUG] Response Body: <empty>")
            else:
                logger.info("🟢 [API-DEBUG] Response Body: <streaming/uncaptured>")
        except Exception as e:
            logger.info(
                f"🟢 [API-DEBUG] Response Body: <could not capture: {e}>"
            )

        # Summary line
        logger.info("🟢 [API-DEBUG] ═══════════════════════════════════════")
        logger.info(
            f"🟢 [API-DEBUG] COMPLETED [ID: {idem}] {request.method} {request.url.path} → {response.status_code} ({process_time:.2f}ms)"
        )
        logger.info("🟢 [API-DEBUG] ═══════════════════════════════════════")

        return response

    except Exception as e:
        process_time = (time.time() - start_time) * 1000

        # ===== ERROR RESPONSE LOGGING =====
        logger.error("🔴 [API-DEBUG] ═══════════════════════════════════════")
        logger.error(f"🔴 [API-DEBUG] ERROR [ID: {idem}]")
        logger.error("🔴 [API-DEBUG] ═══════════════════════════════════════")
        logger.error(f"🔴 [API-DEBUG] Exception: {type(e).__name__}: {str(e)}")
        logger.error(f"🔴 [API-DEBUG] Duration: {process_time:.2f}ms")
        logger.error("🔴 [API-DEBUG] ═══════════════════════════════════════")

        # Re-raise the exception
        raise
    finally:
        # Decrement inflight counter
        try:
            with _inflight_lock:
                if client_ip in _inflight_by_ip:
                    _inflight_by_ip[client_ip] = max(0, _inflight_by_ip[client_ip] - 1)
        except Exception:
            pass


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
                "message": f"An error occurred: {str(e)}",
            },
        )


# Standard response format for all endpoints
standard_response = {
    200: {"description": "Success"},
    400: {"description": "Bad Request"},
    404: {"description": "Not Found"},
    500: {"description": "Internal Server Error"},
}

# todo: To add the ability of updating allowable cors list on the fly
# # Append to the CORS origin
# @app.middleware("http")
# async def update_cors_origin(request, call_next):
#     response = await call_next(request)
#     origin = response.headers.get("Access-Control-Allow-Origin", "")
#     new_origin = ""
#  response.headers["Access-Control-Allow-Origin"] = f"{origin},{new_origin}"
#     return response


@app.on_event("startup")
async def set_api_state_ready():
    state = FeagiStateManager.instance()
    state.set_api_state(ServiceState.READY)
    # Start FD monitor thread
    try:
        t = threading.Thread(target=_fd_monitor_thread, daemon=True)
        t.start()
    except Exception:
        pass


def create_rest_app_direct(config: Dict[str, Any]):
    """RUST/RTOS COMPATIBLE: Factory function for REST app with direct
    dependency injection.

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
    logger.info(
        "[LINK] Creating REST app with direct dependency injection (Rust/RTOS compatible)",
        status="[LINK]",
    )

    #  RUST/RTOS COMPATIBLE: Direct dependency injection instead of environment
    #  lookup
    core_api_service = config["core_api"]
    state_manager = config["state_manager"]
    connectome_manager = config["connectome_manager"]

    if not core_api_service:
        raise RuntimeError(
            "CoreAPIService is required for direct REST app creation"
        )

    logger.info(
        "[OK] Using directly injected CoreAPIService and ConnectomeManager",
        status="[OK]",
    )
    logger.info(
        "[TARGET] All dependencies injected directly - no environment variables needed",
        status="[TARGET]",
    )

    # Set the connectome instance for FastAPI dependency injection
    from feagi.api.rest.dependencies import set_connectome_instance

    set_connectome_instance(connectome_manager)

    # Set core API service for dependency injection
    from feagi.api.rest.dependencies import set_core_api_service_instance

    set_core_api_service_instance(core_api_service)

    #  For now, return the existing app instance (already configured with all
    #  routes)
    # In future iterations, we can create a fresh app instance here
    global app

    # Set state in FeagiStateManager
    state_manager.set_api_state(ServiceState.READY)
    logger.info("REST API state changed: UNAVAILABLE → READY", status="[FAST]")

    return app


def create_rest_app(connectome: ConnectomeManager = None):
    """Factory function to return the FastAPI app instance, with connectome
    dependency injection."""

    #  CRITICAL FIX: Ensure true singleton pattern for mission-critical
    #  reliability
    core_api_service = None

    # Check if we're running as part of the main FEAGI process (singleton mode)
    if os.environ.get("FEAGI_INITIALIZED") == "1":
        logger.info(
            "[LINK] Running in FEAGI subprocess, using singleton ConnectomeManager",
            status="[LINK]",
        )

        #  CRITICAL FIX: In subprocess mode, we can't access parent's
        #  ProcessManager
        # Instead, use the singleton ConnectomeManager directly
        from feagi.bdu.connectome_manager import ConnectomeManager

        connectome = ConnectomeManager.instance()
        logger.info(
            "[TARGET] Created singleton ConnectomeManager instance",
            status="[TARGET]",
        )

        # Create CoreAPIService with singleton instances
        from feagi.core import create_core_api
        from feagi.core.state_manager import FeagiStateManager

        state_manager = FeagiStateManager.instance()
        core_api_service = create_core_api(connectome, {})

        if core_api_service:
            logger.info(
                "[OK] Successfully created CoreAPIService with singleton ConnectomeManager",
                status="[OK]",
            )
        else:
            logger.error(
                "[ERR] Failed to create CoreAPIService", status="[ERR]"
            )
            raise RuntimeError("Failed to create CoreAPIService")

        logger.info(
            "[TARGET] FastAPI app configured with subprocess singleton services",
            status="[TARGET]",
        )

    else:
        # Standalone mode (development/testing)
        logger.info(
            "Running in standalone mode, using singleton ConnectomeManager",
            status="[CONFIG]",
        )

        if connectome is None:
            from feagi.bdu.connectome_manager import ConnectomeManager

            connectome = (
                ConnectomeManager.instance()
            )  # Use singleton pattern instead of direct instantiation

        # Create a basic CoreAPIService for standalone operation
        from feagi.core import create_core_api

        core_api_service = create_core_api(connectome, {})

        if not core_api_service:
            logger.error(
                "[ERR] Failed to create CoreAPIService in standalone mode",
                status="[ERR]",
            )
            raise RuntimeError(
                "Failed to create CoreAPIService in standalone mode"
            )

    # Set the connectome instance for FastAPI dependency injection
    from feagi.api.rest.dependencies import set_connectome_instance

    set_connectome_instance(connectome)

    # Set core API service for dependency injection
    from feagi.api.rest.dependencies import set_core_api_service_instance

    set_core_api_service_instance(core_api_service)

    # Get the global app instance
    global app

    # CRITICAL: Include all routers here instead of at module level
    #  This prevents FastAPI router creation during module import in embedded
    #  mode
    logger.info("[LINK] Including FastAPI routers for REST endpoints")

    app.include_router(
        get_genome_router(),
        prefix="/v1/genome",
        tags=["GENOME"],
        dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
        responses=standard_response,
    )

    app.include_router(
        get_connectome_router(),
        prefix="/v1/connectome",
        tags=["CONNECTOME"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response,
    )

    app.include_router(
        get_burst_engine_router(),
        prefix="/v1/burst_engine",
        tags=["BURST ENGINE"],
        dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
        responses=standard_response,
    )

    # Physiology router (genome parameters)
    from feagi.api.transport.universal_fastapi import UniversalFastAPIWrapper

    physiology_router = UniversalFastAPIWrapper().create_router_for_module(
        "physiology"
    )
    app.include_router(
        physiology_router,
        prefix="/v1/physiology",
        tags=["PHYSIOLOGY"],
        dependencies=[Depends(check_burst_engine_or_allow_genome_ops)],
        responses=standard_response,
    )

    app.include_router(
        get_evolution_router(),
        prefix="/v1/evolution",
        tags=["EVOLUTIONARY"],
        dependencies=[Depends(check_burst_engine)],
        responses=standard_response,
    )

    app.include_router(
        get_feagi_agent_router(),
        prefix="/v1/agent",
        tags=["FEAGI AGENT"],
        dependencies=[Depends(check_burst_engine)],
        responses=standard_response,
    )

    app.include_router(
        get_insights_router(),
        prefix="/v1/insight",
        tags=["INSIGHTS"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response,
    )

    app.include_router(
        get_morphology_router(),
        prefix="/v1/morphology",
        tags=["NEURON MORPHOLOGIES"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_cortical_area_router(),
        prefix="/v1/cortical_area",
        tags=["CORTICAL AREAS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_region_router(),
        prefix="/v1/region",
        tags=["BRAIN REGIONS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_cortical_mapping_router(),
        prefix="/v1/cortical_mapping",
        tags=["CORTICAL MAPPINGS"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_neuroplasticity_router(),
        prefix="/v1/neuroplasticity",
        tags=["NEUROPLASTICITY"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_inputs_router(),
        prefix="/v1/input",
        tags=["INPUT MANAGEMENT"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    app.include_router(
        get_network_router(),
        prefix="/v1/network",
        tags=["NETWORK"],
        dependencies=[],
        responses=standard_response,
    )

    app.include_router(
        get_simulation_router(),
        prefix="/v1/simulation",
        tags=["SIMULATION"],
        dependencies=[Depends(check_brain_running)],
        responses=standard_response,
    )

    app.include_router(
        get_system_router(),
        prefix="/v1/system",
        tags=["SYSTEM"],
        dependencies=[],
        responses=standard_response,
    )

    app.include_router(
        get_training_router(),
        prefix="/v1/training",
        tags=["TRAINING"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    # Add the missing outputs router
    app.include_router(
        get_outputs_router(),
        prefix="/v1/output",
        tags=["OUTPUT MANAGEMENT"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    # Add the missing monitoring router
    app.include_router(
        get_monitoring_router(),
        prefix="/v1/monitoring",
        tags=["MONITORING"],
        dependencies=[Depends(check_active_genome)],
        responses=standard_response,
    )

    # Add the snapshot router
    from feagi.api.transport.universal_fastapi import get_snapshot_router

    app.include_router(
        get_snapshot_router(),
        prefix="/v1/snapshots",
        tags=["SNAPSHOTS"],
        dependencies=[],
        responses=standard_response,
    )

    # Add the visualization router
    app.include_router(
        visualization_router,
        prefix="/v1/visualization",
        tags=["VISUALIZATION"],
        dependencies=[],
        responses=standard_response,
    )

    # Add the debug control router
    from feagi.api.rest.v1.debug import router as debug_router

    app.include_router(
        debug_router,
        prefix="/v1",
        tags=["DEBUG"],
        dependencies=[],
        responses=standard_response,
    )

    logger.info("[OK] All FastAPI routers included successfully")

    # Set state in FeagiStateManager
    from feagi.core.state_manager import FeagiStateManager, ServiceState

    state_manager = FeagiStateManager.instance()
    state_manager.set_api_state(ServiceState.READY)

    # Log debug information about the created app
    logger.info("[APP-CREATION] FastAPI app created successfully")
    logger.debug(
        f"[APP-CREATION] Debug API enabled: {state_manager.is_debug_api_enabled()}"
    )
    logger.debug(
        f"[APP-CREATION] App middleware count: {len(app.user_middleware)}"
    )
    logger.debug(
        f"[APP-CREATION] Middleware types: {[str(type(m)) for m in app.user_middleware]}"
    )

    return app


def get_core_api():
    """Dependency placeholder for the core API service.

    Should be overridden in tests.
    """
    raise NotImplementedError(
        "get_core_api must be overridden in tests with a mock implementation."
    )


@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    return JSONResponse(
        status_code=exc.status_code,
        content=error_response(
            message=exc.detail, error_code=f"HTTP_{exc.status_code}"
        ),
    )


@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    return JSONResponse(
        status_code=422,
        content=error_response(
            message="Validation error",
            error_code="VALIDATION_ERROR",
            metadata={"errors": exc.errors()},
        ),
    )


@app.exception_handler(Exception)
async def generic_exception_handler(request, exc):
    # Log the exception here
    return JSONResponse(
        status_code=500,
        content=error_response(message=str(exc), error_code="INTERNAL_ERROR"),
    )


@app.middleware("http")
async def standardize_response_format(request, call_next):
    """Middleware that standardizes API responses.

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
            return JSONResponse(
                content=content, status_code=response.status_code
            )
    except Exception:
        # If we can't parse JSON or other issues, just return original response
        return response

    # Skip standardization for v1 routes to maintain backward compatibility
    if "/v1/" in request.url.path:
        # Need to rebuild the response since we've consumed the body
        return Response(
            content=body,
            status_code=response.status_code,
            headers=dict(response.headers),
            media_type=response.media_type,
        )

    # For v2+ routes, apply standardization for success responses
    if response.status_code < 400:
        try:
            content = json.loads(body)
            return JSONResponse(
                content=success_response(data=content),
                status_code=response.status_code,
            )
        except Exception:
            # If we can't standardize, return original response rebuilt
            return Response(
                content=body,
                status_code=response.status_code,
                headers=dict(response.headers),
                media_type=response.media_type,
            )

    # For any other case, rebuild the original response
    return Response(
        content=body,
        status_code=response.status_code,
        headers=dict(response.headers),
        media_type=response.media_type,
    )
