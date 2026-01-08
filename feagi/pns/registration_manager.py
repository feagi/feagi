"""
Registration Manager - Rust-backed with Python orchestration

Agent storage: Rust PyAgentRegistry (single source of truth)
Orchestration: Python (FQ samplers, state, capability rates)

No duplication, no fallbacks, no dead code.
"""

import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional, Set

# Initialize logger first
logger = logging.getLogger(__name__)

# Make Rust registry optional - HTTP API registration doesn't require it
try:
    from feagi_rust import PyAgentRegistry
    RUST_REGISTRY_AVAILABLE = True
except ImportError:
    PyAgentRegistry = None
    RUST_REGISTRY_AVAILABLE = False
    logger.warning(
        "⚠️ feagi_rust module not available. HTTP API registration will work, "
        "but local Rust registry operations will be disabled."
    )

load_feagi_config = None
get_agent_config = None


class AgentRegistrationRequest:
    """Agent registration request"""

    def __init__(
        self,
        agent_id: str,
        agent_type: str,
        agent_ip: str,
        capabilities: Dict[str, Any],
        agent_data_port: Optional[int] = None,
        agent_version: Optional[str] = None,
        controller_version: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.agent_ip = agent_ip
        self.capabilities = capabilities
        self.agent_data_port = agent_data_port
        self.agent_version = agent_version
        self.controller_version = controller_version
        self.metadata = metadata or {}
        self.registration_timestamp = datetime.now(timezone.utc).isoformat()


class AgentRegistrationResponse:
    """Agent registration response"""

    def __init__(
        self,
        success: bool,
        message: str,
        agent_id: str,
        cortical_areas: Dict[str, Any],  # Required in FEAGI 2.0 - must come before optional params
        fq_samplers_enabled: Optional[Dict[str, bool]] = None,
        error_code: Optional[str] = None,
        transport_info: Optional[Dict[str, Any]] = None,
    ):
        self.success = success
        self.message = message
        self.agent_id = agent_id
        self.cortical_areas = cortical_areas  # Cortical area availability status (required)
        self.fq_samplers_enabled = fq_samplers_enabled or {}
        self.transport_info = transport_info or {}
        self.error_code = error_code


class RegistrationManager:
    """
    🦀 Rust-backed Registration Manager
    
    Storage: PyAgentRegistry (Rust) - THE ONLY agent storage
    Orchestration: Python (FQ samplers, state, capability rates)
    """

    def __init__(self, state_manager=None, process_manager=None):
        """Initialize Rust-backed Registration Manager"""
        self._lock = threading.RLock()
        self._state_manager = state_manager
        self._process_manager = process_manager

        # 🦀 Rust agent registry - Lazy loaded to handle startup order
        # PNS might not be initialized yet when this runs, so we load on first access
        self._rust_registry = None
        self._registry_initialized = False
        self._shutting_down = False  # Flag to prevent lazy-load during shutdown
        
        logger.info("🦀 Registration Manager initialized (registry will lazy-load from PNS)")

        # Capability tracking for FQ sampler decisions (NOT agent storage!)
        self._capability_counts = {
            "visualization": 0,
            "motor": 0,
            "sensory": 0,
            "video": 0,
            "feagi": 0,
        }

        # FQ sampler states
        self._fq_sampler_states = {
            "visualization_enabled": False,
            "motor_enabled": False,
        }

        # Event listeners
        self._state_change_listeners: Set[Callable] = set()

        # FEAGI readiness
        self._feagi_ready = True
        
        # Heartbeat management (keeps agents registered)
        self._heartbeat_threads: Dict[str, threading.Thread] = {}
        self._heartbeat_running: Dict[str, bool] = {}
        # Safety mode: no implicit defaults. Call configure_heartbeat(...) explicitly.
        self._heartbeat_interval: Optional[float] = None
        self._heartbeat_join_timeout_s: Optional[float] = None
        self._feagi_api_urls: Dict[str, str] = {}  # Store API URL per agent
        self._agent_metadata: Dict[str, Dict[str, Any]] = {}  # Store metadata per agent (timeouts, endpoints, etc.)

        logger.info("🦀 Rust-backed Registration Manager fully initialized")

    def shutdown(self):
        """Signal shutdown to prevent lazy-load deadlocks."""
        self._shutting_down = True
        # Stop all heartbeat threads
        for agent_id in list(self._heartbeat_running.keys()):
            self.stop_heartbeat(agent_id)

    def configure_heartbeat(self, *, interval_s: float, join_timeout_s: float) -> None:
        """Configure heartbeat timings.

        Safety mode: callers must explicitly set these values (no defaults).
        """
        if interval_s <= 0:
            raise ValueError("interval_s must be > 0 (no defaults in safety mode).")
        if join_timeout_s <= 0:
            raise ValueError("join_timeout_s must be > 0 (no defaults in safety mode).")
        self._heartbeat_interval = interval_s
        self._heartbeat_join_timeout_s = join_timeout_s

    def _flatten_rust_capabilities(self, rust_caps: Dict[str, Any]) -> Dict[str, Any]:
        """Flatten Rust AgentCapabilities struct to legacy dict format.
        
        Extracts visualization, motor, sensory, vision from struct fields
        and merges with custom capabilities.
        """
        flattened = {}
        
        if "visualization" in rust_caps and rust_caps["visualization"]:
            flattened["visualization"] = rust_caps["visualization"]
        if "motor" in rust_caps and rust_caps["motor"]:
            flattened["motor"] = rust_caps["motor"]
        if "sensory" in rust_caps and rust_caps["sensory"]:
            flattened["sensory"] = rust_caps["sensory"]
        if "vision" in rust_caps and rust_caps["vision"]:
            flattened["vision"] = rust_caps["vision"]
        
        # Merge custom capabilities
        if "custom" in rust_caps and rust_caps["custom"]:
            flattened.update(rust_caps["custom"])
        
        return flattened
    
    def _get_registry(self):
        """Lazy load registry from PNS (handles startup order issues)
        
        Returns None if Rust registry is not available (HTTP-only mode).
        HTTP API registration works without local Rust registry.
        """
        # If Rust registry is not available, return None (HTTP-only mode)
        if not RUST_REGISTRY_AVAILABLE:
            return None
            
        # Don't attempt lazy-load during shutdown to avoid deadlocks
        if self._shutting_down and not self._registry_initialized:
            return None
            
        if not self._registry_initialized:
            import time
            start = time.time()
            logger.debug("⏱️  [TIMING] Registry lazy-load started")
            with self._lock:
                if not self._registry_initialized:  # Double-check locking
                    try:
                        if (
                            self._process_manager
                            and hasattr(self._process_manager, "_pns")
                            and self._process_manager._pns
                        ):
                            # Use shared registry from PNS (production path)
                            t1 = time.time()
                            self._rust_registry = self._process_manager._pns.get_shared_registry()
                            t2 = time.time()
                            logger.info(f"🦀 ✓ Using shared agent registry from Rust PNS (single source of truth) - took {(t2-t1)*1000:.1f}ms")
                        else:
                            # Safety mode: no implicit fallbacks.
                            self._rust_registry = None
                        self._registry_initialized = True
                    except Exception as e:
                        logger.error(f"❌ Failed to initialize Rust registry: {e}")
                        raise
            elapsed = time.time() - start
            logger.debug(f"⏱️  [TIMING] Registry lazy-load completed in {elapsed*1000:.1f}ms")
        return self._rust_registry

    def register_agent(
        self, request: AgentRegistrationRequest
    ) -> AgentRegistrationResponse:
        """Register agent using Rust registry"""
        with self._lock:
            logger.info(
                f"🤖 REGISTRATION REQUEST: {request.agent_id} (type: {request.agent_type})"
            )

            try:
                # 1. Validate
                validation_result = self._validate_agent_request(request)
                if not validation_result.get("valid", False):
                    return AgentRegistrationResponse(
                        success=False,
                        message=validation_result.get("error", "Validation failed"),
                        agent_id=request.agent_id,
                        cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                        error_code="VALIDATION_ERROR",
                    )

                # 2. Check FEAGI readiness
                if not self._check_feagi_readiness():
                    return AgentRegistrationResponse(
                        success=False,
                        message="FEAGI system not ready",
                        agent_id=request.agent_id,
                        cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                        error_code="FEAGI_NOT_READY",
                    )

                # 3. Check if re-registration (skip if Rust registry not available)
                registry = self._get_registry()
                if registry:
                    try:
                        existing_agent_json = registry.get_agent_json(request.agent_id)
                        existing_agent = json.loads(existing_agent_json)
                        logger.warning(f"⚠️ Agent '{request.agent_id}' re-registering")
                        
                        # Subtract old capability counts (flatten Rust struct to get all caps)
                        old_rust_caps = existing_agent.get("capabilities", {})
                        old_caps = self._flatten_rust_capabilities(old_rust_caps)
                        self._update_capability_counts(old_caps, increment=False)
                    except Exception:
                        logger.info(f"✅ New agent '{request.agent_id}' registering")
                else:
                    logger.info(f"✅ New agent '{request.agent_id}' registering (HTTP-only mode)")

                # 4. Extract capability rates BEFORE sanitization
                capability_rates_to_register = []
                try:
                    from feagi.core.capability_rate_manager import get_capability_rate_manager
                    from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec
                    
                    for cap_name, cap_config in request.capabilities.items():
                        cap_type = None
                        requested_rate = 10.0
                        
                        if cap_name.lower() in ["sensory", "sensor", "input", "sensors"]:
                            cap_type = CapabilityType.SENSORY
                        elif cap_name.lower() in ["motor", "output", "actuator", "motors"]:
                            cap_type = CapabilityType.MOTOR
                        elif cap_name.lower() in ["visualization", "viz"]:
                            cap_type = CapabilityType.VISUALIZATION
                        
                        if cap_type and isinstance(cap_config, dict) and "rate_hz" in cap_config:
                            requested_rate = float(cap_config["rate_hz"])
                            capability_rates_to_register.append(CapabilityRateSpec(
                                capability_type=cap_type,
                                agent_id=request.agent_id,
                                requested_rate_hz=requested_rate
                            ))
                except ImportError:
                    pass

                # 5. Sanitize capabilities
                sanitized_caps = self._sanitize_capabilities(request.capabilities)

                # 6. Normalize capabilities to feagi-sensorimotor format: {"input": [...], "output": [...]}
                # FEAGI HTTP API expects ONLY "input"/"output" keys (not "sensory"/"motor")
                normalized_caps = {}
                
                # First pass: Process "input"/"output" keys (prioritize feagi-sensorimotor format)
                for cap_name, cap_config in sanitized_caps.items():
                    cap_lower = cap_name.lower()
                    
                    if cap_lower == "input":
                        # Already in correct format
                        if isinstance(cap_config, list):
                            normalized_caps["input"] = cap_config
                    elif cap_lower == "output":
                        # Already in correct format
                        if isinstance(cap_config, list):
                            normalized_caps["output"] = cap_config
                
                # Second pass: Convert legacy "sensory"/"motor" keys (only if "input"/"output" not already set)
                for cap_name, cap_config in sanitized_caps.items():
                    cap_lower = cap_name.lower()
                    
                    # Normalize legacy formats to feagi-sensorimotor format
                    if cap_lower in ["sensory", "sensor", "sensors"]:
                        # Skip if "input" already exists (prioritize feagi-sensorimotor format)
                        if "input" not in normalized_caps:
                            # Convert to "input" format: array of cortical IDs
                            if isinstance(cap_config, dict):
                                # Extract cortical_areas if present
                                if "cortical_areas" in cap_config:
                                    normalized_caps["input"] = cap_config["cortical_areas"]
                                elif cap_config:  # Non-empty dict, create empty array
                                    normalized_caps["input"] = []
                            elif isinstance(cap_config, list):
                                # Already in array format
                                normalized_caps["input"] = cap_config
                    elif cap_lower in ["motor", "actuator", "motors"]:
                        # Skip if "output" already exists (prioritize feagi-sensorimotor format)
                        if "output" not in normalized_caps:
                            # Convert to "output" format: array of cortical IDs
                            if isinstance(cap_config, dict):
                                # Extract source_cortical_areas if present
                                if "source_cortical_areas" in cap_config:
                                    normalized_caps["output"] = cap_config["source_cortical_areas"]
                                elif cap_config:  # Non-empty dict, create empty array
                                    normalized_caps["output"] = []
                            elif isinstance(cap_config, list):
                                # Already in array format
                                normalized_caps["output"] = cap_config
                    elif cap_lower in ["visualization", "viz"]:
                        # Keep visualization as-is (not part of input/output)
                        normalized_caps["visualization"] = cap_config
                    elif cap_lower in ["vision"]:
                        # Keep vision as-is
                        normalized_caps["vision"] = cap_config
                    elif cap_lower not in ["input", "output"]:  # Skip already processed keys
                        # Store non-standard capabilities in custom
                        if "custom" not in normalized_caps:
                            normalized_caps["custom"] = {}
                        normalized_caps["custom"][cap_name] = cap_config
                
                # Prepare metadata for registration
                metadata = {}
                if request.agent_data_port is not None:
                    metadata["agent_data_port"] = request.agent_data_port
                if request.agent_version:
                    metadata["agent_version"] = request.agent_version
                if request.controller_version:
                    metadata["controller_version"] = request.controller_version
                if request.agent_ip:
                    metadata["agent_ip"] = request.agent_ip
                if request.metadata:
                    metadata.update(request.metadata)
                
                # FEAGI 2.0: Register via HTTP API endpoint to trigger auto-creation of missing cortical areas
                # This ensures the registration endpoint processes the request and creates missing IPU/OPU areas
                cortical_areas = {"required_ipu_areas": [], "required_opu_areas": []}  # Default
                
                try:
                    import requests
                    
                    # Safety mode: require explicit API URL (no defaults/fallbacks).
                    feagi_api_url = None
                    if request.metadata and "feagi_api_url" in request.metadata:
                        feagi_api_url = request.metadata["feagi_api_url"]

                    if not feagi_api_url:
                        return AgentRegistrationResponse(
                            success=False,
                            message=(
                                "Missing required configuration: request.metadata['feagi_api_url'] "
                                "must be provided (no defaults in safety mode)."
                            ),
                            agent_id=request.agent_id,
                            cortical_areas=cortical_areas,
                            error_code="MISSING_CONFIG",
                        )

                    http_timeout_s = None
                    if request.metadata and "feagi_http_timeout_s" in request.metadata:
                        http_timeout_s = float(request.metadata["feagi_http_timeout_s"])

                    if not http_timeout_s or http_timeout_s <= 0:
                        return AgentRegistrationResponse(
                            success=False,
                            message=(
                                "Missing required configuration: request.metadata['feagi_http_timeout_s'] "
                                "must be provided and > 0 (no defaults in safety mode)."
                            ),
                            agent_id=request.agent_id,
                            cortical_areas=cortical_areas,
                            error_code="MISSING_CONFIG",
                        )
                    
                    # Build registration payload for HTTP API
                    # agent_data_port is required (u16) - use default if not provided
                    agent_data_port = request.agent_data_port if request.agent_data_port is not None else 0
                    
                    registration_payload = {
                        "agent_id": request.agent_id,
                        "agent_type": request.agent_type,
                        "capabilities": normalized_caps,  # Use feagi-sensorimotor format: {"input": [...], "output": [...]}
                        "agent_data_port": agent_data_port,  # Must be u16, not null
                        "agent_version": request.agent_version or "1.0.0",
                        "controller_version": request.controller_version or "1.0.0",
                        "metadata": metadata,
                    }
                    
                    # Call FEAGI HTTP API registration endpoint
                    api_endpoint = f"{feagi_api_url}/v1/agent/register"
                    logger.info(f"📡 Registering agent via HTTP API: {api_endpoint}")
                    
                    response = requests.post(
                        api_endpoint,
                        json=registration_payload,
                        timeout=http_timeout_s,
                        headers={"Content-Type": "application/json"}
                    )
                    
                    if response.status_code != 200:
                        error_msg = f"HTTP registration failed: {response.status_code} - {response.text}"
                        logger.error(error_msg)
                        return AgentRegistrationResponse(
                            success=False,
                            message=error_msg,
                            agent_id=request.agent_id,
                            cortical_areas=cortical_areas,
                            error_code="HTTP_REGISTRATION_ERROR",
                        )
                    
                    # Parse response
                    api_response = response.json()
                    
                    # Extract cortical area information from response (FEAGI 2.0 feature)
                    cortical_areas = api_response.get("cortical_areas", {
                        "required_ipu_areas": [],
                        "required_opu_areas": []
                    })
                    
                    # Check if registration was successful
                    if api_response.get("status") != "success":
                        error_msg = api_response.get("message", "Registration failed")
                        logger.error(f"Registration failed: {error_msg}")
                        return AgentRegistrationResponse(
                            success=False,
                            message=error_msg,
                            agent_id=request.agent_id,
                            cortical_areas=cortical_areas,
                            error_code="REGISTRATION_REJECTED",
                        )
                    
                    logger.info(f"✅ Agent registered via HTTP API: {request.agent_id}")
                    logger.info(f"   Cortical areas: {len(cortical_areas.get('required_ipu_areas', []))} IPU, {len(cortical_areas.get('required_opu_areas', []))} OPU")
                    
                    # Store API URL for heartbeat
                    self._feagi_api_urls[request.agent_id] = feagi_api_url
                    self._agent_metadata[request.agent_id] = dict(request.metadata or {})
                    
                    # Extract transport info from API response (contains actual ZMQ ports)
                    transport_info_from_api = {}
                    if "transports" in api_response:
                        transport_info_from_api["transports"] = api_response["transports"]
                    if "zmq_ports" in api_response:
                        transport_info_from_api["zmq_ports"] = api_response["zmq_ports"]
                        logger.info(f"   ZMQ ports from API: {api_response.get('zmq_ports', {})}")
                    
                    # Now also register in local Rust registry for backward compatibility
                    # (This is for internal consistency, not the primary registration)
                    # Skip if Rust registry is not available (HTTP-only mode)
                    registry = self._get_registry()
                    if registry:
                        metadata_json = json.dumps(metadata) if metadata else None
                        rust_capabilities_json = json.dumps(sanitized_caps)
                        try:
                            result_json = registry.register_agent_direct(
                                request.agent_id,
                                self._map_agent_type_to_rust(request.agent_type),
                                rust_capabilities_json,
                                metadata_json
                            )
                            result = json.loads(result_json)
                            if result.get("success"):
                                logger.info(f"🦀 Agent also stored in local Rust registry: {request.agent_id}")
                        except Exception as e:
                            logger.warning(f"Failed to store in local Rust registry (non-fatal): {e}")
                    else:
                        logger.debug("Skipping local Rust registry storage (HTTP-only mode)")
                    
                except ImportError:
                    logger.error("'requests' library not available. Cannot register via HTTP API.")
                    return AgentRegistrationResponse(
                        success=False,
                        message="HTTP registration requires 'requests' library. Install with: pip install requests",
                        agent_id=request.agent_id,
                        cortical_areas=cortical_areas,
                        error_code="MISSING_DEPENDENCY",
                    )
                except requests.exceptions.RequestException as e:
                    error_msg = f"HTTP registration request failed: {e}"
                    logger.error(error_msg)
                    return AgentRegistrationResponse(
                        success=False,
                        message=error_msg,
                        agent_id=request.agent_id,
                        cortical_areas=cortical_areas,
                        error_code="HTTP_REQUEST_ERROR",
                    )

                # 7. Update capability counts
                self._update_capability_counts(sanitized_caps, increment=True)

                # 8. Register capability rates
                if capability_rates_to_register:
                    try:
                        rate_mgr = get_capability_rate_manager()
                        if rate_mgr:
                            approved, rejections = rate_mgr.register_agent_capabilities(
                                request.agent_id,
                                capability_rates_to_register
                            )
                            logger.info(f"Registered {len(approved)} capability rates for agent {request.agent_id}")
                            if rejections:
                                logger.warning(f"Rejected capabilities for {request.agent_id}: {rejections}")
                    except Exception as e:
                        logger.warning(f"Failed to register capability rate: {e}")

                # 9. Coordinate FQ samplers
                fq_changes = self._coordinate_fq_samplers_for_registration(sanitized_caps)

                # 10. Update State Manager
                self._update_state_manager()

                # 11. Notify listeners
                self._notify_state_change("agent_registered", request.agent_id)

                registry = self._get_registry()
                total_agents = registry.agent_count() if registry else "N/A (HTTP-only mode)"
                logger.info(
                    f"✅ Agent registered: {request.agent_id} "
                    f"(total: {total_agents})"
                )

                # Get transport info - prefer API response, fallback to config-based
                if not transport_info_from_api:
                    return AgentRegistrationResponse(
                        success=False,
                        message=(
                            "Registration API response did not include transport endpoints "
                            "(no fallback transport defaults in safety mode)."
                        ),
                        agent_id=request.agent_id,
                        cortical_areas=cortical_areas,
                        error_code="MISSING_TRANSPORT_INFO",
                    )

                transport_info = transport_info_from_api
                
                # Return response with cortical area information from HTTP API
                return AgentRegistrationResponse(
                    success=True,
                    message=f"Agent {request.agent_id} registered successfully",
                    agent_id=request.agent_id,
                    cortical_areas=cortical_areas,  # Use cortical areas from HTTP API response
                    fq_samplers_enabled=fq_changes,
                    transport_info=transport_info,  # Use transport info from API response if available
                )

            except Exception as e:
                logger.error(f"❌ Registration failed for {request.agent_id}: {e}", exc_info=True)
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Registration failed: {str(e)}",
                    agent_id=request.agent_id,
                    cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                    error_code="INTERNAL_ERROR",
                )

    def deregister_agent(self, agent_id: str) -> AgentRegistrationResponse:
        """Deregister agent from Rust registry"""
        with self._lock:
            logger.info(f"🔌 DEREGISTRATION REQUEST: {agent_id}")

            try:
                # Check if shutting down and skip registry access
                registry = self._get_registry()
                if registry is None:
                    # System is shutting down, skip deregistration
                    return AgentRegistrationResponse(
                        success=True,
                        message="Deregistration skipped (system shutting down)",
                        agent_id=agent_id,
                        cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                        fq_samplers_enabled=False,
                    )
                
                # Get agent info before deletion
                agent_json = registry.get_agent_json(agent_id)
                agent_info = json.loads(agent_json)
                rust_caps = agent_info.get("capabilities", {})
                caps = self._flatten_rust_capabilities(rust_caps)

                # Update capability counts
                self._update_capability_counts(caps, increment=False)

                # 🦀 Remove from Rust registry
                result_json = registry.deregister_agent_direct(agent_id)
                result = json.loads(result_json)
                
                if not result.get("success"):
                    return AgentRegistrationResponse(
                        success=False,
                        message=result.get("message", "Deregistration failed"),
                        agent_id=agent_id,
                        cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                        error_code="RUST_REGISTRY_ERROR",
                    )

                # Coordinate FQ samplers
                fq_changes = self._coordinate_fq_samplers_for_deregistration(caps)

                # Update State Manager
                self._update_state_manager()

                # Notify listeners
                self._notify_state_change("agent_deregistered", agent_id)

                logger.info(f"✅ Agent deregistered: {agent_id}")

                return AgentRegistrationResponse(
                    success=True,
                    message=f"Agent {agent_id} deregistered successfully",
                    agent_id=agent_id,
                    cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                    fq_samplers_enabled=fq_changes,
                )

            except Exception as e:
                logger.error(f"❌ Deregistration failed for {agent_id}: {e}")
                return AgentRegistrationResponse(
                    success=False,
                    message=f"Deregistration failed: {str(e)}",
                    agent_id=agent_id,
                    cortical_areas={"required_ipu_areas": [], "required_opu_areas": []},
                    error_code="AGENT_NOT_FOUND",
                )

    def heartbeat_agent(self, agent_id: str, feagi_api_url: Optional[str] = None) -> bool:
        """Send heartbeat to FEAGI API to keep agent registered"""
        # Use stored API URL or provided one
        api_url = feagi_api_url or self._feagi_api_urls.get(agent_id)
        if not api_url:
            raise RuntimeError(
                "Missing FEAGI API URL for heartbeat (no defaults in safety mode). "
                "Provide feagi_api_url or ensure the agent was registered with metadata['feagi_api_url']."
            )

        http_timeout_s = None
        if agent_id in self._agent_metadata and "feagi_http_timeout_s" in self._agent_metadata[agent_id]:
            http_timeout_s = float(self._agent_metadata[agent_id]["feagi_http_timeout_s"])
        if not http_timeout_s or http_timeout_s <= 0:
            raise RuntimeError(
                "Missing heartbeat HTTP timeout (no defaults in safety mode). "
                "Provide metadata['feagi_http_timeout_s'] during registration."
            )
        
        try:
            import requests
            heartbeat_endpoint = f"{api_url}/v1/agent/heartbeat"
            response = requests.post(
                heartbeat_endpoint,
                json={"agent_id": agent_id},
                timeout=http_timeout_s,
                headers={"Content-Type": "application/json"}
            )
            if response.status_code == 200:
                logger.debug(f"💓 [HEARTBEAT] Sent heartbeat for agent '{agent_id}'")
                return True
            else:
                logger.warning(f"💓 [HEARTBEAT] Heartbeat failed for '{agent_id}': HTTP {response.status_code}")
                return False
        except ImportError:
            logger.warning("💓 [HEARTBEAT] 'requests' library not available. Cannot send heartbeat via HTTP API.")
            return False
        except Exception as e:
            logger.warning(f"💓 [HEARTBEAT] Heartbeat failed for '{agent_id}': {e}")
            return False
    
    def start_heartbeat(self, agent_id: str, feagi_host: str, feagi_api_port: int):
        """Start background heartbeat thread for an agent"""
        if agent_id in self._heartbeat_running and self._heartbeat_running[agent_id]:
            return  # Already running
        
        feagi_api_url = f"http://{feagi_host}:{feagi_api_port}"
        self._feagi_api_urls[agent_id] = feagi_api_url
        
        self._heartbeat_running[agent_id] = True
        heartbeat_interval = self._heartbeat_interval
        if heartbeat_interval is None or heartbeat_interval <= 0:
            raise RuntimeError(
                "Heartbeat interval must be explicitly configured (no defaults in safety mode). "
                "Call RegistrationManager.configure_heartbeat(...) before starting heartbeat."
            )
        
        def heartbeat_loop():
            """Background thread that sends periodic heartbeats"""
            while self._heartbeat_running.get(agent_id, False):
                try:
                    time.sleep(heartbeat_interval)
                    if not self._heartbeat_running.get(agent_id, False):
                        break
                    
                    # Send heartbeat via HTTP API
                    self.heartbeat_agent(agent_id, feagi_api_url)
                except Exception as e:
                    logger.error(f"💓 [HEARTBEAT] Error in heartbeat loop for '{agent_id}': {e}", exc_info=True)
                    # Continue heartbeat loop even on error
                    time.sleep(heartbeat_interval)
        
        heartbeat_thread = threading.Thread(
            target=heartbeat_loop,
            daemon=True,
            name=f"heartbeat-{agent_id}"
        )
        heartbeat_thread.start()
        self._heartbeat_threads[agent_id] = heartbeat_thread
        logger.info(f"💓 [HEARTBEAT] Started heartbeat thread for agent '{agent_id}' (interval: {heartbeat_interval}s, API: {feagi_api_url})")
    
    def stop_heartbeat(self, agent_id: str):
        """Stop background heartbeat thread for an agent"""
        if agent_id in self._heartbeat_running:
            self._heartbeat_running[agent_id] = False
            if agent_id in self._heartbeat_threads:
                thread = self._heartbeat_threads[agent_id]
                if thread.is_alive():
                    if self._heartbeat_join_timeout_s is None:
                        raise RuntimeError(
                            "Heartbeat join timeout must be explicitly configured "
                            "(no defaults in safety mode). Call RegistrationManager.configure_heartbeat(...)."
                        )
                    thread.join(timeout=self._heartbeat_join_timeout_s)
                del self._heartbeat_threads[agent_id]
            if agent_id in self._feagi_api_urls:
                del self._feagi_api_urls[agent_id]
            if agent_id in self._agent_metadata:
                del self._agent_metadata[agent_id]
            logger.debug(f"💓 [HEARTBEAT] Stopped heartbeat thread for agent '{agent_id}'")

    def get_agent_properties(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """Get agent properties from Rust registry"""
        try:
            agent_json = self._get_registry().get_agent_json(agent_id)
            agent = json.loads(agent_json)
            
            # Extract custom capabilities for backward compatibility
            if "capabilities" in agent and "custom" in agent["capabilities"]:
                agent["capabilities"] = agent["capabilities"]["custom"]
            
            # Metadata is now properly stored in agent["metadata"] by Rust
            # Extract common metadata fields to top level for convenience
            if "metadata" in agent and agent["metadata"]:
                metadata = agent["metadata"]
                if "agent_data_port" in metadata:
                    agent["agent_data_port"] = metadata["agent_data_port"]
                if "agent_version" in metadata:
                    agent["agent_version"] = metadata["agent_version"]
                if "controller_version" in metadata:
                    agent["controller_version"] = metadata["controller_version"]
                if "agent_ip" in metadata:
                    agent["agent_ip"] = metadata["agent_ip"]
            
            return agent
        except Exception:
            return None

    def list_agents(self) -> Dict[str, Any]:
        """List all registered agents"""
        import time
        start = time.time()
        try:
            t1 = time.time()
            registry = self._get_registry()
            t2 = time.time()
            logger.debug(f"⏱️  [TIMING] _get_registry() took {(t2-t1)*1000:.1f}ms")
            
            # Handle shutdown gracefully
            if registry is None:
                return {"agents": [], "count": 0, "capability_summary": {}, "fq_sampler_states": {}}
            
            agents_json = registry.get_all_agents_json()
            t3 = time.time()
            logger.debug(f"⏱️  [TIMING] get_all_agents_json() took {(t3-t2)*1000:.1f}ms")
            
            agents_list = json.loads(agents_json)
            t4 = time.time()
            logger.debug(f"⏱️  [TIMING] json.loads() took {(t4-t3)*1000:.1f}ms")
            
            # Extract and flatten capabilities for backward compatibility
            for agent in agents_list:
                # Flatten Rust AgentCapabilities struct to legacy dict format
                if "capabilities" in agent:
                    agent["capabilities"] = self._flatten_rust_capabilities(agent["capabilities"])
                
                # Extract common metadata fields to top level for convenience
                if "metadata" in agent and agent["metadata"]:
                    metadata = agent["metadata"]
                    if "agent_data_port" in metadata:
                        agent["agent_data_port"] = metadata["agent_data_port"]
                    if "agent_version" in metadata:
                        agent["agent_version"] = metadata["agent_version"]
                    if "controller_version" in metadata:
                        agent["controller_version"] = metadata["controller_version"]
                    if "agent_ip" in metadata:
                        agent["agent_ip"] = metadata["agent_ip"]
            
            result = {
                "agents": agents_list,
                "count": len(agents_list),
                "capability_summary": self._capability_counts.copy(),
                "fq_sampler_states": self._fq_sampler_states.copy(),
            }
            elapsed = time.time() - start
            logger.debug(f"⏱️  [TIMING] list_agents() TOTAL: {elapsed*1000:.1f}ms")
            return result
        except Exception as e:
            logger.error(f"Failed to list agents: {e}")
            return {"agents": [], "count": 0}

    def get_fq_sampler_coordination_status(self) -> Dict[str, Any]:
        """Get FQ sampler status"""
        return {
            "fq_samplers": self._fq_sampler_states.copy(),
            "capability_counts": self._capability_counts.copy(),
            "agent_count": self._get_registry().agent_count(),
        }

    def register_state_change_listener(self, listener: Callable) -> None:
        """Register a state change listener"""
        self._state_change_listeners.add(listener)

    def unregister_state_change_listener(self, listener: Callable) -> None:
        """Unregister a state change listener"""
        self._state_change_listeners.discard(listener)

    # === Helper methods ===
    
    def _map_agent_type_to_rust(self, python_type: str) -> str:
        """Map Python agent type to Rust AgentType"""
        type_map = {
            "sensory": "sensory",
            "motor": "motor",
            "both": "both",
            "multimodal": "both",
            "visualization": "visualization",
            "infrastructure": "infrastructure",
            "feagi_bridge": "infrastructure",  # Bridge is infrastructure type
        }
        return type_map.get(python_type.lower(), "sensory")

    def _sanitize_capabilities(self, capabilities: Dict[str, Any]) -> Dict[str, Any]:
        """Sanitize and normalize capabilities"""
        sanitized = {}
        for cap_name, cap_value in capabilities.items():
            if cap_value is None:
                continue
            if isinstance(cap_value, dict):
                # Remove rate_hz from storage (handled separately)
                cleaned = {k: v for k, v in cap_value.items() if k != "rate_hz"}
                if cleaned:
                    sanitized[cap_name] = cleaned
            else:
                sanitized[cap_name] = cap_value
        return sanitized

    def _validate_agent_request(self, request: AgentRegistrationRequest) -> Dict[str, Any]:
        """Validate agent registration request"""
        if not request.agent_id or not isinstance(request.agent_id, str):
            return {"valid": False, "error": "Invalid agent_id"}
        if not request.agent_type:
            return {"valid": False, "error": "Missing agent_type"}
        if not request.capabilities or not isinstance(request.capabilities, dict):
            return {"valid": False, "error": "Invalid capabilities"}
        return {"valid": True}

    def _check_feagi_readiness(self) -> bool:
        """Check if FEAGI is ready"""
        return self._feagi_ready

    def _get_transport_info(self, agent_type: str, capabilities: Dict[str, Any]) -> Dict[str, Any]:
        """Get transport information for agent.

        Safety mode: transport endpoints must come from the FEAGI registration API response.
        This method is retained only for legacy call sites and must not be used.
        """
        raise RuntimeError(
            "Transport endpoints must be provided by FEAGI during registration "
            "(no default/fallback endpoints in safety mode)."
        )

    def _update_capability_counts(self, capabilities: Dict[str, Any], increment: bool) -> None:
        """Update capability counts"""
        delta = 1 if increment else -1
        
        for cap_name in capabilities.keys():
            cap_lower = cap_name.lower()
            if cap_lower in ["visualization", "viz"]:
                self._capability_counts["visualization"] += delta
            elif cap_lower in ["motor", "motors", "output"]:
                self._capability_counts["motor"] += delta
            elif cap_lower in ["sensory", "sensor", "input", "sensors"]:
                self._capability_counts["sensory"] += delta
            elif cap_lower == "video":
                self._capability_counts["video"] += delta
            elif cap_lower == "feagi":
                self._capability_counts["feagi"] += delta

    def _has_visualization_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has visualization capabilities"""
        return any(k.lower() in ["visualization", "viz"] for k in capabilities.keys())

    def _has_motor_capabilities(self, capabilities: Dict[str, Any]) -> bool:
        """Check if agent has motor capabilities"""
        return any(k.lower() in ["motor", "motors", "output"] for k in capabilities.keys())

    def _coordinate_fq_samplers_for_registration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent registers"""
        changes = {}
        
        # Visualization FQ sampler
        has_viz = self._has_visualization_capabilities(capabilities)
        if has_viz and not self._fq_sampler_states["visualization_enabled"]:
            if self._process_manager:
                try:
                    # Create visualization FQ sampler (Rust burst engine + SHM writer)
                    # Default to 60Hz for visualization (will be throttled per-agent by capability manager)
                    self._process_manager.create_fq_sampler("visualization", 60.0)
                    self._fq_sampler_states["visualization_enabled"] = True
                    changes["visualization_fq_enabled"] = True
                    logger.info("🎨 Visualization FQ sampler ENABLED")
                except Exception as e:
                    logger.error(f"Failed to enable visualization FQ sampler: {e}")

        # Motor FQ sampler
        has_motor = self._has_motor_capabilities(capabilities)
        if has_motor and not self._fq_sampler_states["motor_enabled"]:
            if self._process_manager:
                try:
                    # Create motor FQ sampler (Rust burst engine handles motor output)
                    self._process_manager.create_fq_sampler("motor", 60.0)
                    self._fq_sampler_states["motor_enabled"] = True
                    changes["motor_fq_enabled"] = True
                    logger.info("🦾 Motor FQ sampler ENABLED")
                except Exception as e:
                    logger.error(f"Failed to enable motor FQ sampler: {e}")

        return changes

    def _coordinate_fq_samplers_for_deregistration(
        self, capabilities: Dict[str, Any]
    ) -> Dict[str, bool]:
        """Coordinate FQ samplers when agent deregisters"""
        changes = {}
        
        # Check if we should disable visualization FQ
        has_viz = self._has_visualization_capabilities(capabilities)
        if has_viz:
            if self._capability_counts["visualization"] <= 0 and self._fq_sampler_states["visualization_enabled"]:
                if self._process_manager:
                    try:
                        self._process_manager.disable_fq_sampler("visualization")
                        self._fq_sampler_states["visualization_enabled"] = False
                        changes["visualization_fq_disabled"] = True
                        logger.info("🎨 Visualization FQ sampler DISABLED")
                    except Exception as e:
                        logger.error(f"Failed to disable visualization FQ sampler: {e}")

        # Check if we should disable motor FQ
        has_motor = self._has_motor_capabilities(capabilities)
        if has_motor:
            if self._capability_counts["motor"] <= 0 and self._fq_sampler_states["motor_enabled"]:
                if self._process_manager:
                    try:
                        self._process_manager.disable_fq_sampler("motor")
                        self._fq_sampler_states["motor_enabled"] = False
                        changes["motor_fq_disabled"] = True
                        logger.info("🦾 Motor FQ sampler DISABLED")
                    except Exception as e:
                        logger.error(f"Failed to disable motor FQ sampler: {e}")

        return changes

    def _update_state_manager(self) -> None:
        """Update State Manager with current registry state"""
        if self._state_manager:
            try:
                agents_json = self._get_registry().get_all_agents_json()
                agents_list = json.loads(agents_json)
                
                # Convert list to dict (keyed by agent_id) for state manager
                agents_dict = {}
                for agent in agents_list:
                    agent_id = agent.get("agent_id")
                    if agent_id:
                        # Extract custom capabilities for backward compatibility
                        if "capabilities" in agent and "custom" in agent["capabilities"]:
                            agent["capabilities"] = agent["capabilities"]["custom"]
                        
                        # Extract common metadata fields to top level for convenience
                        if "metadata" in agent and agent["metadata"]:
                            metadata = agent["metadata"]
                            if "agent_data_port" in metadata:
                                agent["agent_data_port"] = metadata["agent_data_port"]
                            if "agent_version" in metadata:
                                agent["agent_version"] = metadata["agent_version"]
                            if "controller_version" in metadata:
                                agent["controller_version"] = metadata["controller_version"]
                            if "agent_ip" in metadata:
                                agent["agent_ip"] = metadata["agent_ip"]
                        
                        agents_dict[agent_id] = agent
                
                # Use the correct state manager method
                result = self._state_manager.set_connected_agents(agents_dict)
                if result.is_err:
                    logger.warning(f"Failed to update State Manager: {result.error}")
            except Exception as e:
                logger.warning(f"Failed to update State Manager: {e}")

    def _notify_state_change(self, event_type: str, agent_id: str) -> None:
        """Notify state change listeners"""
        for listener in self._state_change_listeners:
            try:
                listener(event_type, agent_id)
            except Exception as e:
                logger.warning(f"State change listener failed: {e}")


# === Global instance management ===

_registration_manager_instance: Optional[RegistrationManager] = None
_registration_manager_lock = threading.RLock()


def get_registration_manager() -> Optional[RegistrationManager]:
    """Get the global Registration Manager instance"""
    global _registration_manager_instance
    with _registration_manager_lock:
        return _registration_manager_instance


def set_registration_manager(manager: Optional[RegistrationManager]) -> None:
    """Set the global Registration Manager instance"""
    global _registration_manager_instance
    with _registration_manager_lock:
        _registration_manager_instance = manager
        if manager:
            logger.info("🦀 Global Rust-backed Registration Manager set")
        else:
            logger.info("🔄 Registration Manager cleared")


def create_registration_manager(
    state_manager=None, process_manager=None
) -> RegistrationManager:
    """Create and initialize the global Rust-backed Registration Manager"""
    global _registration_manager_instance

    with _registration_manager_lock:
        if _registration_manager_instance is not None:
            logger.warning("🦀 Registration Manager already exists - returning existing")
            return _registration_manager_instance

        _registration_manager_instance = RegistrationManager(
            state_manager=state_manager, process_manager=process_manager
        )

        logger.info("🦀 Global Rust-backed Registration Manager created and initialized")
        return _registration_manager_instance


def reset_registration_manager() -> None:
    """Reset the global Registration Manager (for testing)"""
    global _registration_manager_instance
    with _registration_manager_lock:
        _registration_manager_instance = None
        logger.info("🔄 Registration Manager reset")
