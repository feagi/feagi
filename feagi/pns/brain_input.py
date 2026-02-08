"""
FEAGI Brain Input Manager

Global manager for all FEAGI inputs (sensory data sources).
Uses Rust IOCache for high-performance encoding.
"""

from typing import Any, Dict, List, Optional, TYPE_CHECKING
import base64
import binascii
import json
import logging
import os
import time
from datetime import datetime

if TYPE_CHECKING:
    from feagi.pns.inputs.base import BaseInput
    from feagi.pns.observability.monitor import Monitor

logger = logging.getLogger("feagi.pns.brain_input")

# @ruff-skip: module has >100 E501 line-length violations - cleanup task: sdk-lint-cleanup-brain-input


class BrainInput:
    """
    Global brain input manager.
    
    Manages all registered inputs and handles automatic encoding
    and transmission to FEAGI. Uses Rust IOCache for performance.
    
    This is a singleton - use the module-level `brain_input` instance.
    
    Example:
        from feagi.pns.inputs import Camera
        from feagi.pns import brain_input
        
        # Register inputs
        camera = Camera.register(resolution=(1920, 1080))
        
        # Configure and connect
        brain_input.configure(feagi_host="localhost")
        brain_input.connect()
        
        # Main loop
        while True:
            camera.set_frame(frame)
            brain_input.send()  # Encodes and sends all inputs
    """
    
    def __init__(self):
        # Rust IOCache (lazy-initialized)
        self._cache = None
        self._cache_available = False
        
        # Registry of all inputs
        self._inputs: List['BaseInput'] = []
        
        # Transport (ZMQ/WebSocket)
        self._transport = None
        self._zmq_context = None  # MUST keep context alive!
        self._connected = False

        # Rust-backed agent client (preferred transport)
        self._agent_client = None
        self._agent_capabilities: Optional[Dict[str, Any]] = None
        self._agent_type: Optional[str] = None
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Configuration
        # @safety: No implicit defaults. Commercial deployments must be explicitly configured.
        self._feagi_host: Optional[str] = None
        self._feagi_port: Optional[int] = None
        self._transport_type: Optional[str] = None
        self._api_port: Optional[int] = None  # FEAGI API port for registration
        self._feagi_http_timeout_s: Optional[float] = None
        self._heartbeat_interval_s: Optional[float] = None
        self._heartbeat_join_timeout_s: Optional[float] = None
        
        # Agent registration (REQUIRED in FEAGI 2.0)
        self._agent_registered = False
        self._agent_id = None
        self._registration_response = None
        self._registration_manager = None
        
        # Observability monitors
        self._monitors: List['Monitor'] = []
    
    def _init_cache(self):
        """Initialize Rust ConnectorAgent (lazy)."""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            
            # Initialize Rust tracing logging (if available)
            try:
                # Try top-level function first (available in latest version)
                if hasattr(frpl, 'init_rust_logging'):
                    frpl.init_rust_logging()
                    logger.debug("[OK] Rust tracing logging initialized")
                # Fallback to connector_core module
                elif (
                    hasattr(frpl, "connector_core")
                    and hasattr(frpl.connector_core, "init_rust_logging")
                ):
                    frpl.connector_core.init_rust_logging()
                    logger.debug("[OK] Rust tracing logging initialized via connector_core")
                # Fallback to feagi_agent_sdk
                # (may be commented out in some builds)
                elif (
                    hasattr(frpl, "feagi_agent_sdk")
                    and hasattr(frpl.feagi_agent_sdk, "init_rust_logging")
                ):
                    frpl.feagi_agent_sdk.init_rust_logging()
                    logger.debug("[OK] Rust tracing logging initialized via feagi_agent_sdk")
            except Exception as log_init_err:
                logger.debug(
                    "Could not initialize Rust logging (non-fatal): %s",
                    log_init_err,
                )
            
            # Use ConnectorAgent which provides sensor methods.
            # NOTE: frpl.connector_core.caching.IOCache() does not exist in current API
            # Using ConnectorAgent instead - encoding methods still need to be added to rust-py-libs
            agent_descriptor_b64 = self._resolve_agent_descriptor_b64()
            # ConnectorAgent may be built with or without constructor arg; env var works for both.
            os.environ["FEAGI_AGENT_DESCRIPTOR_B64"] = agent_descriptor_b64
            self._cache = frpl.connector_core.ConnectorAgent()
            self._cache_available = True
            logger.info("[OK] Rust ConnectorAgent initialized")
        except (ImportError, AttributeError) as e:
            logger.error(f"[FAIL] Failed to initialize Rust ConnectorAgent: {e}")
            logger.error("   Install with: pip install feagi_rust_py_libs")
            raise ImportError(
                "Rust SDK (feagi_rust_py_libs) is required for brain_input.\n"
                "Install with: pip install feagi_rust_py_libs"
            ) from e

    def _resolve_agent_descriptor_b64(self) -> str:
        """
        Resolve the base64 AgentDescriptor required by the Rust ConnectorAgent.

        Priority:
        1) self._agent_id (must already be base64 AgentDescriptor)
        2) FEAGI_AGENT_DESCRIPTOR_B64 environment variable
        """
        if self._agent_id:
            return self._validate_agent_descriptor_b64(self._agent_id)

        env_b64 = os.environ.get("FEAGI_AGENT_DESCRIPTOR_B64")
        if env_b64:
            return self._validate_agent_descriptor_b64(env_b64)

        raise RuntimeError(
            "Missing AgentDescriptor base64. Provide a base64 AgentDescriptor as agent_id "
            "or set FEAGI_AGENT_DESCRIPTOR_B64 before registering inputs."
        )

    def _validate_agent_descriptor_b64(self, value: str) -> str:
        """
        Validate that a string is a base64-encoded AgentDescriptor.

        AgentDescriptor is 48 bytes (4 + 20 + 20 + 4). This is enforced strictly.
        """
        try:
            raw = base64.b64decode(value, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise RuntimeError(
                "agent_id must be a base64 AgentDescriptor (48 bytes)."
            ) from exc
        if len(raw) != 48:
            raise RuntimeError("agent_id must decode to 48 bytes (AgentDescriptor).")
        return value
    
    def _allocate_group_id(self) -> int:
        """Allocate next cortical group ID"""
        group_id = self._next_group_id
        self._next_group_id += 1
        return group_id
    
    def configure(
        self,
        *,
        feagi_host: str,
        feagi_port: int,
        transport: str,
        api_port: int,
        feagi_http_timeout_s: float,
        heartbeat_interval_s: float,
        heartbeat_join_timeout_s: float,
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            feagi_host: FEAGI server hostname or IP
            feagi_port: Sensory input port (default: 5558)
            transport: Transport type - "zmq" or "websocket"
            api_port: FEAGI API port for registration (default: 8000)
        """
        if not feagi_host:
            raise ValueError("feagi_host must be provided (no defaults in safety mode).")
        if feagi_port <= 0:
            raise ValueError("feagi_port must be a positive integer (no defaults in safety mode).")
        if api_port <= 0:
            raise ValueError("api_port must be a positive integer (no defaults in safety mode).")
        if not transport:
            raise ValueError("transport must be provided (no defaults in safety mode).")
        if feagi_http_timeout_s <= 0:
            raise ValueError("feagi_http_timeout_s must be > 0 (no defaults in safety mode).")
        if heartbeat_interval_s <= 0:
            raise ValueError("heartbeat_interval_s must be > 0 (no defaults in safety mode).")
        if heartbeat_join_timeout_s <= 0:
            raise ValueError("heartbeat_join_timeout_s must be > 0 (no defaults in safety mode).")

        self._feagi_host = feagi_host
        self._feagi_port = feagi_port
        self._transport_type = transport
        self._api_port = api_port
        self._feagi_http_timeout_s = feagi_http_timeout_s
        self._heartbeat_interval_s = heartbeat_interval_s
        self._heartbeat_join_timeout_s = heartbeat_join_timeout_s
        
        logger.info(f"📡 Configured: {transport}://{feagi_host}:{feagi_port} (API: {feagi_host}:{api_port})")
    
    def register_agent(
        self,
        agent_id: str,
        agent_type: str = "sensory",
        capabilities: Optional[Dict[str, Any]] = None,
        agent_version: Optional[str] = None,
        controller_version: Optional[str] = None,
    ) -> bool:
        """
        Register as an agent with FEAGI (REQUIRED in FEAGI 2.0).
        
        This MUST be called before connect() or any other operations.
        Registration triggers auto-creation of missing IPU/OPU cortical areas.
        
        Args:
            agent_id: Base64-encoded AgentDescriptor (48-byte payload)
            agent_type: Agent type ("sensory", "motor", "both", "visualization", "infrastructure")
            capabilities: Agent capabilities dict in structured format (e.g., {"vision": {"modality": "camera", "target_cortical_area": "isvi", ...}})
                          DEPRECATED: {"input": ["cortical_id"]} format is no longer supported - use structured capabilities like VSG
            agent_version: Optional agent version string
            controller_version: Optional controller version string
            
        Returns:
            True if registration succeeded, False otherwise
            
        Raises:
            RuntimeError: If registration fails (FEAGI 2.0 requires successful registration)
        """
        if self._agent_registered:
            logger.warning(f"Agent '{self._agent_id}' already registered. Re-registering...")
        
        # Import once (avoid redefinitions and ensure symbols exist even if we don't
        # need to instantiate a new RegistrationManager in this call).
        try:
            from feagi.pns.registration_manager import (  # noqa: PLC0415
                AgentRegistrationRequest,
                RegistrationManager,
            )
        except ImportError as e:
            logger.error(f"Failed to import RegistrationManager: {e}")
            raise RuntimeError(
                "RegistrationManager not available. Agent registration is required in FEAGI 2.0."
            ) from e

        # Initialize registration manager
        if self._registration_manager is None:
            self._registration_manager = RegistrationManager()

        if (
            self._feagi_http_timeout_s is None
            or self._heartbeat_interval_s is None
            or self._heartbeat_join_timeout_s is None
        ):
            raise RuntimeError(
                "brain_input.configure(...) must be called with explicit feagi_http_timeout_s, "
                "heartbeat_interval_s, and heartbeat_join_timeout_s before agent registration "
                "(no defaults in safety mode)."
            )

        self._registration_manager.configure_heartbeat(
            interval_s=self._heartbeat_interval_s,
            join_timeout_s=self._heartbeat_join_timeout_s,
        )
        
        # Validate capabilities format - REJECT legacy "input": ["cortical_id"] format
        # FEAGI 2.0 requires structured capabilities like VSG (vision, motor, etc.)
        if capabilities is not None:
            # Check for deprecated "input": ["cortical_id"] format
            if "input" in capabilities and isinstance(capabilities["input"], list):
                # Check if it's an array of base64 cortical IDs (deprecated format)
                input_list = capabilities["input"]
                if input_list and isinstance(input_list[0], str) and len(input_list[0]) > 8:
                    # Likely base64 cortical IDs - this format is deprecated
                    raise ValueError(
                        "[FAIL] DEPRECATED CAPABILITIES FORMAT DETECTED!\n"
                        "The format {'input': ['cortical_id']} is no longer supported.\n"
                        "Please use structured capabilities like VSG:\n"
                        "  {'vision': {'modality': '...', 'target_cortical_area': 'isvi', ...}}\n"
                        "or other structured capability types.\n"
                        "See Visual Sensory Generator for reference implementation."
                    )
        
        # Build capabilities from registered inputs if not provided
        if capabilities is None:
            # Auto-generate from registered inputs (for Camera, etc.)
            # This follows VSG pattern
            if self._inputs:
                # Check if we have vision inputs
                has_vision = any(
                    hasattr(inp, '__class__') and 'Camera' in inp.__class__.__name__
                    for inp in self._inputs
                )
                
                if has_vision:
                    # Use vision capability format (like VSG)
                    vision_input = next(
                        (inp for inp in self._inputs if hasattr(inp, '__class__') and 'Camera' in inp.__class__.__name__),
                        None
                    )
                    if vision_input:
                        cortical_area = getattr(vision_input, '_cortical_area', 'unknown')
                        resolution = getattr(vision_input, 'resolution', (64, 64))
                        capabilities = {
                            "vision": {
                                "modality": "camera",
                                "dimensions": [resolution[0], resolution[1]],
                                "channels": 3,
                                "target_cortical_area": cortical_area
                            }
                        }
                else:
                    raise ValueError(
                        "No capabilities provided and no registered inputs found.\n"
                        "Please provide structured capabilities (e.g., vision, motor) or register inputs first."
                    )
            else:
                raise ValueError(
                    "No capabilities provided and no registered inputs found.\n"
                    "Please provide structured capabilities (e.g., vision, motor) or register inputs first."
                )
        
        if self._feagi_host is None or self._api_port is None:
            raise RuntimeError(
                "brain_input.configure(...) must be called with explicit FEAGI host/api_port "
                "before agent registration (no defaults in safety mode)."
            )

        # Create registration request with API URL in metadata
        metadata = {
            "feagi_api_url": f"http://{self._feagi_host}:{self._api_port}",
            "feagi_api_port": self._api_port,
            "feagi_http_timeout_s": self._feagi_http_timeout_s,
        }
        request = AgentRegistrationRequest(
            agent_id=agent_id,
            agent_type=agent_type,
            agent_ip=self._feagi_host,
            capabilities=capabilities,
            agent_version=agent_version,
            controller_version=controller_version,
            metadata=metadata,
        )
        
        # Register with FEAGI
        try:
            response = self._registration_manager.register_agent(request)
            
            if not response.success:
                error_msg = (
                    f"Agent registration FAILED: {response.message}\n"
                    f"Error code: {response.error_code}\n"
                    f"FEAGI 2.0 requires successful agent registration before any operations.\n"
                    f"Please ensure FEAGI is running and the cortical areas exist (or auto-create is enabled)."
                )
                logger.error(error_msg)
                raise RuntimeError(error_msg)
            
            # Registration succeeded
            self._agent_registered = True
            self._agent_id = agent_id
            self._registration_response = response
            self._agent_type = agent_type
            self._agent_capabilities = capabilities
            
            # Log cortical area status
            if response.cortical_areas:
                ipu_areas = response.cortical_areas.get("required_ipu_areas", [])
                opu_areas = response.cortical_areas.get("required_opu_areas", [])
                
                logger.info(f"[OK] Agent '{agent_id}' registered successfully")
                logger.info(f"   IPU areas: {len(ipu_areas)} required")
                logger.info(f"   OPU areas: {len(opu_areas)} required")
                
                # Log status of each area
                for area in ipu_areas:
                    status = area.get("status", "unknown")
                    area_name = area.get("area_name", "unknown")
                    if status == "Created":
                        logger.info(f"   [OK] IPU area '{area_name}' was auto-created")
                    elif status == "Existing":
                        logger.info(f"   [OK] IPU area '{area_name}' exists")
                    elif status == "Missing":
                        logger.warning(f"   [WARN] IPU area '{area_name}' is missing (auto-create disabled?)")
                    elif status == "Error":
                        logger.error(f"   [FAIL] IPU area '{area_name}' creation failed: {area.get('message', 'unknown error')}")
            
            return True
            
        except Exception as e:
            error_msg = (
                f"Agent registration FAILED with exception: {e}\n"
                f"FEAGI 2.0 requires successful agent registration before any operations."
            )
            logger.error(error_msg, exc_info=True)
            raise RuntimeError(error_msg) from e
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Initializes transport and establishes connection.
        
        REQUIRES: Agent registration must succeed before calling this.
        """
        if not self._agent_registered:
            raise RuntimeError(
                "Agent registration required before connecting.\n"
                "Call brain_input.register_agent(agent_id='<agent_descriptor_b64>', ...) first.\n"
                "FEAGI 2.0 requires successful agent registration before any operations."
            )
        
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        # Initialize transport
        if self._transport_type is None or self._feagi_host is None or self._feagi_port is None:
            raise RuntimeError(
                "brain_input.configure(...) must be called with explicit FEAGI host/port/transport "
                "before connect() (no defaults in safety mode)."
            )

        if self._transport_type == "zmq":
            if self._registration_response is None:
                raise RuntimeError(
                    "Missing registration response (register_agent must succeed before connect)."
                )

            transport_info = getattr(self._registration_response, "transport_info", None)
            if not isinstance(transport_info, dict) or "transports" not in transport_info:
                raise RuntimeError(
                    "Registration response did not include transport endpoints (safety mode: "
                    "endpoints must come from registration response)."
                )

            transports = transport_info.get("transports")
            if not isinstance(transports, list):
                raise RuntimeError("Invalid transport_info['transports'] format")

            zmq_transport = None
            for transport in transports:
                if not isinstance(transport, dict):
                    continue
                if transport.get("transport_type") == "zmq" and transport.get(
                    "enabled", True
                ):
                    zmq_transport = transport
                    break

            if not zmq_transport:
                raise RuntimeError(
                    "Registration response did not include an enabled ZMQ transport."
                )

            zmq_ports = zmq_transport.get("ports")
            if not isinstance(zmq_ports, dict):
                raise RuntimeError("Invalid ZMQ transport ports format")

            registration_port = zmq_ports.get("registration")
            sensory_port = zmq_ports.get("sensory")
            if registration_port is None or sensory_port is None:
                raise RuntimeError(
                    f"Missing required ZMQ ports in registration response: {zmq_ports}"
                )

            # Rust-backed client (single source of truth for ZMQ send semantics)
            try:
                from feagi_rust_py_libs.feagi_rust_py_libs import (  # noqa: PLC0415
                    feagi_agent as rust_sdk,
                )
            except ImportError as e:
                raise ImportError(
                    "Rust SDK (feagi_rust_py_libs) is required for ZMQ transport. "
                    "Install with: pip install feagi_rust_py_libs"
                ) from e

            if not self._agent_id:
                raise RuntimeError("Agent ID missing (register_agent must be called first).")

            if self._agent_capabilities is None:
                raise RuntimeError("Agent capabilities missing (register_agent must be called first).")

            rust_agent_type = rust_sdk.AgentType.sensory()

            config = rust_sdk.PyAgentConfig(self._agent_id, rust_agent_type)
            config.with_registration_endpoint(
                f"tcp://{self._feagi_host}:{int(registration_port)}"
            )
            config.with_sensory_endpoint(
                f"tcp://{self._feagi_host}:{int(sensory_port)}"
            )

            # Enforce deterministic heartbeat config (caller provided explicit interval)
            config.with_heartbeat_interval(float(self._heartbeat_interval_s))

            # Capabilities: map vision when schema matches, store everything else as custom.
            caps = dict(self._agent_capabilities)
            if "vision" in caps:
                vision = caps.get("vision")
                if not isinstance(vision, dict):
                    raise RuntimeError("capabilities['vision'] must be a dict")

                modality = vision.get("modality")
                dims = vision.get("dimensions")
                channels = vision.get("channels")
                cortical_area = vision.get("target_cortical_area")

                if (
                    not isinstance(modality, str)
                    or not isinstance(dims, list)
                    or len(dims) != 2
                    or not isinstance(channels, int)
                    or not isinstance(cortical_area, str)
                ):
                    raise RuntimeError(
                        "Invalid vision capability schema. Expected keys: "
                        "{modality:str, dimensions:[w,h], channels:int, target_cortical_area:str}"
                    )

                config.with_vision_capability(
                    modality,
                    int(dims[0]),
                    int(dims[1]),
                    int(channels),
                    cortical_area,
                )

            for key, value in caps.items():
                if key == "vision":
                    continue
                config.with_custom_capability(key, json.dumps(value))

            # Validate and connect (registers with FEAGI and starts heartbeat in Rust)
            config.validate()
            self._agent_client = rust_sdk.PyAgentClient(config)
            self._agent_client.connect()
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")
        
        self._connected = True
    
    def disconnect(self):
        """Disconnect from FEAGI"""
        # Drop Rust client to trigger deregistration/cleanup in Rust.
        self._agent_client = None
        self._transport = None
        self._zmq_context = None
        self._connected = False
        logger.info("[DISCONN] Disconnected from FEAGI")
    
    def register_input(self, input_instance: 'BaseInput', group_id: Optional[int] = None):
        """
        Register an input (called internally by input classes).
        
        NOTE: In FEAGI 2.0, you must call register_agent() AFTER registering all inputs
        so that the registration includes the correct cortical area information.
        
        Args:
            input_instance: Input instance to register
            group_id: Optional group ID to use. If None, auto-allocates next available ID.
        """
        self._init_cache()
        
        # Use provided group_id or allocate new one
        if group_id is None:
            group_id = self._allocate_group_id()
        else:
            logger.debug(f"Using provided group_id={group_id} for {input_instance.__class__.__name__}")
        
        # Register with Rust cache
        input_instance._register_with_cache(self._cache, group_id)
        input_instance._mark_registered(group_id)
        
        # Add to registry
        self._inputs.append(input_instance)
        
        logger.debug(f"[OK] Registered input: {input_instance.__class__.__name__} (group={group_id})")
        
        # Note: Agent registration should be called after all inputs are registered
        # so that the registration includes complete capability information
    
    def attach_monitor(self, monitor: 'Monitor'):
        """
        Attach an observability monitor.
        
        Args:
            monitor: Monitor instance to attach
        """
        self._monitors.append(monitor)
        logger.debug(f"Attached monitor: {monitor.__class__.__name__}")
    
    def detach_monitor(self, monitor: 'Monitor'):
        """
        Detach an observability monitor.
        
        Args:
            monitor: Monitor instance to detach
        """
        if monitor in self._monitors:
            self._monitors.remove(monitor)
            logger.debug(f"Detached monitor: {monitor.__class__.__name__}")
    
    def _notify_monitors_send_start(self, data: Dict[str, Any]):
        """Notify all monitors of send start."""
        for monitor in self._monitors:
            try:
                monitor.on_send_start(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def _notify_monitors_send_complete(self, data: Dict[str, Any]):
        """Notify all monitors of send complete."""
        for monitor in self._monitors:
            try:
                monitor.on_send_complete(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def send(self):
        """
        Send all input data to FEAGI.
        
        This is the main loop method:
        1. Updates all inputs to cache
        2. Encodes to neurons (Rust - fast!)
        3. Serializes to bytes
        4. Sends via transport
        
        Call this in your main loop after updating all input values.
        
        REQUIRES: Agent registration must succeed before calling this.
        """
        if not self._agent_registered:
            raise RuntimeError(
                "Agent registration required before sending data.\n"
                "Call brain_input.register_agent(agent_id='<agent_descriptor_b64>', ...) first.\n"
                "FEAGI 2.0 requires successful agent registration before any operations."
            )
        
        if not self._connected:
            raise RuntimeError(
                "Not connected to FEAGI. Call brain_input.connect() first."
            )
        
        # Start timing
        start_time = time.perf_counter()
        
        # Notify monitors of send start
        cortical_areas = [inp._cortical_area for inp in self._inputs if hasattr(inp, '_cortical_area')]
        self._notify_monitors_send_start({
            'timestamp': datetime.now(),
            'input_count': len(self._inputs),
            'cortical_areas': cortical_areas
        })
        
        # Reserved for future detailed timing breakdowns
        # (kept disabled to avoid adding overhead in the hot path)
        try:
            # Update all inputs to cache
            for input_instance in self._inputs:
                try:
                    input_instance._write_to_cache(self._cache)
                except Exception as e:
                    logger.error(f"[FAIL] [BRAIN-INPUT] Error writing {input_instance.__class__.__name__} to cache: {e}", exc_info=True)
                    raise
            
            # Encode cached data to bytes
            # Note: Method name was changed to sensors_encode_cached_sensor_data_to_bytes in recent refactor
            try:
                if not hasattr(self._cache, 'sensors_encode_cached_sensor_data_to_bytes'):
                    raise AttributeError(
                        "ConnectorAgent missing required method: sensors_encode_cached_sensor_data_to_bytes()\n"
                        "This method needs to be added to rust-py-libs ConnectorAgent.\n"
                        "See MISSING_RUST_PY_LIBS_API.md for details."
                    )
                
                self._cache.sensors_encode_cached_sensor_data_to_bytes()
            except AttributeError:
                raise
            except Exception as e:
                logger.error(f"[FAIL] [BRAIN-INPUT] Error encoding cached data to bytes: {e}", exc_info=True)
                raise
            
            # Get encoded bytes (Rust)
            # Note: API changed to sensors_read_bytes() which directly returns Vec<u8>
            try:
                if not hasattr(self._cache, 'sensors_read_bytes'):
                    raise AttributeError(
                        "ConnectorAgent missing required method: sensors_read_bytes()\n"
                        "This method needs to be added to rust-py-libs ConnectorAgent.\n"
                        "See MISSING_RUST_PY_LIBS_API.md for details."
                    )
                py_bytes = self._cache.sensors_read_bytes()
                serialized = bytes(py_bytes)
                
                # Logging removed for hot path
            except AttributeError:
                raise
            except Exception as e:
                logger.error(f"[FAIL] [BRAIN-INPUT] Error reading sensor bytes: {e}", exc_info=True)
                raise
            
            # Check if we have data
            if not serialized:
                logger.warning("[WARN] [BRAIN-INPUT] No sensor data to send (serialized data is empty)")
                return
            
            # Send via transport
            if self._agent_client is None:
                logger.error(
                    "Rust agent client not initialized - cannot send %d inputs",
                    len(self._inputs),
                )
                raise RuntimeError("Rust agent client not initialized. Call connect() first.")

            # Real-time: Rust client drops on backpressure (no blocking, no buffering).
            self._agent_client.send_sensory_bytes(serialized)
            
            # Calculate metrics
            duration_ms = (time.perf_counter() - start_time) * 1000.0
            packet_size = len(serialized) if serialized else 0
            
            # Estimate neuron count (this is approximate)
            neuron_count = packet_size // 4  # Rough estimate
            
            # Notify monitors of send complete
            self._notify_monitors_send_complete({
                'timestamp': datetime.now(),
                'packet_size_bytes': packet_size,
                'neuron_count': neuron_count,
                'duration_ms': duration_ms,
                'cortical_areas': cortical_areas
            })
            
        except Exception as e:
            # Notify monitors of error
            for monitor in self._monitors:
                try:
                    monitor.on_error(e, {'operation': 'send', 'stage': 'processing'})
                except Exception:
                    pass
            raise
    
    def send_raw_bytes(self, binary_data: bytes):
        """
        Send raw binary data directly to FEAGI via ZMQ.
        
        This method is for advanced use cases where you're sending pre-serialized
        FEAGI Byte Container (FBC) data directly, bypassing the normal sensor
        encoding pipeline.
        
        Args:
            binary_data: Raw FBC-formatted bytes to send
            
        Example:
            # Spike Train Generator use case
            data = CorticalMappedXYZPNeuronVoxels()
            # ... populate data ...
            binary = data.serialize_to_bytes()
            brain_input.send_raw_bytes(binary)
        
        REQUIRES: Agent must be registered and connected.
        """
        if not self._agent_registered:
            raise RuntimeError(
                "Agent registration required before sending data.\n"
                "Call brain_input.register_agent(agent_id='<agent_descriptor_b64>', ...) first."
            )
        
        if not self._connected:
            raise RuntimeError(
                "Not connected to FEAGI. Call brain_input.connect() first."
            )
        
        if not binary_data:
            logger.warning("[WARN] [BRAIN-INPUT] No data to send (binary_data is empty)")
            return
        
        # Send via transport
        if self._agent_client is None:
            raise RuntimeError("Rust agent client not initialized. Call connect() first.")

        self._agent_client.send_sensory_bytes(binary_data)
    
    def get_input_count(self) -> int:
        """Get number of registered inputs"""
        return len(self._inputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected
    
    def is_registered(self) -> bool:
        """Check if agent is registered with FEAGI (REQUIRED in FEAGI 2.0)"""
        return self._agent_registered
    
    def get_registration_info(self) -> Optional[Dict[str, Any]]:
        """Get registration response information"""
        if not self._agent_registered or self._registration_response is None:
            return None
        
        return {
            "agent_id": self._agent_id,
            "success": self._registration_response.success,
            "message": self._registration_response.message,
            "cortical_areas": self._registration_response.cortical_areas,
            "transport_info": self._registration_response.transport_info,
        }


# Global singleton instance
brain_input = BrainInput()

