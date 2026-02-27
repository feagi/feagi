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

# Cortical ID subtype (bytes 1-3) -> candidate sensor register method names
# Rust macro exposes PascalCase; try both for compatibility
_SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES = {
    b"svi": ["sensor_segmented_vision_register", "sensor_SegmentedVision_register"],
    b"img": ["sensor_vision_register", "sensor_Vision_register"],
    b"mis": ["sensor_misc_data_register", "sensor_MiscData_register"],
}


def decode_cortical_id_to_subtype(cortical_area_id: str) -> bytes:
    """
    Decode base64 cortical ID to subtype bytes (bytes 1-3).

    Cortical ID layout: byte 0 = category (i/o), bytes 1-3 = unit identifier.
    e.g. aXN2aQkAAAA= (vision_LL) -> b'svi' (SegmentedVision)

    Args:
        cortical_area_id: Base64-encoded cortical ID string.

    Returns:
        3-byte subtype (e.g. b'svi', b'img', b'mis').
    """
    decoded = base64.b64decode(cortical_area_id)
    if len(decoded) < 4:
        raise ValueError(
            f"Cortical ID must be at least 4 bytes, got {len(decoded)}"
        )
    return bytes(decoded[1:4])


def register_cortical_areas_with_cache(cache: Any, cortical_area_ids: List[str]) -> None:
    """
    Register cortical areas with ConnectorAgent cache by decoding each ID
    to subtype and calling the appropriate sensor_*_register method.

    Groups areas by subtype; each unique subtype gets one registration (group_id).
    e.g. vision_LL (aXN2aQkAAAA=) -> svi -> sensor_SegmentedVision_register

    Args:
        cache: ConnectorAgent instance from feagi_rust_py_libs.
        cortical_area_ids: List of base64 cortical area IDs to register.
    """
    import feagi_rust_py_libs as frpl

    subtype_to_ids: Dict[bytes, List[str]] = {}
    for cid in cortical_area_ids:
        subtype = decode_cortical_id_to_subtype(cid)
        if subtype not in subtype_to_ids:
            subtype_to_ids[subtype] = []
        subtype_to_ids[subtype].append(cid)

    cc_desc = frpl.connector_core.data_types.descriptors
    cc_data_types = frpl.connector_core.data_types
    frame_enum = frpl.data_structures.genomic.cortical_area.FrameChangeHandling
    frame = frame_enum.Absolute()

    for group_id, (subtype, ids) in enumerate(subtype_to_ids.items()):
        candidates = _SUBTYPE_TO_SENSOR_REGISTER_CANDIDATES.get(subtype)
        if candidates is None:
            subtype_str = subtype.decode("ascii", errors="replace")
            logger.warning(
                "Unknown cortical subtype '%s' (from %s), skipping cache registration",
                subtype_str,
                ids[0][:12],
            )
            continue

        method_name = None
        for candidate in candidates:
            if hasattr(cache, candidate):
                method_name = candidate
                break
        if method_name is None:
            subtype_str = subtype.decode("ascii", errors="replace")
            raise AttributeError(
                f"ConnectorAgent has no sensor registration for subtype '{subtype_str}' "
                f"(tried: {candidates}). Rebuild feagi-rust-py-libs from source: "
                "cd feagi-rust-py-libs && maturin develop --release"
            )

        register_method = getattr(cache, method_name)
        if subtype == b"svi":
            input_props = cc_desc.ImageFrameProperties(
                cc_desc.ImageXYResolution(32, 32),
                cc_desc.ColorSpace.Gamma,
                cc_desc.ColorChannelLayout.RGB,
            )
            center_res = cc_desc.ImageXYResolution(32, 32)
            peripheral_res = cc_desc.ImageXYResolution(32, 32)
            segmented_res = cc_desc.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
                center_res, peripheral_res
            )
            segmented_props = cc_desc.SegmentedImageFrameProperties(
                segmented_res,
                input_props.channel_layout,
                input_props.channel_layout,
                input_props.color_space,
            )
            pct = cc_data_types.Percentage.new_from_0_1
            gaze = cc_data_types.GazeProperties(
                cc_data_types.Percentage2D(pct(0.5), pct(0.5)),
                pct(1.0),
            )
            register_method(
                group=group_id,
                number_channels=1,
                frame_change_handling=frame,
                input_image_properties=input_props,
                segmented_image_properties=segmented_props,
                initial_gaze=gaze,
            )
        elif subtype == b"img":
            input_props = cc_desc.ImageFrameProperties(
                cc_desc.ImageXYResolution(32, 32),
                cc_desc.ColorSpace.Gamma,
                cc_desc.ColorChannelLayout.RGB,
            )
            register_method(
                group=group_id,
                number_channels=1,
                frame_change_handling=frame,
                image_properties=input_props,
            )
        elif subtype == b"mis":
            dims = cc_desc.MiscDataDimensions(1, 1, 1)
            register_method(
                group=group_id,
                number_channels=1,
                frame_change_handling=frame,
                misc_data_dimensions=dims,
            )
        else:
            subtype_str = subtype.decode("ascii", errors="replace")
            raise NotImplementedError(
                f"Cache registration for subtype '{subtype_str}' not implemented"
            )

        subtype_str = subtype.decode("ascii", errors="replace")
        logger.info(
            "Registered cortical area(s) with cache: subtype=%s, group=%d, method=%s",
            subtype_str,
            group_id,
            method_name,
        )


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
        
        # Configure and connect (all params required; see examples/video_streamer)
        brain_input.configure(
            feagi_host="localhost",
            feagi_port=5558,
            motor_port=5564,
            transport="zmq",
            api_port=8000,
            feagi_http_timeout_s=10.0,
            heartbeat_interval_s=5.0,
            heartbeat_join_timeout_s=2.0,
        )
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
        self._rust_logging_initialized = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Configuration
        # @safety: No implicit defaults. Commercial deployments must be explicitly configured.
        self._feagi_host: Optional[str] = None
        self._feagi_port: Optional[int] = None
        self._transport_type: Optional[str] = None
        self._api_port: Optional[int] = None  # FEAGI API port for registration
        self._registration_port: Optional[int] = None
        self._feagi_http_timeout_s: Optional[float] = None
        self._heartbeat_interval_s: Optional[float] = None
        self._heartbeat_join_timeout_s: Optional[float] = None
        self._connection_timeout_ms: Optional[int] = None
        self._registration_retries: Optional[int] = None
        self._auth_token_b64: Optional[str] = None
        self._feagi_motor_port: Optional[int] = None
        
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
            
            # Initialize Rust tracing logging once per BrainInput instance.
            if not self._rust_logging_initialized:
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
                    self._rust_logging_initialized = True
                except BaseException as log_init_err:
                    if "global default trace dispatcher has already been set" in str(
                        log_init_err
                    ):
                        self._rust_logging_initialized = True
                        logger.debug(
                            "Rust tracing subscriber already initialized in process."
                        )
                    else:
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

    def register_cortical_areas(self, cortical_area_ids: List[str]) -> None:
        """
        Register cortical areas with the ConnectorAgent cache.

        Decodes each base64 cortical ID to subtype and calls the appropriate
        sensor_*_register method. Use for agents that send to specific cortical
        areas (e.g. spike train, MuJoCo) without using standard inputs like Camera.

        Args:
            cortical_area_ids: List of base64 cortical area IDs to register.
        """
        self._init_cache()
        if self._cache is not None:
            register_cortical_areas_with_cache(self._cache, cortical_area_ids)

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

    def _parse_agent_descriptor_fields(
        self,
        agent_descriptor_b64: str,
    ) -> tuple[str, str, int]:
        """
        Parse a base64 AgentDescriptor (48 bytes) into descriptor fields.

        Expected byte layout:
          - [0:4] instance id (unused)
          - [4:24] manufacturer (null-padded ASCII)
          - [24:44] agent name (null-padded ASCII)
          - [44:48] version (little-endian u32)
        """
        raw = base64.b64decode(agent_descriptor_b64, validate=True)
        if len(raw) != 48:
            raise RuntimeError(
                "agent_descriptor_b64 must decode to 48 bytes (AgentDescriptor)."
            )

        manufacturer = raw[4:24].decode("ascii").rstrip("\x00").strip()
        agent_name = raw[24:44].decode("ascii").rstrip("\x00").strip()
        agent_version = int.from_bytes(raw[44:48], byteorder="little")

        if not manufacturer:
            raise RuntimeError("Agent descriptor manufacturer cannot be empty.")
        if not agent_name:
            raise RuntimeError("Agent descriptor agent_name cannot be empty.")
        if agent_version <= 0:
            raise RuntimeError("Agent descriptor version must be > 0.")

        return manufacturer, agent_name, agent_version

    def _resolve_required_auth_token_b64(self) -> str:
        """
        Resolve the auth token required by Rust PyAgentConfig.
        """
        token = self._auth_token_b64 or os.environ.get("FEAGI_AUTH_TOKEN_B64")
        if not token:
            raise RuntimeError(
                "Missing auth token base64. Provide auth_token_b64 in brain_input.configure(...) "
                "or set FEAGI_AUTH_TOKEN_B64 before connect()."
            )
        return token

    def _resolve_required_connection_timeout_ms(self) -> int:
        """
        Resolve the connection timeout required by Rust PyAgentConfig.
        """
        timeout_ms = self._connection_timeout_ms
        if timeout_ms is None:
            env_value = os.environ.get("FEAGI_CONNECTION_TIMEOUT_MS")
            if env_value is not None:
                timeout_ms = int(env_value)
        if timeout_ms is None or timeout_ms <= 0:
            raise RuntimeError(
                "Missing connection timeout. Provide connection_timeout_ms in brain_input.configure(...) "
                "or set FEAGI_CONNECTION_TIMEOUT_MS to a positive integer."
            )
        return timeout_ms

    def _resolve_required_registration_retries(self) -> int:
        """
        Resolve the registration retries required by Rust PyAgentConfig.
        """
        retries = self._registration_retries
        if retries is None:
            env_value = os.environ.get("FEAGI_REGISTRATION_RETRIES")
            if env_value is not None:
                retries = int(env_value)
        if retries is None or retries <= 0:
            raise RuntimeError(
                "Missing registration retries. Provide registration_retries in brain_input.configure(...) "
                "or set FEAGI_REGISTRATION_RETRIES to a positive integer."
            )
        return retries
    
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
        motor_port: int,
        registration_port: Optional[int] = None,
        transport: str,
        api_port: int,
        feagi_http_timeout_s: float,
        heartbeat_interval_s: float,
        heartbeat_join_timeout_s: float,
        connection_timeout_ms: Optional[int] = None,
        registration_retries: Optional[int] = None,
        auth_token_b64: Optional[str] = None,
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            feagi_host: FEAGI server hostname or IP
            feagi_port: Sensory input port (default: 5558)
            motor_port: Motor output port (required by FeagiAgentClient)
            transport: Transport type - "zmq" or "websocket"
            api_port: FEAGI API port for registration (default: 8000)
        """
        if not feagi_host:
            raise ValueError("feagi_host must be provided (no defaults in safety mode).")
        if feagi_port <= 0:
            raise ValueError("feagi_port must be a positive integer (no defaults in safety mode).")
        if motor_port <= 0:
            raise ValueError("motor_port must be a positive integer (no defaults in safety mode).")
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
        self._feagi_motor_port = motor_port
        self._registration_port = registration_port
        self._transport_type = transport
        self._api_port = api_port
        self._feagi_http_timeout_s = feagi_http_timeout_s
        self._heartbeat_interval_s = heartbeat_interval_s
        self._heartbeat_join_timeout_s = heartbeat_join_timeout_s
        self._connection_timeout_ms = connection_timeout_ms
        self._registration_retries = registration_retries
        self._auth_token_b64 = auth_token_b64
        
        logger.info("[CFG] Configured: %s://%s:%s (API: %s:%s)", transport, feagi_host, feagi_port, feagi_host, api_port)

    def _derive_capabilities_from_inputs(self) -> Dict[str, Any]:
        """
        Derive structured capabilities from registered inputs for runtime ZMQ
        registration.
        """
        if not self._inputs:
            raise RuntimeError(
                "No registered inputs found. Register inputs before connect()."
            )
        vision_input = next(
            (
                input_item
                for input_item in self._inputs
                if hasattr(input_item, "__class__")
                and "Camera" in input_item.__class__.__name__
            ),
            None,
        )
        if vision_input is None:
            raise RuntimeError(
                "No supported sensory input type found for capability derivation."
            )
        resolution = getattr(vision_input, "resolution", None)
        group_id = getattr(vision_input, "group_id", None)
        if (
            resolution is None
            or not isinstance(resolution, tuple)
            or len(resolution) != 2
            or group_id is None
        ):
            raise RuntimeError(
                "Registered Camera input is missing required resolution/group metadata."
            )
        return {
            "vision": {
                "modality": "camera",
                "dimensions": [int(resolution[0]), int(resolution[1])],
                "channels": 3,
                "unit": "vision",
                "group": int(group_id),
            }
        }
    
    def register_agent(
        self,
        agent_id: str,
        agent_type: str = "sensory",
        capabilities: Optional[Dict[str, Any]] = None,
        agent_version: Optional[str] = None,
        controller_version: Optional[str] = None,
    ) -> bool:
        """
        Cache registration metadata for unified transport registration flow.

        Runtime registration now happens through `FeagiAgentClient.connect()`
        over the configured registration transport port (ZMQ/WebSocket), not
        through REST registration.
        """
        if not agent_id:
            raise ValueError(
                "agent_id must be provided as base64 AgentDescriptor."
            )
        self._agent_id = self._validate_agent_descriptor_b64(agent_id)
        self._agent_type = agent_type
        if capabilities is None:
            capabilities = self._derive_capabilities_from_inputs()
        self._agent_capabilities = capabilities
        self._agent_registered = False
        self._registration_response = None
        _ = agent_version
        _ = controller_version
        logger.info(
            "[REG] Cached agent metadata for transport registration: agent_id=%s, agent_type=%s",
            self._agent_id,
            self._agent_type,
        )
        return True
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Initializes transport and establishes connection.
        
        Uses direct Rust client registration over ZMQ/WebSocket registration
        endpoint.
        """
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        # Initialize transport
        if self._transport_type is None or self._feagi_host is None or self._feagi_port is None:
            raise RuntimeError(
                "brain_input.configure(...) must be called with explicit FEAGI host/port/transport "
                "before connect() (no defaults in safety mode)."
            )

        if self._transport_type == "zmq":
            from feagi.pns.client import AgentType, FeagiAgentClient  # noqa: PLC0415

            motor_port = self._feagi_motor_port
            if motor_port is None or motor_port <= 0:
                raise RuntimeError(
                    "Missing motor port. Provide motor_port in brain_input.configure(...) "
                    "(required by FeagiAgentClient even for sensory-only agents)."
                )
            registration_port = self._registration_port
            if registration_port is None:
                env_registration_port = os.environ.get("FEAGI_REGISTRATION_PORT")
                if env_registration_port is not None:
                    registration_port = int(env_registration_port)
            if registration_port is None or registration_port <= 0:
                raise RuntimeError(
                    "Missing ZMQ registration port. Provide registration_port in "
                    "brain_input.configure(...) or set FEAGI_REGISTRATION_PORT."
                )

            caps = (
                dict(self._agent_capabilities)
                if self._agent_capabilities is not None
                else self._derive_capabilities_from_inputs()
            )
            vision_cap = caps.get("vision")
            if not isinstance(vision_cap, dict):
                raise RuntimeError(
                    "Vision capability is required for brain_input sensory registration."
                )
            modality = vision_cap.get("modality")
            dims = vision_cap.get("dimensions")
            channels = vision_cap.get("channels")
            unit = vision_cap.get("unit")
            group = vision_cap.get("group")
            if (
                not isinstance(modality, str)
                or not isinstance(dims, list)
                or len(dims) != 2
                or not isinstance(channels, int)
                or not isinstance(unit, str)
                or not isinstance(group, int)
            ):
                raise RuntimeError(
                    "Vision capability must define modality/dimensions/channels/unit/group for transport registration."
                )

            if not self._agent_id:
                self._agent_id = self._resolve_agent_descriptor_b64()

            auth_token_b64 = self._resolve_required_auth_token_b64()
            connection_timeout_ms = self._resolve_required_connection_timeout_ms()
            registration_retries = self._resolve_required_registration_retries()

            custom_capabilities = {
                key: value for key, value in caps.items() if key != "vision"
            }
            client = FeagiAgentClient(self._agent_id, AgentType.SENSORY)
            client.configure(
                feagi_host=self._feagi_host,
                registration_port=int(registration_port),
                sensory_port=int(self._feagi_port),
                motor_port=int(motor_port),
                vision_unit=(
                    modality,
                    int(dims[0]),
                    int(dims[1]),
                    int(channels),
                    unit,
                    int(group),
                ),
                custom_capabilities=custom_capabilities or None,
                heartbeat_interval=float(self._heartbeat_interval_s),
                connection_timeout_ms=int(connection_timeout_ms),
                registration_retries=int(registration_retries),
                agent_descriptor_b64=self._agent_id,
                auth_token_b64=auth_token_b64,
            )
            client.connect()
            self._agent_client = client
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")
        
        self._agent_registered = True
        self._connected = True
    
    def disconnect(self):
        """Disconnect from FEAGI"""
        if self._agent_client is not None:
            try:
                self._agent_client.disconnect()
            except Exception as disconnect_error:
                logger.warning(
                    "Rust agent client disconnect failed: %s",
                    disconnect_error,
                )
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

