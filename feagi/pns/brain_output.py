"""
FEAGI Brain Output Manager

Global manager for all FEAGI outputs (motor/action targets).
Uses Rust MotorDeviceCache for high-performance decoding.
"""

from typing import Callable, List, Optional, TYPE_CHECKING, Dict, Any
import base64
import binascii
import json
import logging
import os
import sys

if TYPE_CHECKING:
    from feagi.pns.outputs.base import BaseOutput
    from feagi.pns.observability.monitor import Monitor

# Configure SDK logger with console handler (ensures output is visible)
logger = logging.getLogger("feagi.pns.brain_output")
if not logger.handlers:
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False


class BrainOutput:
    """
    Global brain output manager.
    
    Manages all registered outputs and handles automatic receiving
    and decoding of motor commands from FEAGI. Uses Rust IOCache.
    
    This is a singleton - use the module-level `brain_output` instance.
    
    Example:
        from feagi.pns.outputs import ServoMotor
        from feagi.pns import brain_output
        
        # Register outputs
        servo = ServoMotor.register(range=(0, 180))
        
        # Configure and connect
        brain_output.configure(feagi_host="localhost")
        brain_output.connect()
        
        # Main loop
        while True:
            brain_output.receive()  # Receives and decodes all outputs
            angle = servo.get_angle()
    """
    
    def __init__(self):
        # Rust MotorDeviceCache (lazy-initialized)
        self._cache = None
        self._cache_available = False
        
        # Registry of all outputs
        self._outputs: List['BaseOutput'] = []
        
        # Transport (ZMQ/WebSocket)
        self._client = None
        self._connected = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Channel index counter for motors (all motors use group=0, differentiated by channel)
        self._next_motor_channel = 0
        self._next_motor_channel_by_group: Dict[int, int] = {}
        
        # Motor decoder tracking
        self._motor_total_channels = 0
        self._motor_decoder_registered = False
        #: True when ``connect()`` used :class:`~feagi.pns.client.AgentType.SENSORY` (no motor OPUs).
        self._sensory_only_mode: bool = False

        # Latest motor data (for debugging)
        self._motor_data: Dict[str, Any] = {}
        self._last_logged_motor_values: Dict[str, float] = {}
        
        # Configuration
        self._agent_id = None
        # @safety: No implicit defaults. Commercial deployments must be explicitly configured.
        self._feagi_host: Optional[str] = None
        self._feagi_registration_port: Optional[int] = None
        self._feagi_sensory_port: Optional[int] = None
        self._feagi_motor_port: Optional[int] = None
        self._transport_type: Optional[str] = None
        self._feagi_connection_timeout_ms: Optional[int] = None
        self._feagi_registration_retries: Optional[int] = None
        self._feagi_heartbeat_interval_s: Optional[float] = None
        self._feagi_api_port: Optional[int] = None
        self._feagi_http_timeout_s: Optional[float] = None
        self._auth_token_b64: Optional[str] = None
        self._vision_unit: Optional[tuple[str, int, int, int, str, int]] = None
        self._vision_units: list[tuple[str, int, int, int, str, int]] = []

        # Motor output mapping (channel -> output instance)
        self._motor_outputs_by_channel: Dict[int, 'BaseOutput'] = {}
        self._motor_outputs_by_group_channel: Dict[tuple[int, int], 'BaseOutput'] = {}
        self._device_registration_enricher: Optional[
            Callable[[Dict[str, Any]], Dict[str, Any]]
        ] = None

        # Lazy sensory write helpers (created only when needed)
        self._sensory_percentage_factory = None
        self._sensory_misc_factory = None
        self._vision_image_frame_factory = None
        self._vision_color_space = None
        self._vision_memory_layout = None
        
        # Observability monitors
        self._monitors: List['Monitor'] = []

    def set_device_registration_enricher(
        self, enricher: Callable[[Dict[str, Any]], Dict[str, Any]]
    ) -> None:
        """
        Register a deterministic enricher for exported device_registrations JSON.

        The enricher receives decoded capability JSON and must return the updated payload.
        It is applied immediately before registration payload validation and transmission.
        """
        self._device_registration_enricher = enricher

    @staticmethod
    def _decode_device_property_text(value: Any) -> Optional[str]:
        if isinstance(value, str):
            normalized = value.strip()
            return normalized or None
        if isinstance(value, dict):
            value_type = value.get("type")
            value_content = value.get("value")
            if value_type == "String" and isinstance(value_content, str):
                normalized = value_content.strip()
                return normalized or None
        return None

    @classmethod
    def _normalize_device_property_value(cls, value: Any) -> Any:
        if isinstance(value, dict):
            if "type" in value and "value" in value:
                if value["type"] == "Dictionary" and isinstance(value["value"], dict):
                    return {
                        "type": "Dictionary",
                        "value": {
                            key: cls._normalize_device_property_value(nested)
                            for key, nested in value["value"].items()
                        },
                    }
                return value
            return {
                "type": "Dictionary",
                "value": {
                    key: cls._normalize_device_property_value(nested)
                    for key, nested in value.items()
                },
            }
        if isinstance(value, bool):
            return {"type": "Integer", "value": int(value)}
        if isinstance(value, int):
            return {"type": "Integer", "value": value}
        if isinstance(value, float):
            return {"type": "Float", "value": value}
        return {"type": "String", "value": str(value)}

    @classmethod
    def _normalize_device_registration_properties(cls, registrations: Dict[str, Any]) -> Dict[str, Any]:
        def normalize_section(section_key: str) -> None:
            section = registrations.get(section_key)
            if not isinstance(section, dict):
                return
            for unit_entries in section.values():
                if not isinstance(unit_entries, list):
                    continue
                for entry in unit_entries:
                    if not isinstance(entry, list) or not entry:
                        continue
                    unit_def = entry[0]
                    if not isinstance(unit_def, dict):
                        continue
                    grouping = unit_def.get("device_grouping")
                    if not isinstance(grouping, list):
                        continue
                    for channel in grouping:
                        if not isinstance(channel, dict):
                            continue
                        props = channel.get("device_properties")
                        if not isinstance(props, dict):
                            continue
                        channel["device_properties"] = {
                            key: cls._normalize_device_property_value(raw)
                            for key, raw in props.items()
                        }

        normalize_section("output_units_and_decoder_properties")
        normalize_section("input_units_and_encoder_properties")
        return registrations

    @staticmethod
    def _validate_non_empty_text(
        value: Any, field_path: str, errors: List[str]
    ) -> Optional[str]:
        normalized = BrainOutput._decode_device_property_text(value)
        if normalized is None:
            errors.append(f"{field_path} must be a string")
            return None
        return normalized

    def _validate_registration_units(
        self, section: Any, section_name: str, errors: List[str]
    ) -> None:
        if section is None:
            return
        if not isinstance(section, dict):
            errors.append(f"{section_name} must be an object")
            return

        required_device_properties = (
            "bundle_type",
            "bundle_id",
            "modality",
            "signal_type",
            "source_model",
            "source_entity",
        )
        seen_unit_names: set[str] = set()

        for unit_key in sorted(section):
            unit_entries = section.get(unit_key)
            if not isinstance(unit_entries, list):
                errors.append(f"{section_name}.{unit_key} must be an array")
                continue
            for unit_idx, entry in enumerate(unit_entries):
                unit_path = f"{section_name}.{unit_key}[{unit_idx}]"
                if not isinstance(entry, list) or not entry:
                    errors.append(
                        f"{unit_path} must be [unit_definition, coder_properties]"
                    )
                    continue
                unit_def = entry[0]
                if not isinstance(unit_def, dict):
                    errors.append(f"{unit_path}[0] must be an object")
                    continue

                unit_name = self._validate_non_empty_text(
                    unit_def.get("friendly_name"),
                    f"{unit_path}[0].friendly_name",
                    errors,
                )
                if unit_name:
                    scoped_name = (
                        f"{section_name}:{unit_key}:"
                        f"{unit_def.get('cortical_unit_index')}:{unit_name.lower()}"
                    )
                    if scoped_name in seen_unit_names:
                        errors.append(
                            f"Duplicate unit friendly_name '{unit_name}' in {section_name}.{unit_key}"
                        )
                    seen_unit_names.add(scoped_name)

                grouping = unit_def.get("device_grouping")
                if not isinstance(grouping, list) or not grouping:
                    errors.append(f"{unit_path}[0].device_grouping must be a non-empty array")
                    continue

                seen_channel_names: set[str] = set()
                for channel_idx, channel in enumerate(grouping):
                    channel_path = f"{unit_path}[0].device_grouping[{channel_idx}]"
                    if not isinstance(channel, dict):
                        errors.append(f"{channel_path} must be an object")
                        continue
                    channel_name = self._validate_non_empty_text(
                        channel.get("friendly_name"),
                        f"{channel_path}.friendly_name",
                        errors,
                    )
                    if channel_name:
                        key = channel_name.lower()
                        if key in seen_channel_names:
                            errors.append(
                                f"Duplicate channel friendly_name '{channel_name}' in {unit_path}"
                            )
                        seen_channel_names.add(key)

                    device_properties = channel.get("device_properties")
                    if not isinstance(device_properties, dict):
                        errors.append(f"{channel_path}.device_properties must be an object")
                        continue
                    for required_key in required_device_properties:
                        self._validate_non_empty_text(
                            device_properties.get(required_key),
                            f"{channel_path}.device_properties.{required_key}",
                            errors,
                        )

    def _validate_device_registration_contract(
        self, registrations: Dict[str, Any]
    ) -> Dict[str, Any]:
        errors: List[str] = []
        self._validate_registration_units(
            registrations.get("output_units_and_decoder_properties"),
            "output_units_and_decoder_properties",
            errors,
        )
        self._validate_registration_units(
            registrations.get("input_units_and_encoder_properties"),
            "input_units_and_encoder_properties",
            errors,
        )
        if errors:
            raise RuntimeError(
                "Invalid device_registrations contract:\n- " + "\n- ".join(errors)
            )
        return registrations
    
    def _init_cache(self):
        """Initialize Rust ConnectorAgent (lazy)."""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            # Use ConnectorAgent which provides motor methods
            # NOTE: frpl.connector_core.caching.MotorDeviceCache() does not exist in current API
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
                "Rust SDK (feagi_rust_py_libs) is required for brain_output.\n"
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
            "or set FEAGI_AGENT_DESCRIPTOR_B64 before registering outputs."
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

    def _collect_motor_cortical_ids(self) -> List[str]:
        """Collect motor cortical IDs for registration."""
        cortical_ids = set()
        for output in self._outputs:
            if hasattr(output, '_get_cortical_id'):
                cortical_ids.add(output._get_cortical_id())
            else:
                # Construct cortical ID matching Rust cache registration
                from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
                group_id = int(getattr(output, "group_id", 0) or 0)
                if isinstance(output, ServoMotor):
                    # PositionalServo uses Percentage in registration payload.
                    # FEAGI auto-create yields both Absolute and Incremental areas.
                    # CorticalID: [o,p,s,e, config_lo, config_hi, sub_unit, unit_index]
                    for sub in (0, 1):  # 0=Absolute, 1=Incremental
                        cfg = 1 | (sub << 8)  # variant=1 (Percentage), frame=sub
                        cid_bytes = bytes(
                            [
                                111,
                                112,
                                115,
                                101,
                                cfg & 0xFF,
                                (cfg >> 8) & 0xFF,
                                sub,
                                group_id & 0xFF,
                            ]
                        )
                        cortical_ids.add(base64.b64encode(cid_bytes).decode())
                elif isinstance(output, RotaryMotor):
                    # RotaryMotor: construct similarly (may need adjustment based on actual requirements)
                    # For now, use default "omot" format
                    cid_bytes = bytes([111, 109, 111, 116, 0, 0, 0, group_id & 0xFF])
                    cortical_ids.add(base64.b64encode(cid_bytes).decode())

        # If no specific IDs, use default PositionalServo Percentage areas
        if not cortical_ids:
            for sub in (0, 1):  # 0=Absolute, 1=Incremental
                cfg = 1 | (sub << 8)
                cid_bytes = bytes(
                    [111, 112, 115, 101, cfg & 0xFF, (cfg >> 8) & 0xFF, sub, 0]
                )
                cortical_ids.add(base64.b64encode(cid_bytes).decode())

        return list(cortical_ids)

    def _collect_motor_unit_specs(self) -> List[tuple[str, int]]:
        """Collect motor unit specs (unit, group) for registration."""
        from feagi.pns.outputs.motor import ServoMotor, RotaryMotor

        unit_specs: set[tuple[str, int]] = set()
        for output in self._outputs:
            if isinstance(output, ServoMotor):
                unit_specs.add(("positional_servo", int(getattr(output, "group_id", 0) or 0)))
            elif isinstance(output, RotaryMotor):
                unit_specs.add(("rotary_motor", int(getattr(output, "group_id", 0) or 0)))
        return sorted(unit_specs)
    
    def _allocate_group_id(self) -> int:
        """Allocate next cortical group ID"""
        group_id = self._next_group_id
        self._next_group_id += 1
        return group_id
    
    def _register_motor_decoder(self):
        """Register motor decoder with Rust cache (called once with total channel count)"""
        import feagi_rust_py_libs as frpl
        from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
        frame_mode = frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute()
        positioning = frpl.data_structures.genomic.cortical_area.PercentageNeuronPositioning.Linear()
        z_neuron_resolution = 10

        servo_counts_by_group: Dict[int, int] = {}
        rotary_counts_by_group: Dict[int, int] = {}
        for output in self._outputs:
            if not isinstance(output, (ServoMotor, RotaryMotor)):
                continue
            group_id = int(getattr(output, "group_id", 0) or 0)
            if isinstance(output, ServoMotor):
                servo_counts_by_group[group_id] = servo_counts_by_group.get(group_id, 0) + 1
            else:
                rotary_counts_by_group[group_id] = rotary_counts_by_group.get(group_id, 0) + 1

        for group_id, count in sorted(servo_counts_by_group.items()):
            if count <= 0:
                continue
            self._cache.motor_positional_servo_register(
                group_id,
                count,
                frame_mode,
                z_neuron_resolution,
                positioning,
            )

        for group_id, count in sorted(rotary_counts_by_group.items()):
            if count <= 0:
                continue
            self._cache.motor_rotary_motor_register(
                group_id,
                count,
                frame_mode,
                z_neuron_resolution,
                positioning,
            )

        for output in self._outputs:
            if isinstance(output, (ServoMotor, RotaryMotor)):
                output._register_with_cache(
                    self._cache,
                    output.group_id,
                    output.channel,
                    decoder_registered=True,
                )
    
    def configure(
        self,
        agent_id: str,
        *,
        feagi_host: str,
        feagi_registration_port: int,
        feagi_sensory_port: int,
        feagi_motor_port: int,
        transport: str,
        feagi_connection_timeout_ms: int,
        feagi_registration_retries: int,
        feagi_heartbeat_interval_s: float,
        feagi_api_port: Optional[int] = None,
        feagi_http_timeout_s: Optional[float] = None,
        auth_token_b64: Optional[str] = None,
        vision_unit: Optional[tuple[str, int, int, int, str, int]] = None,
        vision_units: Optional[list[tuple[str, int, int, int, str, int]]] = None,
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            agent_id: Base64-encoded AgentDescriptor (required for registration)
            feagi_host: FEAGI server hostname or IP
            feagi_registration_port: ZMQ registration/heartbeat port
            feagi_sensory_port: ZMQ sensory port (required by SDK config)
            feagi_motor_port: ZMQ motor output port
            transport: Transport type - "zmq" or "websocket"
        """
        if not agent_id:
            raise ValueError("agent_id must be provided (no defaults in safety mode).")
        if not feagi_host:
            raise ValueError("feagi_host must be provided (no defaults in safety mode).")
        if feagi_registration_port <= 0:
            raise ValueError("feagi_registration_port must be a positive integer (no defaults in safety mode).")
        if feagi_sensory_port <= 0:
            raise ValueError("feagi_sensory_port must be a positive integer (no defaults in safety mode).")
        if feagi_motor_port <= 0:
            raise ValueError("feagi_motor_port must be a positive integer (no defaults in safety mode).")
        if not transport:
            raise ValueError("transport must be provided (no defaults in safety mode).")
        if feagi_connection_timeout_ms <= 0:
            raise ValueError("feagi_connection_timeout_ms must be > 0 (no defaults in safety mode).")
        if feagi_registration_retries <= 0:
            raise ValueError("feagi_registration_retries must be > 0 (no defaults in safety mode).")
        if feagi_heartbeat_interval_s <= 0:
            raise ValueError("feagi_heartbeat_interval_s must be > 0 (no defaults in safety mode).")

        self._agent_id = agent_id
        self._init_cache()
        self._feagi_host = feagi_host
        self._feagi_registration_port = feagi_registration_port
        self._feagi_sensory_port = feagi_sensory_port
        self._feagi_motor_port = feagi_motor_port
        self._transport_type = transport
        self._feagi_connection_timeout_ms = feagi_connection_timeout_ms
        self._feagi_registration_retries = feagi_registration_retries
        self._feagi_heartbeat_interval_s = feagi_heartbeat_interval_s
        self._feagi_api_port = feagi_api_port
        self._feagi_http_timeout_s = feagi_http_timeout_s
        self._auth_token_b64 = auth_token_b64
        self._vision_unit = vision_unit
        self._vision_units = list(vision_units or [])
        if vision_unit is not None:
            self._vision_units.append(vision_unit)

        logger.info(
            "[CFG] Configured: agent=%s, %s://%s (registration=%s, motor=%s)",
            agent_id,
            transport,
            feagi_host,
            feagi_registration_port,
            feagi_motor_port,
        )
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Registers agent with FEAGI PNS and establishes transport connection.
        """
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        if not self._agent_id:
            raise RuntimeError(
                "Agent ID not set. Call configure(agent_id='<agent_descriptor_b64>') first."
            )
        if (
            self._feagi_host is None
            or self._feagi_registration_port is None
            or self._feagi_sensory_port is None
            or self._feagi_motor_port is None
        ):
            raise RuntimeError(
                "brain_output.configure(...) must be called with explicit FEAGI host/ports "
                "before connect() (no defaults in safety mode)."
            )
        if self._transport_type is None:
            raise RuntimeError(
                "brain_output.configure(...) must be called with explicit transport "
                "before connect() (no defaults in safety mode)."
            )
        if (
            self._feagi_connection_timeout_ms is None
            or self._feagi_registration_retries is None
            or self._feagi_heartbeat_interval_s is None
        ):
            raise RuntimeError(
                "brain_output.configure(...) must be called with explicit ZMQ timing "
                "before connect() (no defaults in safety mode)."
            )

        self._sensory_only_mode = False

        # Step 0: Register motor decoder with total channel count (if motors were registered)
        if self._motor_total_channels > 0 and not self._motor_decoder_registered:
            self._register_motor_decoder()
            self._motor_decoder_registered = True
        
        # Step 1: Initialize transport
        if self._transport_type == "zmq":
            from feagi.pns.client import AgentType, FeagiAgentClient

            # Build expected cortical IDs from the active ConnectorAgent when
            # available, so registration verification can assert both motor and
            # sensory auto-created areas.
            motor_cortical_ids = self._collect_motor_cortical_ids()
            expected_cortical_ids: list[str] = list(motor_cortical_ids)
            derived_sensory_ids: list[str] = []
            if self._cache is not None:
                if hasattr(self._cache, "get_motor_cortical_ids_for_verification"):
                    try:
                        derived_motor_ids = list(
                            self._cache.get_motor_cortical_ids_for_verification()
                        )
                        if derived_motor_ids:
                            motor_cortical_ids = derived_motor_ids
                    except Exception as exc:
                        logger.warning(
                            "Failed to derive motor cortical IDs from cache: %s",
                            exc,
                        )
                if hasattr(self._cache, "get_sensory_cortical_ids_for_verification"):
                    try:
                        derived_sensory_ids = list(
                            self._cache.get_sensory_cortical_ids_for_verification()
                        )
                        expected_cortical_ids = sorted(
                            set(motor_cortical_ids) | set(derived_sensory_ids)
                        )
                    except Exception as exc:
                        logger.warning(
                            "Failed to derive sensory cortical IDs from cache: %s",
                            exc,
                        )
                else:
                    expected_cortical_ids = list(motor_cortical_ids)
            output_count = self._motor_total_channels or len(motor_cortical_ids)
            has_vision = bool(self._vision_units)
            has_sensory_cache = bool(derived_sensory_ids)
            sensory_only_eligible = (
                output_count <= 0 and (has_sensory_cache or has_vision)
            )

            if output_count <= 0 and not sensory_only_eligible:
                raise RuntimeError(
                    "No motor outputs registered and no sensory/vision registrations "
                    "in ConnectorAgent cache (cannot connect without motors or sensory outputs)."
                )

            if sensory_only_eligible:
                self._sensory_only_mode = True
                agent_type = AgentType.SENSORY
                motor_units_payload = None
            else:
                motor_units = self._collect_motor_unit_specs()
                if not motor_units:
                    raise RuntimeError(
                        "Motor units are required for FEAGI 2.0 registration."
                    )
                agent_type = (
                    AgentType.BOTH
                    if self._vision_units
                    else AgentType.MOTOR
                )
                motor_units_payload = ("motor", output_count, motor_units)

            client = FeagiAgentClient(self._agent_id, agent_type)
            resolved_auth_token_b64 = self._auth_token_b64
            if not resolved_auth_token_b64:
                raise RuntimeError(
                    "Missing auth token base64. Provide auth_token_b64 in "
                    "brain_output.configure(...)."
                )
            configure_kwargs = dict(
                feagi_host=self._feagi_host,
                registration_port=self._feagi_registration_port,
                sensory_port=self._feagi_sensory_port,
                motor_port=self._feagi_motor_port,
                agent_descriptor_b64=self._agent_id,
                heartbeat_interval=self._feagi_heartbeat_interval_s,
                connection_timeout_ms=self._feagi_connection_timeout_ms,
                registration_retries=self._feagi_registration_retries,
                feagi_api_port=self._feagi_api_port,
                feagi_http_timeout_s=self._feagi_http_timeout_s,
                auth_token_b64=resolved_auth_token_b64,
                vision_unit=None,
                vision_units=self._vision_units,
            )
            if motor_units_payload is not None:
                configure_kwargs["motor_units"] = motor_units_payload
            client.configure(**configure_kwargs)
            client.connect()
            if hasattr(client, "set_motor_cortical_ids") and not self._sensory_only_mode:
                client.set_motor_cortical_ids(motor_cortical_ids)
            self._client = client
            # Send device registrations via ZMQ so motor cortical IDs are derived correctly
            if (self._motor_total_channels > 0 or self._sensory_only_mode) and self._cache:
                if self._feagi_api_port is None or self._feagi_http_timeout_s is None:
                    raise RuntimeError(
                        "brain_output.configure(...) must include feagi_api_port "
                        "and feagi_http_timeout_s for deterministic motor IO "
                        "registration."
                    )
                device_regs_str = self._cache.export_capabilities_json()
                if device_regs_str:
                    device_regs = json.loads(device_regs_str)
                    if self._device_registration_enricher is not None:
                        device_regs = self._device_registration_enricher(device_regs)
                    device_regs = self._normalize_device_registration_properties(device_regs)
                    device_regs = self._validate_device_registration_contract(device_regs)
                    logger.info(
                        "[CFG] Verifying cortical auto-create for %d IDs "
                        "(motor=%d, sensory=%d).",
                        len(expected_cortical_ids),
                        len(motor_cortical_ids),
                        max(0, len(expected_cortical_ids) - len(motor_cortical_ids)),
                    )
                    client.send_device_configuration(
                        json.dumps(device_regs, sort_keys=True),
                        expected_cortical_ids=expected_cortical_ids,
                    )
                    logger.info(
                        "[CFG] Sent and verified device_registrations via ZMQ"
                    )
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")

        self._connected = True

    def disconnect(self):
        """Disconnect from FEAGI"""
        if self._client:
            try:
                self._client.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting client: {e}")
            self._client = None
        self._connected = False
        self._sensory_only_mode = False

    def register_sensor_units(
        self,
        unit_channel_counts: Dict[str, int],
        *,
        z_neuron_resolution: int,
        group_index_start: int = 0,
        image_resolution_xy: tuple[int, int] = (32, 32),
        misc_dimensions_xyz: tuple[int, int, int] = (1, 1, 1),
    ) -> Dict[str, int]:
        """
        Register sensory units in ConnectorAgent cache using SDK-owned Rust bindings.

        This exposes the cache registration surface to controllers so they can avoid
        importing ``feagi_rust_py_libs`` directly.

        Args:
            unit_channel_counts: Mapping of FEAGI unit key -> channel count.
                Supported keys: ``Vision``, ``Gyroscope``, ``Proximity``, ``Shock``,
                ``MiscData``.
            z_neuron_resolution: Resolution parameter required by scalar sensory units.
            group_index_start: Starting sensory group index. Use this to reserve
                lower group IDs for other sensory unit families.
            image_resolution_xy: Vision registration resolution as (x, y).
            misc_dimensions_xyz: MiscData registration dimensions as (x, y, z).

        Returns:
            Mapping of unit key -> assigned cache group index (deterministic sorted order).
        """
        self._init_cache()
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        if z_neuron_resolution <= 0:
            raise ValueError("z_neuron_resolution must be > 0.")
        if group_index_start < 0:
            raise ValueError("group_index_start must be >= 0.")

        import feagi_rust_py_libs as frpl

        frame_mode = frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute()
        positioning = frpl.data_structures.genomic.cortical_area.PercentageNeuronPositioning.Linear()
        descriptors = frpl.connector_core.data_types.descriptors
        image_props = descriptors.ImageFrameProperties(
            descriptors.ImageXYResolution(
                int(image_resolution_xy[0]),
                int(image_resolution_xy[1]),
            ),
            descriptors.ColorSpace.Gamma,
            descriptors.ColorChannelLayout.RGB,
        )
        misc_dims = descriptors.MiscDataDimensions(
            int(misc_dimensions_xyz[0]),
            int(misc_dimensions_xyz[1]),
            int(misc_dimensions_xyz[2]),
        )

        sensory_registers = {
            "Vision": lambda group, count: self._cache.sensor_Vision_register(
                group,
                count,
                frame_mode,
                image_props,
            ),
            "Gyroscope": lambda group, count: self._cache.sensor_Gyroscope_register(
                group,
                count,
                frame_mode,
                z_neuron_resolution,
                positioning,
            ),
            "Proximity": lambda group, count: self._cache.sensor_Proximity_register(
                group,
                count,
                frame_mode,
                z_neuron_resolution,
                positioning,
            ),
            "Shock": lambda group, count: self._cache.sensor_Shock_register(
                group,
                count,
                frame_mode,
                z_neuron_resolution,
                positioning,
            ),
            "MiscData": lambda group, count: self._cache.sensor_MiscData_register(
                group,
                count,
                frame_mode,
                misc_dims,
            ),
        }

        unit_groups: Dict[str, int] = {}
        for relative_index, unit_key in enumerate(sorted(unit_channel_counts.keys())):
            group_index = int(group_index_start) + int(relative_index)
            channel_count = int(unit_channel_counts[unit_key])
            if channel_count <= 0:
                continue
            register = sensory_registers.get(unit_key)
            if register is None:
                raise ValueError(
                    f"Unsupported sensory unit '{unit_key}'. "
                    f"Supported units: {sorted(sensory_registers.keys())}"
                )
            register(group_index, channel_count)
            unit_groups[unit_key] = group_index
        return unit_groups

    def register_motor_groups(
        self,
        group_channels: Dict[int, Dict[str, List[str]]],
        *,
        z_neuron_resolution: int,
    ) -> None:
        """
        Register grouped motor channels in ConnectorAgent without exposing Rust APIs.

        This supports advanced controllers that need deterministic group/channel
        registration before FEAGI auto-creation verification.
        """
        self._init_cache()
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        if z_neuron_resolution <= 0:
            raise ValueError("z_neuron_resolution must be > 0.")

        import feagi_rust_py_libs as frpl

        frame_mode = frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute()
        positioning = frpl.data_structures.genomic.cortical_area.PercentageNeuronPositioning.Linear()

        for group_id, channels in sorted(group_channels.items()):
            group_servo_count = len(channels.get("positional_servo", []))
            group_rotary_count = len(channels.get("rotary_motor", []))
            if group_servo_count > 0:
                self._cache.motor_positional_servo_register(
                    group_id,
                    group_servo_count,
                    frame_mode,
                    z_neuron_resolution,
                    positioning,
                )
            if group_rotary_count > 0:
                self._cache.motor_rotary_motor_register(
                    group_id,
                    group_rotary_count,
                    frame_mode,
                    z_neuron_resolution,
                    positioning,
                )

        # Mark as externally registered to avoid callback registration path that
        # depends on removed MotorCorticalType symbols in some builds.
        self._motor_decoder_registered = True

    def _init_sensory_write_helpers(self) -> None:
        """Lazy-init write helper factories for sensory cache writes."""
        if (
            self._sensory_percentage_factory is not None
            and self._sensory_misc_factory is not None
        ):
            return
        import feagi_rust_py_libs as frpl

        self._sensory_percentage_factory = frpl.connector_core.data_types.Percentage
        self._sensory_misc_factory = frpl.connector_core.data_types.MiscData

    def _init_vision_write_helpers(self) -> None:
        """Lazy-init write helper factories for vision cache writes."""
        if (
            self._vision_image_frame_factory is not None
            and self._vision_color_space is not None
            and self._vision_memory_layout is not None
        ):
            return
        import feagi_rust_py_libs as frpl

        self._vision_image_frame_factory = frpl.connector_core.data_types.ImageFrame
        self._vision_color_space = (
            frpl.connector_core.data_types.descriptors.ColorSpace.Gamma
        )
        self._vision_memory_layout = (
            frpl.connector_core.data_types.descriptors.MemoryOrderLayout
            .HeightsWidthsChannels
        )

    def register_vision_groups(
        self,
        vision_units: list[tuple[str, int, int, int, str, int]],
        peripheral_resolution: tuple[int, int] | None = None,
    ) -> list[int]:
        """
        Register segmented vision groups with deterministic group IDs.

        Uses FEAGI standard segmented topology:
        - center: 128x128x3
        - peripherals: 32x32x1

        Args:
            vision_units: Registered camera input units.
            peripheral_resolution: Optional segmented peripheral resolution override
                as ``(width, height)``. Defaults to ``(32, 32)``.
        """
        self._init_cache()
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        if not vision_units:
            return []

        import feagi_rust_py_libs as frpl

        frame_mode = (
            frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute()
        )
        descriptors = frpl.connector_core.data_types.descriptors
        data_types = frpl.connector_core.data_types
        register_method = None
        for candidate in (
            "sensor_segmented_vision_register",
            "sensor_SegmentedVision_register",
        ):
            if hasattr(self._cache, candidate):
                register_method = getattr(self._cache, candidate)
                break
        if register_method is None:
            raise RuntimeError(
                "ConnectorAgent cache does not expose segmented vision registration."
            )

        center_resolution = descriptors.ImageXYResolution(128, 128)
        peripheral_width, peripheral_height = (32, 32)
        if peripheral_resolution is not None:
            raw_width, raw_height = peripheral_resolution
            if int(raw_width) <= 0 or int(raw_height) <= 0:
                raise ValueError(
                    "peripheral_resolution values must be positive integers."
                )
            peripheral_width = int(raw_width)
            peripheral_height = int(raw_height)
        peripheral_resolution_descriptor = descriptors.ImageXYResolution(
            peripheral_width,
            peripheral_height,
        )
        segmented_resolutions = (
            descriptors.SegmentedXYImageResolutions
            .create_with_same_sized_peripheral(
                center_resolution,
                peripheral_resolution_descriptor,
            )
        )
        center_layout = descriptors.ColorChannelLayout.RGB
        peripheral_layout = descriptors.ColorChannelLayout.GrayScale
        color_space = descriptors.ColorSpace.Gamma
        segmented_properties = descriptors.SegmentedImageFrameProperties(
            segmented_resolutions,
            center_layout,
            peripheral_layout,
            color_space,
        )
        pct = data_types.Percentage.new_from_0_1
        initial_gaze = data_types.GazeProperties(
            data_types.Percentage2D(pct(0.5), pct(0.5)),
            pct(1.0),
        )
        registered_groups: list[int] = []
        for (
            _modality,
            width,
            height,
            _channels,
            _unit,
            group,
        ) in vision_units:
            image_props = descriptors.ImageFrameProperties(
                descriptors.ImageXYResolution(int(width), int(height)),
                color_space,
                center_layout,
            )
            register_method(
                group=int(group),
                number_channels=1,
                frame_change_handling=frame_mode,
                input_image_properties=image_props,
                segmented_image_properties=segmented_properties,
                initial_gaze=initial_gaze,
            )
            registered_groups.append(int(group))
        return registered_groups

    def write_sensor_vision_frame(
        self,
        *,
        group: int,
        channel_index: int,
        frame_rgb,
    ) -> None:
        """Write one RGB image frame into vision sensory cache."""
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        self._init_vision_write_helpers()

        import numpy as np

        frame_array = np.asarray(frame_rgb, dtype=np.uint8)
        if frame_array.ndim != 3 or frame_array.shape[2] != 3:
            raise ValueError(
                f"Vision frame must be an RGB ndarray with shape (H, W, 3), got {frame_array.shape}"
            )

        vision_frame = self._vision_image_frame_factory.new_from_array(
            frame_array,
            self._vision_color_space,
            self._vision_memory_layout,
        )
        write_method = None
        for candidate in ("sensor_segmented_vision_write", "sensor_vision_write"):
            if hasattr(self._cache, candidate):
                write_method = getattr(self._cache, candidate)
                break
        if write_method is None:
            raise RuntimeError("ConnectorAgent cache does not expose a vision write method.")

        write_method(
            group=int(group),
            channel_index=int(channel_index),
            data=vision_frame,
        )

    def write_sensor_scalar(
        self,
        *,
        unit_key: str,
        group: int,
        channel_index: int,
        scalar_0_1: float,
    ) -> None:
        """
        Write one normalized scalar sample into the sensory cache.

        Args:
            unit_key: Sensory unit key (``Proximity``, ``Shock``, ``MiscData``).
            group: Registered sensory group index.
            channel_index: Channel index within group.
            scalar_0_1: Normalized scalar value in ``[0.0, 1.0]``.
        """
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        self._init_sensory_write_helpers()
        scalar = max(0.0, min(1.0, float(scalar_0_1)))

        if unit_key == "Proximity":
            self._cache.sensor_proximity_write(
                group=int(group),
                channel_index=int(channel_index),
                data=self._sensory_percentage_factory.new_from_0_1(scalar),
            )
            return
        if unit_key == "Shock":
            self._cache.sensor_shock_write(
                group=int(group),
                channel_index=int(channel_index),
                data=self._sensory_percentage_factory.new_from_0_1(scalar),
            )
            return
        if unit_key == "MiscData":
            import numpy as np

            misc_data = self._sensory_misc_factory.new_from_array(
                np.array([[[scalar]]], dtype=np.float32)
            )
            self._cache.sensor_misc_data_write(
                group=int(group),
                channel_index=int(channel_index),
                data=misc_data,
            )
            return

        raise ValueError(
            f"Unsupported sensor write unit '{unit_key}'. "
            "Supported units: ['Proximity', 'Shock', 'MiscData']"
        )

    def flush_sensory_bytes(self) -> int:
        """
        Encode cached sensory data and send it via active FEAGI client.

        Returns:
            Number of encoded bytes sent in this flush (0 when empty).
        """
        if self._cache is None:
            raise RuntimeError("ConnectorAgent cache is not initialized.")
        if self._client is None:
            raise RuntimeError("FEAGI client is not initialized. Call connect() first.")
        # Avoid noisy client-level errors in motor-only sessions: those
        # connections intentionally have no sensory socket.
        client_agent_type = getattr(self._client, "agent_type", None)
        if (
            hasattr(client_agent_type, "value")
            and str(client_agent_type.value) == "motor"
        ):
            return 0

        self._cache.sensors_encode_cached_sensor_data_to_bytes()
        encoded_bytes = bytes(self._cache.sensors_read_bytes())
        if not encoded_bytes:
            return 0
        try:
            self._client.send_sensory_bytes(encoded_bytes)
            return len(encoded_bytes)
        except Exception as exc:
            # Motor-only agents do not expose a sensory socket. Keep the control
            # loop running without noisy hard failures in those sessions.
            if "Sensory socket is not available for this agent type" in str(exc):
                return 0
            raise
    
    def register_output(self, output_instance: 'BaseOutput'):
        """
        Register an output (called internally by output classes).
        
        Args:
            output_instance: Output instance to register
        """
        self._init_cache()
        
        # Check if this is a motor output (ServoMotor or RotaryMotor)
        from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
        is_motor = isinstance(output_instance, (ServoMotor, RotaryMotor))
        
        if is_motor:
            # Motors: Use group=0, assign unique channel index
            group_id = int(getattr(output_instance, "preferred_group_id", 0) or 0)
            preferred_channel_index = getattr(output_instance, "preferred_channel_index", None)
            if preferred_channel_index is None:
                channel_index = self._next_motor_channel_by_group.get(group_id, 0)
                self._next_motor_channel_by_group[group_id] = channel_index + 1
            else:
                channel_index = int(preferred_channel_index)
                next_index = self._next_motor_channel_by_group.get(group_id, 0)
                if channel_index >= next_index:
                    self._next_motor_channel_by_group[group_id] = channel_index + 1
            output_instance.channel = channel_index
            if group_id == 0:
                self._motor_outputs_by_channel[channel_index] = output_instance
            self._motor_outputs_by_group_channel[(group_id, channel_index)] = output_instance
            
            # Track total motor count for decoder registration
            self._motor_total_channels += 1
        else:
            # Other outputs: Use auto-incrementing group IDs
            group_id = self._allocate_group_id()
            channel_index = 0
        
        # Add to registry first (needed for connect() to know total count)
        self._outputs.append(output_instance)
        
        # Register with Rust cache (motors will defer decoder registration until connect())
        output_instance._register_with_cache(self._cache, group_id, channel_index, self._motor_decoder_registered)
        output_instance._mark_registered(group_id)
        
        if is_motor:
            logger.debug(f"[OK] Registered output: {output_instance.__class__.__name__} (group={group_id}, channel={channel_index})")
        else:
            logger.debug(f"[OK] Registered output: {output_instance.__class__.__name__} (group={group_id})")
    
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
    
    def _notify_monitors_receive_start(self, data: Dict[str, Any]):
        """Notify all monitors of receive start."""
        for monitor in self._monitors:
            try:
                monitor.on_receive_start(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def _notify_monitors_receive_complete(self, data: Dict[str, Any]):
        """Notify all monitors of receive complete."""
        for monitor in self._monitors:
            try:
                monitor.on_receive_complete(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def receive(self):
        """
        Receive motor commands from FEAGI.
        
        This is the main loop method:
        1. Receives bytes from transport
        2. Decodes bytes to neurons (Rust)
        3. Updates motor cache
        4. Reads values from cache to outputs
        
        Call this in your main loop to update all output values.
        """
        if not self._connected:
            raise RuntimeError(
                "Not connected to FEAGI. Call brain_output.connect() first."
            )

        if self._sensory_only_mode:
            return

        # All timing and monitoring removed for performance

        try:
            if not self._client:
                raise RuntimeError("Client not initialized. Call connect() first.")

            motor_data = self._client.receive_motor_data()
            if not motor_data:
                return

            motor_map = motor_data.get("motor")
            if not isinstance(motor_map, dict):
                raise RuntimeError("Motor data format invalid (expected dict).")
            # Canonical snapshot keyed by normalized "group:channel:mode".
            # Keep this deterministic for controller-side diagnostics.
            self._motor_data = {}

            # Aggregate commands by (group, channel) first so mixed mode updates
            # in the same receive cycle can be resolved deterministically.
            pending_by_channel: dict[tuple[int, int], dict[str, tuple[float, str]]] = {}

            for key, value in motor_map.items():
                if value is None:
                    continue
                key_str = str(key)
                command_mode = None
                parts = key_str.split(":")
                if len(parts) == 3:
                    group_str, channel_str, command_mode = parts
                    try:
                        group_id = int(group_str)
                        channel_index = int(channel_str)
                    except ValueError:
                        continue
                elif len(parts) == 2:
                    left, right = parts
                    if right in ("absolute", "incremental"):
                        group_id = 0
                        command_mode = right
                        try:
                            channel_index = int(left)
                        except ValueError:
                            continue
                    else:
                        try:
                            group_id = int(left)
                            channel_index = int(right)
                        except ValueError:
                            continue
                else:
                    group_id = 0
                    try:
                        channel_index = int(key_str)
                    except ValueError:
                        continue

                output = self._motor_outputs_by_group_channel.get((group_id, channel_index))
                if output is None and group_id == 0:
                    output = self._motor_outputs_by_channel.get(channel_index)
                if output is None:
                    continue
                value_f = float(value)
                mode_key = (
                    command_mode if command_mode in ("absolute", "incremental") else "none"
                )
                pending_by_channel.setdefault((group_id, channel_index), {})[mode_key] = (
                    value_f,
                    key_str,
                )

            for (group_id, channel_index), mode_entries in pending_by_channel.items():
                output = self._motor_outputs_by_group_channel.get((group_id, channel_index))
                if output is None and group_id == 0:
                    output = self._motor_outputs_by_channel.get(channel_index)
                if output is None:
                    continue

                # Arbitration: if both absolute and incremental are present
                # for the same channel in this cycle, absolute takes precedence.
                if "absolute" in mode_entries:
                    selected_mode = "absolute"
                elif "incremental" in mode_entries:
                    selected_mode = "incremental"
                else:
                    selected_mode = "none"

                value_f, key_str = mode_entries[selected_mode]
                canonical_key = f"{group_id}:{channel_index}:{selected_mode}"
                self._motor_data[canonical_key] = value_f

                if "absolute" in mode_entries and "incremental" in mode_entries:
                    inc_val, _ = mode_entries["incremental"]
                    logger.info(
                        (
                            "[MOTOR-RX-ARBITRATION] group=%d channel=%d "
                            "absolute=%.6f incremental=%.6f selected=absolute"
                        ),
                        group_id,
                        channel_index,
                        value_f,
                        inc_val,
                    )

                log_key = f"{group_id}:{channel_index}:{selected_mode}"
                prev_value = self._last_logged_motor_values.get(log_key)
                if prev_value is None or abs(value_f - prev_value) > 1e-6:
                    logger.info(
                        "[MOTOR-RX] group=%d channel=%d mode=%s value=%.6f key=%s",
                        group_id,
                        channel_index,
                        selected_mode,
                        value_f,
                        key_str,
                    )
                    self._last_logged_motor_values[log_key] = value_f
                try:
                    output._on_motor_command(
                        value_f,
                        command_mode=None if selected_mode == "none" else selected_mode,
                    )
                except TypeError:
                    # Backward compatibility for output classes with old callback signature.
                    output._on_motor_command(value_f)
                except Exception as e:
                    logger.debug(f"Error updating motor output {key_str}: {e}")
            
            # Note: _read_from_cache() is no longer needed as callbacks handle updates
            # The motor values are already updated via callbacks during process_neurons()
            
        except Exception:
            # Monitor error notifications disabled - was causing performance issues
            raise
    
    def get_output_count(self) -> int:
        """Get number of registered outputs"""
        return len(self._outputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_output = BrainOutput()

