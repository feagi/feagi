"""
FEAGI Agent Client - Rust-Backed Implementation

This is the new recommended client for building FEAGI agents. It uses the
Rust-backed SDK for high performance and reliability.

Features:
- Automatic registration with retry
- Background heartbeat (no manual management needed)
- Reconnection logic with exponential backoff
- High-performance Rust core
- Simple, Pythonic API

Migration from legacy FeagiClient:
- Much simpler API (no manual socket management)
- Automatic heartbeat (no periodic tasks needed)
- Better error handling with retries
- 10-100x faster for message processing

Example:
    from feagi_connector import FeagiAgentClient, AgentType
    
    # Create and connect
    client = FeagiAgentClient("my_camera", AgentType.SENSORY)
    client.configure(
        feagi_host="localhost",
        vision_unit=("camera", 640, 480, 3, "vision", 0),
        heartbeat_interval=5.0
    )
    client.connect()
    
    # Send sensory data
    client.send_sensory_data([(neuron_id, potential), ...])
    
    # Auto-deregisters on exit
"""

import logging
import json
import time
import urllib.error
import urllib.request
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from .xyzp_decoders import decode_motor_xyzp

# Try to import Rust SDK (optional dependency)
_rust_sdk_available = False
_rust_import_error = None

try:
    # Preferred: feagi_agent submodule (feagi-rust-py-libs 0.0.90+)
    from feagi_rust_py_libs.feagi_agent import (
        PyAgentClient as _PyAgentClient,
        PyAgentConfig as _PyAgentConfig,
        AgentType as _RustAgentType,
        init_rust_logging as _init_rust_logging,
    )
    rust_sdk = type("rust_sdk", (), {
        "PyAgentClient": _PyAgentClient,
        "PyAgentConfig": _PyAgentConfig,
        "AgentType": _RustAgentType,
        "init_rust_logging": _init_rust_logging,
    })()
    _rust_sdk_available = True
except ImportError:
    try:
        # Fallback: nested layout (some wheel builds)
        from feagi_rust_py_libs.feagi_rust_py_libs import feagi_agent as rust_sdk
        _rust_sdk_available = True
    except ImportError:
        try:
            from feagi_rust_py_libs import feagi_agent_sdk as rust_sdk
            _rust_sdk_available = True
        except ImportError:
            try:
                import feagi_agent_sdk as rust_sdk
                _rust_sdk_available = True
            except ImportError as e2:
                _rust_import_error = str(e2)
                rust_sdk = None

# Only define these if Rust SDK is available
if _rust_sdk_available:
    PyAgentClient = rust_sdk.PyAgentClient
    PyAgentConfig = rust_sdk.PyAgentConfig
    RustAgentType = rust_sdk.AgentType
    
    # Initialize Rust logging (safe to call multiple times)
    try:
        rust_sdk.init_rust_logging()
        print("[PYTHON-SDK] [OK] Rust logging initialized", flush=True)
    except BaseException as e:
        if "global default trace dispatcher has already been set" in str(e):
            print(
                "[PYTHON-SDK] [OK] Rust logging already initialized",
                flush=True,
            )
        else:
            print(
                f"[PYTHON-SDK] [FAIL] Failed to init Rust logging: {e}",
                flush=True,
            )
            logging.getLogger("feagi.pns.client").warning(
                "Failed to init Rust logging: %s",
                e,
            )
else:
    # No fallback - Rust SDK is required (will raise ImportError in __init__)
    PyAgentClient = None
    PyAgentConfig = None
    RustAgentType = None

logger = logging.getLogger("feagi.pns.client")


class AgentType(Enum):
    """Agent type enum"""
    SENSORY = "sensory"
    MOTOR = "motor"
    BOTH = "both"


class FeagiAgentClient:
    """
    High-performance FEAGI agent client (Rust-backed)
    
    This client provides a simple, production-ready interface for building
    FEAGI agents. All heavy lifting is done in Rust for maximum performance.
    
    Features:
    - Automatic registration with retry logic
    - Background heartbeat (no manual management)
    - Reconnection with exponential backoff
    - Thread-safe operations
    - Graceful shutdown
    
    Example:
        ```python
        from feagi_connector import FeagiAgentClient, AgentType
        
        # Check if FEAGI is reachable first (prevents thread blocking)
        if not FeagiAgentClient.is_feagi_reachable("localhost"):
            print("FEAGI not available, running in standalone mode")
            # ... standalone logic ...
            return
        
        # Create client
        client = FeagiAgentClient("video_camera_01", AgentType.SENSORY)
        
        # Configure
        client.configure(
            feagi_host="localhost",
            vision_unit=("camera", 640, 480, 3, "vision", 0),
            heartbeat_interval=5.0
        )
        
        # Connect (with automatic retry)
        success = client.connect(graceful=True)
        # Returns False instead of raising.
        if not success:
            # Handle connection failure gracefully
            pass
        
        # Send data
        client.send_sensory_data([(0, 50.0), (1, 75.0)])
        
        # Client auto-deregisters on exit
        ```
    """
    
    @staticmethod
    def is_feagi_reachable(
        host: str,
        port: int,
        timeout: float,
    ) -> bool:
        """
        Quick lightweight check if FEAGI is reachable
        
        This method performs a TCP socket connection test WITHOUT initializing
        the SDK or creating any background threads. Use this before creating
        a client to determine if FEAGI is available.
        
        Args:
            host: FEAGI hostname or IP address.
            port: FEAGI registration port.
            timeout: Connection timeout in seconds.
        
        Returns:
            True if FEAGI registration port is reachable, False otherwise
        
        Example:
            ```python
            if FeagiAgentClient.is_feagi_reachable("192.168.1.100"):
                client = FeagiAgentClient("my_agent", AgentType.SENSORY)
                client.configure(feagi_host="192.168.1.100")
                client.connect()
            else:
                print("Running in standalone mode")
                run_without_feagi()
            ```
        
        Note:
            This is a pure Python implementation using socket module,
            NO Rust SDK or ZMQ dependencies are initialized.
        """
        import socket
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception as e:
            logger.debug(f"FEAGI reachability check failed: {e}")
            return False
    
    def __init__(self, agent_id: str, agent_type: AgentType):
        """
        Create a new FEAGI agent client
        
        Args:
            agent_id: Unique identifier for this agent
            agent_type: Type of agent (SENSORY, MOTOR, or BOTH)
        
        Raises:
            ImportError: If Rust SDK (feagi_rust_py_libs) is not installed
        """
        if not _rust_sdk_available:
            raise ImportError(
                "FEAGI Rust SDK is required but not installed.\n"
                "The Rust SDK (feagi_rust_py_libs) provides high-performance "
                "neural processing capabilities.\n\n"
                "Please install it with:\n"
                "  pip install feagi_rust_py_libs\n\n"
                f"Original error: {_rust_import_error}"
            )
        
        self.agent_id = agent_id
        self.agent_type = agent_type
        self._config: Optional[PyAgentConfig] = None
        self._client: Optional[PyAgentClient] = None
        self._connected = False
        
        # Store network config for logging
        self._feagi_host = None
        self._registration_port = None
        self._sensory_port = None
        self._feagi_api_port: Optional[int] = None
        self._feagi_http_timeout_s: Optional[float] = None
        self._motor_registration_retries: Optional[int] = None
        self._motor_registration_retry_interval_s: Optional[float] = None
        # Motor decode subscription filter (base64 cortical IDs).
        # When set, receive_motor_data decodes only these motor cortical areas.
        self._motor_cortical_ids: Optional[List[str]] = None
        
        logger.info(
            "Created agent client: %s (type: %s)",
            agent_id,
            agent_type.value,
        )

    @staticmethod
    def _parse_agent_descriptor_b64(
        agent_descriptor_b64: str,
    ) -> Tuple[str, str, int]:
        """
        Parse base64 AgentDescriptor (48-byte payload) into fields.

        Expected layout:
          - bytes [0:4]: instance id (unused by Python SDK)
          - bytes [4:24]: manufacturer (null-padded ASCII)
          - bytes [24:44]: agent name (null-padded ASCII)
          - bytes [44:48]: agent version (little-endian u32)
        """
        import base64
        import binascii

        try:
            descriptor_bytes = base64.b64decode(
                agent_descriptor_b64,
                validate=True,
            )
        except (ValueError, binascii.Error) as exc:
            raise ValueError(
                "agent_descriptor_b64 must be valid base64 "
                "AgentDescriptor bytes."
            ) from exc

        if len(descriptor_bytes) != 48:
            raise ValueError(
                "agent_descriptor_b64 must decode to 48 bytes."
            )

        manufacturer = (
            descriptor_bytes[4:24]
            .decode("ascii", errors="strict")
            .rstrip("\x00")
            .strip()
        )
        agent_name = (
            descriptor_bytes[24:44]
            .decode("ascii", errors="strict")
            .rstrip("\x00")
            .strip()
        )
        agent_version = int.from_bytes(
            descriptor_bytes[44:48],
            byteorder="little",
        )

        if not manufacturer:
            raise ValueError("Agent descriptor manufacturer cannot be empty.")
        if not agent_name:
            raise ValueError("Agent descriptor agent_name cannot be empty.")
        if agent_version <= 0:
            raise ValueError("Agent descriptor agent_version must be > 0.")

        return manufacturer, agent_name, agent_version
    
    def configure(
        self,
        feagi_host: str,
        registration_port: int,
        sensory_port: int,
        motor_port: int,
        agent_descriptor_b64: str,
        auth_token_b64: str,
        *,
        vision_unit: Optional[Tuple[str, int, int, int, str, int]] = None,
        motor_unit: Optional[Tuple[str, int, str, int]] = None,
        motor_units: Optional[Tuple[str, int, List[Tuple[str, int]]]] = None,
        custom_capabilities: Optional[Dict[str, Any]] = None,
        heartbeat_interval: float = 5.0,
        connection_timeout_ms: int = 5000,
        registration_retries: int = 3,
        sensory_socket_hwm: Optional[int] = None,
        sensory_socket_linger_ms: Optional[int] = None,
        sensory_socket_immediate: Optional[bool] = None,
        feagi_api_port: Optional[int] = None,
        feagi_http_timeout_s: Optional[float] = None,
    ):
        """
        Configure the agent
        
        Args:
            feagi_host: FEAGI hostname or IP address.
            registration_port: Registration/heartbeat port.
            sensory_port: Sensory data input port.
            motor_port: Motor data output port.
            vision_unit: Vision capability tuple using semantic unit + group
                (modality, width, height, channels, unit, group).
            motor_unit: Motor capability tuple using semantic unit + group
                (modality, output_count, unit, group).
            motor_units: Motor capability tuple with multiple unit/group pairs
                (modality, output_count, [(unit, group), ...]).
            custom_capabilities: Dictionary of custom capabilities.
            heartbeat_interval: Heartbeat interval in seconds (0 to disable).
            connection_timeout_ms: Connection timeout in milliseconds.
            registration_retries: Maximum registration retry attempts.
            sensory_socket_hwm: Optional ZMQ PUSH high-water-mark override for
                the sensory socket.
            sensory_socket_linger_ms: Optional linger override in milliseconds
                for the sensory socket.
            sensory_socket_immediate: Optional immediate mode flag for the
                sensory socket.
            agent_descriptor_b64: Base64 AgentDescriptor (48-byte payload).
            auth_token_b64: Base64 auth token (must decode to 32 bytes).
        """
        if not feagi_host:
            raise ValueError("feagi_host must be provided explicitly.")
        if registration_port <= 0:
            raise ValueError("registration_port must be > 0.")
        if sensory_port <= 0:
            raise ValueError("sensory_port must be > 0.")
        if motor_port <= 0:
            raise ValueError("motor_port must be > 0.")
        if heartbeat_interval <= 0:
            raise ValueError("heartbeat_interval must be > 0.")
        if connection_timeout_ms <= 0:
            raise ValueError("connection_timeout_ms must be > 0.")
        if registration_retries <= 0:
            raise ValueError("registration_retries must be > 0.")
        if not agent_descriptor_b64:
            raise ValueError(
                "agent_descriptor_b64 must be provided explicitly.",
            )
        if not auth_token_b64:
            raise ValueError("auth_token_b64 must be provided explicitly.")

        # Map Python AgentType to Rust AgentType
        rust_agent_type = {
            AgentType.SENSORY: RustAgentType.sensory(),
            AgentType.MOTOR: RustAgentType.motor(),
            AgentType.BOTH: RustAgentType.both(),
        }[self.agent_type]
        
        # Create configuration
        self._config = PyAgentConfig(self.agent_id, rust_agent_type)
        
        # Set network configuration
        self._config.with_registration_endpoint(f"tcp://{feagi_host}:{registration_port}")
        self._config.with_sensory_endpoint(f"tcp://{feagi_host}:{sensory_port}")
        self._config.with_motor_endpoint(f"tcp://{feagi_host}:{motor_port}")
        
        # Set reliability configuration
        self._config.with_heartbeat_interval(heartbeat_interval)
        self._config.with_connection_timeout_ms(connection_timeout_ms)
        self._config.with_registration_retries(registration_retries)
        self._motor_registration_retries = registration_retries
        self._motor_registration_retry_interval_s = heartbeat_interval

        manufacturer, descriptor_name, descriptor_version = (
            self._parse_agent_descriptor_b64(agent_descriptor_b64)
        )
        if not hasattr(self._config, "with_agent_descriptor"):
            raise RuntimeError(
                "PyAgentConfig from feagi_rust_py_libs does not have "
                "with_agent_descriptor. Rebuild feagi-rust-py-libs from source: "
                "cd feagi-rust-py-libs && maturin develop --release"
            )
        self._config.with_agent_descriptor(
            manufacturer,
            descriptor_name,
            descriptor_version,
        )

        self._config.with_auth_token_base64(auth_token_b64)

        if any(
            value is not None
            for value in (
                sensory_socket_hwm,
                sensory_socket_linger_ms,
                sensory_socket_immediate,
            )
        ):
            if (
                sensory_socket_hwm is None
                or sensory_socket_linger_ms is None
                or sensory_socket_immediate is None
            ):
                error_message = (
                    "sensory_socket_hwm, sensory_socket_linger_ms, and "
                    "sensory_socket_immediate must all be provided together"
                )
                raise ValueError(error_message)
            self._config.with_sensory_socket_config(
                sensory_socket_hwm,
                sensory_socket_linger_ms,
                sensory_socket_immediate,
            )
        
        # Add capabilities
        if vision_unit:
            (
                modality,
                width,
                height,
                channels,
                unit,
                group,
            ) = vision_unit
            self._config.with_vision_unit(
                modality,
                width,
                height,
                channels,
                unit,
                group,
            )

            # Store for XYZP conversion
            self._vision_width = width
            self._vision_height = height
            self._vision_unit = unit
            self._vision_group = group
            
            logger.debug(
                "Added vision unit capability: %sx%sx%s -> %s:%s",
                width,
                height,
                channels,
                unit,
                group,
            )
        
        # Store network config for direct ZMQ access and logging
        self._feagi_host = feagi_host
        self._registration_port = registration_port
        self._sensory_port = sensory_port
        self._feagi_api_port = feagi_api_port
        self._feagi_http_timeout_s = feagi_http_timeout_s

        if (
            feagi_api_port is None
            and feagi_http_timeout_s is not None
        ) or (
            feagi_api_port is not None
            and feagi_http_timeout_s is None
        ):
            raise ValueError(
                "feagi_api_port and feagi_http_timeout_s must both be "
                "provided together."
            )
        if feagi_api_port is not None and feagi_api_port <= 0:
            raise ValueError("feagi_api_port must be > 0.")
        if feagi_http_timeout_s is not None and feagi_http_timeout_s <= 0:
            raise ValueError("feagi_http_timeout_s must be > 0.")
        
        if motor_unit and motor_units:
            raise ValueError("Provide only one of motor_unit or motor_units.")
        if motor_unit:
            modality, output_count, unit, group = motor_unit
            self._config.with_motor_unit(modality, output_count, unit, group)
            logger.debug(
                "Added motor unit capability: %s outputs <- %s:%s",
                output_count,
                unit,
                group,
            )
        if motor_units:
            modality, output_count, source_units = motor_units
            self._config.with_motor_units(modality, output_count, source_units)
            logger.debug(
                "Added motor unit capabilities: %s outputs <- %s",
                output_count,
                source_units,
            )
        
        if custom_capabilities:
            import json
            for key, value in custom_capabilities.items():
                self._config.with_custom_capability(key, json.dumps(value))
                logger.debug("Added custom capability: %s", key)
        
        # Validate configuration
        try:
            self._config.validate()
            logger.debug("Configuration validated successfully")
        except Exception as e:
            logger.error("Configuration validation failed: %s", e)
            raise
    
    def detect_cortical_area_resolution(
        self,
        cortical_area: str,
        feagi_host: Optional[str] = None,
    ) -> Optional[Tuple[int, int, int]]:
        """
        Detect the resolution (dimensions) of a cortical area from FEAGI
        
        Args:
            cortical_area: Name of the cortical area (e.g., "iic400")
            feagi_host: Optional FEAGI host. If not provided, uses configured
                host.
        
        Returns:
            Tuple of (width, height, depth) if successful, None otherwise
            
        Note:
            Requires FEAGI to be running and accessible via REST API (port
            8000)
        """
        try:
            import requests
            
            # Use provided host or fall back to configured host
            host = feagi_host or self._feagi_host
            if not host:
                logger.warning(
                    "No FEAGI host specified for resolution detection",
                )
                return None
            
            # Query FEAGI REST API for cortical area properties
            api_url = (
                f"http://{host}:8000/v1/cortical_area/multi/"
                "cortical_area_properties"
            )
            payload = [cortical_area]  # API expects array, not object
            
            logger.debug(
                "Querying FEAGI for '%s' dimensions...",
                cortical_area,
            )
            response = requests.post(
                api_url,
                json=payload,
                timeout=2.0,
            )
            
            if response.status_code == 200:
                properties_dict = response.json()
                
                # Response format: dict with cortical_id as key
                # Get the cortical area properties
                area = properties_dict.get(cortical_area)
                if area:
                    dimensions = area.get("dimensions", [])
                    
                    if len(dimensions) == 3:
                        x_dim, y_dim, z_dim = dimensions
                        logger.info(
                            "Detected %s resolution: %sx%sx%s",
                            cortical_area,
                            x_dim,
                            y_dim,
                            z_dim,
                        )
                        return (x_dim, y_dim, z_dim)
                    else:
                        logger.warning(
                            "Cortical area '%s' has invalid dimensions: %s",
                            cortical_area,
                            dimensions,
                        )
                        return None
                
                # Not found in response
                logger.warning(
                    "Cortical area '%s' not found in FEAGI",
                    cortical_area,
                )
                return None
            else:
                if response.status_code == 404:
                    logger.warning(
                        "FEAGI cortical area endpoint not found (HTTP 404) - "
                        "likely no genome loaded yet",
                    )
                else:
                    logger.warning(
                        "Failed to query FEAGI cortical properties: HTTP %s",
                        response.status_code,
                    )
                return None
                
        except ImportError:
            logger.warning(
                "'requests' library not available - install with: pip install "
                "requests",
            )
            return None
        except Exception as e:
            logger.debug("Resolution detection failed: %s", e)
            return None
    
    def connect(self, graceful: bool = False) -> bool:
        """
        Connect to FEAGI and register the agent
        
        This will:
        1. Create ZMQ sockets
        2. Register with FEAGI (with automatic retry)
        3. Start background heartbeat (only after successful registration)
        
        Args:
            graceful: If True, returns False on failure instead of raising
                exceptions. Useful for controllers that can operate without
                FEAGI.
        
        Returns:
            True if connection successful, False if failed (only when
            graceful=True)
        
        Raises:
            RuntimeError: If not configured or connection fails (only when
            graceful=False)
        
        Example:
            ```python
            # Graceful mode (no exceptions)
            if client.connect(graceful=True):
                print("Connected to FEAGI")
            else:
                print("Running without FEAGI")
            
            # Strict mode (raises on failure)
            try:
                client.connect()
            except RuntimeError as e:
                print(f"Connection failed: {e}")
            ```
        """
        if self._config is None:
            error_msg = "Agent not configured. Call configure() first."
            if graceful:
                logger.error(error_msg)
                return False
            raise RuntimeError(error_msg)
        
        if self._connected:
            logger.warning("Agent already connected")
            return True
        
        logger.info(f"Connecting to FEAGI as: {self.agent_id}")
        logger.debug(f"  FEAGI host: {self._feagi_host}")
        logger.debug(f"  Registration port: {self._registration_port}")
        logger.debug(f"  Sensory port: {self._sensory_port}")
        
        # Pre-flight check: Verify FEAGI is reachable
        registration_host = self._feagi_host
        registration_port = self._registration_port
        
        # Note: Skip TCP pre-flight check for ZMQ sockets (they don't behave
        # like regular TCP). Let the Rust SDK handle connection testing.
        
        try:
            # Create client (may start background threads in Rust SDK).
            logger.debug("Creating Rust-backed agent client...")
            self._client = PyAgentClient(self._config)
            
            # Connect (with automatic retry)
            # This is where threads are spawned after successful registration
            logger.debug("Attempting registration with FEAGI...")
            try:
                self._client.connect()
                logger.debug("[OK] Registration successful")
            except Exception as reg_error:
                # Enhanced error message for registration failures
                error_str = str(reg_error).lower()
                
                if (
                    "unknown error" in error_str
                    or "registration failed" in error_str
                ):
                    logger.error(
                        "Registration failed with unclear error from Rust SDK",
                    )
                    logger.error("")
                    logger.error("Common causes:")
                    logger.error("  1. FEAGI's Rust PNS is not running")
                    logger.error(
                        "     Check FEAGI logs for: '[ZMQ-REGISTRATION] "
                        "Listening on...'",
                    )
                    logger.error(
                        "  2. Port mismatch (FEAGI 2.0 uses port 5563, not "
                        "30001)",
                    )
                    logger.error("     Your config: registration_port = ?")
                    logger.error(
                        "     FEAGI listening: Check with 'lsof -i :5563'",
                    )
                    logger.error("  3. ZMQ socket state issue (restart FEAGI)")
                    logger.error("  4. Firewall blocking connection")
                    logger.error("")
                    
                    error_msg = (
                        "Registration failed: %s. Check FEAGI is running with "
                        "Rust PNS on port %s. See logs above for "
                        "troubleshooting steps."
                        % (reg_error, registration_port)
                    )
                    
                    if graceful:
                        logger.warning("%s", error_msg)
                        return False
                    
                    raise RuntimeError(error_msg) from reg_error
                else:
                    # Re-raise with original error
                    if graceful:
                        logger.warning("Connection failed: %s", reg_error)
                        return False
                    raise
            
            self._connected = True
            logger.info("Connected and registered as: %s", self.agent_id)
            logger.info(
                "  Registration: %s:%s",
                registration_host,
                registration_port,
            )
            logger.info(
                "  Sensory data: %s:%s",
                self._feagi_host,
                self._sensory_port,
            )
            heartbeat_interval = (
                self._config.heartbeat_interval
                if hasattr(self._config, "heartbeat_interval")
                else "N/A"
            )
            logger.info("  Heartbeat: %ss", heartbeat_interval)
            return True
            
        except RuntimeError:
            # Re-raise our enhanced RuntimeErrors (unless graceful mode)
            if graceful:
                return False
            raise
        except Exception as e:
            # Catch-all for unexpected errors
            logger.error("Unexpected error during connection: %s", e)
            logger.error("   Error type: %s", type(e).__name__)
            logger.error("")
            logger.error(
                "Please report this error with the following information:",
            )
            logger.error("  - Agent ID: %s", self.agent_id)
            logger.error("  - FEAGI host: %s", self._feagi_host)
            logger.error("  - Registration port: %s", registration_port)
            logger.error("  - Error: %s", e)
            
            if graceful:
                return False
            
            raise RuntimeError(
                "Connection failed with unexpected error: %s" % e
            ) from e
    
    def send_sensory_data(
        self,
        neuron_pairs: List[Tuple[int, float]],
        blocking: bool = True,
    ):
        """
        Send sensory data to FEAGI in binary XYZP format
        
        Args:
            neuron_pairs: List of (neuron_id, potential) tuples
                         Example: [(0, 50.0), (1, 75.0), (2, 30.0)]
            blocking: If False, silently ignores errors (for event loop
                compatibility)
        
        Raises:
            RuntimeError: If not connected (only when blocking=True)
        
        Returns:
            bool: True if send succeeded, False if failed (only relevant when
            blocking=False)
        
        Example:
            ```python
            # Blocking mode (default) - raises on error
            client.send_sensory_data([(0, 50.0), (1, 75.0)])
            
            # Non-blocking mode - for simulator event loops
            # Won't raise exceptions or block the main thread
            success = client.send_sensory_data([(0, 50.0)], blocking=False)
            if not success:
                # Handle failure gracefully
                pass
            ```
        """
        if not self._connected or self._client is None:
            if blocking:
                raise RuntimeError(
                    "Agent not connected. Call connect() first.",
                )
            return False
        
        try:
            # Send via Rust SDK
            self._client.send_sensory_data(neuron_pairs)
            return True
        except Exception as e:
            if blocking:
                logger.error(f"Failed to send sensory data: {e}")
                raise
            else:
                # Non-blocking mode: log warning but don't raise
                logger.debug(f"Non-blocking send failed (continuing): {e}")
                return False

    def send_sensory_bytes(self, payload: bytes):
        """
        Send pre-encoded FEAGI sensory bytes through Rust client.

        Args:
            payload: Serialized FEAGI byte container payload.

        Raises:
            RuntimeError: If not connected.
        """
        if not self._connected or self._client is None:
            raise RuntimeError("Agent not connected. Call connect() first.")

        try:
            self._client.send_sensory_bytes(payload)
        except Exception as e:
            logger.error("Failed to send sensory bytes: %s", e)
            raise

    def set_motor_cortical_ids(self, cortical_ids: List[str]) -> None:
        """
        Configure the motor cortical-ID filter used by receive_motor_data().

        Args:
            cortical_ids: Base64 cortical IDs expected for this agent's motor outputs.
        """
        normalized: List[str] = []
        seen: set[str] = set()
        for cortical_id in cortical_ids:
            cid = str(cortical_id).strip()
            if not cid or cid in seen:
                continue
            seen.add(cid)
            normalized.append(cid)
        self._motor_cortical_ids = normalized or None
    
    def receive_motor_data(
        self,
        blocking: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """
        Receive motor data from FEAGI (non-blocking)
        
        Args:
            blocking: If False, returns None on error instead of raising
        
        Returns:
            Motor data as dictionary if available, None otherwise
            
        Raises:
            RuntimeError: If not connected or not a motor agent
                (only when blocking=True)
        
        Example:
            ```python
            # Blocking mode (default) - raises on error
            motor_data = client.receive_motor_data()
            
            # Non-blocking mode - for simulator event loops
            motor_data = client.receive_motor_data(blocking=False)
            if motor_data:
                # Process commands
                pass
            ```
        """
        if not self._connected or self._client is None:
            if blocking:
                raise RuntimeError(
                    "Agent not connected. Call connect() first.",
                )
            return None
        
        if self.agent_type == AgentType.SENSORY:
            if blocking:
                raise RuntimeError(
                    "Cannot receive motor data - agent is sensory-only",
                )
            return None
        
        try:
            motor_json = self._client.receive_motor_data()
            if motor_json is None:
                return None
            
            # Parse raw XYZP SoA JSON from Rust SDK
            import json
            xyzp_data = json.loads(motor_json)
            
            # Decode XYZP SoA to motor index → power mapping
            # Only decode cortical areas this agent subscribed to
            cortical_ids = self._motor_cortical_ids
            motors = decode_motor_xyzp(xyzp_data, cortical_ids)
            
            # Return in simple format for controllers:
            # {"motor": {0: power, 1: power, ...}}
            return {"motor": motors} if motors else None
            
        except Exception as e:
            if blocking:
                logger.error("Failed to receive motor data: %s", e)
                raise
            else:
                # Non-blocking mode: log debug but don't raise
                logger.debug("Non-blocking receive failed (continuing): %s", e)
                return None

    def _fetch_existing_cortical_areas(
        self,
        cortical_ids: List[str],
    ) -> set[str]:
        """Fetch available cortical areas from FEAGI API."""
        if not self._feagi_host:
            raise RuntimeError("FEAGI host is not configured.")
        if self._feagi_api_port is None:
            raise RuntimeError(
                "feagi_api_port is required for cortical checks.",
            )
        if self._feagi_http_timeout_s is None:
            raise RuntimeError(
                "feagi_http_timeout_s is required for cortical checks.",
            )

        endpoint = (
            f"http://{self._feagi_host}:{self._feagi_api_port}/v1/cortical_area/"
            "multi/cortical_area_properties"
        )
        payload = json.dumps(cortical_ids).encode("utf-8")
        request = urllib.request.Request(
            endpoint,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(
                request,
                timeout=self._feagi_http_timeout_s,
            ) as response:
                body = response.read().decode("utf-8")
        except urllib.error.URLError as e:
            raise RuntimeError(
                f"Failed to query FEAGI cortical areas: {e}",
            ) from e

        try:
            properties = json.loads(body)
        except json.JSONDecodeError as e:
            raise RuntimeError(
                "Invalid FEAGI cortical response (JSON decode failed).",
            ) from e
        if not isinstance(properties, dict):
            raise RuntimeError("Invalid FEAGI cortical response format.")

        available = set()
        for cortical_id, cortical_props in properties.items():
            if cortical_id in cortical_ids and cortical_props is not None:
                available.add(cortical_id)
        return available

    def send_device_configuration(
        self,
        device_registrations_json: str,
        expected_cortical_ids: Optional[List[str]] = None,
    ) -> None:
        """
        Send device/capability registrations to FEAGI via
        ZMQ AgentConfiguration.

        Args:
            device_registrations_json: JSON string matching
                JSONInputOutputDefinition
                (e.g. from ConnectorAgent.export_capabilities_json()).
            expected_cortical_ids: Optional cortical IDs that must exist before
                returning. If provided, device registration is retried and
                verified against FEAGI API.

        Raises:
            RuntimeError: If not connected.
        """
        if not self._connected or self._client is None:
            raise RuntimeError("Agent not connected. Call connect() first.")
        if not expected_cortical_ids:
            self._client.send_device_configuration(device_registrations_json)
            return

        retries = self._motor_registration_retries
        retry_interval_s = self._motor_registration_retry_interval_s
        if retries is None or retries <= 0:
            raise RuntimeError(
                "registration_retries must be configured (> 0).",
            )
        if retry_interval_s is None or retry_interval_s <= 0:
            raise RuntimeError(
                "heartbeat_interval must be configured (> 0).",
            )

        expected_set = sorted(set(expected_cortical_ids))
        missing_set = expected_set
        for attempt in range(1, retries + 1):
            self._client.send_device_configuration(device_registrations_json)
            logger.info(
                "[CFG] Sent device_registrations (attempt %d/%d)",
                attempt,
                retries,
            )

            available = self._fetch_existing_cortical_areas(expected_set)
            missing_set = sorted(set(expected_set) - available)
            if not missing_set:
                logger.info(
                    "[CFG] Device registration verified (%d cortical areas).",
                    len(expected_set),
                )
                return

            logger.warning(
                "[CFG] Missing cortical areas after attempt %d/%d: %s",
                attempt,
                retries,
                missing_set,
            )
            if attempt < retries:
                time.sleep(retry_interval_s)

        raise RuntimeError(
            "Device registration incomplete; missing cortical areas: "
            f"{missing_set}",
        )

    def is_connected(self) -> bool:
        """Check if agent is connected"""
        return (
            self._connected
            and self._client is not None
            and self._client.is_registered()
        )
    
    def disconnect(self, timeout: float = 0.1):
        """
        Disconnect from FEAGI (non-blocking for fast shutdown)
        
        Note: Rust client deregistration is skipped on Ctrl+C shutdown.
        Agent will be cleaned up by FEAGI's heartbeat timeout (60s).
        
        Args:
            timeout: Maximum time to wait (default: 0.1s)
        """
        if self._connected:
            self._connected = False

            # Ensure explicit deregistration/teardown on the Rust side.
            if self._client is not None:
                try:
                    self._client.disconnect()
                except Exception as disconnect_error:
                    logger.warning(
                        "Rust disconnect reported an error for %s: %s",
                        self.agent_id,
                        disconnect_error,
                    )

            self._client = None
            logger.info("[OK] Disconnected %s", self.agent_id)
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - auto-disconnect"""
        self.disconnect()
    
    def __del__(self):
        """Destructor - auto-disconnect"""
        if getattr(self, "_connected", False):
            self.disconnect()
    
    def setup_graceful_shutdown(
        self,
        running_flag: Optional[List[bool]] = None,
    ):
        """
        Setup graceful shutdown with Ctrl+C handling
        
        This installs a signal handler that:
        - First Ctrl+C: Sets running flag to False (graceful shutdown)
        - Second Ctrl+C: Forces immediate exit
        
        Args:
            running_flag: Optional list with single bool element [True/False]
                         If provided, first Ctrl+C will set running_flag[0] =
                         False. If not provided, only the agent's internal
                         state is affected.
        
        Returns:
            The running_flag list (created if not provided)
        
        Example:
            client = FeagiAgentClient("my-agent", AgentType.SENSORY)
            client.configure(...)
            client.connect()
            
            running = client.setup_graceful_shutdown()
            while running[0]:
                # Your agent loop
                pass
        """
        import signal
        import sys
        
        if running_flag is None:
            running_flag = [True]
        
        interrupt_count = [0]
        
        def signal_handler(sig, frame):
            interrupt_count[0] += 1
            if interrupt_count[0] == 1:
                logger.info(
                    "Received interrupt signal - shutting down gracefully...",
                )
                running_flag[0] = False
                self._connected = False  # Stop operations
            else:
                logger.warning("Received second interrupt - forcing exit NOW")
                sys.exit(1)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        return running_flag


# Convenience function for quick agent creation
def create_agent(
    agent_id: str,
    agent_type: AgentType,
    feagi_host: str,
    **kwargs
) -> FeagiAgentClient:
    """
    Quick helper to create and configure an agent
    
    Args:
        agent_id: Unique agent identifier
        agent_type: Agent type (SENSORY, MOTOR, or BOTH)
        feagi_host: FEAGI hostname or IP
        **kwargs: Additional configuration arguments (passed to configure())
    
    Returns:
        Configured (but not connected) FeagiAgentClient
    
    Example:
        ```python
        from feagi_connector import create_agent, AgentType
        
        client = create_agent(
            "my_camera",
            AgentType.SENSORY,
            feagi_host="localhost",
            vision_unit=("camera", 640, 480, 3, "vision", 0)
        )
        client.connect()
        ```
    """
    client = FeagiAgentClient(agent_id, agent_type)
    client.configure(feagi_host=feagi_host, **kwargs)
    return client

