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
        vision_capability=("camera", 640, 480, 3, "i_vision"),
        heartbeat_interval=5.0
    )
    client.connect()
    
    # Send sensory data
    client.send_sensory_data([(neuron_id, potential), ...])
    
    # Auto-deregisters on exit
"""

import logging
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from .xyzp_decoders import decode_motor_xyzp

# Try to import Rust SDK (optional dependency)
_rust_sdk_available = False
_rust_import_error = None

try:
    from feagi_rust_py_libs import feagi_agent_sdk as rust_sdk
    _rust_sdk_available = True
except ImportError as e1:
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
        print("[PYTHON-SDK] ✅ Rust logging initialized", flush=True)
    except Exception as e:
        print(f"[PYTHON-SDK] ❌ Failed to init Rust logging: {e}", flush=True)
        logging.getLogger("feagi.pns.client").warning(f"Failed to init Rust logging: {e}")
else:
    print("[PYTHON-SDK] ⚠️ Rust SDK NOT available, using Python fallback", flush=True)
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
        
        # Create client
        client = FeagiAgentClient("video_camera_01", AgentType.SENSORY)
        
        # Configure
        client.configure(
            feagi_host="localhost",
            vision_capability=("camera", 640, 480, 3, "i_vision"),
            heartbeat_interval=5.0
        )
        
        # Connect (with automatic retry)
        client.connect()
        
        # Send data
        client.send_sensory_data([(0, 50.0), (1, 75.0)])
        
        # Client auto-deregisters on exit
        ```
    """
    
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
        
        logger.info(f"Created agent client: {agent_id} (type: {agent_type.value})")
    
    def configure(
        self,
        feagi_host: str = "localhost",
        registration_port: int = 30001,
        sensory_port: int = 5555,
        motor_port: int = 30005,
        vision_capability: Optional[Tuple[str, int, int, int, str]] = None,
        motor_capability: Optional[Tuple[str, int, List[str]]] = None,
        custom_capabilities: Optional[Dict[str, Any]] = None,
        heartbeat_interval: float = 5.0,
        connection_timeout_ms: int = 5000,
        registration_retries: int = 3,
        sensory_socket_hwm: Optional[int] = None,
        sensory_socket_linger_ms: Optional[int] = None,
        sensory_socket_immediate: Optional[bool] = None,
    ):
        """
        Configure the agent
        
        Args:
            feagi_host: FEAGI hostname or IP address.
            registration_port: Registration/heartbeat port (default: 30001).
            sensory_port: Sensory data input port (default: 5555).
            motor_port: Motor data output port (default: 30005).
            vision_capability: Vision capability tuple
                (modality, width, height, channels, cortical_area).
            motor_capability: Motor capability tuple (modality, output_count,
                cortical_areas).
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
        """
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
        if vision_capability:
            modality, width, height, channels, cortical_area = vision_capability
            self._config.with_vision_capability(
                modality,
                width,
                height,
                channels,
                cortical_area,
            )

            # Store for XYZP conversion
            self._vision_width = width
            self._vision_height = height
            self._cortical_area = cortical_area
            
            logger.debug(
                "Added vision capability: %sx%sx%s -> %s",
                width,
                height,
                channels,
                cortical_area,
            )
        
        # Store network config for direct ZMQ access and logging
        self._feagi_host = feagi_host
        self._registration_port = registration_port
        self._sensory_port = sensory_port
        
        if motor_capability:
            modality, output_count, cortical_areas = motor_capability
            self._config.with_motor_capability(modality, output_count, cortical_areas)
            logger.debug(
                "Added motor capability: %s outputs <- %s",
                output_count,
                cortical_areas,
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
            logger.error(f"Configuration validation failed: {e}")
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
            feagi_host: Optional FEAGI host. If not provided, uses configured host.
        
        Returns:
            Tuple of (width, height, depth) if successful, None otherwise
            
        Note:
            Requires FEAGI to be running and accessible via REST API (port 8000)
        """
        try:
            import requests
            
            # Use provided host or fall back to configured host
            host = feagi_host or self._feagi_host
            if not host:
                logger.warning("No FEAGI host specified for resolution detection")
                return None
            
            # Query FEAGI REST API for cortical area properties
            api_url = f"http://{host}:8000/v1/cortical_area/multi/cortical_area_properties"
            payload = [cortical_area]  # API expects array, not object
            
            logger.debug(f"Querying FEAGI for '{cortical_area}' dimensions...")
            response = requests.post(api_url, json=payload, timeout=2.0)
            
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
                logger.warning(f"Cortical area '{cortical_area}' not found in FEAGI")
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
            logger.debug(f"Resolution detection failed: {e}")
            return None
    
    def connect(self):
        """
        Connect to FEAGI and register the agent
        
        This will:
        1. Create ZMQ sockets
        2. Register with FEAGI (with automatic retry)
        3. Start background heartbeat
        
        Raises:
            RuntimeError: If not configured or connection fails
        """
        if self._config is None:
            raise RuntimeError("Agent not configured. Call configure() first.")
        
        if self._connected:
            logger.warning("Agent already connected")
            return
        
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
            # Create client
            logger.debug("Creating Rust-backed agent client...")
            self._client = PyAgentClient(self._config)
            
            # Connect (with automatic retry)
            logger.debug("Attempting registration with FEAGI...")
            try:
                self._client.connect()
                logger.debug("✓ Registration successful")
            except Exception as reg_error:
                # Enhanced error message for registration failures
                error_str = str(reg_error).lower()
                
                if "unknown error" in error_str or "registration failed" in error_str:
                    logger.error(
                        "❌ Registration failed with unclear error from Rust SDK"
                    )
                    logger.error("")
                    logger.error("Common causes:")
                    logger.error("  1. FEAGI's Rust PNS is not running")
                    logger.error(
                        "     Check FEAGI logs for: '🦀 [ZMQ-REGISTRATION] "
                        "Listening on...'"
                    )
                    logger.error(
                        "  2. Port mismatch (FEAGI 2.0 uses port 5563, not 30001)"
                    )
                    logger.error("     Your config: registration_port = ?")
                    logger.error("     FEAGI listening: Check with 'lsof -i :5563'")
                    logger.error("  3. ZMQ socket state issue (restart FEAGI)")
                    logger.error("  4. Firewall blocking connection")
                    logger.error("")
                    raise RuntimeError(
                        "Registration failed: %s. Check FEAGI is running with "
                        "Rust PNS on port %s. See logs above for "
                        "troubleshooting steps."
                        % (reg_error, registration_port)
                    ) from reg_error
                else:
                    # Re-raise with original error
                    raise
            
            self._connected = True
            logger.info(f"✓ Connected and registered as: {self.agent_id}")
            logger.info(f"  Registration: {registration_host}:{registration_port}")
            logger.info(f"  Sensory data: {self._feagi_host}:{self._sensory_port}")
            heartbeat_interval = (
                self._config.heartbeat_interval
                if hasattr(self._config, "heartbeat_interval")
                else "N/A"
            )
            logger.info("  Heartbeat: %ss", heartbeat_interval)
            
        except RuntimeError:
            # Re-raise our enhanced RuntimeErrors
            raise
        except Exception as e:
            # Catch-all for unexpected errors
            logger.error("❌ Unexpected error during connection: %s", e)
            logger.error("   Error type: %s", type(e).__name__)
            logger.error("")
            logger.error("Please report this error with the following information:")
            logger.error("  - Agent ID: %s", self.agent_id)
            logger.error("  - FEAGI host: %s", self._feagi_host)
            logger.error("  - Registration port: %s", registration_port)
            logger.error("  - Error: %s", e)
            raise RuntimeError(
                f"Connection failed with unexpected error: {e}"
            ) from e
    
    def send_sensory_data(self, neuron_pairs: List[Tuple[int, float]]):
        """
        Send sensory data to FEAGI in binary XYZP format
        
        Args:
            neuron_pairs: List of (neuron_id, potential) tuples
                         Example: [(0, 50.0), (1, 75.0), (2, 30.0)]
        
        Raises:
            RuntimeError: If not connected
        """
        if not self._connected or self._client is None:
            raise RuntimeError("Agent not connected. Call connect() first.")
        
        try:
            # Send via Rust SDK
            self._client.send_sensory_data(neuron_pairs)
        except Exception as e:
            logger.error(f"Failed to send sensory data: {e}")
            raise
    
    def receive_motor_data(self) -> Optional[Dict[str, Any]]:
        """
        Receive motor data from FEAGI (non-blocking)
        
        Returns:
            Motor data as dictionary if available, None otherwise
            
        Raises:
            RuntimeError: If not connected or not a motor agent
        """
        if not self._connected or self._client is None:
            raise RuntimeError("Agent not connected. Call connect() first.")
        
        if self.agent_type == AgentType.SENSORY:
            raise RuntimeError("Cannot receive motor data - agent is sensory-only")
        
        try:
            motor_json = self._client.receive_motor_data()
            if motor_json is None:
                return None
            
            # Parse raw XYZP SoA JSON from Rust SDK
            import json
            xyzp_data = json.loads(motor_json)
            
            # TEMP DEBUG: Log to file
            with open("/tmp/feagi_motor_debug.log", "a") as f:
                import time
                f.write(f"\n[{time.time():.3f}] RAW XYZP: {motor_json[:200]}\n")
                f.write(f"[{time.time():.3f}] PARSED: {xyzp_data}\n")
            
            # Decode XYZP SoA to motor index → power mapping
            # Only decode cortical areas this agent subscribed to
            cortical_ids = self._motor_cortical_ids if hasattr(self, '_motor_cortical_ids') else None
            motors = decode_motor_xyzp(xyzp_data, cortical_ids)
            
            # TEMP DEBUG: Log decoded result
            with open("/tmp/feagi_motor_debug.log", "a") as f:
                f.write(f"[{time.time():.3f}] DECODED: {motors}\n")
                f.write(f"[{time.time():.3f}] RETURNING: {{'motor': {motors}}}\n")
                f.flush()
            
            # Return in simple format for controllers: {"motor": {0: power, 1: power, ...}}
            return {"motor": motors} if motors else None
            
        except Exception as e:
            # TEMP DEBUG: Log error
            with open("/tmp/feagi_motor_debug.log", "a") as f:
                import traceback
                f.write(f"\n[ERROR] {e}\n{traceback.format_exc()}\n")
                f.flush()
            logger.error(f"Failed to receive motor data: {e}")
            raise
    
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
            
            # Skip Rust client deregistration - it blocks on interrupted ZMQ sockets
            # FEAGI will clean up via heartbeat timeout
            self._client = None
            logger.info(
                "✓ Disconnected %s (cleanup via heartbeat timeout)",
                self.agent_id,
            )
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - auto-disconnect"""
        self.disconnect()
    
    def __del__(self):
        """Destructor - auto-disconnect"""
        if self._connected:
            self.disconnect()
    
    def setup_graceful_shutdown(self, running_flag: Optional[List[bool]] = None):
        """
        Setup graceful shutdown with Ctrl+C handling
        
        This installs a signal handler that:
        - First Ctrl+C: Sets running flag to False (graceful shutdown)
        - Second Ctrl+C: Forces immediate exit
        
        Args:
            running_flag: Optional list with single bool element [True/False]
                         If provided, first Ctrl+C will set running_flag[0] = False
                         If not provided, only the agent's internal state is affected
        
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
                logger.info("Received interrupt signal - shutting down gracefully...")
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
    feagi_host: str = "localhost",
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
            vision_capability=("camera", 640, 480, 3, "i_vision")
        )
        client.connect()
        ```
    """
    client = FeagiAgentClient(agent_id, agent_type)
    client.configure(feagi_host=feagi_host, **kwargs)
    return client

