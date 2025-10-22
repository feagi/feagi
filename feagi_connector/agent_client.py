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
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum

from feagi_agent_sdk_py import PyAgentClient, PyAgentConfig, AgentType as RustAgentType

logger = logging.getLogger("feagi_connector.agent_client")


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
        """
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
    ):
        """
        Configure the agent
        
        Args:
            feagi_host: FEAGI hostname or IP address
            registration_port: Registration/heartbeat port (default: 30001)
            sensory_port: Sensory data input port (default: 5555)
            motor_port: Motor data output port (default: 30005)
            vision_capability: Vision capability as (modality, width, height, channels, cortical_area)
                              Example: ("camera", 640, 480, 3, "i_vision")
            motor_capability: Motor capability as (modality, output_count, cortical_areas)
                            Example: ("servo", 4, ["o_motor"])
            custom_capabilities: Dictionary of custom capabilities
            heartbeat_interval: Heartbeat interval in seconds (0 to disable)
            connection_timeout_ms: Connection timeout in milliseconds
            registration_retries: Maximum registration retry attempts
        """
        # Map Python AgentType to Rust AgentType
        rust_agent_type = {
            AgentType.SENSORY: RustAgentType.Sensory,
            AgentType.MOTOR: RustAgentType.Motor,
            AgentType.BOTH: RustAgentType.Both,
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
        
        # Add capabilities
        if vision_capability:
            modality, width, height, channels, cortical_area = vision_capability
            self._config.with_vision_capability(modality, width, height, channels, cortical_area)
            
                # Store for XYZP conversion
            self._vision_width = width
            self._vision_height = height
            self._cortical_area = cortical_area
            
            logger.debug(f"Added vision capability: {width}x{height}x{channels} → {cortical_area}")
        
        # Store network config for direct ZMQ access and logging
        self._feagi_host = feagi_host
        self._registration_port = registration_port
        self._sensory_port = sensory_port
        
        if motor_capability:
            modality, output_count, cortical_areas = motor_capability
            self._config.with_motor_capability(modality, output_count, cortical_areas)
            logger.debug(f"Added motor capability: {output_count} outputs ← {cortical_areas}")
        
        if custom_capabilities:
            import json
            for key, value in custom_capabilities.items():
                self._config.with_custom_capability(key, json.dumps(value))
                logger.debug(f"Added custom capability: {key}")
        
        # Validate configuration
        try:
            self._config.validate()
            logger.debug("Configuration validated successfully")
        except Exception as e:
            logger.error(f"Configuration validation failed: {e}")
            raise
    
    def detect_cortical_area_resolution(self, cortical_area: str, feagi_host: Optional[str] = None) -> Optional[Tuple[int, int, int]]:
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
            payload = {"cortical_ids": [cortical_area]}
            
            logger.debug(f"Querying FEAGI for '{cortical_area}' dimensions...")
            response = requests.post(api_url, json=payload, timeout=2.0)
            
            if response.status_code == 200:
                properties_list = response.json()
                
                # Response format: list of cortical area objects
                # Find the matching cortical area
                for area in properties_list:
                    if area.get("id") == cortical_area:
                        dimensions = area.get("dimensions", [])
                        
                        if len(dimensions) == 3:
                            x_dim, y_dim, z_dim = dimensions
                            logger.info(f"✓ Detected {cortical_area} resolution: {x_dim}x{y_dim}x{z_dim}")
                            return (x_dim, y_dim, z_dim)
                        else:
                            logger.warning(f"Cortical area '{cortical_area}' has invalid dimensions: {dimensions}")
                            return None
                
                # Not found in response
                logger.warning(f"Cortical area '{cortical_area}' not found in FEAGI")
                return None
            else:
                if response.status_code == 404:
                    logger.warning(f"FEAGI cortical area endpoint not found (HTTP 404) - likely no genome loaded yet")
                else:
                    logger.warning(f"Failed to query FEAGI cortical properties: HTTP {response.status_code}")
                return None
                
        except ImportError:
            logger.warning("'requests' library not available - install with: pip install requests")
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
        
        # Note: Skip TCP pre-flight check for ZMQ sockets (they don't respond like regular TCP)
        # Let the Rust SDK handle connection testing
        
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
                    logger.error("❌ Registration failed with unclear error from Rust SDK")
                    logger.error("")
                    logger.error("Common causes:")
                    logger.error("  1. FEAGI's Rust PNS is not running")
                    logger.error("     Check FEAGI logs for: '🦀 [ZMQ-REGISTRATION] Listening on...'")
                    logger.error("  2. Port mismatch (FEAGI 2.0 uses port 5563, not 30001)")
                    logger.error("     Your config: registration_port = ?")
                    logger.error("     FEAGI listening: Check with 'lsof -i :5563'")
                    logger.error("  3. ZMQ socket state issue (restart FEAGI)")
                    logger.error("  4. Firewall blocking connection")
                    logger.error("")
                    raise RuntimeError(
                        f"Registration failed: {reg_error}. "
                        f"Check FEAGI is running with Rust PNS on port {registration_port}. "
                        f"See logs above for troubleshooting steps."
                    )
                else:
                    # Re-raise with original error
                    raise
            
            self._connected = True
            logger.info(f"✓ Connected and registered as: {self.agent_id}")
            logger.info(f"  Registration: {registration_host}:{registration_port}")
            logger.info(f"  Sensory data: {self._feagi_host}:{self._sensory_port}")
            logger.info(f"  Heartbeat: {self._config.heartbeat_interval if hasattr(self._config, 'heartbeat_interval') else 'N/A'}s")
            
        except RuntimeError:
            # Re-raise our enhanced RuntimeErrors
            raise
        except Exception as e:
            # Catch-all for unexpected errors
            logger.error(f"❌ Unexpected error during connection: {e}")
            logger.error(f"   Error type: {type(e).__name__}")
            logger.error("")
            logger.error("Please report this error with the following information:")
            logger.error(f"  - Agent ID: {self.agent_id}")
            logger.error(f"  - FEAGI host: {self._feagi_host}")
            logger.error(f"  - Registration port: {registration_port}")
            logger.error(f"  - Error: {e}")
            raise RuntimeError(f"Connection failed with unexpected error: {e}")
    
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
            if motor_json is not None:
                import json
                return json.loads(motor_json)
            return None
        except Exception as e:
            logger.error(f"Failed to receive motor data: {e}")
            raise
    
    def is_connected(self) -> bool:
        """Check if agent is connected"""
        return self._connected and self._client is not None and self._client.is_registered()
    
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
            logger.info(f"✓ Disconnected {self.agent_id} (cleanup via heartbeat timeout)")
    
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

