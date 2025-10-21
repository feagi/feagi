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
            logger.info(f"Added vision capability: {width}x{height}x{channels} → {cortical_area}")
        
        if motor_capability:
            modality, output_count, cortical_areas = motor_capability
            self._config.with_motor_capability(modality, output_count, cortical_areas)
            logger.info(f"Added motor capability: {output_count} outputs ← {cortical_areas}")
        
        if custom_capabilities:
            import json
            for key, value in custom_capabilities.items():
                self._config.with_custom_capability(key, json.dumps(value))
                logger.info(f"Added custom capability: {key}")
        
        # Validate configuration
        try:
            self._config.validate()
            logger.info("Configuration validated successfully")
        except Exception as e:
            logger.error(f"Configuration validation failed: {e}")
            raise
    
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
        
        try:
            # Create client
            self._client = PyAgentClient(self._config)
            
            # Connect (with automatic retry)
            self._client.connect()
            
            self._connected = True
            logger.info(f"✓ Connected and registered as: {self.agent_id}")
            logger.info(f"  Heartbeat: {self._config.heartbeat_interval if hasattr(self._config, 'heartbeat_interval') else 'N/A'}s")
            
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            raise RuntimeError(f"Connection failed: {e}")
    
    def send_sensory_data(self, neuron_pairs: List[Tuple[int, float]]):
        """
        Send sensory data to FEAGI
        
        Args:
            neuron_pairs: List of (neuron_id, potential) tuples
                         Example: [(0, 50.0), (1, 75.0), (2, 30.0)]
        
        Raises:
            RuntimeError: If not connected
        """
        if not self._connected or self._client is None:
            raise RuntimeError("Agent not connected. Call connect() first.")
        
        try:
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
    
    def disconnect(self):
        """
        Disconnect from FEAGI
        
        This will:
        1. Stop heartbeat
        2. Deregister agent
        3. Close ZMQ sockets
        
        Note: Also called automatically on object deletion
        """
        if self._connected:
            logger.info(f"Disconnecting agent: {self.agent_id}")
            self._connected = False
            # Rust client will auto-deregister on drop
            self._client = None
            logger.info("✓ Disconnected")
    
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

