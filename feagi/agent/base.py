"""
Base Agent Class

Abstract base class for all FEAGI agents.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import os


class BaseAgent(ABC):
    """
    Base class for all FEAGI agents.
    
    Subclass this to create agents for specific embodiments.
    Provides a standard interface for hardware initialization,
    sensor/motor mapping, and FEAGI communication.
    
    Attributes:
        agent_id: Unique identifier for this agent
        feagi_host: FEAGI server host
        client: FEAGI PNS client (initialized in connect())
        running: Agent run state
    
    Example:
        class MyRobotAgent(BaseAgent):
            def initialize_hardware(self):
                # Connect to robot
                pass
            
            def map_sensors(self, hw_data):
                # Convert robot data → FEAGI format
                return {"camera": image_bytes}
            
            def map_motors(self, feagi_output):
                # Convert FEAGI commands → robot format
                return motor_commands
    """
    
    def __init__(self, agent_id: str, feagi_host: str = "localhost", capabilities: Optional[Dict[str, Any]] = None):
        """
        Initialize base agent.
        
        Args:
            agent_id: Unique identifier for this agent
            feagi_host: FEAGI server hostname or IP
            capabilities: Agent capabilities (sensors, motors, etc.)
        """
        self.agent_id = agent_id
        self.feagi_host = feagi_host
        self.capabilities = capabilities or {}
        self.client = None
        self.running = False
    
    @abstractmethod
    def initialize_hardware(self):
        """
        Initialize robot/simulator/device hardware.
        
        Called once during startup before main loop.
        Override this to connect to your hardware.
        """
        pass
    
    @abstractmethod
    def map_sensors(self, hardware_data: Any) -> Dict[str, bytes]:
        """
        Convert hardware sensor data to FEAGI format.
        
        Args:
            hardware_data: Raw sensor data from hardware
        
        Returns:
            Dictionary mapping sensor names to byte arrays
            Example: {"camera": image_bytes, "distance": distance_bytes}
        """
        pass
    
    @abstractmethod
    def map_motors(self, feagi_output: Dict[str, Any]) -> Any:
        """
        Convert FEAGI motor commands to hardware format.
        
        Args:
            feagi_output: Motor commands from FEAGI
        
        Returns:
            Hardware-specific motor commands
        """
        pass
    
    async def connect(self):
        """
        Connect to FEAGI server.
        
        Creates PNS client and establishes connection.
        Call this before run().
        """
        from feagi.pns import FeagiAgentClient, AgentType
        
        # Determine agent type based on FEAGI 2.0 unit/group capability format
        motor_unit = self.capabilities.get("motor_unit")
        motor_units = self.capabilities.get("motor_units")
        vision_unit = self.capabilities.get("vision_unit")
        has_motor = motor_unit is not None or motor_units is not None
        has_vision = vision_unit is not None
        
        if has_motor and has_vision:
            agent_type = AgentType.BOTH
        elif has_motor:
            agent_type = AgentType.MOTOR
        elif has_vision:
            agent_type = AgentType.SENSORY
        else:
            raise ValueError(
                "Agent must define vision_unit and/or motor_unit(s) capabilities"
            )
        
        self.client = FeagiAgentClient(self.agent_id, agent_type)

        auth_token_b64 = self.capabilities.get("auth_token_b64") or os.environ.get(
            "FEAGI_AUTH_TOKEN_B64"
        )
        if not auth_token_b64:
            raise ValueError(
                "Missing auth_token_b64. Provide capabilities['auth_token_b64'] "
                "or set FEAGI_AUTH_TOKEN_B64."
            )
        connection_timeout_raw = self.capabilities.get("connection_timeout_ms")
        if connection_timeout_raw is None:
            connection_timeout_raw = os.environ.get("FEAGI_CONNECTION_TIMEOUT_MS")
        if connection_timeout_raw is None:
            raise ValueError(
                "Missing connection_timeout_ms. Provide capabilities['connection_timeout_ms'] "
                "or set FEAGI_CONNECTION_TIMEOUT_MS."
            )
        registration_retries_raw = self.capabilities.get("registration_retries")
        if registration_retries_raw is None:
            registration_retries_raw = os.environ.get("FEAGI_REGISTRATION_RETRIES")
        if registration_retries_raw is None:
            raise ValueError(
                "Missing registration_retries. Provide capabilities['registration_retries'] "
                "or set FEAGI_REGISTRATION_RETRIES."
            )
        
        # Convert capabilities dict to client config format (FEAGI 2.0)
        motor_cap = None
        motor_caps = None
        if motor_unit:
            motor_cap = (
                motor_unit["modality"],
                motor_unit["output_count"],
                motor_unit["unit"],
                motor_unit["group"],
            )
        if motor_units:
            motor_caps = (
                motor_units["modality"],
                motor_units["output_count"],
                motor_units["source_units"],
            )

        vision_cap = None
        if vision_unit:
            vision_cap = (
                vision_unit["modality"],
                vision_unit["width"],
                vision_unit["height"],
                vision_unit["channels"],
                vision_unit["unit"],
                vision_unit["group"],
            )
        
        # Add custom capabilities for sensors
        custom_caps = {}
        if self.capabilities.get("proximity"):
            custom_caps["proximity"] = {"count": self.capabilities["proximity"].get("count", 1)}
        if self.capabilities.get("infrared"):
            custom_caps["infrared"] = {"count": self.capabilities["infrared"].get("count", 2)}
        
        self.client.configure(
            feagi_host=self.feagi_host,
            motor_port=5564,  # TODO: Get from FEAGI config (currently using transport.zmq_motor_port)
            vision_unit=vision_cap,
            motor_unit=motor_cap,
            motor_units=motor_caps,
            custom_capabilities=custom_caps if custom_caps else None,
            connection_timeout_ms=int(connection_timeout_raw),
            registration_retries=int(registration_retries_raw),
            auth_token_b64=auth_token_b64,
        )
        self.client.connect()  # Synchronous, not async!
    
    async def run(self):
        """
        Main agent loop.
        
        Continuously:
        1. Read sensors from hardware
        2. Convert to FEAGI format
        3. Send to FEAGI
        4. Receive motor commands
        5. Convert and execute
        
        Override for custom behavior.
        """
        self.running = True
        self.initialize_hardware()
        
        while self.running:
            # Get sensor data from hardware
            hw_data = await self.read_sensors()
            
            # Convert to FEAGI format
            feagi_sensors = self.map_sensors(hw_data)
            
            # Send to FEAGI
            await self.client.send_sensory_data(feagi_sensors)
            
            # Receive motor commands
            feagi_motors = await self.client.receive_motor_data()
            
            # Convert and execute
            hw_commands = self.map_motors(feagi_motors)
            await self.execute_commands(hw_commands)
    
    async def stop(self):
        """
        Stop agent gracefully.
        
        Disconnects from FEAGI and shuts down hardware.
        """
        self.running = False
        if self.client:
            self.client.disconnect()  # Synchronous, not async!
    
    async def read_sensors(self) -> Any:
        """
        Read sensor data from hardware.
        
        Override this to implement hardware-specific sensor reading.
        Default implementation returns None.
        
        Returns:
            Hardware sensor data (hardware-specific format)
        """
        return None
    
    async def execute_commands(self, commands: Any):
        """
        Execute motor commands on hardware.
        
        Override this to implement hardware-specific command execution.
        Default implementation does nothing.
        
        Args:
            commands: Hardware-specific motor commands
        """
        pass

