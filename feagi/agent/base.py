"""
Base Agent Class

Abstract base class for all FEAGI agents.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional


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
        
        # Determine agent type based on capabilities (feagi-sensorimotor format: "input"/"output")
        # Support both old format ("motor"/"sensory") and new format ("input"/"output") for backward compatibility
        has_motor = self.capabilities.get("motor") is not None or self.capabilities.get("output") is not None
        has_vision = self.capabilities.get("vision") is not None or self.capabilities.get("input") is not None
        
        if has_motor and has_vision:
            agent_type = AgentType.BOTH
        elif has_motor:
            agent_type = AgentType.MOTOR
        elif has_vision:
            agent_type = AgentType.SENSORY
        else:
            raise ValueError("Agent must have at least motor or vision capabilities")
        
        self.client = FeagiAgentClient(self.agent_id, agent_type)
        
        # Convert capabilities dict to client config format
        # Support both "motor" (old) and "output" (feagi-sensorimotor) formats
        motor_cap = None
        motor_config = self.capabilities.get("motor") or self.capabilities.get("output")
        if motor_config:
            if isinstance(motor_config, list):
                # New format: {"output": ["cortical_id1", "cortical_id2"]}
                motor_count = len(motor_config)
                cortical_ids_b64 = motor_config
            else:
                # Old format: {"motor": {"count": 2, "cortical_ids": [...]}}
                motor_count = motor_config.get("count", 2)
                cortical_ids_b64 = motor_config.get("cortical_ids", [])
                if not cortical_ids_b64:
                    # Fallback to generic names
                    cortical_ids_b64 = [f"m{i}" for i in range(motor_count)]
            
            motor_cap = ("motor", motor_count, cortical_ids_b64)
            print(f"🎮 [SDK DEBUG] Motor capability: count={motor_count}, cortical_ids={cortical_ids_b64}")
        
        # Add custom capabilities for sensors
        custom_caps = {}
        if self.capabilities.get("proximity"):
            custom_caps["proximity"] = {"count": self.capabilities["proximity"].get("count", 1)}
        if self.capabilities.get("infrared"):
            custom_caps["infrared"] = {"count": self.capabilities["infrared"].get("count", 2)}
        
        self.client.configure(
            feagi_host=self.feagi_host,
            motor_port=5564,  # TODO: Get from FEAGI config (currently using transport.zmq_motor_port)
            motor_capability=motor_cap,
            custom_capabilities=custom_caps if custom_caps else None
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

