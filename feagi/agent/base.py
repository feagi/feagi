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
        
        self.client = FeagiAgentClient(self.agent_id, AgentType.BOTH)
        
        # Convert capabilities dict to client config format
        motor_cap = None
        if self.capabilities.get("motor"):
            motor_count = self.capabilities["motor"].get("count", 2)
            motor_cap = ("motor", motor_count, [f"m{i}" for i in range(motor_count)])
        
        # Add custom capabilities for sensors
        custom_caps = {}
        if self.capabilities.get("proximity"):
            custom_caps["proximity"] = {"count": self.capabilities["proximity"].get("count", 1)}
        if self.capabilities.get("infrared"):
            custom_caps["infrared"] = {"count": self.capabilities["infrared"].get("count", 2)}
        
        self.client.configure(
            feagi_host=self.feagi_host,
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

