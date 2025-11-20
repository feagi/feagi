"""
Base Transport Interface

Abstract base class for all transport mechanisms.
"""

from abc import ABC, abstractmethod
from typing import Optional, Callable, Any


class BaseTransport(ABC):
    """
    Abstract base class for robot transport mechanisms.
    
    Subclass this to create transports for different connection types
    (Bluetooth, Serial, WebSocket, etc.)
    """
    
    def __init__(self, config: dict):
        """
        Initialize transport with configuration.
        
        Args:
            config: Transport-specific configuration dictionary
        """
        self.config = config
        self.connected = False
        self.on_data_callback: Optional[Callable[[bytes], None]] = None
    
    @abstractmethod
    async def connect(self) -> None:
        """
        Establish connection to the robot.
        
        Raises:
            ConnectionError: If connection fails
        """
        pass
    
    @abstractmethod
    async def disconnect(self) -> None:
        """
        Close connection to the robot.
        """
        pass
    
    @abstractmethod
    async def send(self, data: bytes) -> None:
        """
        Send data to the robot.
        
        Args:
            data: Raw bytes to send
            
        Raises:
            ConnectionError: If not connected or send fails
        """
        pass
    
    @abstractmethod
    async def receive(self) -> bytes:
        """
        Receive data from the robot.
        
        Returns:
            Raw bytes received from robot
            
        Raises:
            ConnectionError: If not connected or receive fails
        """
        pass
    
    def set_data_callback(self, callback: Callable[[bytes], None]) -> None:
        """
        Set callback for incoming data (for event-driven transports).
        
        Args:
            callback: Function to call when data arrives
        """
        self.on_data_callback = callback
    
    async def __aenter__(self):
        """Async context manager entry"""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        await self.disconnect()

