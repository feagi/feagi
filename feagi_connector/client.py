"""
FEAGI Client - Main client interface for agent connectivity

This module implements the main client interface for connecting to FEAGI.
It abstracts transport details and provides a simple, protocol-agnostic API.
"""

import logging
from typing import Dict, Any, Optional, Callable, Union, List, Tuple
import time

from .zmq import ZmqFeagiClient

# Configure logging
logger = logging.getLogger("feagi_connector")


class FeagiClient:
    """
    Main client interface for connecting to FEAGI.
    
    This class provides a high-level interface for agents to connect to FEAGI,
    abstracting the underlying transport protocol and message formats.
    
    By default, it uses ZeroMQ for communication, but other transports can be
    added in the future.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        agent_id: str = None,
        agent_type: str = "generic",
        transport: str = "zmq",
        **kwargs
    ):
        """
        Initialize the FEAGI client.
        
        Args:
            host: FEAGI host address
            agent_id: Unique identifier for this agent (generated if not provided)
            agent_type: Type of agent (for categorization)
            transport: Transport protocol to use ("zmq", etc.)
            **kwargs: Additional transport-specific parameters
        """
        self.host = host
        self.agent_id = agent_id or f"agent-{int(time.time() * 1000)}"
        self.agent_type = agent_type
        
        # Create client implementation based on transport
        if transport == "zmq":
            self._client = ZmqFeagiClient(host=host, **kwargs)
        else:
            raise ValueError(f"Unsupported transport: {transport}")
        
        # State tracking
        self._registered = False
        
    async def connect(self) -> bool:
        """
        Connect to FEAGI and register the agent.
        
        Returns:
            True if connected and registered successfully, False otherwise
        """
        # Connect the transport
        connected = await self._client.connect()
        if not connected:
            logger.error("Failed to connect to FEAGI")
            return False
            
        # Register the agent
        try:
            response = await self._client.register_agent(
                self.agent_id, 
                self.agent_type
            )
            
            # Process registration response
            if response.get("status") == "success":
                self._registered = True
                logger.info(f"Agent {self.agent_id} registered successfully")
                return True
            else:
                logger.error(f"Agent registration failed: {response}")
                await self._client.disconnect()
                return False
                
        except Exception as e:
            logger.exception(f"Error registering agent: {e}")
            await self._client.disconnect()
            return False
    
    async def disconnect(self) -> bool:
        """
        Disconnect from FEAGI and deregister the agent.
        
        Returns:
            True if disconnected successfully, False otherwise
        """
        try:
            # Only deregister if previously registered
            if self._registered:
                await self._client.deregister_agent(self.agent_id)
                self._registered = False
                
            # Disconnect transport
            await self._client.disconnect()
            logger.info(f"Agent {self.agent_id} disconnected")
            return True
            
        except Exception as e:
            logger.exception(f"Error during disconnect: {e}")
            return False
    
    async def send_sensory_data(self, channel_id: int, data: bytes) -> bool:
        """
        Send sensory data to FEAGI.
        
        Args:
            channel_id: Sensory channel ID
            data: Binary sensory data
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self._registered:
            logger.error("Cannot send sensory data: not registered")
            return False
            
        try:
            await self._client.send_sensory_data(channel_id, data)
            return True
        except Exception as e:
            logger.exception(f"Error sending sensory data: {e}")
            return False
    
    async def register_motor_callback(self, callback: Callable[[int, bytes], None]) -> bool:
        """
        Register a callback for receiving motor data.
        
        Args:
            callback: Function to call when motor data is received
                     (parameters: channel_id, data)
            
        Returns:
            True if registered successfully, False otherwise
        """
        if not self._registered:
            logger.error("Cannot register motor callback: not registered")
            return False
            
        try:
            await self._client.register_motor_callback(callback)
            return True
        except Exception as e:
            logger.exception(f"Error registering motor callback: {e}")
            return False
    
    async def register_visualization_callbacks(
        self,
        activity_callback: Optional[Callable[[bytes], None]] = None,
        structure_callback: Optional[Callable[[bytes], None]] = None
    ) -> bool:
        """
        Register callbacks for receiving visualization data.
        
        Args:
            activity_callback: Function to call when neural activity data is received
            structure_callback: Function to call when brain structure data is received
            
        Returns:
            True if registered successfully, False otherwise
        """
        if not self._registered:
            logger.error("Cannot register visualization callbacks: not registered")
            return False
            
        try:
            await self._client.register_visualization_callbacks(
                activity_callback=activity_callback,
                structure_callback=structure_callback
            )
            return True
        except Exception as e:
            logger.exception(f"Error registering visualization callbacks: {e}")
            return False
    
    async def send_heartbeat(self) -> bool:
        """
        Send a heartbeat to FEAGI to maintain the connection.
        
        Returns:
            True if heartbeat acknowledged, False otherwise
        """
        if not self._registered:
            logger.error("Cannot send heartbeat: not registered")
            return False
            
        try:
            response = await self._client.send_heartbeat(self.agent_id)
            return response.get("type") == "heartbeat_response"
        except Exception as e:
            logger.exception(f"Error sending heartbeat: {e}")
            return False
            
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI status information.
        
        Returns:
            Dictionary with status information
        """
        try:
            return await self._client.get_status()
        except Exception as e:
            logger.exception(f"Error getting status: {e}")
            return {"error": str(e)} 