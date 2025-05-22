"""
FEAGI Client

Main client interface for connecting to FEAGI and exchanging data through four stream types:
1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
2. Sensory Stream: One-directional binary data flow from agent to FEAGI
3. Motor Stream: One-directional binary data flow from FEAGI to agent
4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
"""

import asyncio
import json
import logging
import uuid
from typing import Dict, Any, Optional, List, Union, Tuple, Callable, Awaitable

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

from feagi_connector.protocols import FSMPChannel, FCPMessageType, FVPMessageType
from feagi_connector.api.command_client import FeagiControlClient
from feagi_connector.api.sensory_client import FeagiSensoryClient
from feagi_connector.api.motor_client import FeagiMotorClient
from feagi_connector.api.viz_client import FeagiVizClient
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ
from feagi_connector.utils.processing import (
    infer_byte_structure_type_python as infer_byte_structure_type,
    extract_sub_structures_python as extract_sub_structures
)

# Configure logging
logger = logging.getLogger("feagi_connector")


class FeagiClient:
    """
    High-level client for connecting to FEAGI.
    
    This client provides a unified interface for all FEAGI communication streams:
    1. Control Stream: Bidirectional JSON-based messaging for control commands and health status
    2. Sensory Stream: One-directional binary data flow from agent to FEAGI
    3. Motor Stream: One-directional binary data flow from FEAGI to agent
    4. Visualization Stream: One-directional binary data flow from FEAGI to agent for neuron activity
    
    It handles connection management, agent registration, and data exchange.
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        control_port: int = 5559,
        sensory_port: int = 5558,
        motor_port: int = 5564,
        visualization_port: int = 5560,
        agent_id: Optional[str] = None,
        agent_type: str = "external",
        timeout: int = 5000,
    ):
        """
        Initialize the FEAGI client.
        
        Args:
            host: FEAGI hostname or IP
            control_port: Control stream port (default 5559)
            sensory_port: Sensory stream port (default 5558)
            motor_port: Motor stream port (default 5564)
            visualization_port: Visualization stream port (default 5560)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            agent_type: Agent type for categorization
            timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.agent_type = agent_type
        self.timeout = timeout
        
        # Initialize ZMQ context
        self.context = zmq.asyncio.Context.instance()
        
        # Initialize clients for different streams
        self.control_client = FeagiControlClient(
            host=host,
            port=control_port,
            timeout=timeout,
        )
        
        self.sensory_client = FeagiSensoryClient(
            host=host,
            port=sensory_port,
            agent_id=self.agent_id,
            socket_timeout=timeout,
        )
        
        self.motor_client = FeagiMotorClient(
            host=host,
            port=motor_port,
            agent_id=self.agent_id,
            socket_timeout=timeout,
        )
        
        self.viz_client = FeagiVizClient(
            host=host,
            port=visualization_port,
            agent_id=self.agent_id,
            socket_timeout=timeout,
        )
        
        # State
        self.connected = False
        self.motor_callback = None
        self.visualization_callback = None
        
        # Tasks
        self.heartbeat_task = None
        self.motor_listen_task = None
        self.viz_listen_task = None
    
    async def connect(self) -> bool:
        """
        Connect to FEAGI and register this agent.
        
        Returns:
            True if connection was successful
        """
        try:
            # Connect control client and register agent
            if not await self.control_client.connect():
                logger.error("Failed to connect to FEAGI control stream")
                return False
            
            # Connect sensory client
            if not await self.sensory_client.connect():
                logger.error("Failed to connect to FEAGI sensory stream")
                return False
            
            # Connect motor client
            if not await self.motor_client.connect():
                logger.error("Failed to connect to FEAGI motor stream")
                return False
            
            # Connect visualization client if needed
            if self.visualization_callback:
                if not await self.viz_client.connect():
                    logger.error("Failed to connect to FEAGI visualization stream")
                    return False
            
            # Start heartbeat task
            self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            
            # Start motor listen task if callback is registered
            if self.motor_callback:
                await self.motor_client.subscribe_to_motor_channel("*")
                self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
            
            # Start visualization listen task if callback is registered
            if self.visualization_callback:
                self.viz_listen_task = asyncio.create_task(self._viz_listen_loop())
            
            self.connected = True
            logger.info(f"Connected to FEAGI as agent {self.agent_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to FEAGI: {e}")
            return False
    
    async def disconnect(self) -> None:
        """
        Disconnect from FEAGI.
        """
        self.connected = False
        
        # Send goodbye message
        try:
            await self.control_client.send_goodbye(agent_id=self.agent_id, agent_type=self.agent_type)
        except Exception as e:
            logger.warning(f"Error sending goodbye message: {e}")
        
        # Cancel background tasks
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
            try:
                await self.heartbeat_task
            except asyncio.CancelledError:
                pass
            self.heartbeat_task = None
        
        if self.motor_listen_task:
            self.motor_listen_task.cancel()
            try:
                await self.motor_listen_task
            except asyncio.CancelledError:
                pass
            self.motor_listen_task = None
        
        if self.viz_listen_task:
            self.viz_listen_task.cancel()
            try:
                await self.viz_listen_task
            except asyncio.CancelledError:
                pass
            self.viz_listen_task = None
        
        # Close clients
        await self.control_client.close()
        await self.sensory_client.close()
        await self.motor_client.close()
        await self.viz_client.close()
        
        logger.info("Disconnected from FEAGI")
    
    async def send_sensory_data(
        self, 
        channel: Union[str, FSMPChannel, int],
        data: Union[bytes, Dict[Tuple[int, int, int], float]]
    ) -> bool:
        """
        Send sensory data to FEAGI.
        
        Args:
            channel: Sensory channel ID or enum
            data: Sensory data as bytes or neuron activation dictionary
            
        Returns:
            True if data was sent successfully
        """
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return False
        
        # Convert channel to string if needed
        if isinstance(channel, FSMPChannel):
            channel_str = str(channel.value)
        elif isinstance(channel, int):
            channel_str = str(channel)
        else:
            channel_str = channel
        
        # Convert dictionary to bytes if needed
        if isinstance(data, dict):
            # This is a neuron activation dictionary, encode it
            encoder = ByteStructureEncoder()
            data = encoder.encode_neuron_potentials(data)
        
        # Send through sensory client
        return await self.sensory_client.send_sensory_data(channel_str, data)
    
    def register_motor_callback(
        self, 
        callback: Callable[[str, bytes], None]
    ) -> None:
        """
        Register a callback for motor data.
        
        Args:
            callback: Function to call when motor data is received
                     (parameters: channel_id, data)
        """
        self.motor_callback = callback
        if self.connected and not self.motor_listen_task:
            self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
    
    def register_visualization_callback(
        self,
        callback: Callable[[bytes], None]
    ) -> None:
        """
        Register a callback for visualization data.
        
        Args:
            callback: Function to call when visualization data is received
                     (parameter: data)
        """
        self.visualization_callback = callback
        if self.connected and not self.viz_listen_task:
            self.viz_listen_task = asyncio.create_task(self._viz_listen_loop())
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI system status.
        
        Returns:
            Dictionary with system status information
        """
        if not self.connected:
            logger.error("Not connected to FEAGI")
            return {"error": "Not connected"}
        
        return await self.control_client.get_status()
    
    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeats to keep the connection alive."""
        try:
            while self.connected:
                await self.control_client.make_request("heartbeat", {"agent_id": self.agent_id, "agent_type": self.agent_type})
                await asyncio.sleep(30)  # Send heartbeat every 30 seconds
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in heartbeat loop: {e}")
            self.connected = False
    
    async def _motor_listen_loop(self) -> None:
        """Listen for motor data."""
        try:
            await self.motor_client.start_motor_listener(self.motor_callback)
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in motor listen loop: {e}")
            self.connected = False
    
    async def _viz_listen_loop(self) -> None:
        """Listen for visualization data."""
        try:
            await self.viz_client.start_visualization_listener(self.visualization_callback)
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            logger.error(f"Error in visualization listen loop: {e}")
            self.connected = False 