"""
FEAGI Client

Main client interface for connecting to FEAGI and exchanging data.
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
from feagi_connector.api.command_client import FeagiCommandClient
from feagi_connector.api.sensory_client import FeagiSensoryClient
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
    
    This client provides a unified interface for all FEAGI communication protocols:
    - Command API (FCP)
    - Sensorimotor data (FSMP)
    - Visualization data (FVP)
    
    It handles connection management, agent registration, and data exchange.
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        command_port: int = 5555,
        sensory_port: int = 5558,
        viz_port: int = 5560,
        agent_id: Optional[str] = None,
        agent_type: str = "external",
        timeout: int = 5000,
    ):
        """
        Initialize the FEAGI client.
        
        Args:
            host: FEAGI hostname or IP
            command_port: Command API port (default 5555)
            sensory_port: Sensorimotor data port (default 5558)
            viz_port: Visualization data port (default 5560)
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
        
        # Initialize clients for different protocols
        self.command_client = FeagiCommandClient(
            host=host,
            port=command_port,
            timeout=timeout,
            auth_token=None,  # TODO: Add auth token support
        )
        
        self.sensory_client = FeagiSensoryClient(
            host=host,
            port=sensory_port,
            agent_id=self.agent_id,
            socket_timeout=timeout,
        )
        
        self.viz_client = FeagiVizClient(
            host=host,
            port=viz_port,
            agent_id=self.agent_id,
            socket_timeout=timeout,
        )
        
        # State
        self.connected = False
        self.motor_callback = None
        self.activity_callback = None
        self.structure_callback = None
        
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
            # Check if FEAGI is running
            status = await self.command_client.ping()
            if not status or "error" in status:
                logger.error(f"Failed to connect to FEAGI command API: {status}")
                return False
            
            # Connect sensory client
            if not await self.sensory_client.connect():
                logger.error("Failed to connect to FEAGI sensory API")
                return False
            
            # Register agent
            if not await self.sensory_client.register_agent(
                agent_type=self.agent_type,
            ):
                logger.error("Failed to register agent with FEAGI")
                return False
            
            # Start heartbeat task
            self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            
            # Start motor listen task if callback is registered
            if self.motor_callback:
                self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
            
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
        await self.sensory_client.close()
        await self.viz_client.close()
        
        logger.info("Disconnected from FEAGI")
    
    async def send_sensory_data(
        self, 
        channel: Union[FSMPChannel, int],
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
        
        # Convert channel enum to int if needed
        if isinstance(channel, FSMPChannel):
            channel_id = channel.value
        else:
            channel_id = channel
        
        return await self.sensory_client.send_sensory_data(channel_id, data)
    
    async def register_motor_callback(
        self, 
        callback: Callable[[int, bytes], None]
    ) -> None:
        """
        Register a callback for motor data from FEAGI.
        
        Args:
            callback: Function to call when motor data is received
        """
        self.motor_callback = callback
        
        # Start motor listen task if connected
        if self.connected and not self.motor_listen_task:
            self.motor_listen_task = asyncio.create_task(self._motor_listen_loop())
    
    async def register_visualization_callbacks(
        self,
        activity_callback: Optional[Callable[[Dict], None]] = None,
        structure_callback: Optional[Callable[[Dict], None]] = None,
    ) -> bool:
        """
        Register callbacks for visualization data.
        
        Args:
            activity_callback: Function to call when neural activity data is received
            structure_callback: Function to call when brain structure data is received
            
        Returns:
            True if registration was successful
        """
        self.activity_callback = activity_callback
        self.structure_callback = structure_callback
        
        # Connect to visualization server if needed
        if not self.viz_client.is_connected():
            if not await self.viz_client.connect():
                logger.error("Failed to connect to visualization server")
                return False
        
        # Register with visualization server
        if not await self.viz_client.register_viz_agent():
            logger.error("Failed to register with visualization server")
            return False
        
        # Start visualization listen task
        if not self.viz_listen_task:
            self.viz_listen_task = asyncio.create_task(self._viz_listen_loop())
        
        return True
    
    async def request_visualization(self) -> bool:
        """
        Request visualization data from FEAGI.
        
        Returns:
            True if request was sent successfully
        """
        if not self.viz_client.is_connected():
            logger.error("Not connected to visualization server")
            return False
        
        await self.viz_client.request_visualization()
        return True
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI status.
        
        Returns:
            Status dictionary
        """
        return await self.command_client.get_status()
    
    async def _heartbeat_loop(self) -> None:
        """
        Background task to send periodic heartbeats.
        """
        while self.connected:
            try:
                await self.sensory_client.send_heartbeat()
                await asyncio.sleep(5)  # Send heartbeat every 5 seconds
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in heartbeat loop: {e}")
                await asyncio.sleep(1)
    
    async def _motor_listen_loop(self) -> None:
        """
        Background task to listen for motor data.
        """
        while self.connected:
            try:
                motor_data = await self.sensory_client.receive_motor_data()
                if motor_data and self.motor_callback:
                    channel_id, data = motor_data
                    await asyncio.to_thread(self.motor_callback, channel_id, data)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in motor listen loop: {e}")
                await asyncio.sleep(0.1)
    
    async def _viz_listen_loop(self) -> None:
        """
        Background task to listen for visualization data.
        """
        while self.connected:
            try:
                viz_data = await self.viz_client.receive_visualization_data()
                if not viz_data:
                    await asyncio.sleep(0.1)
                    continue
                
                msg_type = viz_data.get("message_type")
                
                if msg_type == "activity" and self.activity_callback:
                    await asyncio.to_thread(self.activity_callback, viz_data)
                elif msg_type == "structure" and self.structure_callback:
                    await asyncio.to_thread(self.structure_callback, viz_data)
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in visualization listen loop: {e}")
                await asyncio.sleep(0.1) 