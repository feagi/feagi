"""
ZeroMQ Client for FEAGI

This module implements a ZeroMQ-based client for connecting to a FEAGI server.
It supports all FEAGI protocols (FCP, FSMP, FVP) over ZMQ sockets using the
DEALER-ROUTER pattern with custom byte structures.
"""

import asyncio
import logging
import time
import json
from typing import Dict, Any, Optional, Callable, List, Tuple, Union

import zmq
# Try to import asyncio from zmq, use mock if that fails
try:
    import zmq.asyncio
except ImportError:
    logging.warning("zmq.asyncio not found, using mock implementation")
    try:
        # Use the local mock implementation rather than importing from feagi
        from feagi_connector.zmq.mock_asyncio import Context as AsyncioContext
        # Monkey patch zmq to provide asyncio context
        if not hasattr(zmq, 'asyncio'):
            class AsyncioModule:
                Context = AsyncioContext
            zmq.asyncio = AsyncioModule()
    except ImportError:
        logging.warning("Could not import mock_asyncio, creating minimal stub")
        # Create minimal zmq.asyncio stub
        if not hasattr(zmq, 'asyncio'):
            class AsyncioContext(zmq.Context):
                @classmethod
                def instance(cls):
                    return zmq.Context.instance()
                    
            class AsyncioModule:
                Context = AsyncioContext
            zmq.asyncio = AsyncioModule()

# Import protocol definitions
from feagi_connector.protocols import (
    ByteStructureID, ProtocolType, 
    FCPMessageType, FSMPChannel, FVPFrameType,
    ByteStructureEncoder, ByteStructureDecoder, ByteStructureTranslator
)

# Configure logging
logger = logging.getLogger("feagi_connector.zmq")


class ZmqFeagiClient:
    """
    ZeroMQ-based FEAGI Client.
    
    This class implements the client-side ZMQ sockets for connecting
    to a FEAGI server and handling the various protocol streams using
    the DEALER-ROUTER pattern and byte structures.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        control_port: int = 5559,
        sensorimotor_port: int = 5558,
        visualization_port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the ZMQ client.
        
        Args:
            host: Host address of the FEAGI server
            control_port: Port for the FCP control stream
            sensorimotor_port: Port for the FSMP sensorimotor stream
            visualization_port: Port for the FVP visualization stream
            context: Optional ZMQ context to use
        """
        self.host = host
        self.control_port = control_port
        self.sensorimotor_port = sensorimotor_port
        self.visualization_port = visualization_port
        
        # Initialize ZMQ context
        self.context = context or zmq.asyncio.Context.instance()
        
        # Initialize sockets (will be created on connect)
        self.control_socket = None
        self.sensorimotor_socket = None
        self.visualization_socket = None
        
        # Initialize byte structure translator
        self.translator = ByteStructureTranslator()
        
        # Task tracking
        self._tasks = []
        self._running = False
        
        # Callbacks
        self._motor_callback = None
        self._activity_callback = None
        self._structure_callback = None
        
        # Client info
        self.agent_id = None
        
    async def connect(self) -> bool:
        """
        Connect to the FEAGI server.
        
        Returns:
            True if connected successfully, False otherwise
        """
        try:
            logger.debug(f"Connecting to FEAGI server at {self.host}")
            logger.debug(f"Control port: {self.control_port}")
            logger.debug(f"Sensorimotor port: {self.sensorimotor_port}")
            logger.debug(f"Visualization port: {self.visualization_port}")
            
            # Create control socket (DEALER for ROUTER)
            self.control_socket = self.context.socket(zmq.DEALER)
            self.control_socket.connect(f"tcp://{self.host}:{self.control_port}")
            logger.debug("Connected control socket")
            
            # Create sensorimotor socket (DEALER for ROUTER)
            self.sensorimotor_socket = self.context.socket(zmq.DEALER)
            self.sensorimotor_socket.connect(f"tcp://{self.host}:{self.sensorimotor_port}")
            logger.debug("Connected sensorimotor socket")
            
            # Create visualization socket (DEALER for ROUTER)
            self.visualization_socket = self.context.socket(zmq.DEALER)
            self.visualization_socket.connect(f"tcp://{self.host}:{self.visualization_port}")
            logger.debug("Connected visualization socket")
            
            self._running = True
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to FEAGI: {e}")
            await self.disconnect()
            return False
    
    async def disconnect(self):
        """Disconnect from the FEAGI server."""
        # Cancel all running tasks
        for task in self._tasks:
            if not task.done():
                task.cancel()
                
        # Wait for tasks to complete
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
            self._tasks = []
        
        # Close sockets
        for socket in [self.control_socket, self.sensorimotor_socket, self.visualization_socket]:
            if socket:
                socket.close(linger=0)
                
        self.control_socket = None
        self.sensorimotor_socket = None
        self.visualization_socket = None
        
        self._running = False
    
    async def register_agent(self, agent_id: str, agent_type: str) -> Dict[str, Any]:
        """
        Register an agent with the FEAGI server.
        
        Args:
            agent_id: Agent identifier
            agent_type: Agent type
            
        Returns:
            Registration response
        """
        if not self._running or not self.control_socket:
            raise RuntimeError("Client not connected")
        
        logger.debug(f"Registering agent {agent_id} of type {agent_type}")
        
        # Store agent ID
        self.agent_id = agent_id
        
        # Create registration request
        request = {
            "type": FCPMessageType.REGISTER,
            "agent_id": agent_id,
            "agent_type": agent_type,
            "timestamp": int(time.time() * 1000),
            "capabilities": {
                "protocols": {
                    ProtocolType.FCP: True,
                    ProtocolType.FSMP: True,
                    ProtocolType.FVP: True
                },
                "structures": {
                    str(ByteStructureID.JSON): [1],
                    str(ByteStructureID.RAW_IMAGE): [1],
                    str(ByteStructureID.MULTI_HOLDER): [1],
                    str(ByteStructureID.NEURON_FLAT): [1],
                    str(ByteStructureID.NEURON_CATEGORIES): [1]
                }
            }
        }
        
        logger.debug(f"Registration request: {request}")
        
        # For now, send as direct JSON to be compatible with current server
        # This is temporary until the server is updated to support byte structures
        json_data = json.dumps(request).encode('utf-8')
        logger.debug(f"Sending as JSON, size: {len(json_data)} bytes")
        
        # Send with content-type frame for compatibility with current server
        await self.control_socket.send_multipart([b"", b"application/json", json_data])
        logger.debug("Registration request sent, waiting for response")
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        logger.debug(f"Received response with {len(response_frames)} frames")
        
        if len(response_frames) != 3:  # Empty frame, content-type, payload
            logger.error(f"Invalid response format: {response_frames}")
            raise RuntimeError(f"Invalid response format: {response_frames}")
        
        # Extract the actual data (third frame)
        response_data = response_frames[2]
        content_type = response_frames[1].decode('utf-8')
        logger.debug(f"Response content type: {content_type}")
        
        # Parse the response based on content type
        if content_type == "application/json":
            response = json.loads(response_data.decode('utf-8'))
        else:
            # Try to decode as byte structure if not JSON
            try:
                response = self.translator.decode_message(response_data)
            except Exception as e:
                logger.error(f"Failed to decode response: {e}")
                response = {"error": "Failed to decode response", "data": response_data.decode('utf-8', errors='replace')}
        
        logger.debug(f"Decoded response: {response}")
        
        # Start message listeners if registration successful
        if response.get("status") == "success":
            logger.debug("Registration successful, starting listeners")
            self._start_listeners()
            
        return response
    
    def _start_listeners(self):
        """Start asynchronous message listeners."""
        # Start sensorimotor listener if callback registered
        if self._motor_callback:
            task = asyncio.create_task(self._sensorimotor_listener())
            self._tasks.append(task)
            
        # Start visualization listener if callbacks registered
        if self._activity_callback or self._structure_callback:
            task = asyncio.create_task(self._visualization_listener())
            self._tasks.append(task)
    
    async def deregister_agent(self, agent_id: str) -> Dict[str, Any]:
        """
        Deregister an agent from the FEAGI server.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            Deregistration response
        """
        if not self._running or not self.control_socket:
            raise RuntimeError("Client not connected")
        
        # Create deregistration request
        request = {
            "type": FCPMessageType.DEREGISTER,
            "agent_id": agent_id,
            "timestamp": int(time.time() * 1000)
        }
        
        # Encode and send
        data = self.translator.encode_message(request)
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response = self.translator.decode_message(response_frames[1])
        
        return response
    
    async def send_heartbeat(self, agent_id: str) -> Dict[str, Any]:
        """
        Send a heartbeat to the FEAGI server.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            Heartbeat response
        """
        if not self._running or not self.control_socket:
            raise RuntimeError("Client not connected")
        
        # Create heartbeat request
        request = {
            "type": FCPMessageType.HEARTBEAT,
            "agent_id": agent_id,
            "timestamp": int(time.time() * 1000)
        }
        
        # Encode and send
        data = self.translator.encode_message(request)
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response = self.translator.decode_message(response_frames[1])
        
        return response
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get FEAGI status information.
        
        Returns:
            Dictionary with status information
        """
        if not self._running or not self.control_socket:
            raise RuntimeError("Client not connected")
        
        # Create status request
        request = {
            "type": FCPMessageType.STATUS,
            "timestamp": int(time.time() * 1000)
        }
        
        # Encode and send
        data = self.translator.encode_message(request)
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response = self.translator.decode_message(response_frames[1])
        
        return response
    
    async def send_sensory_data(self, channel_id: int, data: bytes) -> None:
        """
        Send sensory data to FEAGI.
        
        Args:
            channel_id: Sensory channel ID
            data: Binary sensory data
        """
        if not self._running or not self.sensorimotor_socket:
            raise RuntimeError("Client not connected")
        
        # Create sensory data message
        message = {
            "type": "sensory_data",
            "channel_id": channel_id,
            "timestamp": int(time.time() * 1000),
            "data_format": "raw"
        }
        
        # Encode message as JSON structure
        message_data = self.translator.encode_message(message)
        
        # Create multi-holder with message and binary data
        image_data = self.translator.encoder.encode_raw_image(
            data, 
            width=0,  # These will be filled in by the agent implementation
            height=0, 
            channels=0
        )
        
        multi_data = self.translator.encoder.encode_multi_holder([message_data, image_data])
        
        # Send to FEAGI
        await self.sensorimotor_socket.send_multipart([b"", multi_data])
    
    async def register_motor_callback(self, callback: Callable[[int, bytes], None]) -> None:
        """
        Register a callback for receiving motor data.
        
        Args:
            callback: Function to call when motor data is received
                     (parameters: channel_id, data)
        """
        self._motor_callback = callback
        
        # Start listener if we're already running
        if self._running and self.sensorimotor_socket:
            task = asyncio.create_task(self._sensorimotor_listener())
            self._tasks.append(task)
    
    async def _sensorimotor_listener(self) -> None:
        """Listen for motor messages from FEAGI."""
        logger.info("Starting sensorimotor listener")
        
        while self._running and self.sensorimotor_socket:
            try:
                # Receive message
                message_parts = await self.sensorimotor_socket.recv_multipart()
                
                if len(message_parts) != 2:
                    logger.error(f"Invalid message format: {message_parts}")
                    continue
                
                # Decode message
                message_data = message_parts[1]
                
                # Check if it's a multi-holder (message + binary data)
                structure_info = self.translator.decoder.decode_header(message_data)
                if structure_info[0] == ByteStructureID.MULTI_HOLDER:
                    # Decode multi-holder
                    multi_data = self.translator.decoder.decode_multi_holder(message_data)
                    structures = multi_data["structures"]
                    
                    if len(structures) != 2:
                        logger.error(f"Expected 2 structures in multi-holder, got {len(structures)}")
                        continue
                    
                    # Decode message part
                    message = self.translator.decode_message(structures[0])
                    
                    # Extract binary data
                    if message.get("type") == "motor_data" and self._motor_callback:
                        channel_id = message.get("channel_id")
                        raw_data = structures[1]
                        
                        # Get raw data from second structure
                        image_data = self.translator.decoder.decode_raw_image(raw_data)
                        
                        # Call callback
                        self._motor_callback(channel_id, image_data["data"])
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.exception(f"Error in sensorimotor listener: {e}")
    
    async def register_visualization_callbacks(
        self,
        activity_callback: Optional[Callable[[bytes], None]] = None,
        structure_callback: Optional[Callable[[bytes], None]] = None
    ) -> None:
        """
        Register callbacks for receiving visualization data.
        
        Args:
            activity_callback: Function to call when neural activity data is received
            structure_callback: Function to call when brain structure data is received
        """
        self._activity_callback = activity_callback
        self._structure_callback = structure_callback
        
        # Start listener if we're already running
        if self._running and self.visualization_socket:
            task = asyncio.create_task(self._visualization_listener())
            self._tasks.append(task)
    
    async def _visualization_listener(self) -> None:
        """Listen for visualization messages from FEAGI."""
        logger.info("Starting visualization listener")
        
        while self._running and self.visualization_socket:
            try:
                # Receive message
                message_parts = await self.visualization_socket.recv_multipart()
                
                if len(message_parts) != 2:
                    logger.error(f"Invalid message format: {message_parts}")
                    continue
                
                # Decode message
                message_data = message_parts[1]
                
                # Determine structure type
                structure_info = self.translator.decoder.decode_header(message_data)
                
                if structure_info[0] == ByteStructureID.MULTI_HOLDER:
                    # Decode multi-holder
                    multi_data = self.translator.decoder.decode_multi_holder(message_data)
                    structures = multi_data["structures"]
                    
                    if len(structures) < 1:
                        logger.error("Empty multi-holder received")
                        continue
                    
                    # Decode message part
                    message = self.translator.decode_message(structures[0])
                    
                    # Process based on message type
                    if message.get("type") == "activity_data" and self._activity_callback:
                        # For activity data, we have neuron data in subsequent structures
                        if len(structures) > 1:
                            # Assuming the second structure is neuron data
                            self._activity_callback(structures[1])
                            
                    elif message.get("type") == "structure_data" and self._structure_callback:
                        # For structure data, we have structure info in subsequent structures
                        if len(structures) > 1:
                            # Assuming the second structure contains structure data
                            self._structure_callback(structures[1])
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.exception(f"Error in visualization listener: {e}") 