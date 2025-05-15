"""
ZeroMQ Client for FEAGI

This module implements a ZeroMQ-based client for connecting to a FEAGI server.
It supports all FEAGI protocols (FCP, FSMP, FVP) over ZMQ sockets.
"""

import asyncio
import logging
import time
import struct
from typing import Dict, Any, Optional, Callable, List, Tuple, Union

import zmq
import zmq.asyncio

# Import protocol definitions
from protocol import (
    ProtocolID, ProtocolHeader, Timestamp,
    FCPMessageType, FCPMessage, RegisterRequest, RegisterResponse,
    DeregisterRequest, DeregisterResponse, StatusRequest, StatusResponse,
    HeartbeatRequest, HeartbeatResponse, ErrorResponse,
    FSMPMessageType, SensoryChannelType, MotorChannelType, FSMPMessage,
    SensoryData, MotorData,
    FVPMessageType, FVPMessage, StructureData, ActivityData
)
from protocol.fcp.v1.fcp_pb2 import ProtocolVersion

# Configure logging
logger = logging.getLogger("feagi_connector.zmq")


class ZmqFeagiClient:
    """
    ZeroMQ-based FEAGI Client.
    
    This class implements the client-side ZMQ sockets for connecting
    to a FEAGI server and handling the various protocol streams.
    """
    
    def __init__(
        self,
        host: str = "localhost",
        control_port: int = 5559,
        sensorimotor_port: int = 5558,
        viz_port_base: int = 5560,
        context: Optional[zmq.asyncio.Context] = None
    ):
        """
        Initialize the ZMQ client.
        
        Args:
            host: Host address of the FEAGI server
            control_port: Port for the FCP control stream
            sensorimotor_port: Port for the FSMP sensorimotor stream
            viz_port_base: Base port for the FVP visualization stream
            context: Optional ZMQ context to use
        """
        self.host = host
        self.control_port = control_port
        self.sensorimotor_port = sensorimotor_port
        self.viz_port_base = viz_port_base
        
        # Initialize ZMQ context
        self.context = context or zmq.asyncio.Context.instance()
        
        # Initialize sockets (will be created on connect)
        self.control_socket = None
        self.sensory_socket = None
        self.motor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None
        
        # Task tracking
        self._tasks = []
        self._running = False
        
    async def connect(self) -> bool:
        """
        Connect to the FEAGI server.
        
        Returns:
            True if connected successfully, False otherwise
        """
        try:
            # Create control socket (DEALER for ROUTER)
            self.control_socket = self.context.socket(zmq.DEALER)
            self.control_socket.connect(f"tcp://{self.host}:{self.control_port}")
            
            # Create sensorimotor sockets
            self.sensory_socket = self.context.socket(zmq.PUSH)
            self.sensory_socket.connect(f"tcp://{self.host}:{self.sensorimotor_port}")
            
            self.motor_socket = self.context.socket(zmq.SUB)
            self.motor_socket.connect(f"tcp://{self.host}:{self.sensorimotor_port}")
            self.motor_socket.setsockopt(zmq.SUBSCRIBE, b"motor")
            
            # Create visualization sockets
            self.viz_structure_socket = self.context.socket(zmq.SUB)
            self.viz_structure_socket.connect(f"tcp://{self.host}:{self.viz_port_base}")
            self.viz_structure_socket.setsockopt(zmq.SUBSCRIBE, b"structure")
            
            self.viz_activity_socket = self.context.socket(zmq.SUB)
            self.viz_activity_socket.connect(f"tcp://{self.host}:{self.viz_port_base + 1}")
            self.viz_activity_socket.setsockopt(zmq.SUBSCRIBE, b"activity")
            
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
        for socket in [self.control_socket, self.sensory_socket, self.motor_socket,
                      self.viz_structure_socket, self.viz_activity_socket]:
            if socket:
                socket.close(linger=0)
                
        self.control_socket = None
        self.sensory_socket = None
        self.motor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None
        
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
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create protocol versions
        versions = ProtocolVersion()
        versions.fcp_version = 1
        versions.fsmp_version = 1
        versions.fvp_version = 1
        
        # Create registration request
        request = RegisterRequest()
        request.agent_id = agent_id
        request.agent_type = agent_type
        request.protocol_versions.CopyFrom(versions)
        request.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_REGISTER
        message.register_request.CopyFrom(request)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response_message = FCPMessage()
        response_message.ParseFromString(response_frames[1])
        
        # Extract response data
        if response_message.HasField('register_response'):
            return {
                "status": response_message.register_response.status,
                "message": response_message.register_response.message,
                "timestamp": response_message.register_response.timestamp.time_ms / 1000
            }
        else:
            raise RuntimeError(f"Unexpected response type: {response_message.type}")
    
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
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create deregistration request
        request = DeregisterRequest()
        request.agent_id = agent_id
        request.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_DEREGISTER
        message.deregister_request.CopyFrom(request)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response_message = FCPMessage()
        response_message.ParseFromString(response_frames[1])
        
        # Extract response data
        if response_message.HasField('deregister_response'):
            return {
                "status": response_message.deregister_response.status,
                "message": response_message.deregister_response.message,
                "timestamp": response_message.deregister_response.timestamp.time_ms / 1000
            }
        else:
            raise RuntimeError(f"Unexpected response type: {response_message.type}")
    
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
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create heartbeat request
        request = HeartbeatRequest()
        request.agent_id = agent_id
        request.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_HEARTBEAT
        message.heartbeat_request.CopyFrom(request)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response_message = FCPMessage()
        response_message.ParseFromString(response_frames[1])
        
        # Extract response data
        if response_message.HasField('heartbeat_response'):
            return {
                "status": response_message.heartbeat_response.status,
                "timestamp": response_message.heartbeat_response.timestamp.time_ms / 1000
            }
        else:
            raise RuntimeError(f"Unexpected response type: {response_message.type}")
    
    async def get_status(self) -> Dict[str, Any]:
        """
        Get the status of the FEAGI server.
        
        Returns:
            Status information
        """
        if not self._running or not self.control_socket:
            raise RuntimeError("Client not connected")
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create status request
        request = StatusRequest()
        request.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_STATUS_REQUEST
        message.status_request.CopyFrom(request)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.control_socket.send_multipart([b"", data])
        
        # Receive and parse response
        response_frames = await self.control_socket.recv_multipart()
        
        if len(response_frames) != 2:
            raise RuntimeError(f"Invalid response format: {response_frames}")
            
        # Deserialize response
        response_message = FCPMessage()
        response_message.ParseFromString(response_frames[1])
        
        # Extract response data
        if response_message.HasField('status_response'):
            return {
                "status": response_message.status_response.status,
                "runtime": {
                    "cpu_usage": response_message.status_response.runtime.cpu_usage,
                    "memory_usage": response_message.status_response.runtime.memory_usage,
                    "uptime_seconds": response_message.status_response.runtime.uptime_seconds
                },
                "agent_count": response_message.status_response.agent_count,
                "timestamp": response_message.status_response.timestamp.time_ms / 1000
            }
        else:
            raise RuntimeError(f"Unexpected response type: {response_message.type}")
    
    async def send_sensory_data(self, channel_id: int, data: bytes) -> None:
        """
        Send sensory data to the FEAGI server.
        
        Args:
            channel_id: Sensory channel ID
            data: Binary sensory data
        """
        if not self._running or not self.sensory_socket:
            raise RuntimeError("Client not connected")
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create sensory data message
        sensory_data = SensoryData()
        sensory_data.channel_id = channel_id
        sensory_data.data = data
        sensory_data.timestamp.CopyFrom(current_time)
        
        # Create FSMP message
        message = FSMPMessage()
        message.header.protocol_id = ProtocolID.FSMP
        message.header.version = 1
        message.type = FSMPMessageType.FSMP_SENSORY
        message.sensory_data.CopyFrom(sensory_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.sensory_socket.send_multipart([b"sensory", data])
    
    async def register_motor_callback(self, callback: Callable[[int, bytes], None]) -> None:
        """
        Register a callback for receiving motor data.
        
        Args:
            callback: Function to call with (channel_id, data) when motor data is received
        """
        if not self._running or not self.motor_socket:
            raise RuntimeError("Client not connected")
            
        # Create a background task to listen for motor messages
        task = asyncio.create_task(self._motor_listener(callback))
        self._tasks.append(task)
    
    async def _motor_listener(self, callback: Callable[[int, bytes], None]) -> None:
        """Background task to listen for motor messages."""
        try:
            while self._running and self.motor_socket:
                try:
                    # Receive message
                    topic, message_data = await self.motor_socket.recv_multipart()
                    
                    # Deserialize message
                    message = FSMPMessage()
                    message.ParseFromString(message_data)
                    
                    # Verify it's a motor message
                    if message.type != FSMPMessageType.FSMP_MOTOR:
                        logger.warning(f"Received non-motor message type: {message.type}")
                        continue
                    
                    # Call callback with motor data
                    callback(message.motor_data.channel_id, message.motor_data.data)
                    
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in motor listener: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            logger.debug("Motor listener cancelled")
    
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
        if not self._running:
            raise RuntimeError("Client not connected")
            
        # Start activity listener if callback provided
        if activity_callback and self.viz_activity_socket:
            task = asyncio.create_task(
                self._visualization_listener(self.viz_activity_socket, activity_callback)
            )
            self._tasks.append(task)
            
        # Start structure listener if callback provided
        if structure_callback and self.viz_structure_socket:
            task = asyncio.create_task(
                self._visualization_listener(self.viz_structure_socket, structure_callback)
            )
            self._tasks.append(task)
    
    async def _visualization_listener(self, socket: zmq.asyncio.Socket, callback: Callable[[bytes], None]) -> None:
        """Background task to listen for visualization messages."""
        try:
            while self._running and socket:
                try:
                    # Receive message
                    topic, message_data = await socket.recv_multipart()
                    
                    # Deserialize message
                    message = FVPMessage()
                    message.ParseFromString(message_data)
                    
                    # Determine data based on message type
                    if message.type == FVPMessageType.FVP_ACTIVITY:
                        callback(message.activity_data.SerializeToString())
                    elif message.type == FVPMessageType.FVP_STRUCTURE:
                        callback(message.structure_data.SerializeToString())
                    else:
                        logger.warning(f"Unknown visualization message type: {message.type}")
                    
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in visualization listener: {e}")
                    await asyncio.sleep(0.1)
                    
        except asyncio.CancelledError:
            logger.debug("Visualization listener cancelled") 