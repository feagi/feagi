"""
ZeroMQ Server for FEAGI API

This module implements the ZeroMQ server for the FEAGI API, providing
high-performance, real-time communication with clients.
"""

import os
import time
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import threading
import asyncio
import concurrent.futures
from typing import Dict, Any, List, Optional, Union, Callable

import zmq
import zmq.asyncio
from zmq.auth.thread import ThreadAuthenticator

from ..core.service import CoreApiService

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

from feagi.api.zmq.connection_manager import ConnectionManager
from feagi.api.zmq.message_handlers import MessageHandler, start_message_handlers, stop_message_handlers
from feagi.api.protocols.translator import ProtocolTranslator


class ZmqServer:
    """
    ZMQ server for FEAGI with proper event loop management.
    
    This implementation ensures each thread has its own event loop and
    properly manages asyncio resources.
    """
    
    def __init__(
        self,
        core_api: CoreApiService,
        host: str = "*",
        req_rep_port: int = 5555,
        pub_sub_port: int = 5556,
        push_pull_port: int = 5557,
        sensorimotor_port: int = 5558,
        control_port: int = 5559,
        vis_base_port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None,
        schema_path: Optional[str] = None
    ):
        """
        Initialize the ZMQ server.
        
        Args:
            core_api: Core API service for accessing FEAGI
            host: Host to bind to
            req_rep_port: Port for REQ/REP pattern
            pub_sub_port: Port for PUB/SUB pattern
            push_pull_port: Port for PUSH/PULL pattern
            sensorimotor_port: Port for sensorimotor stream
            control_port: Port for control protocol stream
            vis_base_port: Base port for visualization stream
            context: Optional existing ZMQ context to use
            schema_path: Path to Cap'n Proto schemas
        """
        self.core_api = core_api
        self.host = host
        self.req_rep_port = req_rep_port
        self.pub_sub_port = pub_sub_port
        self.push_pull_port = push_pull_port
        self.sensorimotor_port = sensorimotor_port
        self.control_port = control_port
        self.vis_base_port = vis_base_port
        
        # Thread and event loop management
        self._thread = None
        self._loop = None
        self._context = context or zmq.asyncio.Context.instance()
        self._running = False
        self._shutdown_event = threading.Event()
        
        # Pattern managers
        self._req_rep = None
        self._pub_sub = None
        self._push_pull = None
        self._sensorimotor = None
        self._visualization = None
        self._control = None
        
        # Initialize sockets (will be created on start)
        self.control_socket = None
        self.sensorimotor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None
        
        # State tracking
        self.agents = {}  # agent_id -> agent_info
        self.tasks = []
        
        # Callbacks
        self.sensory_callback = None
        
        # Create connection manager
        self.connection_manager = ConnectionManager(
            context=self._context,
            control_port=self.control_port,
            sensorimotor_port=self.sensorimotor_port,
            visualization_port=self.vis_base_port
        )
        
        # Create protocol translator
        self.translator = ProtocolTranslator(schema_path=schema_path)
        
        # Store message handlers (created during start())
        self.message_handlers: Dict[str, MessageHandler] = {}
        
        # Track clients awaiting handshake completion
        self.pending_clients: Dict[str, Dict[str, Any]] = {}
        
        # Message processing callbacks
        self.message_processors = {
            "handshake": self._process_handshake_message,
            "fcp": self._process_fcp_message,
            "fsmp": self._process_fsmp_message,
            "fvp": self._process_fvp_message
        }
        
        # Schema loaders
        self.schema_loaders = {
            "handshake": self.translator.get_schema_loader("handshake"),
            "fcp": self.translator.get_schema_loader("fcp"),
            "fsmp": self.translator.get_schema_loader("fsmp"),
            "fvp": self.translator.get_schema_loader("fvp")
        }
        
        # Cleanup task
        self.cleanup_task = None
    
    def start(self) -> bool:
        """
        Start the ZMQ server in a background thread.
        
        Returns:
            True if started successfully, False otherwise
        """
        if self._running:
            logger.warning("ZMQ server is already running")
            return True
        
        logger.info(f"Starting ZMQ server on {self.host}")
        try:
            # In synchronous mode, create a context and start the server in a background thread
            logger.info("Creating background thread for ZMQ server")
            self._thread = threading.Thread(target=self._run_server_thread, daemon=True)
            self._thread.start()
            
            # Wait briefly to allow the server to start or fail
            time.sleep(0.5)
            
            if not self._running:
                # If the server didn't start properly, the thread will have set _running to False
                logger.error("ZMQ server failed to start")
                return False
                
            return True
        except Exception as e:
            logger.error(f"Error starting ZMQ server: {e}")
            if self._context:
                self._context.term()
                self._context = None
            return False
    
    def _run_server_thread(self):
        """
        Run the ZMQ server in a background thread.
        
        This method creates a new event loop for the thread and runs the server in it.
        """
        # Create a new event loop for this thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            # Initialize and run the server
            self._running = True
            self._loop.run_until_complete(self._start_server())
            self._loop.run_until_complete(self._monitor_loop())
        except Exception as e:
            logger.error(f"Error in ZMQ server thread: {e}")
            self._running = False
        finally:
            self._cleanup()
    
    async def _start_server(self):
        """
        Start all ZMQ services asynchronously.
        
        This is called from the background thread.
        """
        try:
            # We'll import here to avoid circular imports
            from .patterns.req_rep import RequestReplyManager
            from .patterns.pub_sub import PubSubManager
            from .patterns.push_pull import PushPullManager
            from .streams.sensorimotor import SensorimotorStream
            from .streams.visualization import VisualizationStream
            from .streams.control import ControlStream
            
            # Initialize all managers with the current thread's event loop
            self._req_rep = RequestReplyManager(
                core_api=self.core_api,
                host=self.host,
                port=self.req_rep_port,
                context=self._context
            )
            
            self._pub_sub = PubSubManager(
                core_api=self.core_api,
                host=self.host,
                port=self.pub_sub_port,
                context=self._context
            )
            
            self._push_pull = PushPullManager(
                core_api=self.core_api,
                host=self.host,
                port=self.push_pull_port,
                context=self._context
            )
            
            self._sensorimotor = SensorimotorStream(
                core_api=self.core_api,
                host=self.host,
                port=self.sensorimotor_port,
                context=self._context
            )
            
            self._control = ControlStream(
                core_api=self.core_api,
                host=self.host,
                port=self.control_port,
                context=self._context
            )
            
            self._visualization = VisualizationStream(
                core_api=self.core_api,
                host=self.host,
                structure_port=self.vis_base_port,
                activity_port=self.vis_base_port + 1,
                control_port=self.vis_base_port + 2,
                context=self._context
            )
            
            # Start all managers
            await self._req_rep.start()
            await self._pub_sub.start()
            await self._push_pull.start()
            await self._sensorimotor.start()
            await self._control.start()
            await self._visualization.start()
            
            # Create control socket (ROUTER)
            self.control_socket = self._context.socket(zmq.ROUTER)
            self.control_socket.bind(f"tcp://*:{self.control_port}")
            
            # Create sensorimotor socket (XPUB/XSUB pattern)
            self.sensorimotor_socket = self._context.socket(zmq.XPUB)
            self.sensorimotor_socket.bind(f"tcp://*:{self.sensorimotor_port}")
            
            # Create visualization sockets (PUB)
            self.viz_structure_socket = self._context.socket(zmq.PUB)
            self.viz_structure_socket.bind(f"tcp://*:{self.vis_base_port}")
            
            self.viz_activity_socket = self._context.socket(zmq.PUB)
            self.viz_activity_socket.bind(f"tcp://*:{self.vis_base_port + 1}")
            
            # Start message handling tasks
            self.tasks.append(asyncio.create_task(self._handle_control_messages()))
            self.tasks.append(asyncio.create_task(self._handle_sensorimotor_messages()))
            
            # Start message handlers
            self.message_handlers = await start_message_handlers(
                self.connection_manager,
                self.schema_loaders,
                self.message_processors
            )
            
            # Start client cleanup task
            self.cleanup_task = asyncio.create_task(self._cleanup_inactive_clients())
            
            logger.info("ZMQ server started successfully")
        except Exception as e:
            logger.error(f"Failed to start ZMQ services: {e}")
            self._running = False
            raise
    
    async def _monitor_loop(self):
        """
        Monitor loop to keep the server running and handle shutdown requests.
        """
        try:
            while self._running and not self._shutdown_event.is_set():
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            logger.info("Monitor loop cancelled")
        except Exception as e:
            logger.error(f"Error in monitor loop: {e}")
            self._running = False
    
    def shutdown(self):
        """
        Shutdown the ZMQ server.
        
        This method is thread-safe and can be called from any thread.
        """
        if not self._running:
            logger.warning("ZMQ server is not running")
            return
            
        logger.info("Shutting down ZMQ server")
        
        # Signal the monitor loop to stop
        self._shutdown_event.set()
        
        # Create a new event loop for shutdown if we're not in the server thread
        if threading.current_thread() != self._thread:
            # We're in a different thread, create a new event loop
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                # Run the shutdown in this new loop
                loop.run_until_complete(self._stop_services())
            finally:
                loop.close()
        else:
            # We're in the server thread, use its loop
            asyncio.ensure_future(self._stop_services())
            
        # Wait for the server thread to finish
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            
        # Final cleanup
        self._running = False
        logger.info("ZMQ server shutdown complete")
    
    async def _stop_services(self):
        """
        Stop all ZMQ services asynchronously.
        """
        logger.info("Stopping ZMQ services")
        
        # Stop all services
        stop_tasks = []
        
        if self._req_rep:
            stop_tasks.append(self._req_rep.stop())
            
        if self._pub_sub:
            stop_tasks.append(self._pub_sub.stop())
            
        if self._push_pull:
            stop_tasks.append(self._push_pull.stop())
            
        if self._sensorimotor:
            stop_tasks.append(self._sensorimotor.stop())
            
        if self._control:
            stop_tasks.append(self._control.stop())
            
        if self._visualization:
            stop_tasks.append(self._visualization.stop())
            
        # Wait for all services to stop
        if stop_tasks:
            await asyncio.gather(*stop_tasks, return_exceptions=True)
            
        logger.info("All ZMQ services stopped")
        
        # Close sockets
        for socket in [self.control_socket, self.sensorimotor_socket, 
                      self.viz_structure_socket, self.viz_activity_socket]:
            if socket:
                socket.close(linger=0)
                
        self.control_socket = None
        self.sensorimotor_socket = None
        self.viz_structure_socket = None
        self.viz_activity_socket = None
    
    def _cleanup(self):
        """
        Clean up resources.
        
        This is called when the server thread exits.
        """
        try:
            # Ensure services are stopped
            if self._loop and self._running:
                self._loop.run_until_complete(self._stop_services())
                
            # Close the event loop
            if self._loop:
                self._loop.close()
                self._loop = None
                
            # Clear service references
            self._req_rep = None
            self._pub_sub = None
            self._push_pull = None
            self._sensorimotor = None
            self._control = None
            self._visualization = None
            
            # Reset state
            self._running = False
            self._shutdown_event.clear()
            
            # Clear state tracking
            self.agents = {}
            self.tasks = []
            
            # Close connection manager
            self.connection_manager.close()
            
            logger.info("ZMQ server resources cleaned up")
        except Exception as e:
            logger.error(f"Error during ZMQ server cleanup: {e}")
    
    async def publish_event(self, event_type: str, event_data: Dict) -> None:
        """
        Publish an event to subscribers.
        
        Args:
            event_type: Type of event
            event_data: Event data
        """
        if not self._running or not self._pub_sub:
            logger.warning("Cannot publish event: ZMQ server not running")
            return
            
        try:
            await self._pub_sub.publish_event(event_type, event_data)
        except Exception as e:
            logger.error(f"Error publishing event: {e}")
    
    async def queue_work(self, work_type: str, data: Any, priority: int = 0) -> None:
        """
        Queue work for processing.
        
        Args:
            work_type: Type of work
            data: Work data
            priority: Priority level (higher is more important)
        """
        if not self._running or not self._push_pull:
            logger.warning("Cannot queue work: ZMQ server not running")
            return
            
        try:
            await self._push_pull.push_data(work_type, data, priority)
        except Exception as e:
            logger.error(f"Error queueing work: {e}")
            
    async def send_control_message(self, agent_id: str, message_type: str, data: Dict[str, Any] = None) -> bool:
        """
        Send a control message to an agent.
        
        Args:
            agent_id: Agent ID
            message_type: Message type
            data: Message data
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self._running or not self._control:
            logger.warning("Cannot send control message: ZMQ server not running")
            return False
            
        try:
            return await self._control.send_control_message(agent_id, message_type, data)
        except Exception as e:
            logger.error(f"Error sending control message: {e}")
            return False
    
    async def _handle_control_messages(self):
        """Handle incoming control messages."""
        try:
            while self._running and self.control_socket:
                try:
                    # Receive message
                    frames = await self.control_socket.recv_multipart()
                    if len(frames) != 3:  # [identity, empty, message]
                        logger.warning(f"Invalid frame count: {len(frames)}")
                        continue
                    
                    identity, empty, message_data = frames
                    
                    # Parse the message
                    fcp_message = FCPMessage()
                    fcp_message.ParseFromString(message_data)
                    
                    # Handle different message types
                    response = None
                    if fcp_message.type == FCPMessageType.FCP_REGISTER:
                        response = await self._handle_register(identity, fcp_message.register_request)
                    elif fcp_message.type == FCPMessageType.FCP_DEREGISTER:
                        response = await self._handle_deregister(identity, fcp_message.deregister_request)
                    elif fcp_message.type == FCPMessageType.FCP_HEARTBEAT:
                        response = await self._handle_heartbeat(identity, fcp_message.heartbeat_request)
                    elif fcp_message.type == FCPMessageType.FCP_STATUS_REQUEST:
                        response = await self._handle_status_request(identity, fcp_message.status_request)
                    else:
                        logger.warning(f"Unknown control message type: {fcp_message.type}")
                        continue
                    
                    # Send response
                    if response:
                        await self.control_socket.send_multipart([identity, empty, response])
                    
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error handling control message: {e}")
                    
        except asyncio.CancelledError:
            logger.debug("Control message handler cancelled")
    
    async def _handle_sensorimotor_messages(self):
        """Handle incoming sensorimotor messages."""
        try:
            while self._running and self.sensorimotor_socket:
                try:
                    # Receive message
                    frames = await self.sensorimotor_socket.recv_multipart()
                    if len(frames) != 2:  # [topic, message]
                        logger.warning(f"Invalid sensorimotor frame count: {len(frames)}")
                        continue
                    
                    topic, message_data = frames
                    
                    # Handle based on topic
                    if topic == b"sensory":
                        await self._handle_sensory_data(message_data)
                    else:
                        logger.warning(f"Unknown sensorimotor topic: {topic}")
                    
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error handling sensorimotor message: {e}")
                    
        except asyncio.CancelledError:
            logger.debug("Sensorimotor message handler cancelled")
    
    async def _handle_register(self, identity: bytes, request: RegisterRequest) -> bytes:
        """Handle agent registration."""
        agent_id = request.agent_id
        agent_type = request.agent_type
        
        logger.info(f"Registering agent: {agent_id} ({agent_type})")
        
        # Store agent information
        self.agents[agent_id] = {
            "id": agent_id,
            "type": agent_type,
            "identity": identity,
            "last_seen": time.time(),
            "protocol_versions": {
                "FCP": request.protocol_versions.fcp_version,
                "FSMP": request.protocol_versions.fsmp_version,
                "FVP": request.protocol_versions.fvp_version
            }
        }
        
        # Create response
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        response = RegisterResponse()
        response.status = "success"
        response.message = f"Agent {agent_id} registered successfully"
        response.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_REGISTER_RESPONSE
        message.register_response.CopyFrom(response)
        
        return message.SerializeToString()
    
    async def _handle_deregister(self, identity: bytes, request: DeregisterRequest) -> bytes:
        """Handle agent deregistration."""
        agent_id = request.agent_id
        
        logger.info(f"Deregistering agent: {agent_id}")
        
        # Remove agent
        if agent_id in self.agents:
            del self.agents[agent_id]
            status = "success"
            message = f"Agent {agent_id} deregistered successfully"
        else:
            status = "error"
            message = f"Agent {agent_id} not found"
        
        # Create response
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        response = DeregisterResponse()
        response.status = status
        response.message = message
        response.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_DEREGISTER_RESPONSE
        message.deregister_response.CopyFrom(response)
        
        return message.SerializeToString()
    
    async def _handle_heartbeat(self, identity: bytes, request: HeartbeatRequest) -> bytes:
        """Handle heartbeat message."""
        agent_id = request.agent_id
        
        # Update last seen timestamp
        if agent_id in self.agents:
            self.agents[agent_id]["last_seen"] = time.time()
            status = "ok"
        else:
            status = "error"
            
        # Create response
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        response = HeartbeatResponse()
        response.status = status
        response.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_HEARTBEAT_RESPONSE
        message.heartbeat_response.CopyFrom(response)
        
        return message.SerializeToString()
    
    async def _handle_status_request(self, identity: bytes, request: StatusRequest) -> bytes:
        """Handle status request message."""
        # Create response
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        response = StatusResponse()
        response.status = "ok"
        response.runtime.cpu_usage = 10.0  # Example values
        response.runtime.memory_usage = 250.5
        response.runtime.uptime_seconds = 3600
        response.agent_count = len(self.agents)
        response.timestamp.CopyFrom(current_time)
        
        # Create FCP message
        message = FCPMessage()
        message.header.protocol_id = ProtocolID.FCP
        message.header.version = 1
        message.type = FCPMessageType.FCP_STATUS_RESPONSE
        message.status_response.CopyFrom(response)
        
        return message.SerializeToString()
    
    async def _handle_sensory_data(self, message_data: bytes):
        """Handle incoming sensory data."""
        # Parse the message
        fsmp_message = FSMPMessage()
        fsmp_message.ParseFromString(message_data)
        
        # Verify it's a sensory message
        if fsmp_message.type != FSMPMessageType.FSMP_SENSORY:
            logger.warning(f"Received non-sensory message type: {fsmp_message.type}")
            return
        
        # Extract data
        channel_id = fsmp_message.sensory_data.channel_id
        data = fsmp_message.sensory_data.data
        timestamp = fsmp_message.sensory_data.timestamp.time_ms / 1000
        
        logger.debug(f"Received sensory data on channel {channel_id}: {len(data)} bytes")
        
        # Process the data if callback is registered
        if self.sensory_callback:
            await self.sensory_callback(channel_id, data)
    
    async def send_motor_data(self, channel_id: int, data: bytes):
        """
        Send motor data to agents.
        
        Args:
            channel_id: Motor channel ID
            data: Motor data
        """
        if not self._running or not self.sensorimotor_socket:
            logger.warning("Cannot send motor data: server not running")
            return
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create motor data message
        motor_data = MotorData()
        motor_data.channel_id = channel_id
        motor_data.data = data
        motor_data.timestamp.CopyFrom(current_time)
        
        # Create FSMP message
        message = FSMPMessage()
        message.header.protocol_id = ProtocolID.FSMP
        message.header.version = 1
        message.type = FSMPMessageType.FSMP_MOTOR
        message.motor_data.CopyFrom(motor_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.sensorimotor_socket.send_multipart([b"motor", data])
        
    async def send_activity_data(self, data: bytes):
        """
        Send neural activity data to visualization clients.
        
        Args:
            data: Serialized activity data
        """
        if not self._running or not self.viz_activity_socket:
            logger.warning("Cannot send activity data: server not running")
            return
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create activity data message
        activity_data = ActivityData()
        # In a real implementation, you would parse the data and fill in the fields
        # For this example, we'll assume data is pre-serialized ActivityData
        activity_data.ParseFromString(data)
        
        # Update timestamp
        activity_data.timestamp.CopyFrom(current_time)
        
        # Create FVP message
        message = FVPMessage()
        message.header.protocol_id = ProtocolID.FVP
        message.header.version = 1
        message.type = FVPMessageType.FVP_ACTIVITY
        message.activity_data.CopyFrom(activity_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.viz_activity_socket.send_multipart([b"activity", data])
        
    async def send_structure_data(self, data: bytes):
        """
        Send brain structure data to visualization clients.
        
        Args:
            data: Serialized structure data
        """
        if not self._running or not self.viz_structure_socket:
            logger.warning("Cannot send structure data: server not running")
            return
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create structure data message
        structure_data = StructureData()
        # In a real implementation, you would parse the data and fill in the fields
        # For this example, we'll assume data is pre-serialized StructureData
        structure_data.ParseFromString(data)
        
        # Update timestamp
        structure_data.timestamp.CopyFrom(current_time)
        
        # Create FVP message
        message = FVPMessage()
        message.header.protocol_id = ProtocolID.FVP
        message.header.version = 1
        message.type = FVPMessageType.FVP_STRUCTURE
        message.structure_data.CopyFrom(structure_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.viz_structure_socket.send_multipart([b"structure", data])
    
    def register_sensory_callback(self, callback: Callable[[int, bytes], None]):
        """
        Register a callback for processing incoming sensory data.
        
        Args:
            callback: Function to call when sensory data is received
                     (parameters: channel_id, data)
        """
        self.sensory_callback = callback
    
    async def _process_handshake_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process a handshake message.
        
        Args:
            agent_id: Agent ID (may be None for hello messages)
            message: Decoded Cap'n Proto handshake message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        try:
            # Convert message to dictionary for easier handling
            message_dict = self.translator.handshake_message_to_dict(message)
            
            # Handle different message types
            if message.type == message.type.hello:
                client_id = message.hello.agentId
                client_type = message.hello.agentType
                zmq_id = None  # We don't have the ZMQ ID yet, handled by HandshakeMessageHandler
                
                logger.info(f"Processing hello from {client_type} client '{client_id}'")
                
                # Store client ID -> agent ID mapping for subsequent messages
                self.pending_clients[client_id] = {
                    "agent_id": client_id,
                    "agent_type": client_type,
                    "timestamp": asyncio.get_running_loop().time()
                }
                
                # Create welcome message
                welcome_msg = self.translator.create_handshake_welcome(
                    server_id=self.server_id,
                    message=f"Welcome to FEAGI {client_type} '{client_id}'"
                )
                
                # Convert to dictionary for response
                return {
                    "protocolId": welcome_msg.protocolId,
                    "version": welcome_msg.version,
                    "type": welcome_msg.type,
                    "welcome": {
                        "serverId": welcome_msg.welcome.serverId,
                        "message": welcome_msg.welcome.message,
                        "timestamp": welcome_msg.welcome.timestamp
                    }
                }
                
            elif message.type == message.type.capabilities:
                if agent_id not in self.pending_clients:
                    logger.warning(f"Received capabilities from unknown client {agent_id}")
                    return None
                    
                # Extract capabilities
                sensory_channels = list(message.capabilities.supportedSensoryChannels)
                motor_channels = list(message.capabilities.supportedMotorChannels)
                
                # Extract protocol versions
                protocol_versions = {
                    "fcp": message.capabilities.protocolVersions.fcpVersion,
                    "fsmp": message.capabilities.protocolVersions.fsmpVersion,
                    "fvp": message.capabilities.protocolVersions.fvpVersion
                }
                
                logger.info(f"Received capabilities from {agent_id}: "
                           f"sensory={sensory_channels}, motor={motor_channels}, "
                           f"protocols={protocol_versions}")
                
                # Register client in connection manager
                # Note: The HandshakeMessageHandler should have updated the ZMQ ID
                client_info = self.pending_clients.get(agent_id)
                if client_info:
                    # Get ZMQ ID from somewhere (needs to be passed from handler)
                    zmq_id = client_info.get("zmq_id")
                    if zmq_id:
                        # Register client with connection manager
                        self.connection_manager.register_client(
                            agent_id=agent_id,
                            zmq_id=zmq_id,
                            supported_protocols=protocol_versions
                        )
                        
                        # Clean up pending client
                        del self.pending_clients[agent_id]
                        
                        # Create configuration message
                        config_msg = self.translator.create_handshake_configuration({
                            # Add server configuration here
                        })
                        
                        # Convert to dictionary for response
                        return {
                            "protocolId": config_msg.protocolId,
                            "version": config_msg.version,
                            "type": config_msg.type,
                            "configuration": {
                                "timestamp": config_msg.configuration.timestamp
                                # Add configuration fields
                            }
                        }
                
            return None
                
        except Exception as e:
            logger.error(f"Error processing handshake message: {e}")
            return None
    
    async def _process_fcp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FCP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FCP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FCP message from {agent_id}")
        
        # Handle specific FCP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fsmp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FSMP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FSMP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FSMP message from {agent_id}")
        
        # Handle specific FSMP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fvp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FVP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FVP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FVP message from {agent_id}")
        
        # Handle specific FVP message types here
        # For now, just log and return None (no response)
        
        return None
    
    def get_server_stats(self) -> Dict[str, Any]:
        """
        Get server statistics.
        
        Returns:
            Dictionary with server statistics
        """
        return {
            "server_id": self.server_id,
            "running": self._running,
            "connections": self.connection_manager.get_connection_stats(),
            "pending_clients": len(self.pending_clients)
        }
    
    async def broadcast_message(self, 
                              protocol_type: str, 
                              message_data: Dict[str, Any],
                              filter_func: Optional[Callable[[str, Dict[str, Any]], bool]] = None) -> int:
        """
        Broadcast a message to all connected clients or a filtered subset.
        
        Args:
            protocol_type: Protocol type ("fcp", "fsmp", or "fvp")
            message_data: Message data to broadcast
            filter_func: Function to filter clients (agent_id, client_info) -> bool
            
        Returns:
            Number of clients the message was sent to
        """
        # Check protocol type
        if protocol_type not in ["fcp", "fsmp", "fvp"]:
            raise ValueError(f"Invalid protocol type: {protocol_type}")
            
        # Get all connected clients
        count = 0
        for agent_id, client_info in self.connection_manager.connections.items():
            # Apply filter if provided
            if filter_func and not filter_func(agent_id, client_info):
                continue
                
            # Create and encode message
            if protocol_type == "fcp":
                message = self.translator.fcp_schema.FCPMessage.new_message(**message_data)
            elif protocol_type == "fsmp":
                message = self.translator.fsmp_schema.FSMPMessage.new_message(**message_data)
            elif protocol_type == "fvp":
                message = self.translator.fvp_schema.FVPMessage.new_message(**message_data)
                
            # Send message
            encoded_message = message.to_bytes()
            success = await self.connection_manager.send_message(
                agent_id=agent_id,
                protocol_type=protocol_type,
                message=encoded_message
            )
            
            if success:
                count += 1
                
        return count
    
    async def _cleanup_inactive_clients(self) -> None:
        """Periodically clean up inactive clients."""
        try:
            while True:
                # Wait for a while
                await asyncio.sleep(30)
                
                # Find inactive clients
                inactive_clients = self.connection_manager.get_inactive_clients(timeout_seconds=60)
                
                # Deregister inactive clients
                for agent_id in inactive_clients:
                    logger.info(f"Deregistering inactive client {agent_id}")
                    self.connection_manager.deregister_client(agent_id)
                    
                # Clean up pending clients
                now = asyncio.get_running_loop().time()
                for client_id in list(self.pending_clients.keys()):
                    client_info = self.pending_clients[client_id]
                    if now - client_info["timestamp"] > 30:  # 30 seconds timeout
                        logger.info(f"Removing pending client {client_id} due to timeout")
                        del self.pending_clients[client_id]
                        
        except asyncio.CancelledError:
            logger.info("Cleanup task cancelled")
            
        except Exception as e:
            logger.error(f"Error in cleanup task: {e}")
    
    async def _process_fcp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FCP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FCP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FCP message from {agent_id}")
        
        # Handle specific FCP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fsmp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FSMP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FSMP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FSMP message from {agent_id}")
        
        # Handle specific FSMP message types here
        # For now, just log and return None (no response)
        
        return None
    
    async def _process_fvp_message(self, agent_id: str, message: Any) -> Optional[Dict[str, Any]]:
        """
        Process an FVP message.
        
        Args:
            agent_id: Agent ID
            message: Decoded Cap'n Proto FVP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FVP message from {agent_id}")
        
        # Handle specific FVP message types here
        # For now, just log and return None (no response)
        
        return None 