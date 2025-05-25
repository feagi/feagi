"""
ZeroMQ Server for FEAGI API.

Provides ZeroMQ-based API access to FEAGI functionality.
"""

import asyncio
import json
import logging
import sys
import threading
import time
from typing import Dict, Any, Optional, List, Set, Callable, Callable
import uuid
import traceback

import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger

# Set up logger early so it's available for imports
logger = setup_logger()

# Core dependencies
from feagi.core.state_manager import FeagiStateManager, GenomeState
from feagi.bdu.connectome_manager import ConnectomeManager

# Import all stream handlers
from .streams.sensory import SensoryStream
from .streams.motor import MotorStream  
from .streams.visualization import VisualizationStream
from .streams.control import ControlStream
from .streams.rest import RestStream

# Import pattern handlers
from .patterns.req_rep import RequestReplyManager
from .patterns.pub_sub import PubSubManager
from .patterns.push_pull import PushPullManager

# Import connection manager
from .connection_manager import ConnectionManager

# Import the unified CoreAPIService
from ..core.services.core_api_service import CoreAPIService

# Force importing the actual ZMQ module first to avoid circular imports
try:
    import zmq
    try:
        import zmq.asyncio
    except ImportError:
        logger.warning("zmq.asyncio not available - using minimal mock")
        # Simple dummy class when asyncio support is not available
        if not hasattr(zmq, 'asyncio'):
            class DummyAsync:
                class Context:
                    @classmethod
                    def instance(cls):
                        return zmq.Context.instance()
            zmq.asyncio = DummyAsync()
except ImportError:
    logger.warning("zmq package not available - server functionality will be limited")
    import zmq

from zmq.auth.thread import ThreadAuthenticator

from .rest_adapter import ZMQRestAPIAdapter  # Import the REST API adapter

# Import protocol definitions
from feagi.api.protocols.constants import (
    ProtocolID, FCPCommandType, FVPFrameType, FSMPChannelType
)
from feagi.api.protocols.translator import ByteStructureTranslator

# Import ConnectionManager from the new, corrected file
from feagi.api.zmq.connection_manager import ConnectionManager
from feagi.api.zmq.message_handlers import MessageHandler, start_message_handlers, stop_message_handlers

# Define the request/response models needed for ZMQ server
class Timestamp:
    """Timestamp for protocol messages."""
    def __init__(self):
        self.time_ms = 0  # Time in milliseconds
        
    def CopyFrom(self, other):
        self.time_ms = other.time_ms

class ProtocolVersions:
    """Protocol versions supported by an agent."""
    def __init__(self):
        self.fcp_version = 1
        self.fsmp_version = 1 
        self.fvp_version = 1

class RegisterRequest:
    """Request for agent registration."""
    def __init__(self):
        self.agent_id = ""
        self.agent_type = ""
        self.protocol_versions = ProtocolVersions()

class RegisterResponse:
    """Response for agent registration."""
    def __init__(self):
        self.status = ""
        self.message = ""
        self.timestamp = Timestamp()
        
    def CopyFrom(self, other):
        self.status = other.status
        self.message = other.message
        self.timestamp.CopyFrom(other.timestamp)

class DeregisterRequest:
    """Request for agent deregistration."""
    def __init__(self):
        self.agent_id = ""

class DeregisterResponse:
    """Response for agent deregistration."""
    def __init__(self):
        self.status = ""
        self.message = ""
        self.timestamp = Timestamp()
        
    def CopyFrom(self, other):
        self.status = other.status
        self.message = other.message
        self.timestamp.CopyFrom(other.timestamp)

class HeartbeatRequest:
    """Request for agent heartbeat."""
    def __init__(self):
        self.agent_id = ""

class HeartbeatResponse:
    """Response for agent heartbeat."""
    def __init__(self):
        self.status = ""
        self.timestamp = Timestamp()
        
    def CopyFrom(self, other):
        self.status = other.status
        self.timestamp.CopyFrom(other.timestamp)

class RuntimeInfo:
    """Runtime information for status response."""
    def __init__(self):
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        self.uptime_seconds = 0

class StatusRequest:
    """Request for server status."""
    def __init__(self):
        pass

class StatusResponse:
    """Response with server status."""
    def __init__(self):
        self.status = ""
        self.runtime = RuntimeInfo()
        self.agent_count = 0
        self.timestamp = Timestamp()
        
    def CopyFrom(self, other):
        self.status = other.status
        self.runtime.cpu_usage = other.runtime.cpu_usage
        self.runtime.memory_usage = other.runtime.memory_usage
        self.runtime.uptime_seconds = other.runtime.uptime_seconds
        self.agent_count = other.agent_count
        self.timestamp.CopyFrom(other.timestamp)

class ZmqServer:
    """
    ZMQ server for FEAGI with proper event loop management.
    
    This implementation ensures each thread has its own event loop and
    properly manages asyncio resources.
    """
    
    def __init__(
        self,
        core_api: CoreAPIService,
        host: str = "127.0.0.1",
        req_rep_port: int = 5555,
        pub_sub_port: int = 5556,
        push_pull_port: int = 5557,
        sensory_port: Optional[int] = 5558,
        motor_port: Optional[int] = 5564,
        control_port: Optional[int] = 5559,
        rest_port: int = 5563,
        vis_port: Optional[int] = 5562,
        context: Optional[zmq.asyncio.Context] = None,
        fq_sampler: Optional[Any] = None,
        fq_sampler_queue: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize the ZeroMQ server for FEAGI.
        
        Args:
            core_api: Core API service for delegating calls to FEAGI core
            host: Host address to bind to
            req_rep_port: Port for REQ/REP pattern (5555)
            pub_sub_port: Port for PUB/SUB pattern (5556)
            push_pull_port: Port for PUSH/PULL pattern (5557)
            sensory_port: Port for sensory data (5558), None to disable
            motor_port: Port for motor data (5564), None to disable
            control_port: Port for control interface (5559), None to disable
            rest_port: Port for REST API (5563)
            vis_port: Port for visualization data (5562), None to disable
            context: Optional ZeroMQ context to use
            fq_sampler: Optional FQ sampler instance for visualization data
            fq_sampler_queue: Optional queue for FQ data from the sampler
            stream_config: Optional stream configuration from TOML
        """
        self.core_api = core_api
        self.host = host
        self.req_rep_port = req_rep_port
        self.pub_sub_port = pub_sub_port
        self.push_pull_port = push_pull_port
        self.sensory_port = sensory_port
        self.motor_port = motor_port
        self.control_port = control_port
        self.rest_port = rest_port
        self.vis_port = vis_port
        
        # Store stream configuration
        self.stream_config = stream_config or {}
        
        # Create ZeroMQ context
        self._context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._running = False
        
        # Store components (some may be None if disabled)
        self._req_rep = None
        self._pub_sub = None
        self._push_pull = None
        self._sensory = None
        self._motor = None
        self._control = None
        self._visualization = None
        self._rest = None
        
        # FQ Sampler integration
        self._fq_sampler = fq_sampler
        self._fq_sampler_queue = fq_sampler_queue
        
        # Thread and event loop management
        self._thread = None
        self._loop = None
        self._shutdown_event = threading.Event()
        
        # Initialize sockets (will be created on start, may be None if stream disabled)
        self.control_socket = None
        self.sensory_socket = None
        self.motor_socket = None
        self.vis_socket = None
        
        # State tracking
        self.agents = {}  # agent_id -> agent_info
        self.tasks = []
        
        # Callbacks
        self.sensory_callback = None
        
        # Create connection manager (only for enabled streams)
        enabled_ports = {}
        if self.control_port is not None:
            enabled_ports['control'] = self.control_port
        if self.sensory_port is not None:
            enabled_ports['sensory'] = self.sensory_port
        if self.vis_port is not None:
            enabled_ports['visualization'] = self.vis_port
            
        self.connection_manager = ConnectionManager(
            context=self._context,
            control_port=self.control_port,
            sensory_port=self.sensory_port,
            motor_port=self.motor_port,
            visualization_port=self.vis_port
        )
        
        # Create protocol translator
        self.translator = ByteStructureTranslator()
        
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
        Start the ZMQ server components.
        
        This method:
        1. Initializes all the ZMQ patterns (REQ/REP, PUB/SUB, etc.)
        2. Sets up the message handlers
        3. Starts the control and data processing tasks
        """
        try:
            logger.info("Starting ZMQ server")
            self._running = True
            
            # Create REST API adapter
            self.rest_api_adapter = ZMQRestAPIAdapter(self.core_api)
            
            # We'll import here to avoid circular imports
            from .patterns.req_rep import RequestReplyManager
            from .patterns.pub_sub import PubSubManager
            from .patterns.push_pull import PushPullManager
            from .streams.sensory import SensoryStream
            from .streams.motor import MotorStream
            from .streams.control import ControlStream
            from .streams.rest import RestStream
            # VisualizationStream imported conditionally at module level
            
            # Initialize only enabled streams based on port configuration
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
            
            # Only create enabled streams (port != None)
            if self.sensory_port is not None:
                self._sensory = SensoryStream(
                    core_api=self.core_api,
                    host=self.host,
                    port=self.sensory_port,
                    context=self._context
                )
                logger.info(f"Sensory stream enabled on port {self.sensory_port}")
            else:
                logger.info("Sensory stream disabled")
            
            if self.motor_port is not None:
                self._motor = MotorStream(
                    core_api=self.core_api,
                    host=self.host,
                    port=self.motor_port,
                    context=self._context,
                    fq_sampler=self._fq_sampler,
                    fq_sampler_queue=self._fq_sampler_queue,
                    stream_config=self.stream_config.get('motor', {})
                )
                logger.info(f"Motor stream enabled on port {self.motor_port}")
            else:
                logger.info("Motor stream disabled")
            
            if self.control_port is not None:
                self._control = ControlStream(
                    core_api=self.core_api,
                    host=self.host,
                    port=self.control_port,
                    context=self._context
                )
                logger.info(f"Control stream enabled on port {self.control_port}")
            else:
                logger.info("Control stream disabled")
            
            self._rest = RestStream(
                core_api=self.core_api,
                host=self.host,
                port=self.rest_port,
                context=self._context
            )
            
            # Pass ZMQ server reference to REST stream for visualization endpoints
            if hasattr(self._rest, 'set_zmq_server'):
                self._rest.set_zmq_server(self)
                logger.debug("ZMQ server reference passed to REST stream")
            
            if self.vis_port is not None:
                # Use full-featured visualization stream
                self._visualization = VisualizationStream(
                    core_api=self.core_api,
                    host=self.host,
                    port=self.vis_port,
                    fq_sampler=self._fq_sampler,
                    fq_sampler_queue=self._fq_sampler_queue,
                    context=self._context,
                    stream_config=self.stream_config.get('visualization', {})
                )
                logger.info(f"Visualization stream enabled on port {self.vis_port}")
            else:
                logger.info("Visualization stream disabled")
            
            # Start only enabled managers
            await self._req_rep.start()
            await self._pub_sub.start()
            await self._push_pull.start()
            
            if self._sensory:
                await self._sensory.start()
            if self._motor:
                await self._motor.start()
            if self._control:
                await self._control.start()
            
            await self._rest.start()
            
            # RTOS: VisualizationStream is now synchronous
            if self._visualization:
                self._visualization.start()  # No await - synchronous method
            
            # Create sockets only for enabled streams
            if self._control:
                self.control_socket = self._control.router_socket
            
            if self._sensory:
                self.sensory_socket = self._sensory.socket
            
            if self._motor:
                self.motor_socket = self._motor.socket
            
            if self._visualization:
                self.vis_socket = self._visualization.socket
            
            # Start message handling tasks
            # Note: Control messages are handled by ControlStream on port 5559
            self.tasks.append(asyncio.create_task(self._sensory_data_loop()))
            
            # Start message handlers
            self.message_handlers = await start_message_handlers(
                self.connection_manager,
                None,
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
            print("ZMQ server is not running", file=sys.stderr, flush=True)
            return
        
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        print("Shutting down ZMQ server", file=sys.stderr, flush=True)
        
        # Signal the monitor loop to stop
        self._shutdown_event.set()
        
        try:
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
            print("ZMQ server shutdown complete", file=sys.stderr, flush=True)
            
        except Exception as e:
            print(f"Error during ZMQ server shutdown: {e}", file=sys.stderr, flush=True)
    
    async def _stop_services(self):
        """
        Stop all ZMQ services asynchronously.
        """
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        print("Stopping ZMQ services", file=sys.stderr, flush=True)
        
        try:
            # Stop all services
            stop_tasks = []
            
            if self._req_rep:
                stop_tasks.append(self._req_rep.stop())
        
            if self._pub_sub:
                stop_tasks.append(self._pub_sub.stop())
        
            if self._push_pull:
                stop_tasks.append(self._push_pull.stop())
        
            if self._sensory:
                stop_tasks.append(self._sensory.stop())
        
            if self._motor:
                stop_tasks.append(self._motor.stop())
        
            if self._control:
                stop_tasks.append(self._control.stop())
        
            if self._rest:
                stop_tasks.append(self._rest.stop())
        
            # RTOS: VisualizationStream is now synchronous, call stop() directly
            if self._visualization:
                self._visualization.stop()  # Direct call - synchronous method
            
            # Wait for all services
            if stop_tasks:
                await asyncio.gather(*stop_tasks, return_exceptions=True)
            
            print("All ZMQ services stopped", file=sys.stderr, flush=True)
            
            # Close sockets
            for socket in [self.control_socket, self.sensory_socket, 
                          self.motor_socket, self.vis_socket]:
                if socket:
                    socket.close(linger=0)
                
            self.control_socket = None
            self.sensory_socket = None
            self.motor_socket = None
            self.vis_socket = None
            
        except Exception as e:
            print(f"Error stopping ZMQ services: {e}", file=sys.stderr, flush=True)
    
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
            self._sensory = None
            self._motor = None
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
            
            # @cursor:critical-path - Signal-safe cleanup should minimize logging
            print("ZMQ server resources cleaned up", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"Error during ZMQ server cleanup: {e}", file=sys.stderr, flush=True)
    
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
    
    async def _sensory_data_loop(self):
        """Handle incoming sensory data."""
        try:
            while self._running and self.sensory_socket:
                try:
                    # Receive message
                    frames = await self.sensory_socket.recv_multipart()
                    if len(frames) != 2:  # [topic, message]
                        logger.warning(f"Invalid sensory frame count: {len(frames)}")
                        continue
                    
                    topic, message_data = frames
                    
                    # Handle based on topic
                    if topic == b"sensory":
                        await self._handle_sensory_data(message_data)
                    else:
                        logger.warning(f"Unknown sensory topic: {topic}")
                    
                except asyncio.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error handling sensory message: {e}")
        except asyncio.CancelledError:
            logger.debug("Sensory message handler cancelled")
    
    async def _handle_register(self, identity: bytes, request: RegisterRequest) -> bytes:
        """
        Handle an agent registration request.
        
        Args:
            identity: Client identity
            request: Registration request
            
        Returns:
            Registration response as binary data
        """
        # Extract agent information
        agent_id = request.agent_id
        agent_type = request.agent_type
        
        logger.info(f"Registering agent: {agent_id} (type: {agent_type})")
        
        # Register the agent
        timestamp_ms = int(time.time() * 1000)
        
        # Add to agents dictionary
        self.agents[agent_id] = {
            "id": agent_id,
            "type": agent_type,
            "identity": identity,
            "registered_at": timestamp_ms,
            "last_heartbeat": timestamp_ms,
            "capabilities": {},
        }
        
        # Create response as dictionary
        response_dict = {
            "type": "register_response",
            "status": "success",
            "message": f"Agent {agent_id} registered successfully",
            "timestamp": {"time_ms": timestamp_ms}
        }
        
        # Encode as binary using ByteStructureTranslator
        return self.translator.create_message(response_dict)
    
    async def _handle_deregister(self, identity: bytes, request: DeregisterRequest) -> bytes:
        """
        Handle an agent deregistration request.
        
        Args:
            identity: Client identity
            request: Deregistration request
            
        Returns:
            Deregistration response as binary data
        """
        agent_id = request.agent_id
        
        logger.info(f"Deregistering agent: {agent_id}")
        
        # Remove agent if it exists
        if agent_id in self.agents:
            del self.agents[agent_id]
            status = "success"
            message = f"Agent {agent_id} deregistered successfully"
        else:
            status = "error"
            message = f"Agent {agent_id} not found"
        
        # Create response
        response = DeregisterResponse()
        response.status = status
        response.message = message
        response.timestamp.time_ms = int(time.time() * 1000)
        
        # Create JSON response for compatibility
        response_dict = {
            "status": response.status,
            "message": response.message,
            "timestamp": {"time_ms": response.timestamp.time_ms}
        }
        
        return json.dumps(response_dict).encode('utf-8')
    
    async def _handle_heartbeat(self, identity: bytes, request: HeartbeatRequest) -> bytes:
        """
        Handle an agent heartbeat request.
        
        Args:
            identity: Client identity
            request: Heartbeat request
            
        Returns:
            Heartbeat response
        """
        agent_id = request.agent_id
        
        # Update last heartbeat time if agent exists
        if agent_id in self.agents:
            self.agents[agent_id]["last_heartbeat"] = int(time.time() * 1000)
            status = "success"
        else:
            status = "error"
        
        # Create response
        response = HeartbeatResponse()
        response.status = status
        response.timestamp.time_ms = int(time.time() * 1000)
        
        # Create JSON response for compatibility
        response_dict = {
            "status": response.status,
            "timestamp": {"time_ms": response.timestamp.time_ms}
        }
        
        return json.dumps(response_dict).encode('utf-8')
    
    async def _handle_status_request(self, identity: bytes, request: StatusRequest) -> bytes:
        """
        Handle a status request.
        
        Args:
            identity: Client identity
            request: Status request
            
        Returns:
            Status response
        """
        # Get system stats
        stats = self.get_server_stats()
        
        # Create response with properly formatted timestamp
        response = StatusResponse()
        response.status = "ok"
        response.runtime.cpu_usage = stats["cpu_usage"]
        response.runtime.memory_usage = stats["memory_usage"]
        response.runtime.uptime_seconds = stats["uptime_seconds"]
        response.agent_count = len(self.agents)
        response.timestamp.time_ms = int(time.time() * 1000)
        
        # Check if we're dealing with an actual StatusRequest object from byte structure
        if isinstance(request, StatusRequest):
            # Create a JSON representation of the response
            response_dict = {
                "status": response.status,
                "runtime": {
                    "cpu_usage": response.runtime.cpu_usage,
                    "memory_usage": response.runtime.memory_usage,
                    "uptime_seconds": response.runtime.uptime_seconds
                },
                "agent_count": response.agent_count,
                "timestamp": {"time_ms": response.timestamp.time_ms}
            }
            
            # Return JSON-encoded response for compatibility
            return json.dumps(response_dict).encode('utf-8')
        else:
            # For real byte structure messages, need to implement proper serialization
            # This is a placeholder for future implementation
            response_dict = {
                "status": response.status,
                "runtime": {
                    "cpu_usage": response.runtime.cpu_usage,
                    "memory_usage": response.runtime.memory_usage,
                    "uptime_seconds": response.runtime.uptime_seconds
                },
                "agent_count": response.agent_count,
                "timestamp": {"time_ms": response.timestamp.time_ms}
            }
            return json.dumps(response_dict).encode('utf-8')
    
    async def _handle_sensory_data(self, message_data: bytes):
        """Handle incoming sensory data."""
        # Parse the message
        fsmp_message = self.translator.fsmp_schema.FSMPMessage.new_message()
        fsmp_message.ParseFromString(message_data)
        
        # Verify it's a sensory message
        if fsmp_message.type != FSMPChannelType.FSMP_SENSORY:
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
        if not self._running or not self.motor_socket:
            logger.warning("Cannot send motor data: server not running")
            return
            
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create motor data message
        motor_data = self.translator.fsmp_schema.MotorData.new_message()
        motor_data.channel_id = channel_id
        motor_data.data = data
        motor_data.timestamp.CopyFrom(current_time)
        
        # Create FSMP message
        message = self.translator.fsmp_schema.FSMPMessage.new_message()
        message.header.protocol_id = ProtocolID.FSMP
        message.header.version = 1
        message.type = FSMPChannelType.FSMP_MOTOR
        message.motor_data.CopyFrom(motor_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.motor_socket.send_multipart([b"motor", data])
        
    async def send_activity_data(self, data: bytes):
        """
        Send neural activity data to visualization clients.
        
        Args:
            data: Serialized activity data
        """
        if not self._running or not self.vis_socket:
            logger.warning("Cannot send activity data: server not running")
            return
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create activity data message
        activity_data = self.translator.fvp_schema.ActivityData.new_message()
        # In a real implementation, you would parse the data and fill in the fields
        # For this example, we'll assume data is pre-serialized ActivityData
        activity_data.ParseFromString(data)
        
        # Update timestamp
        activity_data.timestamp.CopyFrom(current_time)
        
        # Create FVP message
        message = self.translator.fvp_schema.FVPMessage.new_message()
        message.header.protocol_id = ProtocolID.FVP
        message.header.version = 1
        message.type = FVPFrameType.FVP_ACTIVITY
        message.activity_data.CopyFrom(activity_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.vis_socket.send_multipart([b"activity", data])
        
    async def send_structure_data(self, data: bytes):
        """
        Send brain structure data to visualization clients.
        
        Args:
            data: Serialized structure data
        """
        if not self._running or not self.vis_socket:
            logger.warning("Cannot send structure data: server not running")
            return
        
        # Create timestamp
        current_time = Timestamp()
        current_time.time_ms = int(time.time() * 1000)
        
        # Create structure data message
        structure_data = self.translator.fvp_schema.StructureData.new_message()
        # In a real implementation, you would parse the data and fill in the fields
        # For this example, we'll assume data is pre-serialized StructureData
        structure_data.ParseFromString(data)
        
        # Update timestamp
        structure_data.timestamp.CopyFrom(current_time)
        
        # Create FVP message
        message = self.translator.fvp_schema.FVPMessage.new_message()
        message.header.protocol_id = ProtocolID.FVP
        message.header.version = 1
        message.type = FVPFrameType.FVP_STRUCTURE
        message.structure_data.CopyFrom(structure_data)
        
        # Serialize and send
        data = message.SerializeToString()
        await self.vis_socket.send_multipart([b"structure", data])
    
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
            message: Decoded FCP message
            
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
            message: Decoded FSMP message
            
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
            message: Decoded FVP message
            
        Returns:
            Response data if a response is needed, otherwise None
        """
        logger.info(f"Processing FVP message from {agent_id}")
        
        # Handle specific FVP message types here
        # For now, just log and return None (no response)
        
        return None
    
    def get_server_stats(self) -> Dict[str, Any]:
        """
        Get ZMQ server statistics.
        
        Returns:
            Dictionary containing server statistics
        """
        total_agents = len(self.agents)
        active_agents = len([a for a in self.agents.values() if a.get('last_heartbeat')])
        
        return {
            'running': self._running,
            'total_agents': total_agents,
            'active_agents': active_agents,
            'host': self.host,
            'req_rep_port': self.req_rep_port,
            'pub_sub_port': self.pub_sub_port,
            'push_pull_port': self.push_pull_port,
            'control_port': self.control_port,
            'sensory_port': self.sensory_port,
            'motor_port': self.motor_port,
            'rest_port': self.rest_port,
            'vis_port': self.vis_port,
            'enabled_streams': {
                'sensory': self._sensory is not None,
                'motor': self._motor is not None,
                'control': self._control is not None,
                'visualization': self._visualization is not None,
            }
        }

    def get_visualization_stream(self):
        """
        Get the visualization stream instance.
        
        Returns:
            VisualizationStream instance if visualization is enabled, None otherwise
        """
        return self._visualization
    
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
    
    async def _process_control_message(self, identity: bytes, message: Dict[str, Any]) -> bytes:
        """
        Process a control message and return the appropriate response.
        
        Args:
            identity: Client identity
            message: Parsed message data
            
        Returns:
            Response data to send back to the client
        """
        message_type = message.get("type")
        
        if message_type == "register":
            # Handle registration
            request = RegisterRequest()
            request.agent_id = message.get("agent_id", "")
            request.agent_type = message.get("agent_type", "")
            
            return await self._handle_register(identity, request)
        
        elif message_type == "deregister":
            # Handle deregistration
            request = DeregisterRequest()
            request.agent_id = message.get("agent_id", "")
            
            return await self._handle_deregister(identity, request)
        
        elif message_type == "heartbeat":
            # Handle heartbeat
            request = HeartbeatRequest()
            request.agent_id = message.get("agent_id", "")
            
            return await self._handle_heartbeat(identity, request)
        
        elif message_type == "status_request":
            # Handle status request
            request = StatusRequest()
            
            return await self._handle_status_request(identity, request)
        
        else:
            logger.warning(f"Unknown control message type: {message_type}")
            # Create JSON error response
            response = {"status": "error", "message": f"Unknown message type: {message_type}"}
            return json.dumps(response).encode('utf-8')
    
    async def _receive_with_timeout(self, socket, timeout):
        """
        Wait for a message on a socket with timeout.
        
        Args:
            socket: ZMQ socket to receive from
            timeout: Timeout in seconds
            
        Returns:
            True if a message is available, False on timeout
        """
        try:
            # Set up poller
            poller = zmq.asyncio.Poller()
            poller.register(socket, zmq.POLLIN)
            
            # Wait for events with timeout
            events = await asyncio.wait_for(poller.poll(timeout=timeout * 1000), timeout=timeout+0.1)  # milliseconds
            
            # Return True if this socket has data available
            return len(events) > 0 and dict(events).get(socket) == zmq.POLLIN
            
        except asyncio.CancelledError:
            # Make sure we handle cancellation properly
            logger.debug("_receive_with_timeout cancelled")
            raise  # Re-raise to propagate the cancellation
        except asyncio.TimeoutError:
            # Handle explicit timeout
            return False
        except Exception as e:
            logger.error(f"Error in _receive_with_timeout: {str(e)}")
            return False 