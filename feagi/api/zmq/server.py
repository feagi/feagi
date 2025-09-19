"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

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
from typing import Any, Callable, Dict, Optional

import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger

# Set up logger early so it's available for imports
logger = setup_logger(__name__)


# Core dependencies

# Import the unified CoreAPIService
from ..core.services.core_api_service import CoreAPIService

# Import connection manager
# Import pattern handlers
# Import all stream handlers
from .streams.visualization import VisualizationStream

# Import visualization streams conditionally
try:
    from feagi.api.zmq.streams.visualization import VisualizationStream

    _visualization_available = True
except ImportError:
    _visualization_available = False
    logger = logging.getLogger(__name__)
    logger.debug("Visualization stream not available")

# Force importing the actual ZMQ module first to avoid circular imports
try:
    import zmq

    try:
        import zmq.asyncio
    except ImportError:
        logger.warning("zmq.asyncio not available - using minimal mock")
        # Simple dummy class when asyncio support is not available
        if not hasattr(zmq, "asyncio"):

            class DummyAsync:
                class Context:
                    @classmethod
                    def instance(cls):
                        return zmq.Context.instance()

            zmq.asyncio = DummyAsync()
except ImportError:
    logger.warning(
        "zmq package not available - server functionality will be limited"
    )
    import zmq


# Import protocol definitions
from feagi.api.protocols.constants import (
    FSMPChannelType,
    FVPFrameType,
    ProtocolID,
)
from feagi.api.protocols.translator import ByteStructureTranslator

# Import ConnectionManager from the new, corrected file
from feagi.api.zmq.message_handlers import (
    MessageHandler,
    start_message_handlers,
)

from .rest_adapter import ZMQRestAPIAdapter  # Import the REST API adapter


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
    """ZMQ server for FEAGI with proper event loop management.

    This implementation ensures each thread has its own event loop and properly
    manages asyncio resources.
    """

    def __init__(
        self,
        core_api: CoreAPIService,
        req_rep_port: int,
        pub_sub_port: int,
        push_pull_port: int,
        rest_port: int,
        host: str,
        sensory_port: Optional[int] = None,
        motor_port: Optional[int] = None,
        vis_port: Optional[int] = None,
        context: Optional[zmq.asyncio.Context] = None,
        fq_sampler: Optional[Any] = None,
        fire_queue_provider: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None,
        process_manager: Optional[
            Any
        ] = None,  # Accept process manager for on-demand FQ sampler creation
    ):
        """Initialize the ZeroMQ server for FEAGI.

        Args:
            core_api: Core API service for delegating calls to FEAGI core
            req_rep_port: Port for REQ/REP pattern (from config)
            pub_sub_port: Port for PUB/SUB pattern (from config)
            push_pull_port: Port for PUSH/PULL pattern (from config)
            rest_port: Port for REST API (from config)
            host: Host address to bind to
            sensory_port: Port for sensory data (from config), None to disable
            motor_port: Port for motor data (from config), None to disable
            vis_port: Port for visualization data (from config), None to disable
            context: Optional ZeroMQ context to use
            fq_sampler: Optional FQ sampler instance for visualization data
            fire_queue_provider: Optional fire queue provider for visualization data
            stream_config: Optional stream configuration from TOML
            process_manager: Optional process manager for on-demand FQ sampler creation
        """
        self.core_api = core_api
        self.host = host
        self.req_rep_port = req_rep_port
        self.pub_sub_port = pub_sub_port
        self.push_pull_port = push_pull_port
        self.sensory_port = sensory_port
        self.motor_port = motor_port
        self.rest_port = rest_port
        self.vis_port = vis_port

        # Store stream configuration
        self.stream_config = stream_config or {}

        # Create ZeroMQ context
        self._context = context or zmq.asyncio.Context.instance()

        # State tracking
        self._running = False
        self._shutdown_in_progress = False  # Prevent concurrent shutdowns

        # Store components (some may be None if disabled)
        self._req_rep = None
        self._pub_sub = None
        self._push_pull = None
        self._sensory = None
        self._motor = None
        self._visualization = None
        self._rest = None

        # FQ Sampler integration
        self._fq_sampler = fq_sampler
        self._fire_queue_provider = fire_queue_provider
        self._process_manager = (
            process_manager  # Store process manager reference
        )

        # Thread and event loop management
        self._thread = None
        self._loop = None
        self._shutdown_event = threading.Event()

        #  Initialize sockets (will be created on start, may be None if stream
        #  disabled)
        self.sensory_socket = None
        self.motor_socket = None
        self.vis_socket = None

        # State tracking
        self.agents = {}  # agent_id -> agent_info
        self.tasks = []

        # Callbacks
        self.sensory_callback = None

        # Create connection manager only if control port is enabled
        #  NOTE: ConnectionManager now only handles control port - all other
        #  ports are handled by dedicated streams
        #  UPDATE: Control port is also handled by ControlStream, so
        #  ConnectionManager is not needed
        self.connection_manager = None
        logger.info(
            "ConnectionManager disabled - all ports handled by dedicated streams"
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
            "fvp": self._process_fvp_message,
        }

        # Cleanup task
        self.cleanup_task = None

        logger.info(f"ZMQ Server initialized on {host}:{req_rep_port}")

    def start(self) -> bool:
        """Start the ZMQ server in a background thread.

        Returns:
            True if started successfully, False otherwise
        """
        if self._running:
            logger.warning("ZMQ server is already running")
            return True

        logger.info(f"Starting ZMQ server on {self.host}")
        try:
            #  In synchronous mode, create a context and start the server in a
            #  background thread
            logger.info("Creating background thread for ZMQ server")
            self._thread = threading.Thread(
                target=self._run_server_thread, daemon=True
            )
            self._thread.start()

            #  Wait briefly to allow the server to start or fail - use
            #  configurable timeout
            try:
                from feagi.config.toml_loader import (
                    get_timeout_config,
                    load_feagi_config,
                )

                config = load_feagi_config()
                timeout_config = get_timeout_config(config)
                startup_wait = (
                    timeout_config.service_startup / 6.0
                )  # Brief fraction of service startup timeout
            except Exception:
                startup_wait = (
                    0.5  # @architecture:acceptable - emergency fallback
                )

            time.sleep(startup_wait)

            if not self._running:
                #  If the server didn't start properly, the thread will have
                #  set _running to False
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
        """Run the ZMQ server in a background thread.

        This method creates a new event loop for the thread and runs the server
        in it.
        """
        # CRITICAL: Windows asyncio compatibility fix for ZMQ
        # Must be done BEFORE creating event loop for this thread
        import platform

        if platform.system() == "Windows":
            try:
                #  Set the event loop policy to WindowsSelectorEventLoopPolicy
                #  for ZMQ compatibility
                asyncio.set_event_loop_policy(
                    asyncio.WindowsSelectorEventLoopPolicy()
                )
            except AttributeError:
                # Fallback for older Python versions
                logger.warning(
                    "WindowsSelectorEventLoopPolicy not available - ZMQ may have issues"
                )

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

    async def _monitor_and_create_sensory_stream(self) -> None:
        """Monitor FEAGI readiness and create sensory stream when ready."""
        from feagi.core.state_manager import FeagiStateManager, ServiceState, GenomeState
        from .streams.sensory_neural import SensoryNeuralStream as SensoryStream
        
        logger.info("🔍 Starting FEAGI readiness monitor for sensory stream")
        
        while self._running and not self._sensory:
            try:
                state_manager = FeagiStateManager.instance()
                genome_state = state_manager.get_genome_state()
                burst_engine_state = state_manager.get_burst_engine_state()
                
                genome_loaded = genome_state == GenomeState.LOADED.value
                burst_engine_ready = burst_engine_state == ServiceState.READY.value
                
                if genome_loaded and burst_engine_ready:
                    # FEAGI is now ready - create and start sensory stream
                    logger.info("🚀 FEAGI became ready - creating sensory stream")
                    
                    self._sensory = SensoryStream(
                        core_api=self.core_api,
                        host=self.host,
                        port=self.sensory_port,
                        context=self._context,
                    )
                    
                    await self._sensory.start()
                    
                    logger.info(f"✅ Sensory stream started on port {self.sensory_port} - agents can now connect")
                    break
                else:
                    # Still not ready, wait and check again
                    await asyncio.sleep(2.0)
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in sensory stream monitor: {e}")
                await asyncio.sleep(2.0)

    async def _start_server(self):
        """Start the ZMQ server components.

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
            from .patterns.pub_sub import PubSubManager
            from .patterns.push_pull import PushPullManager
            from .patterns.req_rep import RequestReplyManager
            from .streams.motor import MotorStream
            from .streams.rest import RestStream
            from .streams.sensory_neural import (
                SensoryNeuralStream as SensoryStream,
            )

            # VisualizationStream imported conditionally at module level
            # Initialize only enabled streams based on port configuration
            self._req_rep = RequestReplyManager(
                core_api=self.core_api,
                host=self.host,
                port=self.req_rep_port,
                context=self._context,
            )

            self._pub_sub = PubSubManager(
                core_api=self.core_api,
                host=self.host,
                port=self.pub_sub_port,
                context=self._context,
            )

            self._push_pull = PushPullManager(
                core_api=self.core_api,
                host=self.host,
                port=self.push_pull_port,
                context=self._context,
            )

            # Only create enabled streams (port != None)
            if self.sensory_port is not None:
                # Check if FEAGI is ready before creating sensory stream
                from feagi.core.state_manager import FeagiStateManager, ServiceState, GenomeState
                
                state_manager = FeagiStateManager.instance()
                genome_state = state_manager.get_genome_state()
                burst_engine_state = state_manager.get_burst_engine_state()
                
                genome_loaded = genome_state == GenomeState.LOADED.value
                burst_engine_ready = burst_engine_state == ServiceState.READY.value
                
                if genome_loaded and burst_engine_ready:
                    # FEAGI is ready - create sensory stream normally
                    self._sensory = SensoryStream(
                        core_api=self.core_api,
                        host=self.host,
                        port=self.sensory_port,
                        context=self._context,
                    )
                    logger.info(
                        f"✅ FEAGI ready - sensory stream enabled on port {self.sensory_port}"
                    )
                else:
                    # FEAGI not ready - don't create sensory stream
                    self._sensory = None
                    logger.warning(
                        f"🔒 FEAGI not ready - sensory stream blocked on port {self.sensory_port}"
                    )
                    logger.warning(
                        f"   Genome loaded: {genome_loaded}, Burst engine ready: {burst_engine_ready}"
                    )
                    logger.warning(
                        f"   Agents will not be able to connect until FEAGI is ready"
                    )
                    # Start monitoring task to create stream when ready
                    asyncio.create_task(self._monitor_and_create_sensory_stream())
            else:
                logger.info("Sensory stream disabled")

            if self.motor_port is not None:
                #  Get motor FQ sampler from process manager (created on-demand
                #  when motor agents connect)
                motor_fq_sampler = (
                    self._process_manager.get_motor_fq_sampler()
                    if self._process_manager
                    else None
                )
                self._motor = MotorStream(
                    core_api=self.core_api,
                    host=self.host,
                    port=self.motor_port,
                    context=self._context,
                    fq_sampler=motor_fq_sampler,  # Use existing FQ sampler from Process Manager
                    fire_queue_provider=self._fire_queue_provider,
                    stream_config=self.stream_config.get("motor", {}),
                )
                logger.info(
                    f"Motor stream enabled on port {self.motor_port} using existing FQ sampler"
                )
            else:
                logger.info("Motor stream disabled")

            self._rest = RestStream(
                core_api=self.core_api,
                host=self.host,
                port=self.rest_port,
                context=self._context,
            )

            if self.vis_port is not None:
                # Get visualization-specific stream configuration
                viz_stream_config = (
                    self.stream_config.get("visualization", {})
                    if self.stream_config
                    else {}
                )

                #  Get visualization FQ sampler from process manager (created
                #  on-demand when visualization agents connect)
                self._visualization = VisualizationStream(
                    host=self.host,
                    port=self.vis_port,
                    context=None,  # VisualizationStream creates its own sync context
                    fq_sampler=(
                        self._process_manager.get_viz_fq_sampler()
                        if self._process_manager
                        else None
                    ),  # Use existing FQ sampler
                    core_api=self.core_api,  # Pass core_api for coordinate extraction and genome state
                    connectome_manager=self.core_api.get_connectome_manager(),  # Pass connectome_manager for cortical areas
                    process_manager=self._process_manager,  # Pass process manager for enable/disable control
                    stream_config=viz_stream_config,  # Pass visualization-specific configuration
                )
                logger.info(
                    f"Primary visualization stream enabled on port {self.vis_port} using existing FQ sampler"
                )
            else:
                logger.info("Visualization stream disabled")

            #  Pass ZMQ server reference to REST stream for visualization
            #  endpoints
            #  IMPORTANT: This must happen AFTER visualization stream is
            #  created!
            if hasattr(self._rest, "set_zmq_server"):
                self._rest.set_zmq_server(self)
                logger.debug("ZMQ server reference passed to REST stream")

            # Start only enabled managers
            await self._req_rep.start()
            await self._pub_sub.start()
            await self._push_pull.start()

            if self._sensory:
                await self._sensory.start()
            if self._motor:
                await self._motor.start()

            await self._rest.start()

            # RTOS: VisualizationStream is now synchronous
            if self._visualization:
                self._visualization.start()  # No await - synchronous method

            # Create sockets only for enabled streams
            if self._sensory:
                self.sensory_socket = self._sensory.socket

            if self._motor:
                self.motor_socket = self._motor.socket

            if self._visualization:
                self.vis_socket = self._visualization.socket

            # Start message handling tasks
            # Note: Each stream handles its own message processing
            # No central data loop needed since streams are autonomous

            # Start message handlers
            if self.connection_manager:
                self.message_handlers = await start_message_handlers(
                    self.connection_manager, None, self.message_processors
                )
            else:
                logger.info(
                    "Message handlers disabled - connection manager not available"
                )
                self.message_handlers = {}

            # Start client cleanup task
            self.cleanup_task = asyncio.create_task(
                self._cleanup_inactive_clients()
            )

            logger.info("ZMQ server started successfully")
        except Exception as e:
            logger.error(f"Failed to start ZMQ services: {e}")
            self._running = False
            raise

    async def _monitor_loop(self):
        """Monitor loop to keep the server running and handle shutdown
        requests."""
        try:
            while self._running and not self._shutdown_event.is_set():
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            logger.info("Monitor loop cancelled")
        except Exception as e:
            logger.error(f"Error in monitor loop: {e}")
            self._running = False

    def shutdown(self):
        """Shutdown the ZMQ server with improved handling for multiple
        attempts.

        This method is thread-safe and can be called from any thread.
        """
        # Prevent concurrent shutdowns
        if self._shutdown_in_progress:
            print(
                "Shutdown already in progress, ignoring signal",
                file=sys.stderr,
                flush=True,
            )
            return

        if not self._running:
            print("ZMQ server is not running", file=sys.stderr, flush=True)
            return

        # Set shutdown flag atomically
        self._shutdown_in_progress = True

        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        print("Shutting down FEAGI servers...", file=sys.stderr, flush=True)

        try:
            # Signal the monitor loop to stop
            self._shutdown_event.set()

            #  Create a new event loop for shutdown if we're not in the server
            #  thread
            if threading.current_thread() != self._thread:
                # We're in a different thread, create a new event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    # Run the shutdown in this new loop with timeout
                    loop.run_until_complete(
                        asyncio.wait_for(self._stop_services(), timeout=15.0)
                    )
                except asyncio.TimeoutError:
                    print(
                        "[WARN]  Shutdown timed out after 15 seconds - forcing exit",
                        file=sys.stderr,
                        flush=True,
                    )
                finally:
                    try:
                        loop.close()
                    except Exception:
                        pass  # Ignore loop closing errors
            else:
                # We're in the server thread, use its loop
                if self._loop and self._loop.is_running():
                    asyncio.ensure_future(self._stop_services())

            # Wait for the server thread to finish with timeout
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=8.0)
                if self._thread.is_alive():
                    print(
                        "[WARN]  Server thread didn't stop within timeout",
                        file=sys.stderr,
                        flush=True,
                    )

            # Final cleanup
            self._running = False
            print("ZMQ server shutdown complete", file=sys.stderr, flush=True)

        except Exception as e:
            print(
                f"Error during ZMQ server shutdown: {e}",
                file=sys.stderr,
                flush=True,
            )
        finally:
            # Always reset shutdown flag
            self._shutdown_in_progress = False

    async def _stop_services(self):
        """Stop all ZMQ services and streams."""
        print("Stopping ZMQ services...", file=sys.stderr, flush=True)

        try:
            #  Collect async stop tasks for services that have async stop
            #  methods
            stop_tasks = []

            # Add pattern services (async stop methods)
            if self._req_rep:
                stop_tasks.append(self._req_rep.stop())
            if self._pub_sub:
                stop_tasks.append(self._pub_sub.stop())
            if self._push_pull:
                stop_tasks.append(self._push_pull.stop())

            # Add stream services (async stop methods)
            if self._sensory:
                stop_tasks.append(self._sensory.stop())
            if self._motor:
                stop_tasks.append(self._motor.stop())
            if self._rest:
                stop_tasks.append(self._rest.stop())

            # Handle visualization stream separately with timeout
            if self._visualization:
                import asyncio as asyncio_module

                def stop_visualization_with_timeout():
                    """Stop visualization in a separate thread with timeout."""
                    try:
                        logger.debug(
                            "Stopping visualization stream with timeout..."
                        )

                        # Check if visualization has async stop method
                        if hasattr(
                            self._visualization, "stop"
                        ) and asyncio.iscoroutinefunction(
                            self._visualization.stop
                        ):
                            #  If stop is async, we need to run it in the event
                            #  loop
                            loop = asyncio_module.new_event_loop()
                            try:
                                asyncio_module.set_event_loop(loop)
                                loop.run_until_complete(
                                    self._visualization.stop()
                                )
                            finally:
                                loop.close()
                        else:
                            # If stop is sync, call it directly
                            self._visualization.stop()

                        logger.debug(
                            "Visualization stream stopped successfully"
                        )
                    except Exception as e:
                        logger.error(
                            f"Error stopping visualization stream: {e}"
                        )

                # Run visualization stop in thread pool with timeout
                try:
                    await asyncio_module.wait_for(
                        asyncio_module.get_event_loop().run_in_executor(
                            None, stop_visualization_with_timeout
                        ),
                        timeout=5.0,  # 5 second timeout for visualization shutdown
                    )
                except asyncio_module.TimeoutError:
                    logger.error(
                        "[WARN]  Visualization stream shutdown timed out after 5 seconds - forcing cleanup"
                    )
                    # Force cleanup by setting visualization to None
                    self._visualization = None
                except Exception as e:
                    logger.error(f"Error during visualization shutdown: {e}")
                    self._visualization = None

            # Wait for all other services with timeout
            if stop_tasks:
                try:
                    results = await asyncio.wait_for(
                        asyncio.gather(*stop_tasks, return_exceptions=True),
                        timeout=8.0,  # 8 second timeout for other services
                    )
                    # Check for exceptions in results
                    for i, result in enumerate(results):
                        if isinstance(result, Exception):
                            logger.warning(
                                f"Service {i} stop returned exception: {result}"
                            )
                except asyncio.TimeoutError:
                    logger.error(
                        "[WARN]  Some services didn't stop within timeout - forcing cleanup"
                    )

            print("All ZMQ services stopped", file=sys.stderr, flush=True)

            # Final task cancellation to catch any stragglers
            await self._cancel_all_tasks(wait_time=1.0)

            # Close sockets with error handling
            for socket_name, socket in [
                ("sensory", self.sensory_socket),
                ("motor", self.motor_socket),
                ("visualization", self.vis_socket),
            ]:
                if socket:
                    try:
                        socket.close(linger=0)
                        logger.debug(f"Closed {socket_name} socket")
                    except Exception as e:
                        logger.warning(
                            f"Error closing {socket_name} socket: {e}"
                        )

            self.sensory_socket = None
            self.motor_socket = None
            self.vis_socket = None

        except Exception as e:
            print(
                f"Error stopping ZMQ services: {e}",
                file=sys.stderr,
                flush=True,
            )
            logger.error(f"Critical error in service shutdown: {e}")
            # Force cleanup even if there were errors
            self._visualization = None

    async def _cancel_all_tasks(self, wait_time: float = 2.0):
        """Cancel all running asyncio tasks except the current one.

        Args:
            wait_time: Maximum time to wait for tasks to complete cancellation
        """
        try:
            # Get all tasks in the current event loop
            current_task = asyncio.current_task()
            all_tasks = [
                task
                for task in asyncio.all_tasks()
                if task != current_task and not task.done()
            ]

            if not all_tasks:
                print("No tasks to cancel", file=sys.stderr, flush=True)
                return

            print(
                f"Cancelling {len(all_tasks)} running tasks...",
                file=sys.stderr,
                flush=True,
            )

            # Cancel all tasks
            for task in all_tasks:
                if not task.done():
                    task.cancel()

            # Wait for tasks to complete cancellation
            if all_tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*all_tasks, return_exceptions=True),
                        timeout=wait_time,
                    )
                    print(
                        "All tasks cancelled successfully",
                        file=sys.stderr,
                        flush=True,
                    )
                except asyncio.TimeoutError:
                    remaining_tasks = [
                        task for task in all_tasks if not task.done()
                    ]
                    print(
                        f"[WARN]  {len(remaining_tasks)} tasks didn't cancel within {wait_time}s - forcing shutdown",
                        file=sys.stderr,
                        flush=True,
                    )

                    # Log which tasks are still running for debugging
                    for task in remaining_tasks:
                        task_info = getattr(task, "_coro", "unknown")
                        print(
                            f"  - Stuck task: {task_info}",
                            file=sys.stderr,
                            flush=True,
                        )

        except Exception as e:
            print(f"Error cancelling tasks: {e}", file=sys.stderr, flush=True)

    def _cleanup(self):
        """Clean up resources.

        This is called when the server thread exits.
        """
        try:
            # Ensure services are stopped
            if self._loop and self._running:
                # Run the async shutdown process
                self._loop.run_until_complete(self._stop_services())

            # Close the event loop
            if self._loop:
                try:
                    # Cancel any remaining tasks before closing the loop
                    #  Use qualified asyncio module import to avoid variable
                    #  shadowing
                    import asyncio as asyncio_module

                    pending_tasks = [
                        task
                        for task in asyncio_module.all_tasks(self._loop)
                        if not task.done()
                    ]
                    if pending_tasks:
                        print(
                            f"Force-cancelling {len(pending_tasks)} remaining tasks before loop close",
                            file=sys.stderr,
                            flush=True,
                        )
                        for task in pending_tasks:
                            task.cancel()

                        # Give tasks a brief moment to cancel
                        if not self._loop.is_closed():
                            try:
                                self._loop.run_until_complete(
                                    asyncio_module.gather(
                                        *pending_tasks, return_exceptions=True
                                    )
                                )
                            except Exception as e:
                                print(
                                    f"Error during final task cleanup: {e}",
                                    file=sys.stderr,
                                    flush=True,
                                )

                    self._loop.close()
                    print("Event loop closed", file=sys.stderr, flush=True)
                except Exception as e:
                    print(
                        f"Error closing event loop: {e}",
                        file=sys.stderr,
                        flush=True,
                    )
                finally:
                    self._loop = None

            # Clear service references
            self._req_rep = None
            self._pub_sub = None
            self._push_pull = None
            self._sensory = None
            self._motor = None
            self._visualization = None

            # Reset state
            self._running = False
            self._shutdown_event.clear()

            # Clear state tracking
            self.agents = {}
            self.tasks = []

            # Close connection manager
            if self.connection_manager:
                try:
                    self.connection_manager.close()
                except Exception as e:
                    print(
                        f"Error closing connection manager: {e}",
                        file=sys.stderr,
                        flush=True,
                    )

            #  @cursor:critical-path - Signal-safe cleanup should minimize
            #  logging
            print(
                "ZMQ server resources cleaned up", file=sys.stderr, flush=True
            )
        except Exception as e:
            print(
                f"Error during ZMQ server cleanup: {e}",
                file=sys.stderr,
                flush=True,
            )

    async def publish_event(self, event_type: str, event_data: Dict) -> None:
        """Publish an event to subscribers.

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

    async def queue_work(
        self, work_type: str, data: Any, priority: int = 0
    ) -> None:
        """Queue work for processing.

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

    async def send_motor_data(self, channel_id: int, data: bytes):
        """Send motor data to agents.

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
        """Send neural activity data to visualization clients.

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
        #  In a real implementation, you would parse the data and fill in the
        #  fields
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
        """Send brain structure data to visualization clients.

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
        #  In a real implementation, you would parse the data and fill in the
        #  fields
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

    def register_sensory_callback(
        self, callback: Callable[[int, bytes], None]
    ):
        """Register a callback for processing incoming sensory data.

        Args:
            callback: Function to call when sensory data is received
                     (parameters: channel_id, data)
        """
        self.sensory_callback = callback

    async def _process_handshake_message(
        self, agent_id: str, message: Any
    ) -> Optional[Dict[str, Any]]:
        """Process a handshake message.

        Args:
            agent_id: Agent ID (may be None for hello messages)
            message: Decoded Cap'n Proto handshake message

        Returns:
            Response data if a response is needed, otherwise None
        """
        try:
            # Convert message to dictionary for easier handling
            #  message_dict =
            #  self.translator.handshake_message_to_dict(message) # Unused
            #  variable removed

            # Handle different message types
            if message.type == message.type.hello:
                client_id = message.hello.agentId
                client_type = message.hello.agentType
                zmq_id = None  # We don't have the ZMQ ID yet, handled by HandshakeMessageHandler

                logger.info(
                    f"Processing hello from {client_type} client '{client_id}'"
                )

                # Store client ID -> agent ID mapping for subsequent messages
                self.pending_clients[client_id] = {
                    "agent_id": client_id,
                    "agent_type": client_type,
                    "timestamp": asyncio.get_running_loop().time(),
                }

                # Create welcome message
                welcome_msg = self.translator.create_handshake_welcome(
                    server_id=self.server_id,
                    message=f"Welcome to FEAGI {client_type} '{client_id}'",
                )

                # Convert to dictionary for response
                return {
                    "protocolId": welcome_msg.protocolId,
                    "version": welcome_msg.version,
                    "type": welcome_msg.type,
                    "welcome": {
                        "serverId": welcome_msg.welcome.serverId,
                        "message": welcome_msg.welcome.message,
                        "timestamp": welcome_msg.welcome.timestamp,
                    },
                }

            elif message.type == message.type.capabilities:
                if agent_id not in self.pending_clients:
                    logger.warning(
                        f"Received capabilities from unknown client {agent_id}"
                    )
                    return None

                # Extract capabilities
                sensory_channels = list(
                    message.capabilities.supportedSensoryChannels
                )
                motor_channels = list(
                    message.capabilities.supportedMotorChannels
                )

                # Extract protocol versions
                protocol_versions = {
                    "fcp": message.capabilities.protocolVersions.fcpVersion,
                    "fsmp": message.capabilities.protocolVersions.fsmpVersion,
                    "fvp": message.capabilities.protocolVersions.fvpVersion,
                }

                logger.info(
                    f"Received capabilities from {agent_id}: "
                    f"sensory={sensory_channels}, motor={motor_channels}, "
                    f"protocols={protocol_versions}"
                )

                # Register client in connection manager
                #  Note: The HandshakeMessageHandler should have updated the
                #  ZMQ ID
                client_info = self.pending_clients.get(agent_id)
                if client_info and self.connection_manager:
                    #  Get ZMQ ID from somewhere (needs to be passed from
                    #  handler)
                    zmq_id = client_info.get("zmq_id")
                    if zmq_id:
                        # Register client with connection manager
                        self.connection_manager.register_client(
                            agent_id=agent_id,
                            zmq_id=zmq_id,
                            supported_protocols=protocol_versions,
                        )

                        # Clean up pending client
                        del self.pending_clients[agent_id]

                        # Create configuration message
                        config_msg = self.translator.create_handshake_configuration(
                            {
                                # Add server configuration here
                            }
                        )

                        # Convert to dictionary for response
                        return {
                            "protocolId": config_msg.protocolId,
                            "version": config_msg.version,
                            "type": config_msg.type,
                            "configuration": {
                                "timestamp": config_msg.configuration.timestamp
                                # Add configuration fields
                            },
                        }

            return None

        except Exception as e:
            logger.error(f"Error processing handshake message: {e}")
            return None

    async def _process_fcp_message(
        self, agent_id: str, message: Any
    ) -> Optional[Dict[str, Any]]:
        """Process an FCP message.

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

    async def _process_fsmp_message(
        self, agent_id: str, message: Any
    ) -> Optional[Dict[str, Any]]:
        """Process an FSMP message.

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

    async def _process_fvp_message(
        self, agent_id: str, message: Any
    ) -> Optional[Dict[str, Any]]:
        """Process an FVP message.

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
        """Get ZMQ server statistics.

        Returns:
            Dictionary containing server statistics
        """
        total_agents = len(self.agents)
        active_agents = len(
            [a for a in self.agents.values() if a.get("last_heartbeat")]
        )

        return {
            "running": self._running,
            "total_agents": total_agents,
            "active_agents": active_agents,
            "host": self.host,
            "req_rep_port": self.req_rep_port,
            "pub_sub_port": self.pub_sub_port,
            "push_pull_port": self.push_pull_port,
            "sensory_port": self.sensory_port,
            "motor_port": self.motor_port,
            "rest_port": self.rest_port,
            "vis_port": self.vis_port,
            "enabled_streams": {
                "sensory": self._sensory is not None,
                "motor": self._motor is not None,
                "visualization": self._visualization is not None,
            },
        }

    def get_visualization_stream(self):
        """Get the visualization stream instance.

        Returns:
            VisualizationStream instance if visualization is enabled, None otherwise
        """
        return self._visualization

    async def broadcast_message(
        self,
        protocol_type: str,
        message_data: Dict[str, Any],
        filter_func: Optional[Callable[[str, Dict[str, Any]], bool]] = None,
    ) -> int:
        """Broadcast a message to all connected clients or a filtered subset.

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

        # Return 0 if connection manager is disabled
        if not self.connection_manager:
            logger.debug(
                "Connection manager disabled - cannot broadcast messages"
            )
            return 0

        # Get all connected clients
        count = 0
        for (
            agent_id,
            client_info,
        ) in self.connection_manager.connections.items():
            # Apply filter if provided
            if filter_func and not filter_func(agent_id, client_info):
                continue

            # Create and encode message
            if protocol_type == "fcp":
                message = self.translator.fcp_schema.FCPMessage.new_message(
                    **message_data
                )
            elif protocol_type == "fsmp":
                message = self.translator.fsmp_schema.FSMPMessage.new_message(
                    **message_data
                )
            elif protocol_type == "fvp":
                message = self.translator.fvp_schema.FVPMessage.new_message(
                    **message_data
                )

            # Send message
            encoded_message = message.to_bytes()
            success = await self.connection_manager.send_message(
                agent_id=agent_id,
                protocol_type=protocol_type,
                message=encoded_message,
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

                # Skip cleanup if connection manager is disabled
                if not self.connection_manager:
                    continue

                # Find inactive clients
                inactive_clients = (
                    self.connection_manager.get_inactive_clients(
                        timeout_seconds=60
                    )
                )

                # Deregister inactive clients
                for agent_id in inactive_clients:
                    logger.info(f"Deregistering inactive client {agent_id}")
                    self.connection_manager.deregister_client(agent_id)

                # Clean up pending clients
                now = asyncio.get_running_loop().time()
                for client_id in list(self.pending_clients.keys()):
                    client_info = self.pending_clients[client_id]
                    if (
                        now - client_info["timestamp"] > 30
                    ):  # 30 seconds timeout
                        logger.info(
                            f"Removing pending client {client_id} due to timeout"
                        )
                        del self.pending_clients[client_id]

        except asyncio.CancelledError:
            logger.info("Cleanup task cancelled")

        except Exception as e:
            logger.error(f"Error in cleanup task: {e}")

    async def _process_control_message(
        self, identity: bytes, message: Dict[str, Any]
    ) -> bytes:
        """Process a control message and return the appropriate response.

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
            response = {
                "status": "error",
                "message": f"Unknown message type: {message_type}",
            }
            return json.dumps(response).encode("utf-8")

    async def _receive_with_timeout(self, socket, timeout):
        """Wait for a message on a socket with timeout.

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
            events = await asyncio.wait_for(
                poller.poll(timeout=timeout * 1000), timeout=timeout + 0.1
            )  # milliseconds

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
