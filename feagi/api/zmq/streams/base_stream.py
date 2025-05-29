"""
Base ZMQ Stream Implementation for FEAGI

This module provides specialized base classes for different ZMQ communication patterns:
- UnidirectionalStream: For motor, sensory, visualization (one-way data flow)
- BidirectionalStream: For control (request/response patterns)
- BaseZMQStream: Common functionality for all streams

The architecture properly accounts for the fundamental differences in communication patterns.
"""

import abc
import time
import threading
import asyncio
from typing import Dict, Any, Optional, Union, List, Callable
from enum import Enum

import zmq
import zmq.asyncio

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)


class StreamMode(Enum):
    """Stream execution modes."""
    SYNC = "synchronous"
    ASYNC = "asynchronous"


class SocketType(Enum):
    """Supported socket types with their ZMQ constants."""
    PUB = zmq.PUB
    SUB = zmq.SUB
    PUSH = zmq.PUSH
    PULL = zmq.PULL
    ROUTER = zmq.ROUTER
    DEALER = zmq.DEALER
    REQ = zmq.REQ
    REP = zmq.REP


class DataDirection(Enum):
    """Data flow directions for unidirectional streams."""
    INBOUND = "inbound"   # Data flows TO FEAGI (e.g., sensory)
    OUTBOUND = "outbound" # Data flows FROM FEAGI (e.g., motor, visualization)


class BaseZMQStream(abc.ABC):
    """
    Base class for all FEAGI ZMQ streams providing common functionality.
    
    This class handles shared functionality like:
    - Basic state management
    - Health monitoring
    - Configuration management
    - Genome state tracking
    """
    
    def __init__(
        self,
        host: str = "*",
        port: int = 5559,
        mode: StreamMode = StreamMode.ASYNC,
        core_api: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """Initialize base ZMQ stream."""
        # Basic configuration
        self.host = host
        self.port = port
        self.mode = mode
        self.core_api = core_api
        self.stream_config = stream_config or {}
        
        # State management
        self.running = False
        self._active_mode = False
        
        # Health monitoring
        self.stats = {
            'start_time': 0,
            'data_processed': 0,
            'bytes_processed': 0,
            'errors': 0,
            'last_activity': 0
        }
        
        # Register for genome state changes if available
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        self._update_active_mode()

    def _update_active_mode(self) -> None:
        """Update active mode based on genome availability."""
        old_mode = self._active_mode
        
        try:
            if self.core_api and hasattr(self.core_api, 'genome_is_loaded'):
                self._active_mode = self.core_api.genome_is_loaded()
            else:
                self._active_mode = True  # Default to active if no core API
        except Exception as e:
            logger.warning(f"Error checking genome state: {e}")
            self._active_mode = False
        
        if old_mode != self._active_mode:
            mode_str = "ACTIVE" if self._active_mode else "STANDBY"
            logger.info(f"{self.__class__.__name__} entering {mode_str} mode")

    def _on_genome_state_change(self, old_state: Any, new_state: Any) -> None:
        """Handle genome state changes."""
        try:
            from feagi.core.state_manager import GenomeState
            self._active_mode = (new_state == GenomeState.LOADED)
            
            if self.running:
                mode_str = "ACTIVE" if self._active_mode else "STANDBY"
                logger.info(f"{self.__class__.__name__} entering {mode_str} mode")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            self._active_mode = False

    @abc.abstractmethod
    async def start(self) -> None:
        """Start the stream."""
        pass

    @abc.abstractmethod
    async def stop(self) -> None:
        """Stop the stream."""
        pass

    def get_health_status(self) -> Dict[str, Any]:
        """Get basic health status."""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        
        return {
            'stream_type': self.__class__.__name__,
            'mode': self.mode.value,
            'running': self.running,
            'active_mode': self._active_mode,
            'runtime_seconds': runtime,
            'stats': {
                **self.stats,
                'data_per_second': self.stats['data_processed'] / max(runtime, 1),
                'bytes_per_second': self.stats['bytes_processed'] / max(runtime, 1)
            }
        }


class UnidirectionalStream(BaseZMQStream):
    """
    Base class for unidirectional ZMQ streams (Motor, Sensory, Visualization).
    
    Unidirectional streams have:
    - Single socket for data flow
    - Simple publish/subscribe or push/pull patterns
    - No request/response semantics
    - Race condition protection for single socket
    """
    
    def __init__(
        self,
        host: str = "*",
        port: int = 5559,
        direction: DataDirection = DataDirection.OUTBOUND,
        socket_type: SocketType = SocketType.PUB,
        mode: StreamMode = StreamMode.ASYNC,
        core_api: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """Initialize unidirectional stream."""
        super().__init__(host, port, mode, core_api, stream_config)
        
        self.direction = direction
        self.socket_type = socket_type
        
        # Single socket management with race condition protection
        self.socket = None
        self._socket_lock = threading.Lock()
        self.context = None
        
        # Worker management
        self.worker_threads: List[threading.Thread] = []
        self.async_tasks: List[asyncio.Task] = []
        self._stop_event = threading.Event() if mode == StreamMode.SYNC else None
        
        # Initialize
        self._initialize_context()
        self._create_socket()

    def _initialize_context(self) -> None:
        """Initialize ZMQ context based on execution mode."""
        if self.mode == StreamMode.SYNC:
            self.context = zmq.Context()
        else:
            self.context = zmq.asyncio.Context.instance()
        
        logger.debug(f"Initialized {self.mode.value} ZMQ context for {self.direction.value} stream")

    def _create_socket(self) -> None:
        """Create the single socket for unidirectional communication."""
        try:
            socket = self.context.socket(self.socket_type.value)
            
            # Apply socket options based on direction and type
            self._configure_socket(socket)
            
            # Bind or connect based on direction
            address = f"tcp://{self.host}:{self.port}"
            if self.direction == DataDirection.INBOUND:
                # FEAGI receives data - bind to accept connections
                socket.bind(address)
                logger.info(f"{self.socket_type.name} socket bound to {address}")
            else:
                # FEAGI sends data - bind for subscribers to connect
                socket.bind(address)
                logger.info(f"{self.socket_type.name} socket bound to {address}")
            
            # Store socket safely
            with self._socket_lock:
                self.socket = socket
                
        except Exception as e:
            logger.error(f"Failed to create {self.direction.value} socket: {e}")
            raise

    def _configure_socket(self, socket) -> None:
        """Configure socket options based on type and direction."""
        # Common options
        socket.setsockopt(zmq.LINGER, 1000)
        
        # Direction-specific configuration
        if self.direction == DataDirection.OUTBOUND:
            # For outbound streams (motor, visualization)
            if self.socket_type == SocketType.PUB:
                socket.setsockopt(zmq.SNDHWM, 1000)  # Send buffer
        else:
            # For inbound streams (sensory)
            if self.socket_type == SocketType.PULL:
                socket.setsockopt(zmq.RCVHWM, 100)   # Receive buffer

    def _safe_socket_operation(
        self, 
        operation: Callable, 
        *args, 
        **kwargs
    ) -> Any:
        """
        Perform socket operation with comprehensive error handling.
        
        This prevents the race condition that caused the original error.
        """
        # Check if stream is running
        if not self.running:
            logger.debug(f"Skipping {operation.__name__}: stream not running")
            return None
            
        # Get socket with thread safety
        with self._socket_lock:
            socket = self.socket
            
        if not socket:
            logger.debug(f"Cannot perform {operation.__name__}: socket is None")
            return None
            
        try:
            # Atomic socket reference to prevent mid-operation changes
            socket_ref = socket
            if not socket_ref:
                logger.debug(f"Socket became None during operation")
                return None
                
            # Perform the operation
            result = operation(socket_ref, *args, **kwargs)
            
            # Update stats
            self.stats['last_activity'] = time.time()
            
            return result
            
        except AttributeError as e:
            if "'NoneType' object has no attribute" in str(e):
                logger.debug(f"Socket became None during {operation.__name__} (race condition prevented)")
                return None
            else:
                logger.error(f"Unexpected AttributeError in {operation.__name__}: {e}")
                self.stats['errors'] += 1
                
        except zmq.ZMQError as e:
            if e.errno == zmq.ETERM:
                logger.debug(f"ZMQ context terminated - stopping {operation.__name__}")
                return None
            elif e.errno == zmq.EAGAIN:
                logger.warning(f"Socket not ready for {operation.__name__} (EAGAIN)")
                return None
            else:
                logger.error(f"ZMQ error in {operation.__name__}: {e} (errno: {e.errno})")
                self.stats['errors'] += 1
                
        except Exception as e:
            logger.error(f"Error in {operation.__name__}: {e}")
            self.stats['errors'] += 1
            
        return None

    async def start(self) -> None:
        """Start unidirectional stream."""
        if self.running:
            return
            
        logger.info(f"Starting {self.__class__.__name__} ({self.direction.value})")
        self.running = True
        self.stats['start_time'] = time.time()
        
        if self.mode == StreamMode.SYNC:
            if self._stop_event:
                self._stop_event.clear()
            self._start_workers()
        else:
            await self._start_async_tasks()
        
        logger.info(f"{self.__class__.__name__} started")

    async def stop(self) -> None:
        """Stop unidirectional stream with race condition prevention."""
        if not self.running:
            return
            
        logger.info(f"Stopping {self.__class__.__name__}")
        self.running = False
        
        if self.mode == StreamMode.SYNC:
            if self._stop_event:
                self._stop_event.set()
            self._stop_workers()
        else:
            await self._stop_async_tasks()
        
        # Clean up resources
        self._cleanup_resources()
        
        # Close socket LAST (after workers stopped)
        self._close_socket()
        
        logger.info(f"{self.__class__.__name__} stopped")

    def _start_workers(self) -> None:
        """Start sync worker threads."""
        # Subclasses override this
        pass

    async def _start_async_tasks(self) -> None:
        """Start async tasks."""
        # Subclasses override this
        pass

    def _stop_workers(self) -> None:
        """Stop worker threads with timeout."""
        if not self.worker_threads:
            return
            
        logger.debug(f"Stopping {len(self.worker_threads)} worker threads...")
        
        MAX_WAIT = 3.0
        per_thread_timeout = min(1.0, MAX_WAIT / max(len(self.worker_threads), 1))
        
        for i, thread in enumerate(self.worker_threads, 1):
            if thread.is_alive():
                logger.debug(f"Waiting for thread {i}/{len(self.worker_threads)}: {thread.name}")
                thread.join(timeout=per_thread_timeout)
                
                if thread.is_alive():
                    logger.warning(f"Thread {thread.name} didn't stop after {per_thread_timeout:.1f}s")
        
        self.worker_threads.clear()

    async def _stop_async_tasks(self) -> None:
        """Stop async tasks."""
        if not self.async_tasks:
            return
            
        logger.debug(f"Stopping {len(self.async_tasks)} async tasks...")
        
        for task in self.async_tasks:
            task.cancel()
        
        if self.async_tasks:
            await asyncio.gather(*self.async_tasks, return_exceptions=True)
        
        self.async_tasks.clear()

    def _cleanup_resources(self) -> None:
        """Clean up stream-specific resources."""
        # Subclasses override this
        pass

    def _close_socket(self) -> None:
        """Close socket safely."""
        with self._socket_lock:
            if self.socket:
                try:
                    self.socket.close(linger=0)
                    logger.debug("Socket closed")
                except Exception as e:
                    logger.warning(f"Error closing socket: {e}")
                finally:
                    self.socket = None
        
        # Close context
        if self.context:
            try:
                self.context.term()
                logger.debug("ZMQ context terminated")
            except Exception as e:
                logger.warning(f"Error terminating context: {e}")

    # Data operations for subclasses
    def publish_data(self, data: bytes, topic: bytes = b"") -> bool:
        """Publish data for outbound streams (PUB sockets)."""
        if self.direction != DataDirection.OUTBOUND:
            logger.error("publish_data() only available for outbound streams")
            return False
            
        if self.socket_type != SocketType.PUB:
            logger.error("publish_data() only available for PUB sockets")
            return False
            
        result = self._safe_socket_operation(
            lambda socket: socket.send_multipart([topic, data])
        )
        
        if result is not None:
            self.stats['data_processed'] += 1
            self.stats['bytes_processed'] += len(data)
            return True
        
        return False

    async def receive_data(self, timeout: float = 0.5) -> Optional[bytes]:
        """Receive data for inbound streams (PULL sockets)."""
        if self.direction != DataDirection.INBOUND:
            logger.error("receive_data() only available for inbound streams")
            return None
            
        if self.socket_type != SocketType.PULL:
            logger.error("receive_data() only available for PULL sockets")
            return None
            
        try:
            with self._socket_lock:
                socket = self.socket
                
            if not socket:
                return None
                
            frames = await asyncio.wait_for(socket.recv_multipart(), timeout=timeout)
            
            if frames:
                data = frames[-1]  # Last frame is the data
                self.stats['data_processed'] += 1
                self.stats['bytes_processed'] += len(data)
                return data
                
        except asyncio.TimeoutError:
            pass
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            self.stats['errors'] += 1
            
        return None


class BidirectionalStream(BaseZMQStream):
    """
    Base class for bidirectional ZMQ streams (Control, REST).
    
    Bidirectional streams have:
    - Multiple sockets (ROUTER/DEALER pattern typically)
    - Request/response semantics
    - Client session management
    - More complex state tracking
    """
    
    def __init__(
        self,
        host: str = "*",
        port: int = 5559,
        mode: StreamMode = StreamMode.ASYNC,
        core_api: Optional[Any] = None,
        stream_config: Optional[Dict[str, Any]] = None
    ):
        """Initialize bidirectional stream."""
        super().__init__(host, port, mode, core_api, stream_config)
        
        # Multiple socket management
        self.sockets: Dict[str, Union[zmq.Socket, zmq.asyncio.Socket]] = {}
        self._socket_lock = threading.Lock()
        self.context = None
        
        # Client session management
        self.clients: Dict[str, Dict[str, Any]] = {}
        self._client_lock = threading.Lock()
        
        # Worker management
        self.async_tasks: List[asyncio.Task] = []
        
        # Message handlers
        self.message_handlers: Dict[str, Callable] = {}
        
        # Initialize
        self._initialize_context()

    def _initialize_context(self) -> None:
        """Initialize ZMQ context."""
        if self.mode == StreamMode.SYNC:
            self.context = zmq.Context()
        else:
            self.context = zmq.asyncio.Context.instance()
        
        logger.debug(f"Initialized {self.mode.value} ZMQ context for bidirectional stream")

    @abc.abstractmethod
    def _create_sockets(self) -> None:
        """Create sockets for bidirectional communication."""
        pass

    @abc.abstractmethod
    async def _handle_request(self, client_id: bytes, message: Dict[str, Any]) -> Dict[str, Any]:
        """Handle a request from a client."""
        pass

    def register_message_handler(self, message_type: str, handler: Callable) -> None:
        """Register a handler for a specific message type."""
        self.message_handlers[message_type] = handler
        logger.debug(f"Registered handler for message type: {message_type}")

    def get_connected_clients(self) -> List[Dict[str, Any]]:
        """Get information about connected clients."""
        with self._client_lock:
            return list(self.clients.values())

    async def start(self) -> None:
        """Start bidirectional stream."""
        if self.running:
            return
            
        logger.info(f"Starting {self.__class__.__name__}")
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Create sockets
        self._create_sockets()
        
        # Start async tasks
        await self._start_async_tasks()
        
        logger.info(f"{self.__class__.__name__} started")

    async def stop(self) -> None:
        """Stop bidirectional stream."""
        if not self.running:
            return
            
        logger.info(f"Stopping {self.__class__.__name__}")
        self.running = False
        
        # Stop async tasks
        await self._stop_async_tasks()
        
        # Close sockets
        self._close_sockets()
        
        logger.info(f"{self.__class__.__name__} stopped")

    async def _start_async_tasks(self) -> None:
        """Start async tasks for bidirectional communication."""
        # Subclasses override this
        pass

    async def _stop_async_tasks(self) -> None:
        """Stop async tasks."""
        if not self.async_tasks:
            return
            
        logger.debug(f"Stopping {len(self.async_tasks)} async tasks...")
        
        for task in self.async_tasks:
            task.cancel()
        
        if self.async_tasks:
            await asyncio.gather(*self.async_tasks, return_exceptions=True)
        
        self.async_tasks.clear()

    def _close_sockets(self) -> None:
        """Close all sockets safely."""
        with self._socket_lock:
            for name, socket in self.sockets.items():
                if socket:
                    try:
                        socket.close(linger=0)
                        logger.debug(f"Socket '{name}' closed")
                    except Exception as e:
                        logger.warning(f"Error closing socket '{name}': {e}")
            
            self.sockets.clear()
        
        # Close context
        if self.context:
            try:
                self.context.term()
                logger.debug("ZMQ context terminated")
            except Exception as e:
                logger.warning(f"Error terminating context: {e}")

    def get_health_status(self) -> Dict[str, Any]:
        """Get comprehensive health status for bidirectional streams."""
        status = super().get_health_status()
        
        # Add bidirectional-specific info
        status.update({
            'socket_count': len(self.sockets),
            'client_count': len(self.clients),
            'async_task_count': len(self.async_tasks)
        })
        
        # Socket details
        socket_status = {}
        with self._socket_lock:
            for name, socket in self.sockets.items():
                if socket:
                    try:
                        socket_status[name] = {
                            'type': socket.socket_type,
                            'closed': socket.closed if hasattr(socket, 'closed') else False
                        }
                    except Exception as e:
                        socket_status[name] = {'error': str(e)}
                else:
                    socket_status[name] = {'status': 'None'}
        
        status['sockets'] = socket_status
        return status 