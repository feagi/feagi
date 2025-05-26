#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
ZMQ Manager for FEAGI

This module provides the core ZMQ management functionality for FEAGI,
handling socket creation, management, and process-priority-aware configuration.
"""

import os
import zmq
import zmq.asyncio
import asyncio
import threading
from typing import Dict, List, Optional, Set, Tuple, Any, Union, Callable, Callable
from enum import IntEnum

from feagi.utils.logger import setup_logger
from feagi.api.protocols.base import ProtocolID

logger = setup_logger(__name__)


class ProcessPriority(IntEnum):
    """Process priority levels for FEAGI components."""
    CRITICAL = 1     # Priority 1: Core neural simulation processes (real-time)
    IMPORTANT = 2    # Priority 2: Near real-time processes
    BACKGROUND = 3   # Priority 3: Non-critical background processes


class TopicType(str, IntEnum):
    """Topic categories for ZMQ communication."""
    CONTROL = "control"           # Management and control messages
    HEALTHCHECK = "healthcheck"   # Status and health monitoring
    BURST = "burst"               # Neural activity data (high priority)
    FCL = "fcl"                   # Fire candidate list data (high priority)
    MOTOR = "motor"               # Motor output data
    SENSORY = "sensory"           # Sensory input data 
    VISUALIZATION = "viz"         # Visualization data (lowest priority)


class ZMQPortManager:
    """
    Manages port allocation for ZMQ connections.
    
    Ensures there are no port conflicts by tracking allocated ports
    and providing clean allocation/deallocation.
    """
    
    def __init__(self, min_port: int = 40001, max_port: int = 40050):
        """
        Initialize the port manager.
        
        Args:
            min_port: Minimum port number in range
            max_port: Maximum port number in range
        """
        self.min_port = min_port
        self.max_port = max_port
        self.used_ports: Set[int] = set()
        self._lock = threading.RLock()
        
    def get_available_port(self) -> int:
        """
        Get an available port in the range.
        
        Returns:
            Available port number
            
        Raises:
            RuntimeError: If no ports are available
        """
        with self._lock:
            for port in range(self.min_port, self.max_port + 1):
                if port not in self.used_ports:
                    self.used_ports.add(port)
                    return port
        raise RuntimeError(f"No available ports in range {self.min_port}-{self.max_port}")
        
    def release_port(self, port: int) -> None:
        """
        Release a used port.
        
        Args:
            port: Port number to release
        """
        with self._lock:
            if port in self.used_ports:
                self.used_ports.remove(port)


class ZMQSocketConfig:
    """Configuration for ZMQ sockets based on process priority."""
    
    # Socket options by priority
    PRIORITY_OPTIONS = {
        ProcessPriority.CRITICAL: {
            # For critical processes, optimize for low latency
            zmq.LINGER: 0,           # Don't linger when closed
            zmq.RCVHWM: 10000,       # High receive high water mark
            zmq.SNDHWM: 10000,       # High send high water mark
            zmq.IMMEDIATE: 1,        # Don't queue messages if no connection
            zmq.RCVTIMEO: 100,       # Low receive timeout (100ms)
            zmq.SNDTIMEO: 100,       # Low send timeout (100ms)
            zmq.TCP_KEEPALIVE: 1,    # Enable TCP keepalive
            zmq.TCP_KEEPALIVE_IDLE: 30,  # 30s idle before keepalive
            zmq.TCP_KEEPALIVE_INTVL: 5,  # 5s between keepalive probes
        },
        ProcessPriority.IMPORTANT: {
            # For important processes, balanced settings
            zmq.LINGER: 1000,        # 1s linger when closed
            zmq.RCVHWM: 5000,        # Medium receive high water mark
            zmq.SNDHWM: 5000,        # Medium send high water mark
            zmq.IMMEDIATE: 0,        # Queue messages if no connection
            zmq.RCVTIMEO: 500,       # Medium receive timeout (500ms)
            zmq.SNDTIMEO: 500,       # Medium send timeout (500ms)
            zmq.TCP_KEEPALIVE: 1,    # Enable TCP keepalive
        },
        ProcessPriority.BACKGROUND: {
            # For background processes, optimize for efficiency
            zmq.LINGER: 5000,        # 5s linger when closed
            zmq.RCVHWM: 1000,        # Low receive high water mark
            zmq.SNDHWM: 1000,        # Low send high water mark
            zmq.IMMEDIATE: 0,        # Queue messages if no connection
            zmq.RCVTIMEO: 1000,      # Higher receive timeout (1s)
            zmq.SNDTIMEO: 1000,      # Higher send timeout (1s)
        }
    }
    
    # Default topic to priority mapping
    TOPIC_PRIORITY = {
        TopicType.CONTROL: ProcessPriority.IMPORTANT,
        TopicType.HEALTHCHECK: ProcessPriority.IMPORTANT,
        TopicType.BURST: ProcessPriority.CRITICAL,
        TopicType.FCL: ProcessPriority.CRITICAL,
        TopicType.MOTOR: ProcessPriority.IMPORTANT,
        TopicType.SENSORY: ProcessPriority.IMPORTANT,
        TopicType.VISUALIZATION: ProcessPriority.BACKGROUND,
    }
    
    # Protocol to topic type mapping
    PROTOCOL_TOPIC = {
        ProtocolID.FCP: [TopicType.CONTROL, TopicType.HEALTHCHECK],
        ProtocolID.FSMP: [TopicType.MOTOR, TopicType.SENSORY],
        ProtocolID.FVP: [TopicType.VISUALIZATION],
    }
    
    @classmethod
    def get_socket_options(cls, priority: ProcessPriority) -> Dict[int, Any]:
        """
        Get ZMQ socket options for a given priority.
        
        Args:
            priority: Process priority level
            
        Returns:
            Dictionary of ZMQ socket options
        """
        return cls.PRIORITY_OPTIONS.get(priority, cls.PRIORITY_OPTIONS[ProcessPriority.IMPORTANT])
    
    @classmethod
    def get_topic_priority(cls, topic: Union[str, TopicType]) -> ProcessPriority:
        """
        Get priority level for a topic.
        
        Args:
            topic: Topic name or TopicType
            
        Returns:
            ProcessPriority level
        """
        if isinstance(topic, str):
            # Try to convert string to TopicType
            try:
                topic = TopicType(topic.lower())
            except (ValueError, AttributeError):
                # If conversion fails, use default priority
                return ProcessPriority.IMPORTANT
                
        return cls.TOPIC_PRIORITY.get(topic, ProcessPriority.IMPORTANT)


class ZMQManager:
    """
    ZeroMQ manager for FEAGI.
    
    Handles socket creation, connection management, and process-priority-aware 
    socket configuration.
    """
    
    def __init__(self, 
                 context: Optional[zmq.Context] = None,
                 port_range: Tuple[int, int] = (40001, 40050)):
        """
        Initialize the ZMQ manager.
        
        Args:
            context: Optional existing ZMQ context to use
            port_range: Tuple of (min_port, max_port) for port allocation
        """
        self.context = context or zmq.Context.instance()
        self.async_context = zmq.asyncio.Context.instance()
        self.port_manager = ZMQPortManager(port_range[0], port_range[1])
        self.sockets: Dict[str, Dict[str, Any]] = {}  # {id: {socket, type, priority, address}}
        self.connections: Dict[str, Dict[str, Any]] = {}  # {agent_id: {address, topics, sockets}}
        self._lock = threading.RLock()
    
    def create_socket(self, 
                      socket_type: int, 
                      socket_id: str,
                      priority: ProcessPriority = ProcessPriority.IMPORTANT,
                      bind: bool = True,
                      address: Optional[str] = None) -> zmq.Socket:
        """
        Create a ZMQ socket with priority-specific configuration.
        
        Args:
            socket_type: ZMQ socket type (zmq.PUB, zmq.SUB, etc.)
            socket_id: Unique identifier for the socket
            priority: Process priority level
            bind: Whether to bind or connect the socket
            address: Optional address to bind/connect to
            
        Returns:
            Configured ZMQ socket
            
        Raises:
            ValueError: If socket_id already exists
        """
        with self._lock:
            if socket_id in self.sockets:
                raise ValueError(f"Socket with ID '{socket_id}' already exists")
                
            # Create socket
            socket = self.context.socket(socket_type)
            
            # Apply priority-specific options
            options = ZMQSocketConfig.get_socket_options(priority)
            for option, value in options.items():
                try:
                    socket.setsockopt(option, value)
                except zmq.ZMQError as e:
                    logger.warning(f"Could not set option {option} to {value}: {e}")
            
            # Generate address if not provided
            if not address:
                port = self.port_manager.get_available_port()
                interface = os.environ.get('FEAGI_ZMQ_INTERFACE', '*')
                address = f"tcp://{interface}:{port}"
            
            # Bind or connect
            if bind:
                socket.bind(address)
                logger.info(f"Bound socket {socket_id} to {address}")
            else:
                socket.connect(address)
                logger.info(f"Connected socket {socket_id} to {address}")
            
            # Store socket information
            self.sockets[socket_id] = {
                'socket': socket,
                'type': socket_type,
                'priority': priority,
                'address': address,
                'bind': bind
            }
            
            return socket

    def create_async_socket(self, 
                           socket_type: int, 
                           socket_id: str,
                           priority: ProcessPriority = ProcessPriority.IMPORTANT,
                           bind: bool = True,
                           address: Optional[str] = None) -> zmq.asyncio.Socket:
        """
        Create an async ZMQ socket with priority-specific configuration.
        
        Args:
            socket_type: ZMQ socket type (zmq.PUB, zmq.SUB, etc.)
            socket_id: Unique identifier for the socket
            priority: Process priority level
            bind: Whether to bind or connect the socket
            address: Optional address to bind/connect to
            
        Returns:
            Configured async ZMQ socket
        """
        with self._lock:
            if socket_id in self.sockets:
                raise ValueError(f"Socket with ID '{socket_id}' already exists")
                
            # Create socket
            socket = self.async_context.socket(socket_type)
            
            # Apply priority-specific options
            options = ZMQSocketConfig.get_socket_options(priority)
            for option, value in options.items():
                try:
                    socket.setsockopt(option, value)
                except zmq.ZMQError as e:
                    logger.warning(f"Could not set option {option} to {value}: {e}")
            
            # Generate address if not provided
            if not address:
                port = self.port_manager.get_available_port()
                interface = os.environ.get('FEAGI_ZMQ_INTERFACE', '*')
                address = f"tcp://{interface}:{port}"
            
            # Bind or connect
            if bind:
                socket.bind(address)
                logger.info(f"Bound async socket {socket_id} to {address}")
            else:
                socket.connect(address)
                logger.info(f"Connected async socket {socket_id} to {address}")
            
            # Store socket information
            self.sockets[socket_id] = {
                'socket': socket,
                'type': socket_type,
                'priority': priority,
                'address': address,
                'bind': bind,
                'async': True
            }
            
            return socket
            
    def create_pub_socket(self, 
                         socket_id: str, 
                         topic: Union[str, TopicType],
                         address: Optional[str] = None,
                         bind: bool = True) -> zmq.Socket:
        """
        Create a PUB socket for a specific topic.
        
        Args:
            socket_id: Unique identifier for the socket
            topic: Topic this socket will publish on
            address: Optional address to bind/connect to
            bind: Whether to bind or connect the socket
            
        Returns:
            Configured ZMQ PUB socket
        """
        priority = ZMQSocketConfig.get_topic_priority(topic)
        return self.create_socket(zmq.PUB, socket_id, priority, bind, address)
        
    def create_sub_socket(self, 
                         socket_id: str, 
                         topics: List[Union[str, TopicType]],
                         address: Optional[str] = None,
                         bind: bool = False) -> zmq.Socket:
        """
        Create a SUB socket for specific topics.
        
        Args:
            socket_id: Unique identifier for the socket
            topics: List of topics to subscribe to
            address: Optional address to bind/connect to
            bind: Whether to bind or connect the socket
            
        Returns:
            Configured ZMQ SUB socket
        """
        # Use highest priority from topics
        priority = max(ZMQSocketConfig.get_topic_priority(topic) for topic in topics)
        
        socket = self.create_socket(zmq.SUB, socket_id, priority, bind, address)
        
        # Subscribe to topics
        for topic in topics:
            if isinstance(topic, TopicType):
                topic_str = topic.value
            else:
                topic_str = str(topic)
                
            socket.setsockopt(zmq.SUBSCRIBE, topic_str.encode('utf-8'))
            logger.debug(f"Socket {socket_id} subscribed to topic: {topic_str}")
            
        return socket
        
    def create_req_socket(self, 
                         socket_id: str, 
                         priority: ProcessPriority = ProcessPriority.IMPORTANT,
                         address: Optional[str] = None,
                         bind: bool = False) -> zmq.Socket:
        """
        Create a REQ socket.
        
        Args:
            socket_id: Unique identifier for the socket
            priority: Process priority level
            address: Optional address to bind/connect to
            bind: Whether to bind or connect the socket
            
        Returns:
            Configured ZMQ REQ socket
        """
        return self.create_socket(zmq.REQ, socket_id, priority, bind, address)
        
    def create_rep_socket(self, 
                         socket_id: str, 
                         priority: ProcessPriority = ProcessPriority.IMPORTANT,
                         address: Optional[str] = None,
                         bind: bool = True) -> zmq.Socket:
        """
        Create a REP socket.
        
        Args:
            socket_id: Unique identifier for the socket
            priority: Process priority level
            address: Optional address to bind/connect to
            bind: Whether to bind or connect the socket
            
        Returns:
            Configured ZMQ REP socket
        """
        return self.create_socket(zmq.REP, socket_id, priority, bind, address)

    def register_agent_connection(self, 
                                 agent_id: str, 
                                 protocols: Dict[ProtocolID, int],
                                 address_base: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
        """
        Register an agent connection and create necessary sockets.
        
        Args:
            agent_id: Agent identifier
            protocols: Dictionary of protocol IDs to version numbers 
            address_base: Optional base address for connections
            
        Returns:
            Dictionary of connection information
        """
        with self._lock:
            if agent_id in self.connections:
                raise ValueError(f"Agent '{agent_id}' is already registered")
                
            # Generate base address if not provided
            if not address_base:
                interface = os.environ.get('FEAGI_ZMQ_INTERFACE', '*')
                port_base = self.port_manager.get_available_port()
                address_base = f"tcp://{interface}:{port_base}"
            
            # Create connection record
            connection = {
                'agent_id': agent_id,
                'protocols': protocols,
                'address_base': address_base,
                'sockets': {},
                'ports': {}
            }
            
            # Create sockets for each protocol
            for protocol_id, version in protocols.items():
                # Get topics for this protocol
                topics = ZMQSocketConfig.PROTOCOL_TOPIC.get(protocol_id, [])
                
                for topic in topics:
                    # Create unique socket IDs
                    pub_id = f"{agent_id}_{topic}_pub"
                    sub_id = f"{agent_id}_{topic}_sub"
                    
                    # Get port for this topic
                    port = self.port_manager.get_available_port()
                    topic_address = address_base.rsplit(':', 1)[0] + f":{port}"
                    
                    # Create sockets
                    pub_socket = self.create_pub_socket(pub_id, topic, topic_address, bind=True)
                    
                    # Store in connection record
                    connection['sockets'][topic] = {
                        'pub': pub_id,
                        'sub': sub_id,  # Reserve ID, socket will be created by agent
                        'address': topic_address
                    }
                    connection['ports'][topic] = port
            
            # Store connection
            self.connections[agent_id] = connection
            
            return connection
            
    def deregister_agent_connection(self, agent_id: str) -> bool:
        """
        Deregister an agent connection and clean up resources.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            True if deregistered, False if agent not found
        """
        with self._lock:
            if agent_id not in self.connections:
                return False
                
            # Get connection info
            connection = self.connections[agent_id]
            
            # Close all sockets
            for topic, sockets in connection['sockets'].items():
                for socket_type, socket_id in sockets.items():
                    if socket_id in self.sockets:
                        self._close_socket(socket_id)
                        
                # Release port
                if topic in connection['ports']:
                    self.port_manager.release_port(connection['ports'][topic])
            
            # Remove connection
            del self.connections[agent_id]
            
            return True
            
    def _close_socket(self, socket_id: str) -> None:
        """
        Close a socket and clean up resources.
        
        Args:
            socket_id: Socket identifier
        """
        if socket_id not in self.sockets:
            return
            
        socket_info = self.sockets[socket_id]
        socket = socket_info['socket']
        
        # Close socket
        try:
            socket.close()
            logger.info(f"Closed socket {socket_id}")
        except Exception as e:
            logger.error(f"Error closing socket {socket_id}: {e}")
            
        # Remove from registry
        del self.sockets[socket_id]
            
    def close_socket(self, socket_id: str) -> bool:
        """
        Close a socket by ID.
        
        Args:
            socket_id: Socket identifier
            
        Returns:
            True if socket was closed, False if not found
        """
        with self._lock:
            if socket_id not in self.sockets:
                return False
                
            self._close_socket(socket_id)
            return True
            
    def get_socket_info(self, socket_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a socket.
        
        Args:
            socket_id: Socket identifier
            
        Returns:
            Socket information dictionary or None if not found
        """
        with self._lock:
            return self.sockets.get(socket_id)
            
    def get_agent_connection(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about an agent connection.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            Connection information dictionary or None if not found
        """
        with self._lock:
            return self.connections.get(agent_id)
            
    def get_socket(self, socket_id: str) -> Optional[zmq.Socket]:
        """
        Get a socket by ID.
        
        Args:
            socket_id: Socket identifier
            
        Returns:
            ZMQ socket or None if not found
        """
        with self._lock:
            socket_info = self.sockets.get(socket_id)
            if socket_info:
                return socket_info['socket']
            return None
            
    def cleanup(self) -> None:
        """
        Clean up all resources.
        """
        with self._lock:
            # Close all sockets
            for socket_id in list(self.sockets.keys()):
                self._close_socket(socket_id)
                
            # Clear connections
            self.connections.clear() 