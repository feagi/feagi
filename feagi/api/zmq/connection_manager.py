"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
ZeroMQ Connection Manager for FEAGI API.

This module manages ZeroMQ client and server connections.
It handles:
- Central connection/socket management
- State synchronization across streams
- Socket cleanup and lifecycle management
"""

import asyncio
import time
import logging
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
from typing import Dict, Optional, List, Set, Any, Tuple, TYPE_CHECKING, Callable

import zmq
import zmq.asyncio

# Use TYPE_CHECKING to avoid circular imports
if TYPE_CHECKING:
    from .server import ZMQServer

from ..core.services.core_api_service import CoreAPIService
from feagi.core.state_manager import GenomeState
from feagi.api.protocols import ProtocolID


class ConnectionManager:
    """
    Connection manager for multiple ZMQ clients using ROUTER-DEALER pattern.
    
    This class handles:
    - Creation of sockets for different protocols
    - Client identity tracking (mapping ZMQ identities to agent IDs)
    - Message routing to specific clients
    - Connection state management
    """
    
    def __init__(self, 
                 context: Optional[zmq.asyncio.Context] = None,
                 control_port: int = 5559,
                 sensory_port: int = 5558,
                 motor_port: int = 5564,
                 visualization_port: int = 5560):
        """
        Initialize the connection manager.
        
        Args:
            context: ZMQ context (will create a new one if None)
            control_port: Port for control messages (FCP protocol)
            sensory_port: Port for sensory data input (FSMP protocol)
            motor_port: Port for motor data output (FSMP protocol)
            visualization_port: Port for visualization data (FVP protocol)
        """
        self.context = context or zmq.asyncio.Context.instance()
        self.connections: Dict[str, Dict[str, Any]] = {}  # agent_id -> connection_info
        
        # Create control socket (ROUTER) for bidirectional control messages
        self.control_socket = self.context.socket(zmq.ROUTER)
        self.control_socket.setsockopt(zmq.ROUTER_MANDATORY, 1)  # Raise error if recipient not found
        self.control_socket.setsockopt(zmq.SNDHWM, 0)  # Unlimited send queue
        self.control_socket.bind(f"tcp://*:{control_port}")
        logger.info(f"Control socket (ROUTER) bound to port {control_port}")
        
        # Create sensory socket (PULL) for receiving sensory data
        self.sensory_socket = self.context.socket(zmq.PULL)
        self.sensory_socket.setsockopt(zmq.RCVHWM, 1)  # Minimal receive queue
        self.sensory_socket.bind(f"tcp://*:{sensory_port}")
        logger.info(f"Sensory socket (PULL) bound to port {sensory_port}")
        
        # Create motor socket (PUB) for broadcasting motor commands
        self.motor_socket = self.context.socket(zmq.PUB)
        self.motor_socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        self.motor_socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        self.motor_socket.bind(f"tcp://*:{motor_port}")
        logger.info(f"Motor socket (PUB) bound to port {motor_port}")
        
        # Create visualization socket (PUB) for broadcasting visualization data
        self.visualization_socket = self.context.socket(zmq.PUB)
        self.visualization_socket.setsockopt(zmq.SNDHWM, 1)
        self.visualization_socket.setsockopt(zmq.CONFLATE, 1)
        self.visualization_socket.bind(f"tcp://*:{visualization_port}")
        logger.info(f"Visualization socket (PUB) bound to port {visualization_port}")
        
        # Store port information
        self.ports = {
            "control": control_port,
            "sensory": sensory_port,
            "motor": motor_port,
            "visualization": visualization_port
        }
    
    def register_client(self, 
                         agent_id: str, 
                         zmq_id: bytes, 
                         supported_protocols: Dict[str, int]) -> None:
        """
        Register a new client connection.
        
        Args:
            agent_id: Unique identifier for the agent
            zmq_id: ZMQ identity frame for this client
            supported_protocols: Dictionary mapping protocol names to version numbers
        """
        self.connections[agent_id] = {
            "zmq_id": zmq_id,
            "protocols": supported_protocols,
            "last_active": time.time(),
            "connected_since": time.time(),
            "message_count": {"sent": 0, "received": 0}
        }
        logger.info(f"Registered client {agent_id} with ZMQ ID {zmq_id.hex()}")
    
    def deregister_client(self, agent_id: str) -> bool:
        """
        Deregister a client.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            True if client was successfully deregistered, False if not found
        """
        if agent_id in self.connections:
            del self.connections[agent_id]
            logger.info(f"Deregistered client {agent_id}")
            return True
        return False
    
    def get_client_info(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a client by agent ID.
        
        Args:
            agent_id: Agent identifier
            
        Returns:
            Client information dictionary or None if not found
        """
        return self.connections.get(agent_id)
    
    def get_client_by_zmq_id(self, zmq_id: bytes) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
        """
        Find client by ZMQ identity.
        
        Args:
            zmq_id: ZMQ identity frame
            
        Returns:
            Tuple of (agent_id, client_info) or (None, None) if not found
        """
        for agent_id, info in self.connections.items():
            if info["zmq_id"] == zmq_id:
                return agent_id, info
        return None, None
    
    def update_client_activity(self, agent_id: str) -> None:
        """
        Update the last activity timestamp for a client.
        
        Args:
            agent_id: Agent identifier
        """
        if agent_id in self.connections:
            self.connections[agent_id]["last_active"] = time.time()
            self.connections[agent_id]["message_count"]["received"] += 1
    
    async def send_message(self, 
                          agent_id: str, 
                          protocol_type: str, 
                          message: bytes) -> bool:
        """
        Send a message to a specific client.
        
        Args:
            agent_id: Agent identifier
            protocol_type: Protocol type ("fcp", "fsmp_sensory", "fsmp_motor", or "fvp")
            message: Serialized message
            
        Returns:
            True if message was sent, False if client not found
        """
        client_info = self.connections.get(agent_id)
        if not client_info:
            logger.warning(f"Client {agent_id} not found")
            return False
            
        zmq_id = client_info["zmq_id"]
        
        try:
            if protocol_type == "fcp":
                # Control messages use ROUTER-DEALER pattern
                await self.control_socket.send_multipart([zmq_id, b"", message])
            elif protocol_type == "fsmp_sensory":
                # Sensory data is received, not sent to clients
                logger.warning("Cannot send sensory data to client (one-way from client to FEAGI)")
                return False
            elif protocol_type == "fsmp_motor":
                # Motor data uses PUB-SUB pattern with channel as topic
                channel_id = agent_id.encode()  # Use agent_id as topic
                await self.motor_socket.send_multipart([channel_id, message])
            elif protocol_type == "fvp":
                # Visualization data uses PUB-SUB pattern with topic
                topic = b"activity"  # Default topic
                await self.visualization_socket.send_multipart([topic, message])
            else:
                logger.error(f"Unknown protocol type: {protocol_type}")
                return False
                
            # Update client info
            client_info["last_active"] = time.time()
            client_info["message_count"]["sent"] += 1
            return True
            
        except zmq.ZMQError as e:
            logger.error(f"Failed to send message to client {agent_id}: {e}")
            return False
    
    def get_inactive_clients(self, timeout_seconds: int = 30) -> List[str]:
        """
        Get a list of clients that have been inactive for longer than the specified timeout.
        
        Args:
            timeout_seconds: Number of seconds of inactivity to consider a client inactive
            
        Returns:
            List of agent IDs for inactive clients
        """
        now = time.time()
        return [
            agent_id for agent_id, info in self.connections.items()
            if now - info["last_active"] > timeout_seconds
        ]
    
    def get_connection_stats(self) -> Dict[str, Any]:
        """
        Get statistics about current connections.
        
        Returns:
            Dictionary of connection statistics
        """
        return {
            "active_connections": len(self.connections),
            "ports": self.ports,
            "clients": [
                {
                    "agent_id": agent_id,
                    "connected_since": info["connected_since"],
                    "last_active": info["last_active"],
                    "message_count": info["message_count"],
                    "protocols": info["protocols"]
                }
                for agent_id, info in self.connections.items()
            ]
        }
    
    def close(self) -> None:
        """Close all sockets and clean up resources."""
        logger.info("Closing ConnectionManager sockets")
        for socket_name in ["control_socket", "sensory_socket", "motor_socket", "visualization_socket"]:
            socket = getattr(self, socket_name, None)
            if socket:
                try:
                    socket.close(linger=0)
                except Exception as e:
                    logger.error(f"Error closing {socket_name}: {e}")
        
        # Clear connection tracking
        self.connections.clear()
        logger.info("ConnectionManager closed")


class ZMQConnectionManager:
    """
    ZeroMQ Connection Manager.
    
    This singleton manages all ZeroMQ connections for FEAGI API.
    It ensures consistent state across all streams and handles
    lifecycle management for connections.
    """
    _instance = None
    
    @classmethod
    def instance(cls, 
                 core_api: Optional[CoreAPIService] = None,
                 host: str = "*"):
        """Get the singleton instance."""
        if cls._instance is None:
            cls._instance = ZMQConnectionManager(core_api, host)
        return cls._instance
    
    def __init__(self, 
                 core_api: Optional[CoreAPIService] = None,
                 host: str = "*"):
        """
        Initialize the connection manager.
        
        Args:
            core_api: Core API service
            host: Host to bind to
        """
        if ZMQConnectionManager._instance is not None:
            raise RuntimeError("Use ZMQConnectionManager.instance() to get the singleton instance")
            
        self.host = host
        self.core_api = core_api
        
        # Standard ports
        self.control_port = 5559
        self.sensory_port = 5558
        self.motor_port = 5564
        self.visualization_port = 5560
        
        # Active mode flag
        self._active_mode = False
        
        # ZMQ servers
        self.servers = {}
        
        # Register for genome state change notifications
        if core_api and hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
            
        # Initialize active mode
        self._update_active_mode()
        
    def create_server(self, server_type: str = "default", **kwargs) -> Any:
        """
        Create a ZMQ server instance.
        
        Args:
            server_type: Type of server to create
            **kwargs: Additional arguments for server creation
            
        Returns:
            Server instance
        """
        if server_type in self.servers:
            logger.warning(f"Server of type {server_type} already exists, returning existing instance")
            return self.servers[server_type]
            
        # Import server implementation here to avoid circular imports
        from .server import ZmqServer
        
        # Create server with default ports if not specified
        server = ZmqServer(
            core_api=self.core_api,
            host=kwargs.get("host", self.host),
            control_port=kwargs.get("control_port", self.control_port),
            sensory_port=kwargs.get("sensory_port", self.sensory_port),
            motor_port=kwargs.get("motor_port", self.motor_port),
            vis_port=kwargs.get("vis_port", self.visualization_port)
        )
        
        # Store server
        self.servers[server_type] = server
        
        return server
    
    async def start_all(self) -> None:
        """Start all ZMQ servers."""
        self.running = True
        
        # Update state from CoreAPI
        self._update_active_mode()
        
        # Start all servers
        for server_id, server in self.servers.items():
            logger.info(f"Starting ZMQ server: {server_id}")
            task = asyncio.create_task(server.start())
            self._running_tasks.add(task)
            
    async def stop_all(self) -> None:
        """Stop all ZMQ servers."""
        self.running = False
        
        # Stop all servers
        for server_id, server in self.servers.items():
            logger.info(f"Stopping ZMQ server: {server_id}")
            await server.stop()
            
        # Cancel all running tasks
        for task in self._running_tasks:
            if not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        
        self._running_tasks.clear()
        
    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        if not self.core_api:
            return
            
        old_mode = self._active_mode
        self._active_mode = self.core_api.genome_is_loaded()
        
        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("ZMQConnectionManager: Entering ACTIVE mode (genome loaded)")
            else:
                logger.info("ZMQConnectionManager: Entering STANDBY mode (no genome loaded)")
                
            # Propagate state change to all servers
            for server in self.servers.values():
                if hasattr(server, '_update_active_mode'):
                    server._update_active_mode()
    
    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes.
        
        Args:
            old_state: Previous genome state
            new_state: New genome state
        """
        logger.debug(f"ZMQConnectionManager received genome state change: {old_state} → {new_state}")
        
        # Only care about LOADED vs other states
        if new_state == GenomeState.LOADED:
            # Transition to active mode when genome is loaded
            self._active_mode = True
            logger.info("ZMQConnectionManager: Entering ACTIVE mode (genome loaded)")
        else:
            # Any other state means genome not fully loaded
            self._active_mode = False 
            logger.info("ZMQConnectionManager: Entering STANDBY mode (genome not loaded)")
            
        # Propagate state change to all servers
        for server in self.servers.values():
            if hasattr(server, '_update_active_mode'):
                server._update_active_mode() 