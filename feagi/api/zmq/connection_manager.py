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
from typing import Dict, Optional, List, Set, Any, Tuple, TYPE_CHECKING

import zmq
import zmq.asyncio

# Use TYPE_CHECKING to avoid circular imports
if TYPE_CHECKING:
    from .server import ZMQServer

from ..core.service import CoreApiService
from feagi.core.state_manager import GenomeState
from feagi.api.protocols import ProtocolID


class ConnectionManager:
    """
    Connection manager for multiple ZMQ clients using ROUTER-DEALER pattern.
    
    This class handles:
    - Creation of ROUTER sockets for different protocols
    - Client identity tracking (mapping ZMQ identities to agent IDs)
    - Message routing to specific clients
    - Connection state management
    """
    
    def __init__(self, 
                 context: Optional[zmq.asyncio.Context] = None,
                 control_port: int = 5559,
                 sensorimotor_port: int = 5558,
                 visualization_port: int = 5560):
        """
        Initialize the connection manager.
        
        Args:
            context: ZMQ context (will create a new one if None)
            control_port: Port for control messages (FCP protocol)
            sensorimotor_port: Port for sensorimotor data (FSMP protocol)
            visualization_port: Base port for visualization data (FVP protocol)
        """
        self.context = context or zmq.asyncio.Context.instance()
        self.connections: Dict[str, Dict[str, Any]] = {}  # agent_id -> connection_info
        
        # Create ROUTER sockets for each protocol
        self.control_socket = self.context.socket(zmq.ROUTER)
        self.control_socket.setsockopt(zmq.ROUTER_MANDATORY, 1)  # Raise error if recipient not found
        self.control_socket.setsockopt(zmq.SNDHWM, 0)  # Unlimited send queue
        self.control_socket.bind(f"tcp://*:{control_port}")
        logger.info(f"Control socket bound to port {control_port}")
        
        self.sensorimotor_socket = self.context.socket(zmq.ROUTER)
        self.sensorimotor_socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
        self.sensorimotor_socket.setsockopt(zmq.SNDHWM, 0)
        self.sensorimotor_socket.bind(f"tcp://*:{sensorimotor_port}")
        logger.info(f"Sensorimotor socket bound to port {sensorimotor_port}")
        
        self.visualization_socket = self.context.socket(zmq.ROUTER)
        self.visualization_socket.setsockopt(zmq.ROUTER_MANDATORY, 1)
        self.visualization_socket.setsockopt(zmq.SNDHWM, 0)
        self.visualization_socket.bind(f"tcp://*:{visualization_port}")
        logger.info(f"Visualization socket bound to port {visualization_port}")
        
        # Store port information
        self.ports = {
            "control": control_port,
            "sensorimotor": sensorimotor_port,
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
            protocol_type: Protocol type ("fcp", "fsmp", or "fvp")
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
                await self.control_socket.send_multipart([zmq_id, b"", message])
            elif protocol_type == "fsmp":
                await self.sensorimotor_socket.send_multipart([zmq_id, b"", message])
            elif protocol_type == "fvp":
                await self.visualization_socket.send_multipart([zmq_id, b"", message])
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
        """
        Close all sockets and clean up resources.
        """
        logger.info("Closing connection manager")
        
        # Close all sockets
        for socket in [self.control_socket, self.sensorimotor_socket, self.visualization_socket]:
            try:
                socket.close()
            except Exception as e:
                logger.error(f"Error closing socket: {e}")
        
        # Clear connections
        self.connections.clear()
        logger.info("Connection manager closed")


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
                 core_api: Optional[CoreApiService] = None,
                 host: str = "*"):
        """Get singleton instance of ZMQConnectionManager."""
        if cls._instance is None:
            cls._instance = cls(core_api, host)
        return cls._instance
        
    def __init__(self, 
                 core_api: Optional[CoreApiService] = None,
                 host: str = "*"):
        """
        Initialize the ZMQ Connection Manager.
        
        Args:
            core_api: The CoreApiService instance
            host: Host address for binding
        """
        self.core_api = core_api
        self.host = host
        self.context = zmq.asyncio.Context.instance()
        self._servers = {}
        self._running_tasks = set()
        self.running = False
        
        # State tracking
        self._active_mode = False  # True when genome is loaded
        
        # Register for genome state change notifications if CoreAPI is provided
        if core_api and hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        if core_api:
            self._update_active_mode()
        
    def create_server(self, server_type: str = "default", **kwargs) -> Any:
        """
        Create a new ZMQ server.
        
        Args:
            server_type: Type of server to create
            **kwargs: Additional arguments for server
            
        Returns:
            The created server
        """
        if server_type in self._servers:
            # Return existing server
            return self._servers[server_type]
            
        # Pass state information to server
        kwargs['core_api'] = self.core_api
        kwargs['host'] = self.host
        kwargs['context'] = self.context
        
        # Import here to avoid circular imports
        from .server import ZMQServer
        
        # Create the server
        server = ZMQServer(**kwargs)
        
        # Store it
        self._servers[server_type] = server
        
        # Inform server of current state
        if hasattr(server, '_update_active_mode'):
            server._update_active_mode()
        
        return server
    
    async def start_all(self) -> None:
        """Start all ZMQ servers."""
        self.running = True
        
        # Update state from CoreAPI
        self._update_active_mode()
        
        # Start all servers
        for server_id, server in self._servers.items():
            logger.info(f"Starting ZMQ server: {server_id}")
            task = asyncio.create_task(server.start())
            self._running_tasks.add(task)
            
    async def stop_all(self) -> None:
        """Stop all ZMQ servers."""
        self.running = False
        
        # Stop all servers
        for server_id, server in self._servers.items():
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
            for server in self._servers.values():
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
        for server in self._servers.values():
            if hasattr(server, '_update_active_mode'):
                server._update_active_mode() 