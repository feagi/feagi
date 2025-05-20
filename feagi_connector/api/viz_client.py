"""
FEAGI Visualization Client

Client for receiving neural activity and brain structure data from FEAGI using ZMQ DEALER/ROUTER pattern.
"""

import json
import logging
import uuid
import asyncio
from typing import Dict, Any, Optional, List, Union, Tuple

import zmq
import zmq.asyncio
from feagi_bytes import ByteStructureDecoder

# Configure logging
logger = logging.getLogger("feagi_connector.viz")


class FeagiVizClient:
    """
    Client for receiving visualization data from FEAGI using DEALER/ROUTER pattern (port 5560).
    
    This client handles neural activity and brain structure data for visualization.
    """
    
    def __init__(
        self, 
        host: str = "127.0.0.1", 
        port: int = 5560, 
        agent_id: Optional[str] = None, 
        socket_timeout: int = 1000
    ):
        """
        Initialize the visualization client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ DEALER/ROUTER port (default 5560)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"viz-{uuid.uuid4().hex[:8]}"
        self.timeout = socket_timeout
        self.context = zmq.asyncio.Context.instance()
        self.socket = None
        self.registered = False
        self._connected = False
        
    async def connect(self) -> bool:
        """
        Create and connect a socket.
        
        Returns:
            True if connection was successful
        """
        try:
            if self.socket:
                self.socket.close()
                
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait for pending messages on close
            
            # Set a connect timeout
            self.socket.setsockopt(zmq.CONNECT_TIMEOUT, 1000)  # 1 second timeout
            
            logger.debug(f"Connecting to visualization server at tcp://{self.host}:{self.port}")
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to visualization server")
            self._connected = True
            return True
            
        except zmq.error.ZMQError as e:
            logger.error(f"ZMQ Error connecting to {self.host}:{self.port}: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            self._connected = False
            return False
    
    def is_connected(self) -> bool:
        """
        Check if the client is connected.
        
        Returns:
            True if connected
        """
        return self._connected and self.socket is not None
            
    async def close(self) -> None:
        """Close the socket."""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.registered = False
            self._connected = False
    
    async def register_viz_agent(self) -> bool:
        """
        Register this agent with FEAGI visualization server.
        
        Returns:
            True if registration was successful
        """
        if self.registered:
            return True
           
        # Make sure we're connected
        if not self.is_connected():
            if not await self.connect():
                return False
            
        # Create registration message
        hello_msg = {
            "message_type": "hello",
            "agent_id": self.agent_id,
            "agent_type": "visualization",
        }
        
        try:
            # Send hello message
            logger.debug(f"Sending hello message to viz server: {hello_msg}")
            await self.socket.send_multipart([
                b"",  # Empty delimiter frame required for ROUTER
                json.dumps(hello_msg).encode('utf-8')
            ])
            
            # Wait for welcome message
            try:
                welcome_frames = await self.socket.recv_multipart()
                
                if len(welcome_frames) < 2:
                    logger.error(f"Invalid welcome message format: {welcome_frames}")
                    return False
                
                # Skip the empty delimiter frame
                welcome_data = welcome_frames[1]
                
                try:
                    welcome_msg = json.loads(welcome_data.decode('utf-8'))
                    
                    if welcome_msg.get("message_type") == "welcome":
                        logger.info(f"Successfully registered visualization agent {self.agent_id} with FEAGI")
                        self.registered = True
                        return True
                    else:
                        logger.error(f"Unexpected response to hello: {welcome_msg}")
                        return False
                        
                except json.JSONDecodeError:
                    logger.error(f"Could not decode welcome message: {welcome_data}")
                    return False
                    
            except zmq.error.Again:
                logger.error("Timed out waiting for welcome message")
                return False
                
        except Exception as e:
            logger.error(f"Error registering visualization agent: {e}")
            return False
    
    async def request_visualization(self, activity: bool = True, structure: bool = False) -> None:
        """
        Request visualization data from FEAGI.
        
        Args:
            activity: Whether to request neural activity data
            structure: Whether to request brain structure data
        """
        if not self.is_connected() or not self.registered:
            logger.error("Not connected or registered")
            return
        
        try:
            # Request activity data
            if activity:
                request_msg = {
                    "message_type": "activity_request",
                    "agent_id": self.agent_id,
                    "timestamp": int(asyncio.get_event_loop().time() * 1000)
                }
                
                await self.socket.send_multipart([
                    b"",  # Empty delimiter frame required for ROUTER
                    json.dumps(request_msg).encode('utf-8')
                ])
            
            # Request structure data
            if structure:
                request_msg = {
                    "message_type": "structure_request",
                    "agent_id": self.agent_id,
                    "timestamp": int(asyncio.get_event_loop().time() * 1000)
                }
                
                await self.socket.send_multipart([
                    b"",  # Empty delimiter frame required for ROUTER
                    json.dumps(request_msg).encode('utf-8')
                ])
                
        except Exception as e:
            logger.error(f"Error requesting visualization data: {e}")
    
    async def receive_visualization_data(self, timeout: int = None) -> Optional[Dict]:
        """
        Receive visualization data from FEAGI.
        
        Args:
            timeout: Receive timeout in milliseconds (None for default)
            
        Returns:
            Visualization data dictionary or None if no data available
        """
        if not self.is_connected() or not self.registered:
            logger.error("Not connected or registered")
            return None
        
        try:
            # Set timeout if provided
            if timeout is not None:
                old_timeout = self.socket.getsockopt(zmq.RCVTIMEO)
                self.socket.setsockopt(zmq.RCVTIMEO, timeout)
            
            # Try to receive a message
            try:
                frames = await self.socket.recv_multipart()
                
                if len(frames) < 3:
                    logger.warning(f"Invalid visualization data format: {frames}")
                    return None
                
                # Skip the empty delimiter frame
                header_data = frames[1]
                viz_data = frames[2]
                
                try:
                    header = json.loads(header_data.decode('utf-8'))
                    message_type = header.get("message_type")
                    
                    if message_type == "activity_data":
                        # Decode neural activity data
                        decoder = ByteStructureDecoder()
                        activity = decoder.decode_neuron_data(viz_data)
                        
                        return {
                            "message_type": "activity",
                            "timestamp": header.get("timestamp"),
                            "data": activity
                        }
                        
                    elif message_type == "structure_data":
                        # Decode brain structure data
                        structure = json.loads(viz_data.decode('utf-8'))
                        
                        return {
                            "message_type": "structure",
                            "timestamp": header.get("timestamp"),
                            "data": structure
                        }
                        
                    else:
                        logger.warning(f"Unexpected message type: {message_type}")
                        return None
                        
                except (json.JSONDecodeError, Exception) as e:
                    logger.error(f"Could not decode visualization data: {e}")
                    return None
                    
            except zmq.error.Again:
                # No data available within timeout
                return None
                
        except Exception as e:
            logger.error(f"Error receiving visualization data: {e}")
            return None
            
        finally:
            # Reset timeout if it was changed
            if timeout is not None:
                self.socket.setsockopt(zmq.RCVTIMEO, old_timeout) 