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
FEAGI Visualization Client

Client for connecting to the FEAGI Visualization Stream, which provides one-directional
binary data flow from FEAGI to agent for neuron activity visualization.
"""

import asyncio
import logging
import time
from typing import Dict, Any, Optional, List, Callable

import zmq
import zmq.asyncio

# Configure logging
logger = logging.getLogger("feagi_connector.viz")


class FeagiVizClient:
    """
    Client for the FEAGI Visualization Stream.
    
    The Visualization Stream handles one-directional flow of neural activity
    data from FEAGI to visualization clients:
    - Neural firing data
    - Brain structure information
    - System status updates
    
    Implementation uses ZMQ SUB socket for efficient reception of data.
    """
    
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5562,
        agent_id: str = "",
        socket_timeout: int = 5000,
    ):
        """
        Initialize the Visualization Client.
        
        Args:
            host: FEAGI hostname or IP
            port: Visualization Stream port (default 5562)
            agent_id: Agent ID for subscription identification
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id
        self.socket_timeout = socket_timeout
        
        # Initialize ZMQ context
        self.context = zmq.asyncio.Context.instance()
        self.socket = None
        
        # State
        self.connected = False
        self.subscribed_topics = set()
        
        # Tasks
        self.listen_task = None
        
        # Callbacks
        self.viz_callback = None
    
    async def connect(self) -> bool:
        """
        Connect to the FEAGI Visualization Stream.
        
        Returns:
            True if connection was successful
        """
        if self.connected:
            return True
            
        try:
            # Create SUB socket
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.RCVTIMEO, self.socket_timeout)
            
            # Connect to FEAGI
            socket_address = f"tcp://{self.host}:{self.port}"
            self.socket.connect(socket_address)
            
            # Subscribe to activity data and system messages
            self.socket.setsockopt(zmq.SUBSCRIBE, b"activity")
            self.socket.setsockopt(zmq.SUBSCRIBE, b"system")
            self.subscribed_topics.add("activity")
            self.subscribed_topics.add("system")
            
            self.connected = True
            logger.info(f"Connected to FEAGI Visualization Stream at {socket_address}")
            logger.debug(f"Socket info: id={id(self.socket)}, agent_id={self.agent_id}, topics={self.subscribed_topics}")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to FEAGI Visualization Stream: {e}")
            await self._cleanup()
            return False
    
    async def close(self) -> None:
        """
        Close the connection to FEAGI.
        """
        await self._cleanup()
    
    async def _cleanup(self) -> None:
        """Clean up resources."""
        self.connected = False
        
        # Cancel listen task
        if self.listen_task:
            self.listen_task.cancel()
            try:
                await self.listen_task
            except asyncio.CancelledError:
                pass
            self.listen_task = None
        
        # Close socket
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def subscribe_to_topic(self, topic: str) -> None:
        """
        Subscribe to a visualization topic.
        
        Args:
            topic: Topic to subscribe to ("activity", "structure", "system")
        """
        if not self.connected or not self.socket:
            logger.error("Not connected to FEAGI")
            return
            
        if topic in self.subscribed_topics:
            logger.warning(f"Already subscribed to topic: {topic}")
            return
            
        # Subscribe to topic
        self.socket.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        self.subscribed_topics.add(topic)
        logger.info(f"Subscribed to visualization topic: {topic}")
    
    def unsubscribe_from_topic(self, topic: str) -> None:
        """
        Unsubscribe from a visualization topic.
        
        Args:
            topic: Topic to unsubscribe from
        """
        if not self.connected or not self.socket:
            logger.error("Not connected to FEAGI")
            return
            
        if topic not in self.subscribed_topics:
            logger.warning(f"Not subscribed to topic: {topic}")
            return
            
        # Do not allow unsubscribing from system messages
        if topic == "system":
            logger.warning("Cannot unsubscribe from system messages")
            return
            
        # Unsubscribe from topic
        self.socket.setsockopt(zmq.UNSUBSCRIBE, topic.encode("utf-8"))
        self.subscribed_topics.remove(topic)
        logger.info(f"Unsubscribed from visualization topic: {topic}")
    
    async def receive_visualization_data(self, timeout: float = 0.5) -> Optional[tuple]:
        """
        Receive visualization data from FEAGI.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Tuple of (topic, data) or None if no data available
        """
        if not self.connected or not self.socket:
            logger.error("Not connected to FEAGI")
            return None
            
        try:
            # Receive message with timeout
            logger.debug(f"Waiting for visualization data on socket {id(self.socket)}...")
            frames = await asyncio.wait_for(
                self.socket.recv_multipart(),
                timeout=timeout
            )
            
            if len(frames) != 2:  # [topic, message]
                logger.warning(f"Invalid frame count: {len(frames)}")
                return None
                
            # Extract topic and data
            topic = frames[0].decode("utf-8")
            data = frames[1]
            
            # Log detailed information about the received data
            data_len = len(data)
            logger.debug(f"Received {data_len} bytes on topic '{topic}'")
            
            # Log header bytes for debugging
            if data_len > 0:
                header_size = min(20, data_len)
                logger.debug(f"Data header: {[b for b in data[:header_size]]}")
                logger.debug(f"Data header (hex): {data[:header_size].hex()}")
                
                # If data is small enough, log the full contents
                if data_len < 100:
                    logger.debug(f"Full data (hex): {data.hex()}")
            else:
                logger.warning("Received empty data payload")
            
            return (topic, data)
            
        except asyncio.TimeoutError:
            # No data available (only log occasionaly to avoid noise)
            return None
            
        except Exception as e:
            logger.error(f"Error receiving visualization data: {e}")
            return None
    
    async def start_visualization_listener(self, callback: Callable[[bytes], None]) -> None:
        """
        Start listening for visualization data.
        
        Args:
            callback: Function to call when visualization data is received
                     (parameter: data)
        """
        if not self.connected or not self.socket:
            logger.error("Not connected to FEAGI")
            return
            
        self.viz_callback = callback
        
        message_count = 0
        last_log_time = time.time()
        
        logger.info(f"Starting visualization listener on socket {id(self.socket)} for topics {self.subscribed_topics}")
        
        try:
            while self.connected:
                try:
                    # Receive data
                    viz_data = await self.receive_visualization_data()
                    
                    # Log periodic status updates
                    current_time = time.time()
                    if current_time - last_log_time > 5.0:
                        logger.debug(f"Visualization listener running, {message_count} messages received so far")
                        last_log_time = current_time
                    
                    if not viz_data:
                        # No data available, sleep and try again
                        await asyncio.sleep(0.01)
                        continue
                    
                    message_count += 1
                    topic, data = viz_data
                    
                    # Handle system messages
                    if topic == "system":
                        message = data.decode("utf-8")
                        await self._handle_system_message(message)
                        continue
                    
                    # Call the callback with the visualization data
                    if self.viz_callback and topic == "activity":
                        logger.debug(f"Calling visualization callback with {len(data)} bytes of data (message #{message_count})")
                        await asyncio.to_thread(self.viz_callback, data)
                        
                except asyncio.CancelledError:
                    # Task was cancelled, exit gracefully
                    break
                    
                except Exception as e:
                    logger.error(f"Error handling visualization data: {e}")
                    await asyncio.sleep(0.1)  # Avoid tight loop on errors
                    
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            logger.info("Visualization listener task cancelled")
            pass
        except Exception as e:
            logger.error(f"Fatal error in visualization listener: {e}")
            
        logger.info(f"Visualization listener exiting, processed {message_count} messages total")
    
    async def _handle_system_message(self, message: str) -> None:
        """
        Handle system messages from FEAGI.
        
        Args:
            message: System message to handle
        """
        logger.debug(f"Received system message: {message}")
        
        # Handle state change messages
        if message.startswith("FEAGI_STATE_CHANGE:"):
            state = message.split(":", 1)[1]
            if state == "standby":
                logger.info("FEAGI entered standby mode")
            elif state == "active":
                logger.info("FEAGI entered active mode")
                
        # Handle other system messages as needed 