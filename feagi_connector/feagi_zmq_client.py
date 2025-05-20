#!/usr/bin/env python3
"""
FEAGI ZMQ Client
----------------

A complete implementation of all ZMQ communication patterns used with FEAGI 2.1.

This module provides three client classes for the different ZMQ patterns used by FEAGI:
1. FeagiCommandClient - REQ/REP pattern for command-based API (port 5555)
2. FeagiSensoryClient - DEALER/ROUTER pattern for sensory data (port 5558)
3. FeagiVizClient - DEALER/ROUTER pattern for visualization data (port 5560)

Author: FEAGI Team
"""

import json
import logging
import os
import sys
import time
import uuid
from typing import Dict, Any, Optional, List, Union, Tuple

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq
import numpy as np
from feagi_bytes import ByteStructureEncoder, ByteStructureDecoder, ByteStructureID

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_zmq_client")


class FeagiCommandClient:
    """
    Command client for FEAGI using REQ/REP pattern (port 5555).
    
    This client implements the command-based API over ZMQ REQ/REP pattern,
    correctly handling message formatting and state.
    """
    
    def __init__(self, host="127.0.0.1", port=5555, timeout=5000, auth_token=None):
        """
        Initialize the command client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ REQ/REP port (default 5555)
            timeout: Request timeout in milliseconds
            auth_token: Authentication token (optional)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.auth_token = auth_token
        self.context = zmq.Context.instance()
        self.next_id = 1
    
    def make_request(self, command: str, params: Optional[Dict] = None) -> Dict:
        """
        Make a direct command request with proper message formatting.
        
        Args:
            command: Command name (e.g., 'ping', 'get_status')
            params: Command parameters
            
        Returns:
            Response data or error dictionary
        """
        # Create a new socket for this request (important for REQ/REP state)
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to the server
            socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to {self.host}:{self.port}")
            
            # Create command request
            request = {
                'command': command,
                'id': self.next_id,
                'timestamp': int(time.time() * 1000),
                'params': params or {}
            }
            self.next_id += 1
                
            # Serialize request data
            request_bytes = json.dumps(request).encode('utf-8')
            content_type = "application/json"
            auth_token_bytes = self.auth_token.encode('utf-8') if self.auth_token else b""
            
            logger.debug(f"Sending command: {command} ({len(request_bytes)} bytes)")
            
            # Send request with correct format: [auth_token, content_type, request_data]
            socket.send_multipart([
                auth_token_bytes,              # Auth token (empty if none)
                content_type.encode('utf-8'),  # Content type
                request_bytes                  # Request data
            ])
            
            # Wait for response
            try:
                response_parts = socket.recv_multipart()
                
                # FEAGI should respond with [content_type, response_data]
                if len(response_parts) < 2:
                    logger.warning(f"Unexpected response format - received {len(response_parts)} parts")
                    return {'error': 'Invalid response format'}
                
                # Parse the response
                resp_content_type = response_parts[0].decode('utf-8')
                resp_data = response_parts[1]
                
                logger.debug(f"Received response: {len(resp_data)} bytes, content-type: {resp_content_type}")
                
                if resp_content_type == "application/json":
                    try:
                        # Parse JSON response
                        response = json.loads(resp_data.decode('utf-8'))
                        return response
                    except json.JSONDecodeError:
                        logger.warning(f"Could not decode JSON response")
                        return {'raw_response': resp_data.decode('utf-8', errors='replace')}
                else:
                    # Return raw response with content type
                    return {
                        'content_type': resp_content_type,
                        'raw_response': resp_data.decode('utf-8', errors='replace')
                    }
                
            except zmq.error.Again:
                logger.error(f"Request timed out after {self.timeout/1000} seconds")
                return {'error': 'timeout'}
                
        except Exception as e:
            logger.error(f"Error: {e}")
            return {'error': 'general', 'message': str(e)}
            
        finally:
            # Always close the socket to prevent resource leaks
            socket.close()
    
    # Convenience methods for common commands
    def ping(self) -> Dict:
        """Send a ping command."""
        return self.make_request("ping")
    
    def get_status(self) -> Dict:
        """Get the simulation status."""
        return self.make_request("get_status")
    
    def get_configuration(self) -> Dict:
        """Get the FEAGI configuration."""
        return self.make_request("get_configuration")
    
    def get_performance(self) -> Dict:
        """Get performance metrics."""
        return self.make_request("get_performance")


class FeagiSensoryClient:
    """
    Client for sending sensory data to FEAGI using DEALER/ROUTER pattern (port 5558).
    
    This client properly handles binary data transmission for sensory input,
    using the correct message framing for DEALER/ROUTER.
    """
    
    def __init__(self, host="127.0.0.1", port=5558, agent_id=None, socket_timeout=1000):
        """
        Initialize the sensory client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ DEALER/ROUTER port (default 5558)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"agent-{uuid.uuid4().hex[:8]}"
        self.timeout = socket_timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.registered = False
        self._connect()
        
    def _connect(self):
        """Create and connect a socket."""
        try:
            if self.socket:
                self.socket.close()
                
            self.socket = self.context.socket(zmq.DEALER)
            self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.LINGER, 0)  # Don't wait for pending messages on close
            
            # Set a connect timeout
            self.socket.setsockopt(zmq.CONNECT_TIMEOUT, 1000)  # 1 second timeout
            
            logger.debug(f"Connecting to sensorimotor server at tcp://{self.host}:{self.port}")
            self.socket.connect(f"tcp://{self.host}:{self.port}")
            logger.debug(f"Connected to sensorimotor server")
            return True
            
        except zmq.error.ZMQError as e:
            logger.error(f"ZMQ Error connecting to {self.host}:{self.port}: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
            
    def close(self):
        """Close the socket."""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def register_agent(self, agent_type="external", sensory_channels=None, motor_channels=None) -> bool:
        """
        Register this agent with FEAGI.
        
        Args:
            agent_type: Type of agent 
            sensory_channels: List of sensory channels
            motor_channels: List of motor channels
            
        Returns:
            True if registration was successful
        """
        if self.registered:
            return True
           
        # Make sure we're connected
        if not self.socket:
            if not self._connect():
                return False
            
        # Create registration message
        hello_msg = {
            "message_type": "hello",
            "agent_id": self.agent_id,
            "agent_type": agent_type,
            "timestamp": time.time()
        }
        
        logger.debug(f"Sending hello message: {hello_msg}")
        
        # Send the message with correct framing for DEALER/ROUTER
        # Empty frame required for DEALER/ROUTER pattern
        try:
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(hello_msg).encode()
            ])
            
            # Wait for welcome response
            logger.debug("Waiting for welcome response...")
            try:
                response = self.socket.recv_multipart()
                logger.debug(f"Received response with {len(response)} frames")
                
                if len(response) < 2:
                    logger.error("Invalid welcome response - not enough frames")
                    return False
                
                # Try to decode the response 
                try:
                    # Skip the first empty frame
                    welcome_data = json.loads(response[1].decode())
                    logger.debug(f"Welcome response: {welcome_data}")
                    
                    if welcome_data.get("message_type") == "welcome":
                        logger.info(f"Received welcome: {welcome_data.get('message')}")
                        
                        # Send capabilities
                        capabilities = {
                            "message_type": "capabilities",
                            "agent_id": self.agent_id,
                            "supported_sensory_channels": sensory_channels or [],
                            "supported_motor_channels": motor_channels or [],
                            "timestamp": time.time()
                        }
                        
                        logger.debug(f"Sending capabilities: {capabilities}")
                        
                        self.socket.send_multipart([
                            b"",  # Empty delimiter frame
                            json.dumps(capabilities).encode()
                        ])
                        
                        # Wait for capabilities_ack
                        logger.debug("Waiting for capabilities_ack...")
                        try:
                            response = self.socket.recv_multipart()
                            logger.debug(f"Received capabilities response with {len(response)} frames")
                            
                            if len(response) < 2:
                                logger.error("Invalid capabilities response - not enough frames")
                                return False
                            
                            try:
                                capabilities_ack = json.loads(response[1].decode())
                                logger.debug(f"Capabilities response: {capabilities_ack}")
                                
                                if capabilities_ack.get("message_type") == "capabilities_ack":
                                    logger.info("Registration complete")
                                    self.registered = True
                                    return True
                                else:
                                    logger.error(f"Unexpected response type: {capabilities_ack.get('message_type')}")
                                    
                            except json.JSONDecodeError as e:
                                logger.error(f"Error decoding capabilities_ack: {e}")
                                
                        except zmq.error.Again:
                            logger.error(f"Timeout waiting for capabilities_ack")
                        
                    else:
                        logger.error(f"Unexpected message type: {welcome_data.get('message_type')}")
                        
                except json.JSONDecodeError as e:
                    logger.error(f"Error decoding welcome message: {e}")
                    
            except zmq.error.Again:
                logger.error(f"Timeout waiting for welcome message")
                
        except Exception as e:
            logger.error(f"Registration error: {e}")
            
        return False
    
    def send_sensory_data(self, cortical_area: str, neuron_data: Dict[Tuple[int, int, int], float]) -> bool:
        """
        Send sensory data to FEAGI.
        
        Args:
            cortical_area: Target cortical area name
            neuron_data: Dictionary mapping (x,y,z) coordinates to activation values
            
        Returns:
            True if data was sent successfully
        """
        if not self.registered:
            logger.warning("Agent not registered")
            if not self.register_agent():
                return False
        
        try:
            # Convert neuron data to the format expected by ByteStructureEncoder
            coordinates = []
            values = []
            
            for coord, value in neuron_data.items():
                coordinates.append(coord)
                values.append(value)
                
            # Create numpy arrays for coordinates and values
            coordinates_array = np.array(coordinates, dtype=np.int32)
            values_array = np.array(values, dtype=np.float32)
            
            # Encode the neuron data using feagi_bytes
            encoder = ByteStructureEncoder()
            encoded_data = encoder.encode_neuron_data(
                ByteStructureID.NEURON_DATA_V1,
                coordinates_array,
                values_array
            )
            
            # Create message with metadata
            message = {
                "message_type": "sensory_data",
                "cortical_area": cortical_area,
                "timestamp": time.time(),
                "data_format": "feagi_bytes"
            }
            
            # Send metadata and binary data
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(message).encode(),
                encoded_data
            ])
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False


class FeagiVizClient:
    """
    Client for receiving visualization data from FEAGI using DEALER/ROUTER pattern (port 5560).
    
    This client receives neural activity visualization data from FEAGI.
    """
    
    def __init__(self, host="127.0.0.1", port=5560, agent_id=None, socket_timeout=1000):
        """
        Initialize the visualization client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ DEALER/ROUTER port for visualization (default 5560)
            agent_id: Agent ID for FEAGI registration (default: auto-generated)
            socket_timeout: Socket timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.agent_id = agent_id or f"viz-{uuid.uuid4().hex[:8]}"
        self.timeout = socket_timeout
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.IDENTITY, self.agent_id.encode())
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        self.registered = False
    
    def close(self):
        """Close the socket."""
        self.socket.close()
    
    def register_viz_agent(self) -> bool:
        """
        Register this agent for visualization with FEAGI.
        
        Returns:
            True if registration was successful
        """
        if self.registered:
            return True
            
        # Create registration message
        hello_msg = {
            "message_type": "hello",
            "agent_id": self.agent_id,
            "agent_type": "visualization",
            "timestamp": time.time()
        }
        
        # Send the message
        try:
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(hello_msg).encode()
            ])
            
            # Wait for welcome response
            response = self.socket.recv_multipart()
            if len(response) < 2:
                logger.error("Invalid welcome response")
                return False
                
            welcome_data = json.loads(response[1].decode())
            
            if welcome_data.get("message_type") == "welcome":
                logger.info(f"Received welcome: {welcome_data.get('message')}")
                
                # For visualization client, no capabilities message is needed
                self.registered = True
                return True
                    
        except Exception as e:
            logger.error(f"Registration error: {e}")
            
        return False
    
    def request_visualization(self) -> None:
        """
        Request visualization data from FEAGI.
        """
        if not self.registered:
            if not self.register_viz_agent():
                logger.warning("Failed to register visualization agent")
                return
        
        try:
            # Create visualization request
            request = {
                "message_type": "visualization_request",
                "timestamp": time.time()
            }
            
            # Send the request
            self.socket.send_multipart([
                b"",  # Empty delimiter frame
                json.dumps(request).encode()
            ])
            
        except Exception as e:
            logger.error(f"Error requesting visualization: {e}")
    
    def receive_visualization_data(self, timeout=1000) -> Optional[Dict]:
        """
        Receive visualization data from FEAGI.
        
        Args:
            timeout: Timeout in milliseconds
            
        Returns:
            Dictionary with visualization data or None if no data received
        """
        if not self.registered:
            if not self.register_viz_agent():
                logger.warning("Failed to register visualization agent")
                return None
                
        # Set timeout for this receive operation
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        
        try:
            # Receive message
            message = self.socket.recv_multipart()
            
            # Skip empty frame
            if len(message) < 2:
                logger.warning("Received malformed message")
                return None
                
            # Parse message
            try:
                # The second frame should contain the message data
                data = json.loads(message[1].decode())
                
                # If there are additional frames, they contain binary data
                if len(message) > 2:
                    data['binary_data'] = message[2:]
                    
                return data
                
            except json.JSONDecodeError:
                logger.warning("Received non-JSON message")
                return None
                
        except zmq.error.Again:
            # Timeout occurred
            return None
            
        except Exception as e:
            logger.error(f"Error receiving visualization data: {e}")
            return None


if __name__ == "__main__":
    # Example usage
    logger.info("Testing FEAGI ZMQ clients")
    
    # Command client example
    logger.info("Testing Command Client on port 5555...")
    cmd_client = FeagiCommandClient()
    response = cmd_client.ping()
    logger.info(f"Ping response: {response}")
    
    response = cmd_client.get_status()
    logger.info(f"Status response: {response}")
    
    # Sensory client example
    logger.info("Testing Sensory Client on port 5558...")
    sensory_client = FeagiSensoryClient()
    if sensory_client.register_agent(sensory_channels=["visual", "touch"]):
        logger.info("Successfully registered agent with FEAGI")
        
        # Send some test data
        logger.info("Sending test sensory data...")
        neuron_data = {
            (0, 0, 0): 1.0,
            (1, 0, 0): 0.5,
            (0, 1, 0): 0.7,
            (1, 1, 0): 0.3
        }
        
        if sensory_client.send_sensory_data("visual_input", neuron_data):
            logger.info("Successfully sent sensory data")
        else:
            logger.error("Failed to send sensory data")
            
        sensory_client.close()
    else:
        logger.error("Failed to register sensory agent")
    
    # Visualization client example
    # (uncomment to test)
    """
    logger.info("Testing Visualization Client on port 5560...")
    viz_client = FeagiVizClient()
    if viz_client.register_viz_agent():
        logger.info("Successfully registered visualization agent")
        viz_client.request_visualization()
        viz_data = viz_client.receive_visualization_data()
        logger.info(f"Visualization data: {viz_data}")
        viz_client.close()
    else:
        logger.error("Failed to register visualization agent")
    """
    
    logger.info("Testing complete") 