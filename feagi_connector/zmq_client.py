#!/usr/bin/env python3
"""
FEAGI 2.1 ZMQ Client

This module provides client classes for connecting to FEAGI 2.1 using ZMQ:

1. FeagiReqRepClient - Uses REQ/REP pattern for REST API calls (port 5555)
2. FeagiSensoryClient - Uses DEALER/ROUTER pattern for sensorimotor data (port 5558)
3. FeagiVizClient - Uses DEALER/ROUTER pattern for visualization data (port 5560)

These clients are designed to be used independently or together depending on
your application's needs.
"""

import sys
import os
import json
import time
import logging
import uuid
from typing import Dict, Any, List, Optional, Union, Tuple

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq

# Try to import feagi_bytes for binary encoding/decoding if available
try:
    from feagi_bytes.serialization import ByteStructureEncoder, ByteStructureDecoder
    from feagi_bytes.constants import ByteStructureID
    FEAGI_BYTES_AVAILABLE = True
except ImportError:
    FEAGI_BYTES_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_zmq_client")

class FeagiReqRepClient:
    """
    ZMQ REQ/REP client for FEAGI REST API.
    
    This client uses the REQ/REP pattern for making REST API calls on port 5555.
    It creates a new socket for each request to ensure proper socket state management.
    """
    
    def __init__(self, host="127.0.0.1", port=5555, timeout=5000):
        """
        Initialize the REQ/REP client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ REQ/REP port (default 5555)
            timeout: Request timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.next_id = 1
    
    def make_request(self, method, route, params=None, query=None, body=None, headers=None):
        """
        Make a REQ/REP request, creating a new socket for each request.
        
        Args:
            method: HTTP method (GET, POST, etc.)
            route: API route
            params: Path parameters
            query: Query parameters
            body: Request body
            headers: Request headers
            
        Returns:
            Response data or error dictionary
        """
        # Create a new socket for this request
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to the server
            socket.connect(f"tcp://{self.host}:{self.port}")
            
            # Create request
            request = {
                'id': self.next_id,
                'method': method.upper(),
                'route': route,
                'timestamp': int(time.time() * 1000)
            }
            self.next_id += 1
            
            # Add optional components
            if params:
                request['params'] = params
            else:
                request['params'] = {}
                
            if query:
                request['query'] = query
            else:
                request['query'] = {}
                
            if body:
                request['body'] = body
            else:
                request['body'] = {}
                
            if headers:
                request['headers'] = headers
            else:
                request['headers'] = {'content-type': 'application/json'}
                
            # Serialize and send
            request_bytes = json.dumps(request).encode('utf-8')
            logger.debug(f"Sending {method} {route}: {len(request_bytes)} bytes")
            
            # Send request
            socket.send(request_bytes)
            
            # Wait for response
            response_bytes = socket.recv()
            logger.debug(f"Received response: {len(response_bytes)} bytes")
            
            try:
                # Try to parse as JSON
                response = json.loads(response_bytes.decode('utf-8'))
                return response
            except json.JSONDecodeError:
                # If we can't parse as JSON, just return the raw response
                return {'raw_response': response_bytes.decode('utf-8', errors='replace')}
                
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                logger.error(f"Request timed out: {e}")
                return {'error': 'timeout', 'message': str(e)}
            else:
                logger.error(f"ZMQ error: {e}")
                return {'error': 'zmq', 'message': str(e)}
                
        except Exception as e:
            logger.error(f"Error: {e}")
            return {'error': 'general', 'message': str(e)}
            
        finally:
            # Always close the socket to prevent resource leaks
            socket.close()
    
    def get(self, route, params=None, query=None, headers=None):
        """Make a GET request."""
        return self.make_request('GET', route, params, query, None, headers)
    
    def post(self, route, body, params=None, query=None, headers=None):
        """Make a POST request."""
        return self.make_request('POST', route, params, query, body, headers)
    
    def put(self, route, body, params=None, query=None, headers=None):
        """Make a PUT request."""
        return self.make_request('PUT', route, params, query, body, headers)
    
    def delete(self, route, params=None, query=None, headers=None):
        """Make a DELETE request."""
        return self.make_request('DELETE', route, params, query, None, headers)


class FeagiSensoryClient:
    """
    ZMQ DEALER/ROUTER client for FEAGI sensorimotor data.
    
    This client uses the DEALER/ROUTER pattern to send sensory data
    to FEAGI via port 5558. It maintains a single persistent connection.
    """
    
    def __init__(self, host="127.0.0.1", port=5558, identity=None, agent_id=None):
        """
        Initialize the sensory client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ sensorimotor port (default 5558)
            identity: ZMQ socket identity (generated if not provided)
            agent_id: Agent ID for FEAGI (generated if not provided)
        """
        self.host = host
        self.port = port
        self.identity = identity or f"sensory-client-{int(time.time())}"
        self.agent_id = agent_id or f"agent-{uuid.uuid4()}"
        self.context = zmq.Context.instance()
        self.socket = None
        
        # Initialize feagi_bytes encoder if available
        self.encoder = ByteStructureEncoder() if FEAGI_BYTES_AVAILABLE else None
    
    def connect(self):
        """Connect to FEAGI's sensorimotor port."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.DEALER)
        
        # Set socket identity for the ROUTER on the server side
        self.socket.setsockopt(zmq.IDENTITY, self.identity.encode('utf-8'))
        
        # Connect to server
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to sensorimotor port at {self.host}:{self.port}")
    
    def disconnect(self):
        """Disconnect from FEAGI."""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def send_sensory_data(self, coordinates, cortical_id=None):
        """
        Send sensory data to FEAGI.
        
        Args:
            coordinates: List of [x,y,z] coordinates to activate
            cortical_id: Target cortical area ID
            
        Returns:
            True if successful, False otherwise
        """
        if not self.socket:
            self.connect()
            
        try:
            if not cortical_id:
                logger.warning("No cortical ID provided, using default")
                cortical_id = "default"
                
            # Create the data dictionary
            data = {
                "agent_id": self.agent_id,
                "coordinates": coordinates,
                "cortical_id": cortical_id,
                "timestamp": int(time.time() * 1000)
            }
            
            # Serialize the data
            if self.encoder and FEAGI_BYTES_AVAILABLE:
                # Use feagi_bytes for proper binary encoding
                # This would be the ideal method, but currently the text-based JSON works too
                try:
                    data_bytes = self.encoder.encode(ByteStructureID.FCL_LIST, data)
                    logger.debug(f"Encoded {len(coordinates)} coordinates as binary FCL_LIST")
                except Exception as e:
                    # Fall back to JSON if binary encoding fails
                    logger.warning(f"Binary encoding failed: {e}, falling back to JSON")
                    data_bytes = json.dumps(data).encode('utf-8')
            else:
                # Use standard JSON encoding
                data_bytes = json.dumps(data).encode('utf-8')
            
            # Send with proper framing for DEALER/ROUTER
            # VERY IMPORTANT: The empty first frame is required for the ROUTER socket
            self.socket.send_multipart([b"", data_bytes])
            
            logger.info(f"Sent {len(coordinates)} activations to {cortical_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False


class FeagiVizClient:
    """
    ZMQ DEALER/ROUTER client for FEAGI visualization data.
    
    This client uses the DEALER/ROUTER pattern to receive visualization
    data from FEAGI via port 5560.
    """
    
    def __init__(self, host="127.0.0.1", port=5560, identity=None):
        """
        Initialize the visualization client.
        
        Args:
            host: FEAGI hostname or IP
            port: ZMQ visualization port (default 5560)
            identity: ZMQ socket identity (generated if not provided)
        """
        self.host = host
        self.port = port
        self.identity = identity or f"viz-client-{int(time.time())}"
        self.context = zmq.Context.instance()
        self.socket = None
        
        # Initialize feagi_bytes decoder if available
        self.decoder = ByteStructureDecoder() if FEAGI_BYTES_AVAILABLE else None
    
    def connect(self):
        """Connect to FEAGI's visualization port."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.DEALER)
        
        # Set socket identity for the ROUTER on the server side
        self.socket.setsockopt(zmq.IDENTITY, self.identity.encode('utf-8'))
        
        # Connect to server
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to visualization port at {self.host}:{self.port}")
    
    def disconnect(self):
        """Disconnect from FEAGI."""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def receive_visualization_data(self, timeout=1000):
        """
        Receive visualization data from FEAGI.
        
        Args:
            timeout: Receive timeout in milliseconds
            
        Returns:
            Tuple of (success, data) where success is a boolean and
            data is either the visualization data or an error message
        """
        if not self.socket:
            self.connect()
            
        try:
            # Set receive timeout
            self.socket.setsockopt(zmq.RCVTIMEO, timeout)
            
            # Receive multipart message
            parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            
            # Must have at least 2 parts (empty delimiter + payload)
            if len(parts) < 2:
                return False, "Invalid message format"
                
            # The payload is the second part
            payload = parts[1]
            
            # Try to decode the payload
            if self.decoder and FEAGI_BYTES_AVAILABLE:
                try:
                    # Check if this is a binary message with a structure header
                    if len(payload) > 4:
                        message_type = int.from_bytes(payload[0:4], byteorder='little')
                        structure_id = ByteStructureID(message_type)
                        data = self.decoder.decode(payload)
                        return True, data
                except Exception as e:
                    logger.warning(f"Binary decoding failed: {e}, trying JSON")
            
            # Fall back to JSON decoding
            try:
                data = json.loads(payload.decode('utf-8'))
                return True, data
            except json.JSONDecodeError:
                # If all else fails, return the raw payload
                return False, {"raw_data": payload}
                
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                # No message available, not an error
                return False, "No data available"
            else:
                logger.error(f"ZMQ error: {e}")
                return False, str(e)
                
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            return False, str(e)


# Example usage
if __name__ == "__main__":
    # Test REST API client
    rest_client = FeagiReqRepClient()
    result = rest_client.get('/v1/system/health_check')
    print(f"Health check result: {result}")
    
    # Test sensory client
    sensory_client = FeagiSensoryClient()
    sensory_client.connect()
    
    # Generate some random coordinates
    import random
    coordinates = [[random.randint(0, 7), random.randint(0, 7), 0] for _ in range(10)]
    
    # Send to a cortical area (replace with an actual ID from your genome)
    sensory_client.send_sensory_data(coordinates, cortical_id="iv00CC")
    sensory_client.disconnect()
    
    print("Test complete") 