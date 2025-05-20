#!/usr/bin/env python3
"""
Simplified FEAGI Sensory Injection Test

This script demonstrates connecting to FEAGI, loading a test genome,
and injecting sensory data using ZMQ for neural activity visualization.
"""

import asyncio
import json
import logging
import os
import random
import sys
import time
from typing import List, Dict, Any, Optional

# We need to ensure we're using the system's ZMQ, not our local module
# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

# Import ZMQ from venv
import zmq
import zmq.asyncio

# Add the parent directory to path for FEAGI modules
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Import FEAGI Bytes library for proper binary encoding/decoding
from feagi_bytes.constants import ByteStructureID
from feagi_bytes.serialization import ByteStructureEncoder, ByteStructureDecoder
from feagi_bytes.translator import ByteStructureTranslator

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,  # Set to DEBUG for more detailed logs
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("feagi_sensory_test.log")
    ]
)
logger = logging.getLogger("sensory_test")

# Configuration
DEFAULT_HOST = "127.0.0.1"
DEFAULT_REST_PORT = 8000  # HTTP REST API port
DEFAULT_ZMQ_REST_PORT = 5555  # ZMQ REQ/REP port for REST API
DEFAULT_ZMQ_SENSORY_PORT = 5558  # ZMQ Sensorimotor port
VISUAL_CORTICAL_AREA = "iv00CC"  # Common visual cortical area in test genomes
NUM_PATTERNS = 20  # Number of patterns to generate
PATTERN_SIZE = 16  # Number of neurons in each pattern
DELAY_BETWEEN_PATTERNS = 0.2  # Seconds
TEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), "test_genome.json")  # Path to test genome file
ZMQ_TIMEOUT = 10000  # 10 seconds timeout for ZMQ sockets (reduced from 30 seconds)


class SimpleZmqRestClient:
    """
    Simple ZMQ REST API client for FEAGI.
    
    This class provides a minimal implementation to send REST-like requests
    over ZMQ DEALER/ROUTER sockets.
    """
    
    def __init__(self, host="localhost", port=5560, timeout=ZMQ_TIMEOUT, identity=None):
        """
        Initialize the ZMQ REST client.
        
        Args:
            host: Host address
            port: ZMQ REST port
            timeout: Request timeout in milliseconds
            identity: Optional client identity
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.identity = identity or f"rest-client-{int(time.time())}"
        self.context = zmq.Context.instance()
        self.socket = None
        self.request_id = 0
        
    def connect(self):
        """Connect to the ZMQ server."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.DEALER)
        
        # Set socket identity for the ROUTER on the server side
        identity_bytes = self.identity.encode('utf-8')
        self.socket.setsockopt(zmq.IDENTITY, identity_bytes)
        logger.debug(f"Setting socket identity: {self.identity}")
        
        # Set timeout
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        # Connect to server
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to ZMQ REST API at {self.host}:{self.port} with identity {self.identity}")
        
    def disconnect(self):
        """Disconnect from the ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None
            
    def request(self, method, route, params=None, query=None, body=None, headers=None):
        """
        Send a REST-like request over ZMQ.
        
        Args:
            method: HTTP method (GET, POST, etc.)
            route: API route
            params: Path parameters
            query: Query parameters
            body: Request body
            headers: Request headers
            
        Returns:
            Response body
        """
        if not self.socket:
            self.connect()
            
        # Create request
        self.request_id += 1
        request = {
            'id': self.request_id,
            'method': method.upper(),  # Ensure method is uppercase
            'route': route,
            'timestamp': int(time.time() * 1000)
        }
        
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
            
        # Debug log the request
        logger.debug(f"ZMQ REST Request: {json.dumps(request, indent=2)}")
            
        # Serialize and send
        request_bytes = json.dumps(request).encode('utf-8')
        try:
            # VERY IMPORTANT: For DEALER/ROUTER pattern, we send:
            # [identity(implicit), empty_delimiter, payload]
            # The identity is automatically handled by ZMQ based on our socket identity
            # The empty delimiter is required to tell the ROUTER socket where the
            # identity part ends and the payload begins
            logger.debug(f"Sending request to {self.host}:{self.port} (size: {len(request_bytes)} bytes)")
            
            # Send message with proper framing - just add the empty delimiter
            # NOTE: No structure headers or binary encoding for REST API
            self.socket.send_multipart([b"", request_bytes])
            
            # Receive response
            logger.debug(f"Waiting for response from {self.host}:{self.port}")
            
            # Receive using the same framing pattern
            # For DEALER/ROUTER, we'll get [empty_delimiter, payload]
            response_parts = self.socket.recv_multipart()
            
            # Verify we got at least two parts (the empty delimiter and the payload)
            if len(response_parts) < 2:
                logger.error(f"Invalid response format: received {len(response_parts)} parts")
                raise ValueError("Invalid response format")
                
            # The payload is the second part
            response_bytes = response_parts[1]
            logger.debug(f"Received {len(response_bytes)} bytes response")
            
            # Debug - print the raw bytes
            logger.debug(f"Raw response bytes: {response_bytes!r}")
            
            try:
                # Parse JSON response
                response = json.loads(response_bytes.decode('utf-8'))
                logger.debug(f"Received response: {json.dumps(response, indent=2)}")
                
                # Check for error
                if response.get('status', 200) >= 400:
                    error = response.get('body', {})
                    message = error.get('message', 'Unknown error')
                    code = error.get('code', 'ERROR')
                    logger.error(f"Error response: {code}: {message}")
                    raise RuntimeError(f"{code}: {message}")
                
                # Return response body
                return response.get('body', {})
            except json.JSONDecodeError:
                # If we're on port 5555, this might be a REQ/REP socket
                # It might need a different message format
                logger.warning(f"Could not parse response as JSON, trying REQ/REP format")
                # Just return the raw bytes for now
                return {"status": "raw_response", "data": str(response_bytes)}
            
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                logger.error(f"Request timed out: {method} {route}")
                raise TimeoutError("Request timed out")
            else:
                logger.error(f"ZMQ error: {e}")
                raise
    
    def get(self, route, params=None, query=None, headers=None):
        """Send a GET request."""
        return self.request('GET', route, params, query, None, headers)
    
    def post(self, route, body, params=None, query=None, headers=None):
        """Send a POST request."""
        return self.request('POST', route, params, query, body, headers)


class SimpleZmqSensoryClient:
    """
    Simple ZMQ client for sending sensory data to FEAGI.
    
    This class provides a minimal implementation to send sensory data
    over ZMQ using the feagi_bytes binary protocol.
    """
    
    def __init__(self, host="localhost", port=5558, identity=None):
        """
        Initialize the sensory client.
        
        Args:
            host: Host address
            port: ZMQ sensorimotor port
            identity: Optional client identity
        """
        self.host = host
        self.port = port
        self.context = zmq.Context.instance()
        self.socket = None
        self.agent_id = f"sensory-test-{int(time.time())}"
        self.identity = identity or f"sensory-client-{int(time.time())}"
        
        # Initialize FEAGI byte structure tools
        self.encoder = ByteStructureEncoder()
        self.translator = ByteStructureTranslator()
        
    def connect(self):
        """Connect to the ZMQ server."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.DEALER)
        
        # Set socket identity for the ROUTER on the server side
        identity_bytes = self.identity.encode('utf-8')
        self.socket.setsockopt(zmq.IDENTITY, identity_bytes)
        logger.debug(f"Setting sensory socket identity: {self.identity}")
        
        # Connect to server
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to ZMQ sensorimotor at {self.host}:{self.port} with identity {self.identity}")
        
    def disconnect(self):
        """Disconnect from the ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None
            
    def send_sensory_data(self, coordinates, cortical_id=None):
        """
        Send sensory data to FEAGI.
        
        Args:
            coordinates: List of [x,y,z] coordinates to activate
            cortical_id: Cortical area ID to target
            
        Returns:
            True if sent successfully
        """
        if not self.socket:
            self.connect()
            
        try:
            if not cortical_id:
                logger.warning("No cortical ID provided, using default")
                cortical_id = "default"
                
            # Create a simple dictionary with the data
            # This matches what FEAGI expects for sensory input
            data_dict = {
                "agent_id": self.agent_id,
                "coordinates": coordinates,
                "cortical_id": cortical_id,
                "timestamp": int(time.time() * 1000)
            }
            
            # Serialize to JSON directly (no byte structure header)
            # This avoids the "Unknown structure type" error
            json_data = json.dumps(data_dict).encode('utf-8')
            
            # Send with proper framing for DEALER/ROUTER
            # VERY IMPORTANT: The empty first frame is critical for ROUTER sockets
            logger.debug(f"Sending {len(coordinates)} sensory activations to {cortical_id}")
            self.socket.send_multipart([b"", json_data])  # [empty_delimiter, payload]
            
            logger.info(f"Sent {len(coordinates)} activations to {cortical_id} via sensorimotor channel")
            return True
            
        except Exception as e:
            logger.error(f"Error sending sensory data: {e}")
            return False
            

async def generate_random_pattern(size, dimensions):
    """Generate a random pattern of neurons for stimulation."""
    pattern = []
    
    # Get dimensions
    x_max = dimensions.get('x', 10)
    y_max = dimensions.get('y', 10)
    
    # Generate a small cluster of neurons for a coherent pattern
    base_x = random.randint(0, max(0, x_max - 4))
    base_y = random.randint(0, max(0, y_max - 4))
    
    for _ in range(size):
        # Use a smaller area to create a cluster effect
        x = min(x_max - 1, max(0, base_x + random.randint(-2, 2)))
        y = min(y_max - 1, max(0, base_y + random.randint(-2, 2)))
        z = 0  # Usually sensory input is on z=0
        
        # Avoid duplicates
        coord = [x, y, z]
        if coord not in pattern:
            pattern.append(coord)
    
    return pattern


class SimpleZmqReqClient:
    """
    Simple ZMQ REQ/REP client for FEAGI.
    
    This class provides a minimal implementation to send requests
    using the standard REQ/REP pattern.
    """
    
    def __init__(self, host="localhost", port=5555, timeout=ZMQ_TIMEOUT):
        """
        Initialize the ZMQ REQ client.
        
        Args:
            host: Host address
            port: ZMQ REQ port
            timeout: Request timeout in milliseconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.request_id = 0
        
    def connect(self):
        """Connect to the ZMQ server."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.REQ)
        
        # Set timeout
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        # Connect to server
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to ZMQ REQ/REP at {self.host}:{self.port}")
        
    def disconnect(self):
        """Disconnect from the ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None
            
    def request(self, method, route, params=None, query=None, body=None, headers=None):
        """
        Send a request over standard REQ/REP.
        
        Args:
            method: HTTP method (GET, POST, etc.)
            route: API route
            params: Path parameters
            query: Query parameters
            body: Request body
            headers: Request headers
            
        Returns:
            Response body
        """
        if not self.socket:
            self.connect()
            
        # Create request
        self.request_id += 1
        request = {
            'id': self.request_id,
            'method': method.upper(),  # Ensure method is uppercase
            'route': route,
            'timestamp': int(time.time() * 1000)
        }
        
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
            
        # Debug log the request
        logger.debug(f"ZMQ REQ Request: {json.dumps(request, indent=2)}")
            
        # Serialize and send
        request_bytes = json.dumps(request).encode('utf-8')
        try:
            # Send with standard REQ socket (no multipart needed)
            logger.debug(f"Sending request to {self.host}:{self.port} (size: {len(request_bytes)} bytes)")
            self.socket.send(request_bytes)
            
            # Receive response
            logger.debug(f"Waiting for response from {self.host}:{self.port}")
            response_bytes = self.socket.recv()
            
            logger.debug(f"Received {len(response_bytes)} bytes response")
            logger.debug(f"Raw response bytes: {response_bytes!r}")
            
            # Try to parse as JSON
            try:
                response = json.loads(response_bytes.decode('utf-8'))
                logger.debug(f"Parsed JSON response: {json.dumps(response, indent=2)}")
                
                # Check for error
                if response.get('status', 200) >= 400:
                    error = response.get('body', {})
                    message = error.get('message', 'Unknown error')
                    code = error.get('code', 'ERROR')
                    logger.error(f"Error response: {code}: {message}")
                    raise RuntimeError(f"{code}: {message}")
                
                # Return response body
                return response.get('body', {})
            except json.JSONDecodeError:
                # If not valid JSON, just return the raw response
                logger.warning(f"Response is not valid JSON")
                return {"raw_response": response_bytes.decode('utf-8', errors='replace')}
            
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                logger.error(f"Request timed out: {method} {route}")
                raise TimeoutError("Request timed out")
            else:
                logger.error(f"ZMQ error: {e}")
                raise
    
    def get(self, route, params=None, query=None, headers=None):
        """Send a GET request."""
        return self.request('GET', route, params, query, None, headers)
    
    def post(self, route, body, params=None, query=None, headers=None):
        """Send a POST request."""
        return self.request('POST', route, params, query, body, headers)


async def run_test():
    """Run the complete test."""
    # For the REST API, let's use the REQ/REP pattern on port 5555 (confirmed working)
    logger.info(f"FEAGI host: {DEFAULT_HOST}")
    logger.info(f"Using ZMQ REQ/REP port: {DEFAULT_ZMQ_REST_PORT}")
    logger.info(f"Sensorimotor port: {DEFAULT_ZMQ_SENSORY_PORT}")
    
    # Create clients
    rest_client = SimpleZmqReqClient(
        host=DEFAULT_HOST,
        port=DEFAULT_ZMQ_REST_PORT,
        timeout=ZMQ_TIMEOUT
    )
    
    sensory_client = SimpleZmqSensoryClient(
        host=DEFAULT_HOST,
        port=DEFAULT_ZMQ_SENSORY_PORT
    )
    
    try:
        # Connect clients
        rest_client.connect()
        sensory_client.connect()
        
        # Test connection with a simple health check
        try:
            logger.info("Testing connection with health check")
            health = rest_client.get('/v1/system/health_check')
            logger.info(f"Health check successful: {health}")
            
            # Try getting FEAGI status to ensure connection is working
            try:
                logger.info("Getting system status")
                status = rest_client.get('/v1/status')
                logger.info(f"FEAGI status: {json.dumps(status, indent=2)}")
            except Exception as e:
                logger.warning(f"Failed to get status, but connection is working: {e}")
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            rest_client.disconnect()
            sensory_client.disconnect()
            return False
        
        # Load genome if needed
        has_genome = False
        try:
            # Check if there are any cortical areas (genome loaded)
            logger.info("Checking for loaded genome")
            areas = rest_client.get('/v1/connectome/cortical_areas')
            has_genome = len(areas) > 0
            logger.info(f"Found {len(areas)} cortical areas")
        except Exception as e:
            logger.warning(f"Error checking for genome: {e}")
        
        if not has_genome:
            # Load the test genome
            logger.info(f"Loading test genome from {TEST_GENOME_PATH}")
            
            try:
                # Try POST first
                logger.info("Trying POST method for genome upload")
                
                # Read the genome data
                with open(TEST_GENOME_PATH, 'r') as f:
                    genome_data = json.load(f)
                
                # For REQ/REP pattern, we can't send very large messages
                # So let's split the genome into smaller pieces
                # First, just try sending a minimal genome structure
                minimal_genome = {
                    "genome_id": genome_data.get("genome_id", "test-genome"),
                    "name": genome_data.get("name", "Test Genome"),
                    "description": genome_data.get("description", "Test genome for FEAGI ZMQ testing"),
                    "version": genome_data.get("version", "0.1.0"),
                    "cortical_areas": {}
                }
                
                # Add just a few cortical areas for testing
                test_areas = ["iv00CC", "iv00TM"]  # Visual cortex areas
                for area_id in test_areas:
                    if area_id in genome_data.get("cortical_areas", {}):
                        minimal_genome["cortical_areas"][area_id] = genome_data["cortical_areas"][area_id]
                
                logger.info(f"Uploading minimal test genome with {len(minimal_genome['cortical_areas'])} areas")
                result = rest_client.post('/v1/genome/upload', minimal_genome)
                
                if 'message' in result and 'success' in result.get('message', '').lower():
                    logger.info("Genome loaded successfully with POST")
                    genome_loaded = True
                else:
                    logger.warning(f"POST method failed: {result}")
                
                # If POST failed, try PUT
                if not genome_loaded:
                    try:
                        logger.info("Trying PUT method for genome upload")
                        result = rest_client.request('PUT', '/v1/genome/upload', body=genome_data)
                        
                        if 'message' in result and 'success' in result.get('message', '').lower():
                            logger.info("Genome loaded successfully with PUT")
                            genome_loaded = True
                        else:
                            logger.warning(f"PUT method failed: {result}")
                    except Exception as e:
                        logger.warning(f"PUT genome upload failed: {e}")
                
                # If both methods failed
                if not genome_loaded:
                    logger.error("Failed to load genome with either POST or PUT")
                    return False
                
                # Wait for the genome to be processed
                logger.info("Waiting for FEAGI to process the genome (5 seconds)")
                await asyncio.sleep(5)
                
            except Exception as e:
                logger.exception(f"Error loading genome: {e}")
                return False
        else:
            logger.info("FEAGI already has a genome loaded")
        
        # Get available cortical areas
        try:
            logger.info("Getting cortical areas")
            cortical_areas = rest_client.get('/v1/connectome/cortical_areas')
            if not cortical_areas:
                logger.error("No cortical areas found")
                return False
                
            logger.info(f"Found {len(cortical_areas)} cortical areas: {[area.get('id') for area in cortical_areas]}")
        except Exception as e:
            logger.exception(f"Error getting cortical areas: {e}")
            return False
        
        # Find the target cortical area (visual cortex)
        target_area = None
        for area in cortical_areas:
            if area.get('id') == VISUAL_CORTICAL_AREA:
                target_area = area
                logger.info(f"Found target cortical area: {VISUAL_CORTICAL_AREA}")
                break
        
        if not target_area:
            logger.warning(f"Target cortical area '{VISUAL_CORTICAL_AREA}' not found")
            
            # If our target wasn't found but we have other areas, use the first one
            if cortical_areas:
                target_area = cortical_areas[0]
                logger.info(f"Using alternate cortical area: {target_area.get('id')}")
            else:
                logger.error("No usable cortical areas found")
                return False
        
        # Extract dimensions
        dimensions = target_area.get('dimensions', {'x': 10, 'y': 10, 'z': 1})
        cortical_id = target_area.get('id')
        logger.info(f"Using cortical area '{cortical_id}' with dimensions {dimensions}")
        
        # Generate and send patterns
        logger.info(f"Generating and sending {NUM_PATTERNS} patterns")
        for i in range(NUM_PATTERNS):
            # Generate a random pattern
            pattern = await generate_random_pattern(PATTERN_SIZE, dimensions)
            
            # Send to FEAGI using both methods
            try:
                # Method 1: REST API direct stimulation
                logger.debug(f"Sending pattern {i+1} via REST API to {cortical_id}")
                rest_client.post(
                    f'/v1/connectome/cortical_area/{cortical_id}/stimulate',
                    {"coordinates": pattern},
                    params={'cortical_id': cortical_id}
                )
                logger.info(f"Pattern {i+1}/{NUM_PATTERNS} sent via REST API")
                
                # Method 2: Sensorimotor stream
                logger.debug(f"Sending pattern {i+1} via sensorimotor to {cortical_id}")
                sensory_client.send_sensory_data(pattern, cortical_id)
                
                # Delay between patterns
                await asyncio.sleep(DELAY_BETWEEN_PATTERNS)
                
            except Exception as e:
                logger.warning(f"Error sending pattern {i+1}: {e}")
                continue
        
        logger.info("Test completed successfully")
        return True
        
    except Exception as e:
        logger.exception(f"Error during test: {e}")
        return False
    finally:
        # Disconnect clients
        rest_client.disconnect()
        sensory_client.disconnect()
        logger.info("Disconnected from FEAGI")


if __name__ == "__main__":
    print("Starting FEAGI Sensory Injection Test")
    try:
        asyncio.run(run_test())
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Unhandled error: {e}")
    print("Test completed") 