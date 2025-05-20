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
import zmq
import zmq.asyncio
from typing import List, Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
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
DEFAULT_ZMQ_REST_PORT = 5560  # ZMQ REST API port
DEFAULT_ZMQ_SENSORY_PORT = 5558  # ZMQ Sensorimotor port
VISUAL_CORTICAL_AREA = "iv00CC"  # Common visual cortical area in test genomes
NUM_PATTERNS = 20  # Number of patterns to generate
PATTERN_SIZE = 16  # Number of neurons in each pattern
DELAY_BETWEEN_PATTERNS = 0.2  # Seconds
TEST_GENOME_PATH = "test_genome.json"  # Path to test genome file


class SimpleZmqRestClient:
    """
    Simple ZMQ REST API client for FEAGI.
    
    This class provides a minimal implementation to send REST-like requests
    over ZMQ DEALER/ROUTER sockets.
    """
    
    def __init__(self, host="localhost", port=5560, timeout=5000):
        """
        Initialize the ZMQ REST client.
        
        Args:
            host: Host address
            port: ZMQ REST port
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
            
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to ZMQ REST API at {self.host}:{self.port}")
        
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
            'method': method,
            'route': route,
            'timestamp': int(time.time() * 1000)
        }
        
        # Add optional components
        if params:
            request['params'] = params
        if query:
            request['query'] = query
        if body:
            request['body'] = body
        if headers:
            request['headers'] = headers
            
        # Serialize and send
        request_bytes = json.dumps(request).encode('utf-8')
        try:
            # Send multipart message with empty frame for DEALER/ROUTER pattern
            self.socket.send_multipart([b"", request_bytes])
            
            # Receive response
            response_parts = self.socket.recv_multipart()
            
            # Parse response (should be [empty_frame, payload])
            if len(response_parts) < 2:
                raise ValueError(f"Invalid response format: {response_parts}")
                
            # Get JSON payload
            response = json.loads(response_parts[1].decode('utf-8'))
            
            # Check for error
            if response.get('status', 200) >= 400:
                error = response.get('body', {})
                message = error.get('message', 'Unknown error')
                code = error.get('code', 'ERROR')
                raise RuntimeError(f"{code}: {message}")
            
            # Return response body
            return response.get('body', {})
            
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
    over ZMQ using the sensorimotor protocol.
    """
    
    def __init__(self, host="localhost", port=5558):
        """
        Initialize the sensory client.
        
        Args:
            host: Host address
            port: ZMQ sensorimotor port
        """
        self.host = host
        self.port = port
        self.context = zmq.Context.instance()
        self.socket = None
        self.agent_id = f"sensory-test-{int(time.time())}"
        
    def connect(self):
        """Connect to the ZMQ server."""
        if self.socket:
            return
            
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.connect(f"tcp://{self.host}:{self.port}")
        logger.info(f"Connected to ZMQ sensorimotor at {self.host}:{self.port}")
        
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
            cortical_id: Optional cortical area ID (only for REST API)
            
        Returns:
            True if sent successfully
        """
        if not self.socket:
            self.connect()
            
        try:
            # Format the data as JSON for simplicity
            data = {
                "agent_id": self.agent_id,
                "coordinates": coordinates,
                "timestamp": int(time.time() * 1000)
            }
            
            # Add cortical area if provided
            if cortical_id:
                data["cortical_id"] = cortical_id
                
            # Serialize and send
            data_bytes = json.dumps(data).encode('utf-8')
            self.socket.send_multipart([b"", data_bytes])
            
            logger.info(f"Sent {len(coordinates)} activations via ZMQ sensorimotor")
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


async def run_test():
    """Run the complete test."""
    # Create clients
    rest_client = SimpleZmqRestClient(
        host=DEFAULT_HOST,
        port=DEFAULT_ZMQ_REST_PORT
    )
    
    sensory_client = SimpleZmqSensoryClient(
        host=DEFAULT_HOST,
        port=DEFAULT_ZMQ_SENSORY_PORT
    )
    
    try:
        # Connect clients
        rest_client.connect()
        sensory_client.connect()
        
        # Load genome if needed
        has_genome = False
        try:
            # Check if there are any cortical areas (genome loaded)
            areas = rest_client.get('/v1/connectome/cortical_areas')
            has_genome = len(areas) > 0
            logger.info(f"Found {len(areas)} cortical areas")
        except Exception as e:
            logger.warning(f"Error checking for genome: {e}")
        
        if not has_genome:
            # Load the test genome
            logger.info(f"Loading test genome from {TEST_GENOME_PATH}")
            
            try:
                # Read the genome file
                with open(TEST_GENOME_PATH, 'r') as f:
                    genome_data = json.load(f)
                    
                # Send the genome to FEAGI using the REST API
                logger.info(f"Uploading genome to FEAGI ({len(json.dumps(genome_data))} bytes)")
                result = rest_client.post('/v1/genome/upload', genome_data)
                
                if 'message' in result and 'success' in result.get('message', '').lower():
                    logger.info("Genome loaded successfully")
                else:
                    logger.error(f"Failed to load genome: {result}")
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
        cortical_areas = rest_client.get('/v1/connectome/cortical_areas')
        if not cortical_areas:
            logger.error("No cortical areas found")
            return False
        
        # Find the target cortical area (visual cortex)
        target_area = None
        for area in cortical_areas:
            if area.get('id') == VISUAL_CORTICAL_AREA:
                target_area = area
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
                rest_client.post(
                    f'/v1/connectome/cortical_area/{cortical_id}/stimulate',
                    {"coordinates": pattern},
                    params={'cortical_id': cortical_id}
                )
                logger.info(f"Pattern {i+1}/{NUM_PATTERNS} sent via REST API")
                
                # Method 2: Sensorimotor stream
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