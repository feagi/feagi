#!/usr/bin/env python3
"""
REQ/REP ZMQ Test for FEAGI

This script properly tests the REQ/REP pattern with FEAGI's ZMQ port 5555
by ensuring we correctly alternate between sending and receiving.
"""

import sys
import os
import json
import time
import logging

# We need to ensure we're using the system's ZMQ, not our local module
# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("req_rep_test")

class FeagiReqRepClient:
    """
    Properly implemented REQ/REP ZMQ client for FEAGI.
    
    This class ensures proper handling of the REQ/REP pattern by:
    1. Creating a new socket for each request/response pair
    2. Alternating between sending and receiving
    3. Using proper timeout handling
    """
    
    def __init__(self, host="127.0.0.1", port=5555, timeout=5000):
        """Initialize the REQ/REP client."""
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.next_id = 1
        
    def make_request(self, method, route, params=None, query=None, body=None, headers=None):
        """
        Make a REQ/REP request, correctly handling socket creation and cleanup.
        
        Args:
            method: HTTP method (GET, POST, etc.)
            route: API route
            params: Path parameters
            query: Query parameters
            body: Request body
            headers: Request headers
            
        Returns:
            Response data
        """
        # Create a new socket for this request
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to the server
            socket.connect(f"tcp://{self.host}:{self.port}")
            logger.info(f"Connected to {self.host}:{self.port}")
            
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
            logger.info(f"Sending request: {method} {route} ({len(request_bytes)} bytes)")
            logger.debug(f"Request data: {json.dumps(request, indent=2)}")
            
            # Send request
            socket.send(request_bytes)
            
            # Wait for response
            logger.info(f"Waiting for response...")
            response_bytes = socket.recv()
            
            logger.info(f"Received response: {len(response_bytes)} bytes")
            
            try:
                # Try to parse as JSON
                response = json.loads(response_bytes.decode('utf-8'))
                logger.debug(f"Response data: {json.dumps(response, indent=2)}")
                return response
            except json.JSONDecodeError:
                # If we can't parse as JSON, just return the raw response
                logger.warning(f"Couldn't parse response as JSON")
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
            # Always close the socket
            socket.close()
    
    def get(self, route, params=None, query=None, headers=None):
        """Make a GET request."""
        return self.make_request('GET', route, params, query, None, headers)
    
    def post(self, route, body, params=None, query=None, headers=None):
        """Make a POST request."""
        return self.make_request('POST', route, params, query, body, headers)

def test_basic_requests():
    """Run basic tests against FEAGI's ZMQ REQ/REP API."""
    client = FeagiReqRepClient()
    
    # Test 1: Health check
    logger.info("TEST 1: Health check")
    response = client.get('/v1/system/health_check')
    logger.info(f"Health check response: {response}")
    
    # Test 2: Get system status
    logger.info("TEST 2: System status")
    response = client.get('/v1/status')
    logger.info(f"Status response: {response}")
    
    # Test 3: Check cortical areas
    logger.info("TEST 3: Get cortical areas")
    response = client.get('/v1/connectome/cortical_areas')
    logger.info(f"Found {len(response.get('body', []))} cortical areas")
    
    # Return list of areas for further tests
    return response.get('body', [])

def test_genome():
    """Test loading a minimal genome."""
    client = FeagiReqRepClient()
    
    # Create a minimal test genome
    minimal_genome = {
        "genome_id": f"test-genome-{int(time.time())}",
        "name": "Minimal Test Genome",
        "description": "Minimal test genome for FEAGI ZMQ testing",
        "version": "0.1.0",
        "cortical_areas": {
            "test1": {
                "name": "Test Area 1",
                "dimensions": {"x": 8, "y": 8, "z": 1},
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "group": "IPU"
            }
        }
    }
    
    # Try uploading
    logger.info("Testing genome upload")
    response = client.post('/v1/genome/upload', minimal_genome)
    logger.info(f"Genome upload response: {response}")
    
    return response

def test_cortical_stimulation():
    """Test stimulating a cortical area directly."""
    client = FeagiReqRepClient()
    
    # First, get available cortical areas
    areas_response = client.get('/v1/connectome/cortical_areas')
    areas = areas_response.get('body', [])
    
    if not areas:
        logger.warning("No cortical areas found, skipping stimulation test")
        return
    
    # Pick the first area
    target_area = areas[0]
    target_id = target_area.get('id')
    logger.info(f"Testing stimulation of cortical area: {target_id}")
    
    # Create some test coordinates
    coordinates = []
    for i in range(10):
        coordinates.append([i, i, 0])
    
    # Send stimulation
    stim_data = {"coordinates": coordinates}
    response = client.post(f'/v1/connectome/cortical_area/{target_id}/stimulate', stim_data)
    logger.info(f"Stimulation response: {response}")
    
    return response

if __name__ == "__main__":
    logger.info("Starting FEAGI REQ/REP ZMQ Test")
    
    try:
        # Test basic requests
        areas = test_basic_requests()
        
        # If no areas were found, test loading a genome
        if not areas:
            logger.info("No cortical areas found, testing genome loading")
            test_genome()
            
            # After loading, get areas again
            client = FeagiReqRepClient()
            areas_response = client.get('/v1/connectome/cortical_areas')
            areas = areas_response.get('body', [])
        
        # If we have areas now, test stimulation
        if areas:
            logger.info(f"Found {len(areas)} cortical areas, testing stimulation")
            test_cortical_stimulation()
    
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.error(f"Unhandled error: {e}")
    
    logger.info("Test completed") 