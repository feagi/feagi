#!/usr/bin/env python3
"""
Fixed ZMQ REQ/REP Client for FEAGI 2.1

This client correctly formats requests according to FEAGI's expected message format:
[auth_token, content_type, request_data]
"""

import sys
import os
import json
import time
import logging

# Add .venv to path to ensure we get the right zmq
venv_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), ".venv", "lib", "python3.9", "site-packages")
sys.path.insert(0, venv_path)

import zmq

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi_req_rep_fixed")

class FeagiReqRepFixedClient:
    """
    Correctly implemented ZMQ REQ/REP client for FEAGI 2.1.
    
    This client formats messages according to FEAGI's expected format:
    [auth_token, content_type, request_data]
    """
    
    def __init__(self, host="127.0.0.1", port=5555, timeout=5000, auth_token=None):
        """
        Initialize the REQ/REP client.
        
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
    
    def make_request(self, method, route, params=None, query=None, body=None, headers=None):
        """
        Make a REQ/REP request with proper message formatting.
        
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
        # Create a new socket for this request (important for REQ/REP state)
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout)
        
        try:
            # Connect to the server
            socket.connect(f"tcp://{self.host}:{self.port}")
            logger.info(f"Connected to {self.host}:{self.port}")
            
            # Create command-based request (FEAGI expects command field)
            command = None
            
            # Map REST route to command
            if route == '/v1/system/health_check':
                command = "ping"
            elif route == '/v1/status':
                command = "get_status"
            elif route == '/v1/system/configuration':
                command = "get_configuration"
            elif route == '/v1/system/performance':
                command = "get_performance"
            elif route.startswith('/v1/connectome/cortical_area'):
                if 'cortical_id' in params:
                    # Handle specific cortical area route
                    cortical_id = params['cortical_id']
                    command = f"get_cortical_area:{cortical_id}"
                else:
                    # It's the general cortical areas route
                    command = "get_status"  # Use status command to get cortical areas
            elif route == '/v1/genome/blueprint':
                command = "get_status"  # Blueprint is part of status
            elif route == '/v1/genome':
                command = "get_status"  # Full genome is part of status
            else:
                # If no direct mapping, use REST format in command
                command = f"{method.lower()}:{route}"
            
            # Build request with required command field
            request = {
                'command': command,
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
                
            # Serialize request data
            request_bytes = json.dumps(request).encode('utf-8')
            content_type = "application/json"
            auth_token_bytes = self.auth_token.encode('utf-8') if self.auth_token else b""
            
            logger.info(f"Sending command: {command} ({len(request_bytes)} bytes)")
            logger.debug(f"Request data: {json.dumps(request, indent=2)}")
            
            # Send request with correct format: [auth_token, content_type, request_data]
            socket.send_multipart([
                auth_token_bytes,              # Auth token (empty if none)
                content_type.encode('utf-8'),  # Content type
                request_bytes                  # Request data
            ])
            
            # Wait for response
            logger.info(f"Waiting for response...")
            try:
                response_parts = socket.recv_multipart()
                
                # FEAGI should respond with [content_type, response_data]
                if len(response_parts) < 2:
                    logger.warning(f"Unexpected response format - received {len(response_parts)} parts")
                    return {'error': 'Invalid response format'}
                
                # Parse the response
                resp_content_type = response_parts[0].decode('utf-8')
                resp_data = response_parts[1]
                
                logger.info(f"Received response: {len(resp_data)} bytes, content-type: {resp_content_type}")
                
                if resp_content_type == "application/json":
                    try:
                        # Parse JSON response
                        response = json.loads(resp_data.decode('utf-8'))
                        logger.debug(f"Response data: {json.dumps(response, indent=2)}")
                        return response
                    except json.JSONDecodeError:
                        logger.warning(f"Could not decode JSON response: {resp_data!r}")
                        return {'raw_response': resp_data.decode('utf-8', errors='replace')}
                else:
                    # Return raw response with content type
                    return {
                        'content_type': resp_content_type,
                        'raw_response': resp_data.decode('utf-8', errors='replace')
                    }
                
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
            logger.debug("Socket closed")
    
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


if __name__ == "__main__":
    logger.info("Testing fixed FEAGI REQ/REP client")
    
    client = FeagiReqRepFixedClient()
    
    # Test health check
    logger.info("Testing health check...")
    response = client.get('/v1/system/health_check')
    logger.info(f"Health check response: {response}")
    
    # Test system status
    logger.info("Testing system status...")
    response = client.get('/v1/status')
    logger.info(f"Status response: {response}")
    
    # Don't test cortical areas directly since it's not a separate command
    # Instead, get the full system status and print what's available
    logger.info("Testing system status with more details...")
    response = client.get('/v1/status', query={"include": "all"})
    
    # Print available data keys in the response
    if 'status' in response:
        logger.info(f"Status response contains: {list(response['status'].keys())}")
    else:
        logger.info(f"Status response: {response}")
    
    # Test genome 
    logger.info("Testing genome blueprint...")
    response = client.get('/v1/genome/blueprint')
    if 'error' not in response:
        logger.info("Successfully retrieved genome blueprint")
    else:
        logger.info(f"Genome blueprint response: {response}")
    
    logger.info("Testing complete") 