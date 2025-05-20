#!/usr/bin/env python3
"""
Comprehensive test for FEAGI's command client (REQ/REP on port 5555)
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
logger = logging.getLogger("command_client_test")

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
    
    def make_request(self, command: str, params=None) -> dict:
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
            logger.debug(f"Request data: {json.dumps(request, indent=2)}")
            
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
                        logger.debug(f"Response data: {json.dumps(response, indent=2)}")
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
    def ping(self) -> dict:
        """Send a ping command."""
        return self.make_request("ping")
    
    def get_status(self) -> dict:
        """Get the simulation status."""
        return self.make_request("get_status")
    
    def get_configuration(self) -> dict:
        """Get the FEAGI configuration."""
        return self.make_request("get_configuration")
    
    def get_performance(self) -> dict:
        """Get performance metrics."""
        return self.make_request("get_performance")

def test_command_client():
    """Run a comprehensive test of the command client."""
    client = FeagiCommandClient()
    
    # Test ping
    logger.info("Testing ping command...")
    response = client.ping()
    if "pong" in response and response["pong"] is True:
        logger.info("Ping test passed!")
    else:
        logger.error(f"Ping test failed: {response}")
    
    # Test get_status
    logger.info("Testing get_status command...")
    response = client.get_status()
    if "status" in response and isinstance(response["status"], dict):
        logger.info(f"Status test passed! Status: {response['status']}")
    else:
        logger.error(f"Status test failed: {response}")
    
    # Test get_configuration (may not be supported)
    logger.info("Testing get_configuration command...")
    response = client.get_configuration()
    logger.info(f"Configuration response: {response}")
    
    # Test get_performance (may not be supported)
    logger.info("Testing get_performance command...")
    response = client.get_performance()
    logger.info(f"Performance response: {response}")
    
    # Test invalid command
    logger.info("Testing invalid command...")
    response = client.make_request("nonexistent_command")
    if "error" in response:
        logger.info(f"Invalid command test passed! Error: {response['error']}")
    else:
        logger.error(f"Invalid command test failed: {response}")

if __name__ == "__main__":
    logger.info("Starting FEAGI command client test")
    test_command_client()
    logger.info("Test complete") 