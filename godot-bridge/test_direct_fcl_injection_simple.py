"""
Simplified test script for direct FCL injection into FEAGI.

This script uses ZMQ to send data directly to FEAGI's FCL manager,
and also checks if the genome is loaded and contains the target cortical area.
"""

import json
import time
import zmq
import random
from typing import List, Tuple, Dict, Any, Optional

# FEAGI ZMQ configuration
FEAGI_HOST = "127.0.0.1"
SENSORIMOTOR_PORT = 5558
CONTROL_PORT = 5555  # For control/query messages

# Use a cortical area that should exist - adjust as needed
CORTICAL_AREA = "iv00CC"  # A common visual cortical area name

# Pattern configuration
PATTERN_SIZE = (10, 10, 1)  # 3D size of the cortical area


def check_genome_status(socket) -> bool:
    """Check if FEAGI has a genome loaded using ZMQ control channel."""
    # Create a status request message
    request = {
        "type": "query",
        "target": "status",
        "timestamp": int(time.time() * 1000)
    }
    
    # Convert to JSON and send
    json_data = json.dumps(request).encode('utf-8')
    # Try different message format based on the error logs
    socket.send_multipart([json_data])
    print("Sent status request")
    
    # Wait for response with timeout
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    if poller.poll(5000):  # 5 second timeout
        # Get response
        response_frames = socket.recv_multipart()
        if len(response_frames) < 1:
            print("Invalid response format")
            return False
            
        try:
            # Parse the single frame response
            payload = response_frames[0]
            
            # Parse JSON response
            response = json.loads(payload.decode('utf-8'))
            
            # Check genome status
            if 'genome_availability' in response and response['genome_availability']:
                if 'genome_validity' in response and response['genome_validity']:
                    if 'brain_readiness' in response and response['brain_readiness']:
                        print("FEAGI has a valid genome loaded and brain is ready")
                        return True
            
            print(f"Genome status: {response.get('genome_availability', False)}")
            print(f"Genome validity: {response.get('genome_validity', False)}")
            print(f"Brain readiness: {response.get('brain_readiness', False)}")
            return False
                
        except Exception as e:
            print(f"Error parsing response: {e}")
            return False
    else:
        print("Timeout waiting for status response")
        return False


def check_cortical_area(socket, cortical_area: str) -> bool:
    """Check if the specified cortical area exists in the genome using ZMQ control channel."""
    # Create a genome blueprint request
    request = {
        "type": "query",
        "target": "genome_blueprint",
        "timestamp": int(time.time() * 1000)
    }
    
    # Convert to JSON and send
    json_data = json.dumps(request).encode('utf-8')
    # Try different message format based on the error logs
    socket.send_multipart([json_data])
    print("Sent genome blueprint request")
    
    # Wait for response with timeout
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    if poller.poll(5000):  # 5 second timeout
        # Get response
        response_frames = socket.recv_multipart()
        if len(response_frames) < 1:
            print("Invalid response format")
            return False
            
        try:
            # Parse the single frame response
            payload = response_frames[0]
            
            # Parse JSON response
            blueprint = json.loads(payload.decode('utf-8'))
            
            # Check if cortical area exists
            if cortical_area in blueprint:
                dims = blueprint.get(cortical_area, {}).get('coordinates', {}).get('dimension', [])
                if dims:
                    global PATTERN_SIZE
                    PATTERN_SIZE = tuple(dims)
                    print(f"Found cortical area '{cortical_area}' with dimensions {PATTERN_SIZE}")
                    return True
                else:
                    print(f"Cortical area '{cortical_area}' exists but dimensions not found")
                    return False
            else:
                print(f"Cortical area '{cortical_area}' not found in genome blueprint")
                return False
                
        except Exception as e:
            print(f"Error parsing response: {e}")
            return False
    else:
        print("Timeout waiting for blueprint response")
        return False


def create_fcl_data() -> List[List[int]]:
    """Create a simple pattern of FCL data."""
    x_max, y_max, z_max = PATTERN_SIZE
    
    # Create a simple square pattern
    fcl_data = []
    
    # Fixed 4x4 square in the middle
    square_size = min(4, x_max - 1, y_max - 1)
    x_offset = max(0, x_max // 2 - square_size // 2)
    y_offset = max(0, y_max // 2 - square_size // 2)
    
    for dx in range(square_size):
        for dy in range(square_size):
            x = x_offset + dx
            y = y_offset + dy
            z = 0
            fcl_data.append([x, y, z])
    
    return fcl_data


def send_fcl_data(socket, cortical_area: str, coordinates: List[List[int]]):
    """Send FCL data directly to FEAGI."""
    # Format the message
    message = {
        "data": {
            "direct_stimulation": {
                cortical_area: coordinates
            }
        }
    }
    
    # Convert to JSON and send - use a simpler format
    json_data = json.dumps(message).encode('utf-8')
    socket.send(json_data)
    print(f"Sent {len(coordinates)} coordinates to {cortical_area}")


def main():
    """Main test function."""
    # Initialize ZMQ control socket for checks
    context = zmq.Context()
    control_socket = context.socket(zmq.REQ)  # Change from DEALER to REQ
    
    # Connect to FEAGI control channel
    print(f"Connecting to FEAGI control channel at {FEAGI_HOST}:{CONTROL_PORT}")
    control_socket.connect(f"tcp://{FEAGI_HOST}:{CONTROL_PORT}")
    
    # Check genome status and cortical area
    genome_ok = check_genome_status(control_socket)
    if not genome_ok:
        print("FEAGI doesn't have a valid genome loaded. Continuing anyway for testing.")
    
    cortical_ok = check_cortical_area(control_socket, CORTICAL_AREA)
    if not cortical_ok:
        print(f"Cortical area {CORTICAL_AREA} may not exist. Continuing anyway for testing.")
    
    # Clean up control socket, we don't need it anymore
    control_socket.close()
    
    # Initialize sensorimotor socket for data sending
    sensorimotor_socket = context.socket(zmq.DEALER)  # Keep this as DEALER
    
    # Connect to FEAGI sensorimotor channel
    print(f"Connecting to FEAGI sensorimotor channel at {FEAGI_HOST}:{SENSORIMOTOR_PORT}")
    sensorimotor_socket.connect(f"tcp://{FEAGI_HOST}:{SENSORIMOTOR_PORT}")
    
    try:
        # Send patterns in a loop
        for i in range(20):  # 20 iterations
            # Generate FCL data
            fcl_data = create_fcl_data()
            
            # Send to FEAGI
            send_fcl_data(sensorimotor_socket, CORTICAL_AREA, fcl_data)
            
            # Wait to see the effect
            time.sleep(0.5)
            
            # Print a progress indicator
            print(f"Iteration {i + 1}/20")
    
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        # Clean up
        sensorimotor_socket.close()
        context.term()
        print("Disconnected from FEAGI")


if __name__ == "__main__":
    main() 