#!/usr/bin/env python3
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
FEAGI Connector Neuron Processing Example

This example demonstrates both Python and Rust implementations for processing 
neuron data in FEAGI Connector.
"""

import asyncio
import logging
import time
import sys
from feagi_connector import FeagiClient
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ
from feagi_connector.protocols import FSMPChannel

# Always import Python implementations
from feagi_connector.utils.processing import (
    encode_neuron_potential_xyz_python,
    decode_neuron_potential_xyz_python
)

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("example")


def print_neuron_data(neuron_data, label="Neuron Data"):
    """Print formatted neuron data"""
    print(f"\n{label}:")
    print("-" * (len(label) + 1))
    for (x, y, z), potential in sorted(neuron_data.items()):
        print(f"  Position ({x:3d}, {y:3d}, {z:3d}): {potential:.2f}")
    print()


def demonstrate_python_implementation(neuron_data):
    """Demonstrate the Python implementation"""
    print("\n===== PYTHON IMPLEMENTATION =====")
    
    # Encode using Python
    start_time = time.time()
    encoded_bytes = encode_neuron_potential_xyz_python(neuron_data)
    encode_time = time.time() - start_time
    
    print(f"Encoded to {len(encoded_bytes)} bytes in {encode_time:.6f} seconds")
    
    # Decode using Python
    start_time = time.time()
    decoded_data = decode_neuron_potential_xyz_python(encoded_bytes)
    decode_time = time.time() - start_time
    
    print(f"Decoded in {decode_time:.6f} seconds")
    print_neuron_data(decoded_data, "Decoded with Python")
    
    return encoded_bytes, encode_time, decode_time


def benchmark_python(neuron_data, iterations=1000):
    """Benchmark Python implementation"""
    print("\n===== PYTHON BENCHMARK =====")
    
    # Benchmark Python encoding
    start_time = time.time()
    for _ in range(iterations):
        encode_neuron_potential_xyz_python(neuron_data)
    py_encode_time = time.time() - start_time
    
    # Benchmark Python decoding
    encoded = encode_neuron_potential_xyz_python(neuron_data)
    start_time = time.time()
    for _ in range(iterations):
        decode_neuron_potential_xyz_python(encoded)
    py_decode_time = time.time() - start_time
    
    print(f"Python encoding: {py_encode_time:.6f} seconds for {iterations} iterations")
    print(f"Python decoding: {py_decode_time:.6f} seconds for {iterations} iterations")


async def feagi_integration_demo(encoded_bytes):
    """Demonstrate integration with FEAGI"""
    print("\n===== FEAGI INTEGRATION DEMO =====")
    
    try:
        # Create FEAGI client
        client = FeagiClient(
            host="localhost",  # Change to your FEAGI host if needed
            agent_id="neuron-example",
            agent_type="example",
            timeout=2000  # Short timeout for quick feedback
        )
        
        # Define motor callback
        async def motor_callback(channel_id, data):
            logger.info(f"Received motor data on channel {channel_id}")
            if isinstance(data, dict) and data.get("type") == "neuron_potential":
                print_neuron_data(data.get("neurons", {}), "Received motor neuron data")
        
        # Register callback
        await client.register_motor_callback(motor_callback)
        
        # Connect to FEAGI
        connected = await client.connect()
        if not connected:
            logger.warning("Could not connect to FEAGI. Demo running in offline mode only.")
            return
            
        logger.info("Connected to FEAGI")
        
        # Send the encoded neuron data
        await client.send_sensory_data(FSMPChannel.VISION, encoded_bytes)
        logger.info("Sent neuron data to FEAGI")
        
        # Wait for any motor responses
        logger.info("Waiting for motor responses (5 seconds)...")
        await asyncio.sleep(5)
        
        # Disconnect
        await client.disconnect()
        logger.info("Disconnected from FEAGI")
        
    except Exception as e:
        logger.error(f"Error in FEAGI integration: {e}")
        logger.info("Demo ran in offline mode only.")


async def main_python():
    """Main function for Python implementation demo"""
    print("==================================================")
    print("  FEAGI CONNECTOR - PYTHON IMPLEMENTATION DEMO")
    print("==================================================")
    
    # Create sample neuron data
    neuron_data = {
        (10, 20, 1): 1.0,   # Fully activated neuron
        (11, 20, 1): 0.75,  # Partially activated neuron
        (12, 20, 1): 0.5,   # Partially activated neuron
        (13, 20, 1): 0.25,  # Weakly activated neuron
        (14, 20, 1): 0.0,   # Inactive neuron
    }
    
    # Display original data
    print_neuron_data(neuron_data, "Original neuron data")
    
    # Demonstrate Python implementation
    py_encoded, py_encode_time, py_decode_time = demonstrate_python_implementation(neuron_data)
    
    # Run Python benchmarks
    benchmark_python(neuron_data)
    
    # Verify byte type
    assert py_encoded[0] == NEURON_POTENTIAL_CATEGORICAL_XYZ
    print(f"\nVerified byte structure type: {py_encoded[0]}")
    
    # Run the FEAGI integration demo
    await feagi_integration_demo(py_encoded)


if __name__ == "__main__":
    asyncio.run(main_python()) 