#!/usr/bin/env python3
"""
FEAGI Connector Neuron Processing Example - Rust Implementation

This example explicitly demonstrates the Rust implementation for processing
neuron data in FEAGI Connector. It will raise ImportError if the Rust
implementation is not available.
"""

import asyncio
import logging
import time
import sys
from feagi_connector import FeagiClient
from feagi_connector.utils import NEURON_POTENTIAL_CATEGORICAL_XYZ
from feagi_connector.protocols import FSMPChannel

# Explicitly import Rust implementations - will raise ImportError if not available
from feagi_connector.utils.rust_processing import (
    encode_neuron_potential_xyz_rust,
    decode_neuron_potential_xyz_rust
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


def demonstrate_rust_implementation(neuron_data):
    """Demonstrate the Rust implementation"""
    print("\n===== RUST IMPLEMENTATION =====")
    
    # Encode using Rust
    start_time = time.time()
    encoded_bytes = encode_neuron_potential_xyz_rust(neuron_data)
    encode_time = time.time() - start_time
    
    print(f"Encoded to {len(encoded_bytes)} bytes in {encode_time:.6f} seconds")
    
    # Decode using Rust
    start_time = time.time()
    decoded_data = decode_neuron_potential_xyz_rust(encoded_bytes)
    decode_time = time.time() - start_time
    
    print(f"Decoded in {decode_time:.6f} seconds")
    print_neuron_data(decoded_data, "Decoded with Rust")
    
    return encoded_bytes, encode_time, decode_time


def benchmark_rust(neuron_data, iterations=1000):
    """Benchmark Rust implementation"""
    print("\n===== RUST BENCHMARK =====")
    
    # Benchmark Rust encoding
    start_time = time.time()
    for _ in range(iterations):
        encode_neuron_potential_xyz_rust(neuron_data)
    rust_encode_time = time.time() - start_time
    
    # Benchmark Rust decoding
    encoded = encode_neuron_potential_xyz_rust(neuron_data)
    start_time = time.time()
    for _ in range(iterations):
        decode_neuron_potential_xyz_rust(encoded)
    rust_decode_time = time.time() - start_time
    
    print(f"Rust encoding: {rust_encode_time:.6f} seconds for {iterations} iterations")
    print(f"Rust decoding: {rust_decode_time:.6f} seconds for {iterations} iterations")


async def feagi_integration_demo(encoded_bytes):
    """Demonstrate integration with FEAGI"""
    print("\n===== FEAGI INTEGRATION DEMO =====")
    
    try:
        # Create FEAGI client
        client = FeagiClient(
            host="localhost",  # Change to your FEAGI host if needed
            agent_id="neuron-example-rust",
            agent_type="example-rust",
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


async def main_rust():
    """Main function for Rust implementation demo"""
    print("==================================================")
    print("  FEAGI CONNECTOR - RUST IMPLEMENTATION DEMO")
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
    
    # Demonstrate Rust implementation
    rust_encoded, rust_encode_time, rust_decode_time = demonstrate_rust_implementation(neuron_data)
    
    # Run Rust benchmarks
    benchmark_rust(neuron_data)
    
    # Verify byte type
    assert rust_encoded[0] == NEURON_POTENTIAL_CATEGORICAL_XYZ
    print(f"\nVerified byte structure type: {rust_encoded[0]}")
    
    # Run the FEAGI integration demo
    await feagi_integration_demo(rust_encoded)


if __name__ == "__main__":
    try:
        asyncio.run(main_rust())
    except ImportError as e:
        print("\n==================================================")
        print("  ERROR: Rust implementation not available")
        print("==================================================")
        print(f"Import error: {e}")
        print("\nTo use this example, install FEAGI Connector with Rust support:")
        print("pip install \"feagi_connector[rust]\"")
        sys.exit(1) 