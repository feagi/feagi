#!/usr/bin/env python3
"""
Test script for FCL Burst Engine with Visualization

This script creates a test FEAGI instance with simulated neural activity and
connects it to the visualization stream for testing purposes.
"""

import os
import sys
import time
import asyncio
import threading
import logging
from queue import Queue
from typing import Dict, Any, List, Set
import zmq

# Add the parent directory to the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import FEAGI components
from feagi.npu.burst_engine import BurstEngine, FCLSampler
from feagi.npu.fcl_manager import FCLManager, BitMap
from feagi.api.zmq.streams.visualization import VisualizationStream
from feagi.api.core.service import CoreApiService
from feagi.utils.logger import setup_logger

# Set up logging
logger = setup_logger()
logger.setLevel(logging.INFO)
logger.info("Starting FCL Burst Engine test with visualization")

# Create a mock connectome manager for testing
class MockConnectomeManager:
    def __init__(self):
        self.fcl_manager = FCLManager(window_size=10)
        self.cortical_areas = {
            'test_area1': MockCorticalArea('test_area1'),
            'test_area2': MockCorticalArea('test_area2'),
            'test_area3': MockCorticalArea('test_area3'),
        }
        self.neuron_positions = {}
        self.neuron_properties = {}
        
        # Initialize some test neurons in each area
        self._setup_test_neurons()
    
    def _setup_test_neurons(self):
        # Create test neurons for each area
        neuron_id_counter = 1
        for area_id, area in self.cortical_areas.items():
            for i in range(20):
                neuron_id = neuron_id_counter
                neuron_id_counter += 1
                
                # Add positions
                x = i % 5
                y = i // 5
                z = 0
                self.neuron_positions[neuron_id] = (x, y, z)
                
                # Add properties
                self.neuron_properties[neuron_id] = {
                    'membrane_potential': 0.8,
                    'area_id': area_id
                }
        
    def update_membrane_potentials(self):
        # Simulate neuron firing by creating random activations
        import random
        
        # Get a random subset of neurons for each area
        fired_neurons = []
        neurons_by_cortical = {}
        
        for area_id, area in self.cortical_areas.items():
            area_neurons = [n for n in self.neuron_positions.keys() 
                          if self.neuron_properties[n]['area_id'] == area_id]
            
            # Randomly select 5-10 neurons to fire in this area
            num_to_fire = random.randint(5, 10)
            if area_neurons:
                area_fired = random.sample(area_neurons, min(num_to_fire, len(area_neurons)))
                fired_neurons.extend(area_fired)
                neurons_by_cortical[area_id] = area_fired
        
        # Update the FCL manager with these neurons
        self.fcl_manager.update_fcl(
            current_timestep=self.fcl_manager.current_timestep + 1,
            neurons_by_cortical=neurons_by_cortical
        )
        
        logger.info(f"Fired {len(fired_neurons)} neurons across {len(neurons_by_cortical)} areas")
        return fired_neurons
    
    def get_neuron_position(self, neuron_id):
        # Return the neuron's position or a default if not found
        return self.neuron_positions.get(neuron_id, (0, 0, 0))
    
    def get_neuron_property(self, neuron_id, property_name, default=None):
        # Return the requested property or default if not found
        if neuron_id in self.neuron_properties:
            return self.neuron_properties[neuron_id].get(property_name, default)
        return default

# Mock cortical area for testing
class MockCorticalArea:
    def __init__(self, area_id):
        self.id = area_id
        self.properties = {
            'fcl_sample_rate': 10.0  # Sample rate in Hz
        }

# Mock core API service for testing
class MockCoreApiService(CoreApiService):
    def __init__(self):
        self.genome_loaded_flag = True
        self.genome_change_listeners = []
    
    def genome_is_loaded(self):
        return self.genome_loaded_flag
    
    def register_genome_change_listener(self, callback):
        self.genome_change_listeners.append(callback)

async def run_test():
    """Run the test with a mock FEAGI instance and visualization."""
    logger.info("Initializing test components")
    
    # Create mock components
    connectome_manager = MockConnectomeManager()
    core_api = MockCoreApiService()
    
    # Create a queue for FCL data
    fcl_queue = Queue()
    
    # Create the burst engine
    burst_engine = BurstEngine(
        connectome_manager=connectome_manager,
        fcl_manager=connectome_manager.fcl_manager,
        config={"desired_frequency_hz": 5.0}  # 5 Hz for testing
    )
    
    # Create the FCL sampler
    fcl_sampler = FCLSampler(
        fcl_manager=connectome_manager.fcl_manager,
        sample_frequency_hz=10.0,
        output_queue=fcl_queue,
        connectome_manager=connectome_manager
    )
    
    # Create the visualization stream
    viz_stream = VisualizationStream(
        core_api=core_api,
        host="*",  # Allow connections from any interface
        port=5571,  # Use a different port to avoid conflicts
        fcl_sampler=fcl_sampler,
        fcl_sampler_queue=fcl_queue
    )
    
    # Start the visualization stream
    logger.info("Starting visualization stream")
    await viz_stream.start()
    
    # Start the FCL sampler in a separate thread
    logger.info("Starting FCL sampler")
    sampler_thread = threading.Thread(target=fcl_sampler.run)
    sampler_thread.daemon = True
    sampler_thread.start()
    
    # Make sure visualization subscribers are enabled
    fcl_sampler.set_visualization_subscribers(True)
    
    # Monkey-patch the send_visualization_data method to use the "activity" topic
    # This is needed because the test_viz_agent.py is subscribing to "activity"
    original_send_viz_data = viz_stream.send_visualization_data
    
    async def patched_send_viz_data(data: bytes) -> None:
        """Patched version that sends on 'activity' topic instead of 'fcl'"""
        if not viz_stream.running or not viz_stream.socket:
            return
            
        try:
            # Send data on the "activity" topic that test_viz_agent.py is expecting
            topic = b"activity"
            print(f"SENDING VISUALIZATION DATA ON 'activity' TOPIC: {len(data)} bytes")
            await viz_stream.socket.send_multipart([topic, data])
            print(f"VISUALIZATION STREAM: Sent {len(data)} bytes on 'activity' topic")
        except Exception as e:
            logger.error(f"Error in patched send_visualization_data: {e}")
    
    # Replace the method with our patched version
    viz_stream.send_visualization_data = patched_send_viz_data
    
    # Send periodic welcome messages to help clients detect the server
    async def send_welcome_messages():
        """Periodically send welcome messages to help clients connect"""
        while viz_stream.running:
            try:
                welcome_msg = f"FEAGI_WELCOME:{time.time()}"
                print(f"VISUALIZATION STREAM: Sending welcome message: {welcome_msg}")
                await viz_stream.socket.send_multipart([
                    b"system",
                    welcome_msg.encode('utf-8')
                ])
                
                # Also send a test activity message
                test_data = bytes([11, 1, 0, 1, 0, 0, 0, 1, 116, 101, 115, 116, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                await viz_stream.socket.send_multipart([
                    b"activity",
                    test_data
                ])
                print(f"VISUALIZATION STREAM: Sent test activity data")
                
                # Let's also send a detailed debug info message about our socket
                if hasattr(viz_stream.socket, 'getsockopt'):
                    try:
                        debug_info = {
                            "type": "PUB",
                            "bound_to": f"tcp://*:{port}",
                            "linger": viz_stream.socket.getsockopt(zmq.LINGER),
                            "hwm": viz_stream.socket.getsockopt(zmq.SNDHWM),
                            "time": time.time()
                        }
                        debug_msg = f"SOCKET_DEBUG:{str(debug_info)}"
                        await viz_stream.socket.send_multipart([
                            b"system",
                            debug_msg.encode('utf-8')
                        ])
                        print(f"VISUALIZATION STREAM: Sent socket debug info")
                    except Exception as e:
                        print(f"VISUALIZATION STREAM: Error getting socket debug info: {e}")
                
            except Exception as e:
                print(f"VISUALIZATION STREAM: Error sending welcome message: {e}")
            await asyncio.sleep(2.0)  # Send every 2 seconds
    
    # Start welcome message task
    welcome_task = asyncio.create_task(send_welcome_messages())
    
    # Run the burst engine for some cycles
    logger.info("Running burst engine for 60 seconds...")
    
    # Run for 60 seconds
    end_time = time.time() + 60
    while time.time() < end_time:
        # Process one burst cycle
        burst_engine.run_test()
        
        # Wait a bit between bursts
        await asyncio.sleep(0.2)  # 5 Hz
        
        # Print queue status occasionally
        if int(time.time()) % 5 == 0:
            if hasattr(fcl_queue, 'qsize'):
                logger.info(f"FCL queue size: {fcl_queue.qsize()}")
    
    # Clean up
    logger.info("Test complete, stopping components")
    fcl_sampler.stop()
    await viz_stream.stop()
    
    logger.info("All components stopped")

if __name__ == "__main__":
    # Run the test
    asyncio.run(run_test()) 