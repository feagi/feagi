"""
Copyright 2023-Present The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================

This module implements a bridge between FEAGI 2.0 and the Godot game engine.
It connects to FEAGI using the feagi_connector library and the ZMQ Visualization Stream,
processes neuron activity data, and forwards it to Godot for visualization.
"""

import os
import json
import asyncio
import threading
import logging
import time
from datetime import datetime
from collections import deque
from typing import Dict, List, Set, Any, Tuple, Optional, Union
import numpy as np

# Import FEAGI Connector - use the modern client instead of legacy modules
import sys
sys.path.append("..")  # Add parent directory to path
from feagi_connector.client import FeagiClient
from feagi_connector.zmq.client import ZmqFeagiClient  # Fixed class name

# Import local modules
from version import __version__
from network_configuration import websocket_operation, bridge_operation, feagi_to_brain_visualizer
from network_configuration import queue_of_recieve_godot_data, send_to_BV_queue, sleep
import godot_bridge_functions_new as bridge  # Updated to use the new file without OpenCV
from FEAGIByteStructures.JSONByteStructure import JSONByteStructure
from FEAGIByteStructures.SVORaymarchingByteStructure import SVORaymarchingByteStructure
from FEAGIByteStructures.MultiByteStructHolder import MultiByteStructHolder
from FEAGIByteStructures.SingleRawImage import SingleRawImage

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("feagi_godot_bridge")

# Runtime data structure
runtime_data = {
    "cortical_data": {},
    "stimulation_period": 0.01,
    "cortical_list": set(),
    "genome_number": 0,
    "connectome_data": {}
}


class GodotBridge:
    """
    Bridge between FEAGI 2.0 and Godot for neural activity visualization.
    
    This class connects to FEAGI's visualization stream, processes neuron
    activity data, and forwards it to Godot for visualization.
    """
    
    def __init__(self, config_file='configuration.json'):
        """
        Initialize the Godot Bridge.
        
        Args:
            config_file: Path to the configuration file
        """
        # Load configuration
        with open(config_file, 'r') as f:
            self.config = json.load(f)
            
        # Extract settings
        self.feagi_settings = self.config["feagi_settings"]
        self.agent_settings = self.config['agent_settings']
        
        # Override with environment variables if available
        self.feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', self.feagi_settings.get('feagi_host', "127.0.0.1"))
        self.feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', self.feagi_settings.get('feagi_api_port', "8000"))
        self.agent_settings['godot_websocket_port'] = os.environ.get('WS_BRIDGE_PORT', self.agent_settings.get('godot_websocket_port', "9050"))
        
        # Capabilities for FEAGI registration
        self.capabilities = {}
        
        # State tracking
        self.running = False
        self.previous_genome_timestamp = 0
        self.current_genome_number = 0
        self.current_register_number = 0
        
        # FEAGI client
        self.feagi_client = None
        self.zmq_client = None
        
        # Performance tracking
        self.timings = {
            'coords': 0,
            'json': 0,
            'send': 0,
            'frame_count': 0
        }
        
        # Neural data
        self.cortical_dimensions = {}
        self.activation_coordinates = {}
        self.cortical_areas = {}
        
        # Cached data for optimization
        self.coord_buffer = np.zeros((10000, 3), dtype=np.int32)
        self.struct_cache = {}
        self.last_coords_by_cortical_id = {}
        
        # Godot data
        self.godot_input = {}
        
        # Initialize threads
        self.threads = []
        
        # Message queue for visualization data
        self.fcl_queue = deque(maxlen=3)
        
    def connect_to_feagi(self):
        """Connect to FEAGI using the FeagiClient."""
        logger.info(f"Connecting to FEAGI at {self.feagi_settings['feagi_host']}")
        
        try:
            # Create FEAGI client
            self.feagi_client = FeagiClient(
                host=self.feagi_settings['feagi_host'],
                agent_id="godot_bridge",
                agent_type="monitor"
            )
            
            # Create ZMQ client for visualization stream
            self.zmq_client = ZmqFeagiClient(
                host=self.feagi_settings['feagi_host'],
                visualization_port=5560  # Default visualization port
            )
            
            # Connect the ZMQ client
            asyncio.run(self.zmq_client.connect())
            
            # Register visualization callback
            asyncio.run(self.zmq_client.register_visualization_callbacks(
                activity_callback=self._on_zmq_message
            ))
            
            logger.info("Connected to FEAGI successfully")
            self.running = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            return False
    
    def _on_zmq_message(self, message):
        """Callback for ZMQ messages from FEAGI."""
        # Store message in queue for processing
        self.fcl_queue.append(message)
        
    def _process_feagi_data(self):
        """Process data received from FEAGI."""
        if not self.fcl_queue:
            return
        
        # Get the latest message
        message = self.fcl_queue.pop()
        
        # Process the message
        # For now, assuming a dict with similar structure to the legacy format
        # Extract activation coordinates if available
        if "godot" in message:
            self.activation_coordinates = message["godot"]
            
        # Extract cortical dimensions if available
        if "cortical_dimensions" in message:
            self.cortical_dimensions = message["cortical_dimensions"]
        
        # Process the data for Godot
        self._process_data_for_godot(message)
        
    def _process_data_for_godot(self, message):
        """Process neuron activity data for Godot visualization."""
        self.timings['frame_count'] += 1
        start = time.perf_counter()
        
        # Create the data structure for Godot
        processed_FEAGI_status_data = {
            "status": {
                "burst_engine": message.get("burst_engine", False),
                "genome_availability": message.get("genome_availability", False),
                "genome_validity": message.get("genome_validity", False),
                "brain_readiness": message.get("brain_readiness", False),
                "genome_timestamp": message.get("genome_changed")
            },
            "activations": []
        }
        
        # Process amalgamation data if available
        if "amalgamation_pending" in message:
            processed_FEAGI_status_data["status"]["amalgamation_pending"] = message.get("amalgamation_pending")
            if 'initiation_time' in processed_FEAGI_status_data["status"]["amalgamation_pending"]:
                processed_FEAGI_status_data["status"]["amalgamation_pending"].pop('initiation_time')
        
        # Create structures to send to Godot
        wrapped_structures_to_send = []
        
        # Add JSON data
        json_start = time.perf_counter()
        json_wrapped = JSONByteStructure.create_from_json_string(json.dumps(processed_FEAGI_status_data))
        wrapped_structures_to_send.append(json_wrapped)
        json_end = time.perf_counter()
        self.timings['json'] += (json_end - json_start)
        
        # Process coordinates
        coords_start = time.perf_counter()
        if self.activation_coordinates:
            for cortical_id, coords in self.activation_coordinates.items():
                if not coords or len(coords) == 0:
                    continue
                
                # Check if coordinates haven't changed since last frame
                current_coords_hash = hash(frozenset(tuple(c) for c in coords))
                dimensions = self.cortical_dimensions.get(cortical_id, (10, 10, 10))
                cache_key = (cortical_id, current_coords_hash, dimensions)
                
                if cache_key in self.struct_cache:
                    wrapped_structures_to_send.append(self.struct_cache[cache_key])
                    continue
                
                # Coordinates changed or not in cache, process them
                coord_count = len(coords)
                
                # Resize buffer if necessary
                if coord_count > self.coord_buffer.shape[0]:
                    self.coord_buffer = np.zeros((coord_count + 1000, 3), dtype=np.int32)
                
                # Fast path using pre-allocated buffer
                for i, (x, y, z) in enumerate(coords):
                    self.coord_buffer[i, 0] = x
                    self.coord_buffer[i, 1] = y
                    self.coord_buffer[i, 2] = z
                
                # Use a view of the buffer
                activation_coordinate = self.coord_buffer[:coord_count]
                
                # Get cortical dimension
                cortical_dimension = np.array(dimensions, dtype=np.int32)
                
                # Create SVO structure
                svo_activations = SVORaymarchingByteStructure.create_from_summary_data(
                    cortical_dimension, activation_coordinate, cortical_id)
                
                # Cache the structure for future reuse
                self.struct_cache[cache_key] = svo_activations
                
                # Limit cache size
                if len(self.struct_cache) > 1000:
                    for old_key in list(self.struct_cache.keys())[:100]:
                        self.struct_cache.pop(old_key)
                
                wrapped_structures_to_send.append(svo_activations)
        
        coords_end = time.perf_counter()
        self.timings['coords'] += (coords_end - coords_start)
        
        # Process image data if available
        if pns.full_list_dimension:
            if 'iv00CC' in pns.full_list_dimension:
                res_json = list(retina.grab_xy_cortical_resolution('iv00CC'))
                resolution = (int(res_json[0]), int(res_json[1]))
                FEAGI_RGB_data = message.get("color_image")  # dict[tuple[int, int, int]: int]
                if FEAGI_RGB_data is not None:
                    image_wrapped = SingleRawImage.create_from_FEAGI_delta_dict(resolution, FEAGI_RGB_data)
                    wrapped_structures_to_send.append(image_wrapped)
        
        # Send data to Godot
        send_start = time.perf_counter()
        if wrapped_structures_to_send:
            multi_wrapped = MultiByteStructHolder(wrapped_structures_to_send)
            if not multi_wrapped.is_empty():
                byte_data = multi_wrapped.to_bytes()
                send_to_BV_queue.append(byte_data)
        send_end = time.perf_counter()
        self.timings['send'] += (send_end - send_start)
        
        # Process incoming data from Godot
        if queue_of_recieve_godot_data:
            obtained_data_from_godot = queue_of_recieve_godot_data[0].decode('UTF-8')
            queue_of_recieve_godot_data.pop()
        else:
            obtained_data_from_godot = "{}"
        
        # Process Godot input if valid
        invalid_values = {"None", "{}", "refresh", "[]"}
        if obtained_data_from_godot not in invalid_values and obtained_data_from_godot != self.godot_input:
            self.godot_input = bridge.godot_data(obtained_data_from_godot)
            
            # Process direct stimulation
            converted_data = bridge.convet_godot_coord_to_feagi_coord(
                stimulation_from_godot=self.godot_input,
                cortical_data_list={}  # Will need to update this with the right data
            )
            
            if converted_data != {}:
                # Send stimulation to FEAGI
                # This will need to be adapted for the new API
                pass
        
        # Log performance if needed
        end = time.perf_counter()
        if self.timings['frame_count'] % 100 == 0:
            total_frames = self.timings['frame_count']
            logger.info("\n=== PERFORMANCE REPORT ===")
            logger.info(f"Coordinates processing: {self.timings['coords']/total_frames:.6f} sec avg")
            logger.info(f"JSON operations: {self.timings['json']/total_frames:.6f} sec avg")
            logger.info(f"Message sending: {self.timings['send']/total_frames:.6f} sec avg")
            logger.info(f"Estimated FPS: {total_frames/(self.timings['coords'] + self.timings['json'] + self.timings['send']):.1f}")
            logger.info("=========================\n")
            
            # Reset timings for next batch
            self.timings = {
                'coords': 0,
                'json': 0,
                'send': 0,
                'frame_count': 0
            }
    
    def run(self):
        """Main run loop for the bridge."""
        # Start Godot WebSocket server
        self.threads.append(threading.Thread(
            target=websocket_operation, 
            args=(self.agent_settings,), 
            daemon=True
        ))
        
        # Start bridge operation for data forwarding
        self.threads.append(threading.Thread(
            target=bridge_operation, 
            args=(runtime_data,), 
            daemon=True
        ))
        
        # Start queue management
        self.threads.append(threading.Thread(
            target=feagi_to_brain_visualizer, 
            args=(runtime_data,), 
            daemon=True
        ))
        
        # Start ZMQ client listener in a separate thread
        self.threads.append(threading.Thread(
            target=self._run_zmq_client,
            daemon=True
        ))
        
        # Start all threads
        for thread in self.threads:
            thread.start()
        
        # Main processing loop
        logger.info("Starting main processing loop")
        try:
            while self.running:
                # Process FEAGI data
                self._process_feagi_data()
                
                # Sleep to control loop speed
                sleep(runtime_data["stimulation_period"])
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt, shutting down")
            self.running = False
        except Exception as e:
            logger.exception(f"Error in main loop: {e}")
            self.running = False
    
    def _run_zmq_client(self):
        """Run the ZMQ client in an event loop."""
        try:
            # Create a new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Start the visualization listener
            loop.run_until_complete(self.zmq_client._visualization_listener())
        except Exception as e:
            logger.exception(f"Error in ZMQ client thread: {e}")
            self.running = False
    
    def disconnect(self):
        """Disconnect from FEAGI."""
        self.running = False
        
        # Disconnect ZMQ client
        if self.zmq_client:
            try:
                asyncio.run(self.zmq_client.disconnect())
            except Exception as e:
                logger.error(f"Error disconnecting ZMQ client: {e}")
                
        logger.info("Disconnected from FEAGI")


def main():
    """Main entry point for the bridge."""
    # Print banner
    print("=" * 80)
    print("=" * 25 + "  FEAGI 2.0 Godot Bridge  " + "=" * 25)
    print("=" * 80)
    
    # Create and run the bridge
    bridge = GodotBridge()
    
    try:
        # Connect to FEAGI
        connected = bridge.connect_to_feagi()
        if not connected:
            logger.error("Failed to connect to FEAGI, exiting")
            return
        
        # Run the bridge
        bridge.run()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down")
    except Exception as e:
        logger.exception(f"Error in bridge: {e}")
    finally:
        # Clean up
        bridge.disconnect()


if __name__ == "__main__":
    # Set runtime data stimulation period default
    runtime_data["stimulation_period"] = 0.01
    
    # Run the main function
    main() 