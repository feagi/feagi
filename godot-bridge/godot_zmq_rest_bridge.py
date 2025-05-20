#!/usr/bin/env python3
"""
Godot Bridge Using ZMQ REST API

This module implements a bridge between FEAGI 2.1 and the Godot game engine,
leveraging the standardized ZMQ REST API protocol for control commands.

It handles both:
1. REST API commands for configuration, genome/connectome data
2. Visualization streaming via ZMQ for neural activity data

This allows Godot to visualize the brain structure and neural activity in real-time.
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

# Add the parent directory to the Python path to find FEAGI modules
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import the ZMQ REST client
from feagi.api.zmq.rest_client import ZMQRestClient

# Import local modules
from version import __version__
from network_configuration import websocket_operation, bridge_operation, feagi_to_brain_visualizer
from network_configuration import queue_of_recieve_godot_data, send_to_BV_queue, sleep
import godot_bridge_functions_new as bridge
from FEAGIByteStructures.JSONByteStructure import JSONByteStructure
from FEAGIByteStructures.SVORaymarchingByteStructure import SVORaymarchingByteStructure
from FEAGIByteStructures.MultiByteStructHolder import MultiByteStructHolder
from FEAGIByteStructures.SingleRawImage import SingleRawImage

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("godot_zmq_rest_bridge")


class GodotZMQRestBridge:
    """
    Bridge between FEAGI 2.1 and Godot using the ZMQ REST API protocol.
    
    This class connects to FEAGI using two mechanisms:
    1. ZMQ REST API for control commands and queries
    2. ZMQ Visualization Stream for neural activity data
    
    It then processes and forwards this data to Godot for visualization.
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
        self.feagi_settings['feagi_api_port'] = int(os.environ.get('FEAGI_API_PORT', self.feagi_settings.get('feagi_api_port', "8000")))
        self.feagi_settings['feagi_zmq_port'] = int(os.environ.get('FEAGI_ZMQ_PORT', self.feagi_settings.get('feagi_zmq_port', "5559")))
        self.feagi_settings['feagi_viz_port'] = int(os.environ.get('FEAGI_VIZ_PORT', self.feagi_settings.get('feagi_viz_port', "5560")))
        self.agent_settings['godot_websocket_port'] = int(os.environ.get('WS_BRIDGE_PORT', self.agent_settings.get('godot_websocket_port', "9050")))
        
        # Setup ZMQ REST client
        self.rest_client = ZMQRestClient(
            host=self.feagi_settings['feagi_host'], 
            port=self.feagi_settings['feagi_zmq_port']
        )
        
        # State tracking
        self.running = False
        self.connected = False
        self.previous_genome_timestamp = 0
        self.current_genome_number = 0
        
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
        
        # Threads for different operations
        self.threads = []
        
        # Message queue for activity data
        self.activity_queue = deque(maxlen=3)
        
        # Initialize ZMQ connection flag
        self.zmq_connected = False
        
        # Store the connectome data from FEAGI
        self.connectome_data = {}
        
        # Visualization stream socket
        self.viz_socket = None
    
    def connect(self):
        """Connect to FEAGI using the ZMQ REST API client."""
        logger.info(f"Connecting to FEAGI at {self.feagi_settings['feagi_host']}:{self.feagi_settings['feagi_zmq_port']}")
        
        try:
            # Connect to FEAGI using ZMQ REST API
            self.rest_client.connect()
            logger.info("Connected to FEAGI ZMQ REST API")
            
            # Test connection by getting system health
            health = self.rest_client.get_health()
            logger.info(f"FEAGI system health: {health}")
            
            # Get initial cortical areas
            try:
                self.cortical_areas = self.rest_client.get_cortical_areas()
                logger.info(f"Retrieved {len(self.cortical_areas)} cortical areas")
                
                # Extract dimensions
                for area in self.cortical_areas:
                    area_id = area['id']
                    dimensions = area['dimensions']
                    self.cortical_dimensions[area_id] = dimensions
            except Exception as e:
                logger.warning(f"Failed to get cortical areas: {e}")
            
            # Get system status
            try:
                status = self.rest_client.get_status()
                logger.info(f"FEAGI status: {status}")
            except Exception as e:
                logger.warning(f"Failed to get system status: {e}")
            
            # Mark as connected
            self.connected = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to FEAGI: {e}")
            return False
    
    def setup_visualization_stream(self):
        """Set up the ZMQ visualization stream to receive activity data."""
        import zmq
        
        logger.info(f"Setting up ZMQ visualization stream on port {self.feagi_settings['feagi_viz_port']}")
        
        try:
            # Create ZMQ context and socket
            context = zmq.Context.instance()
            socket = context.socket(zmq.SUB)
            
            # Connect to the visualization socket
            socket.connect(f"tcp://{self.feagi_settings['feagi_host']}:{self.feagi_settings['feagi_viz_port']}")
            
            # Subscribe to activity data
            socket.setsockopt_string(zmq.SUBSCRIBE, "activity")
            
            # Store the socket
            self.viz_socket = socket
            
            # Start a thread to receive data
            thread = threading.Thread(target=self._receive_visualization_data, daemon=True)
            thread.start()
            self.threads.append(thread)
            
            logger.info("ZMQ visualization stream setup complete")
            self.zmq_connected = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup visualization stream: {e}")
            return False
    
    def _receive_visualization_data(self):
        """Background thread to receive data from the visualization stream."""
        logger.info("Starting visualization data receiver thread")
        
        try:
            while self.running:
                try:
                    # Check if socket exists
                    if not self.viz_socket:
                        logger.warning("Visualization socket not initialized")
                        time.sleep(1)
                        continue
                        
                    # Wait for message with timeout
                    if self.viz_socket.poll(100):  # 100ms timeout
                        # Receive data (topic, message)
                        topic, message_data = self.viz_socket.recv_multipart()
                        
                        if topic == b"activity":
                            try:
                                # Parse the message data
                                message = json.loads(message_data.decode('utf-8'))
                                
                                # Store activation data in queue
                                if "activations" in message:
                                    self.activity_queue.append(message["activations"])
                                    
                                    # Extract coordinates if in new format
                                    if isinstance(message["activations"], dict):
                                        self.activation_coordinates = message["activations"]
                            except json.JSONDecodeError:
                                logger.warning("Failed to parse visualization message as JSON")
                            except Exception as e:
                                logger.warning(f"Error processing visualization message: {e}")
                    else:
                        # No message, sleep a bit
                        time.sleep(0.01)
                except Exception as e:
                    logger.error(f"Error in visualization receiver: {e}")
                    time.sleep(0.5)  # Avoid tight loop on error
        except Exception as e:
            logger.error(f"Visualization thread died: {e}")
    
    def _process_data_for_godot(self):
        """Process neuron activity data for Godot visualization."""
        self.timings['frame_count'] += 1
        start = time.perf_counter()
        
        # Get system status
        try:
            status = self.rest_client.get_status()
        except Exception as e:
            logger.warning(f"Failed to get status: {e}")
            status = {
                "genome_availability": True,
                "genome_validity": True,
                "brain_readiness": True
            }
        
        # Create the data structure for Godot
        processed_FEAGI_status_data = {
            "status": {
                "burst_engine": status.get("burst_engine_status", "running") == "running",
                "genome_availability": status.get("genome_availability", True),
                "genome_validity": status.get("genome_validity", True),
                "brain_readiness": status.get("brain_readiness", True),
                "genome_timestamp": status.get("timestamp", 0)
            },
            "activations": []
        }
        
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
                cache_key = (cortical_id, current_coords_hash, tuple(dimensions))
                
                if cache_key in self.struct_cache:
                    wrapped_structures_to_send.append(self.struct_cache[cache_key])
                    continue
                
                # Coordinates changed or not in cache, process them
                coord_count = len(coords)
                
                # Resize buffer if necessary
                if coord_count > self.coord_buffer.shape[0]:
                    self.coord_buffer = np.zeros((coord_count + 1000, 3), dtype=np.int32)
                
                # Copy coordinates to buffer
                for i, coord in enumerate(coords):
                    self.coord_buffer[i, 0] = coord[0]
                    self.coord_buffer[i, 1] = coord[1]
                    self.coord_buffer[i, 2] = coord[2]
                
                # Create SVO structure
                svo_struct = SVORaymarchingByteStructure.create_from_activations(
                    np.array(coords, dtype=np.int32),
                    cortical_id,
                    dimensions
                )
                
                # Cache the result
                self.struct_cache[cache_key] = svo_struct
                wrapped_structures_to_send.append(svo_struct)
        
        coords_end = time.perf_counter()
        self.timings['coords'] += (coords_end - coords_start)
        
        # Send data to Godot
        if wrapped_structures_to_send:
            send_start = time.perf_counter()
            multi_struct = MultiByteStructHolder.create_from_structs(wrapped_structures_to_send)
            
            # Use the existing send_to_BV_queue to forward to Godot
            send_to_BV_queue.put(multi_struct.to_bytes())
            send_end = time.perf_counter()
            self.timings['send'] += (send_end - send_start)
        
        # Print performance metrics occasionally
        if self.timings['frame_count'] % 100 == 0:
            total = time.perf_counter() - start
            logger.info(f"Performance: json={self.timings['json']/self.timings['frame_count']:.6f}s, "
                       f"coords={self.timings['coords']/self.timings['frame_count']:.6f}s, "
                       f"send={self.timings['send']/self.timings['frame_count']:.6f}s, "
                       f"total={total:.6f}s")
    
    def check_for_genome_updates(self):
        """Check if the genome has been updated."""
        try:
            status = self.rest_client.get_status()
            genome_timestamp = status.get("timestamp", 0)
            
            if genome_timestamp != self.previous_genome_timestamp:
                logger.info(f"Genome updated, refreshing cortical areas")
                self.previous_genome_timestamp = genome_timestamp
                self.current_genome_number += 1
                
                # Refresh cortical areas
                self.cortical_areas = self.rest_client.get_cortical_areas()
                
                # Update dimensions
                for area in self.cortical_areas:
                    area_id = area['id']
                    dimensions = area['dimensions']
                    self.cortical_dimensions[area_id] = dimensions
                
                # Clear cache as dimensions might have changed
                self.struct_cache.clear()
        except Exception as e:
            logger.warning(f"Failed to check for genome updates: {e}")
    
    def run(self):
        """Run the Godot bridge."""
        logger.info("Starting Godot bridge")
        self.running = True
        
        # Connect to FEAGI
        if not self.connect():
            logger.error("Failed to connect to FEAGI. Exiting.")
            return False
        
        # Setup visualization stream
        if not self.setup_visualization_stream():
            logger.warning("Failed to setup visualization stream. Continuing without visualization.")
        
        # Start network threads for Godot communication
        try:
            # Start the websocket server thread for Godot
            websocket_thread = threading.Thread(target=websocket_operation)
            websocket_thread.daemon = True
            websocket_thread.start()
            self.threads.append(websocket_thread)
            
            # Start the bridge operation thread
            bridge_thread = threading.Thread(target=bridge_operation)
            bridge_thread.daemon = True
            bridge_thread.start()
            self.threads.append(bridge_thread)
        except Exception as e:
            logger.error(f"Failed to start networking threads: {e}")
            self.running = False
            return False
        
        # Main processing loop
        try:
            last_genome_check = time.time()
            while self.running:
                # Process activity data for Godot
                if self.activity_queue:
                    self._process_data_for_godot()
                
                # Check for genome updates every few seconds
                current_time = time.time()
                if current_time - last_genome_check > 5:  # Check every 5 seconds
                    self.check_for_genome_updates()
                    last_genome_check = current_time
                
                # Process any input received from Godot
                try:
                    if not queue_of_recieve_godot_data.empty():
                        godot_data = queue_of_recieve_godot_data.get_nowait()
                        self._process_godot_input(godot_data)
                except Exception as e:
                    logger.error(f"Error processing Godot input: {e}")
                
                # Sleep to avoid tight loop
                time.sleep(0.01)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received. Shutting down.")
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
        finally:
            self.running = False
            
        # Cleanup
        self.disconnect()
        return True
    
    def _process_godot_input(self, data):
        """Process input received from Godot."""
        try:
            # Parse the input data
            parsed_data = json.loads(data)
            logger.debug(f"Received data from Godot: {parsed_data}")
            
            # Process commands
            if "command" in parsed_data:
                command = parsed_data["command"]
                
                if command == "get_cortical_areas":
                    # Get cortical areas and send back to Godot
                    areas = self.rest_client.get_cortical_areas()
                    response = {"command_response": "cortical_areas", "data": areas}
                    send_to_BV_queue.put(json.dumps(response).encode('utf-8'))
                
                elif command == "get_genome":
                    # Get genome and send back to Godot
                    genome = self.rest_client.get_genome_blueprint()
                    response = {"command_response": "genome", "data": genome}
                    send_to_BV_queue.put(json.dumps(response).encode('utf-8'))
                
                elif command == "update_configuration":
                    # Update FEAGI configuration
                    if "config" in parsed_data:
                        config = parsed_data["config"]
                        result = self.rest_client.update_configuration(config)
                        response = {"command_response": "configuration_updated", "success": True}
                        send_to_BV_queue.put(json.dumps(response).encode('utf-8'))
                        
            # Process stimulation
            if "stimulation" in parsed_data:
                # This would need to use a different API mechanism to send stimulation
                logger.warning("Stimulation not yet implemented in ZMQ REST bridge")
        except json.JSONDecodeError:
            logger.warning(f"Failed to parse Godot input as JSON: {data}")
        except Exception as e:
            logger.error(f"Error processing Godot input: {e}")
    
    def disconnect(self):
        """Disconnect from FEAGI."""
        logger.info("Disconnecting from FEAGI")
        
        # Disconnect ZMQ REST client
        try:
            if self.connected:
                self.rest_client.disconnect()
                logger.info("Disconnected ZMQ REST client")
        except Exception as e:
            logger.warning(f"Error disconnecting ZMQ REST client: {e}")
        
        # Close visualization socket
        try:
            if self.viz_socket:
                self.viz_socket.close()
                logger.info("Closed visualization socket")
        except Exception as e:
            logger.warning(f"Error closing visualization socket: {e}")
        
        # Set flags
        self.connected = False
        self.zmq_connected = False
        logger.info("Disconnected from FEAGI")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='FEAGI to Godot Bridge using ZMQ REST API')
    parser.add_argument('--config', default='configuration.json', help='Path to configuration file')
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                       help='Set the logging level')
    
    args = parser.parse_args()
    
    # Set log level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Create and run bridge
    bridge = GodotZMQRestBridge(config_file=args.config)
    
    try:
        bridge.run()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down")
    except Exception as e:
        logger.error(f"Error running bridge: {e}")
    finally:
        # Ensure clean disconnect
        if bridge.connected:
            bridge.disconnect()


if __name__ == "__main__":
    import argparse
    main() 