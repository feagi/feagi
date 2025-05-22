"""
ZeroMQ Visualization Stream Implementation for FEAGI API

This module implements a specialized streaming pattern for visualization data.
It provides:
- One-directional flow from FEAGI to agents for neuron activity data
- Efficient binary serialization for high-performance data exchange
- Level-of-detail mechanisms for performance optimization
- Genome-dependent state management (standby when no genome loaded)

Performance Optimization:
- Socket is configured for real-time operation with minimal latency
- Messages are treated as ephemeral - no queueing is performed
- ZMQ_CONFLATE ensures only the latest message is kept, preventing stale data processing
- High water marks (HWM) are set to minimal values to prevent buffer buildup
- Non-blocking operations ensure system responsiveness
"""

import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(__name__)
import time
import struct
import json
from typing import Dict, Any, Optional, Callable, List, Tuple
import random

import zmq
import zmq.asyncio

from ...core.service import CoreApiService
from ...utils.rate_limit import RateLimiter
from feagi.core.state_manager import GenomeState


def extract_neuron_data_from_bytes(data: bytes) -> Dict[str, List]:
    """
    Extract neuron data from raw byte structure.
    
    This function tries to parse the byte data into the format:
    {"cortical_id": [[x values], [y values], [z values], [potentials]], ...}
    
    Args:
        data: Raw byte data
        
    Returns:
        Dictionary mapping cortical IDs to coordinate and potential arrays
    """
    try:
        # Print raw data length for debugging
        print(f"DEBUG: Attempting to extract neuron data from {len(data)} bytes")
        
        # Skip the header (2 bytes)
        if len(data) < 2:
            print("DEBUG: Data too short for header")
            return {}
            
        # Check if this is a neuron structure (structure ID 4 or 5)
        structure_id = data[0]
        version = data[1]
        print(f"DEBUG: Structure ID: {structure_id}, Version: {version}")
        
        if structure_id not in (4, 5):  # Neuron flat or neuron categories
            print(f"DEBUG: Not a neuron structure (ID {structure_id})")
            return {}
            
        # For neuron flat structure (ID 4)
        if structure_id == 4:
            # Skip header (2 bytes) and get count (4 bytes)
            neuron_count = struct.unpack("!I", data[2:6])[0]
            print(f"DEBUG: Neuron count: {neuron_count}")
            
            if neuron_count == 0:
                print("DEBUG: No neurons in data")
                return {}
                
            # Get the length of the cortical ID section
            cortical_id_section_length = struct.unpack("!I", data[6:10])[0]
            print(f"DEBUG: Cortical ID section length: {cortical_id_section_length}")
            
            # Extract cortical IDs
            offset = 10
            cortical_ids = []
            for i in range(neuron_count):
                # Each cortical ID is 6 bytes
                cortical_id = data[offset:offset+6].decode('utf-8')
                cortical_ids.append(cortical_id)
                offset += 6
                # Print the first few cortical IDs for debugging
                if i < 5:
                    print(f"DEBUG: Cortical ID {i}: {cortical_id}")
                
            # After cortical IDs comes the coordinates and potentials
            # Format: [x1, x2, ..., y1, y2, ..., z1, z2, ..., p1, p2, ...]
            
            # Extract X coordinates (4 bytes each)
            x_coords = []
            for i in range(neuron_count):
                x = struct.unpack("!i", data[offset:offset+4])[0]
                x_coords.append(x)
                offset += 4
                # Print the first few coordinates for debugging
                if i < 5:
                    print(f"DEBUG: X coord {i}: {x}")
                
            # Extract Y coordinates (4 bytes each)
            y_coords = []
            for i in range(neuron_count):
                y = struct.unpack("!i", data[offset:offset+4])[0]
                y_coords.append(y)
                offset += 4
                # Print the first few coordinates for debugging
                if i < 5:
                    print(f"DEBUG: Y coord {i}: {y}")
                
            # Extract Z coordinates (4 bytes each)
            z_coords = []
            for i in range(neuron_count):
                z = struct.unpack("!i", data[offset:offset+4])[0]
                z_coords.append(z)
                offset += 4
                # Print the first few coordinates for debugging
                if i < 5:
                    print(f"DEBUG: Z coord {i}: {z}")
                
            # Extract potentials (4 bytes each)
            potentials = []
            for i in range(neuron_count):
                p = struct.unpack("!f", data[offset:offset+4])[0]
                potentials.append(p)
                offset += 4
                # Print the first few potentials for debugging
                if i < 5:
                    print(f"DEBUG: Potential {i}: {p}")
                
            # Group by cortical ID
            result = {}
            for i in range(neuron_count):
                cortical_id = cortical_ids[i][:6].ljust(6)
                if cortical_id not in result:
                    result[cortical_id] = [[], [], [], []]
                
                result[cortical_id][0].append(x_coords[i])
                result[cortical_id][1].append(y_coords[i])
                result[cortical_id][2].append(z_coords[i])
                result[cortical_id][3].append(potentials[i])
            
            # Print summary of result
            print(f"DEBUG: Extracted data for {len(result)} cortical areas")
            for area_id, data in list(result.items())[:3]:  # Show first 3 areas
                print(f"DEBUG: Area {area_id}: {len(data[0])} neurons")
                
            return result
            
        return {}
        
    except Exception as e:
        print(f"DEBUG ERROR: Error extracting neuron data from bytes: {e}")
        import traceback
        print(f"DEBUG ERROR TRACE: {traceback.format_exc()}")
        return {}


class VisualizationStream:
    """
    ZeroMQ Visualization Stream implementation.
    
    This implementation uses a PUB socket for sending neural activity data (FEAGI → agents).
    
    The stream automatically adjusts to the genome availability state:
    - When no genome is loaded, it operates in standby mode, sending status updates but no data
    - When a genome is loaded, it transitions to active mode with full functionality
    """
    
    def __init__(
        self, 
        core_api: CoreApiService,
        host: str = "*", 
        port: int = 5560,
        context: Optional[zmq.asyncio.Context] = None,
        fcl_sampler: Optional[Any] = None,
        fcl_sampler_queue: Optional[Any] = None
    ):
        """
        Initialize a new Visualization Stream.
        
        Args:
            core_api: The CoreApiService instance to delegate calls to
            host: Host address to bind to
            port: Port for visualization data
            context: Optional existing ZMQ context to use
            fcl_sampler: Optional FCL sampler instance for visualization data
            fcl_sampler_queue: Optional queue for FCL data from the sampler
        """
        self.core_api = core_api
        self.host = host
        self.port = port
        self.running = False
        self.context = context or zmq.asyncio.Context.instance()
        
        # State tracking
        self._active_mode = False  # True when genome is loaded and ready
        
        # Connected clients tracking (estimated, since ZMQ PUB doesn't track subscribers)
        self.client_last_heartbeat = {}  # Mapping of client_id -> last heartbeat time
        self.client_heartbeat_timeout = 30  # Consider clients disconnected after 30s without heartbeat
        
        # Socket for visualization data
        self.socket = self._setup_socket()
        
        # Rate limiter for throttling high-frequency data
        self.rate_limiter = RateLimiter()
        
        # FCL Sampler integration
        self.fcl_sampler = fcl_sampler
        self.fcl_sampler_queue = fcl_sampler_queue
        
        # Tasks
        self.tasks = []
        
        # Register for genome state change notifications
        if hasattr(core_api, 'register_genome_change_listener'):
            core_api.register_genome_change_listener(self._on_genome_state_change)
        
        # Initialize state based on current genome availability
        self._update_active_mode()

    def _setup_socket(self) -> zmq.asyncio.Socket:
        """
        Set up a visualization socket with real-time optimization.
        
        Returns:
            Configured ZMQ socket
        """
        socket = self.context.socket(zmq.PUB)
        
        # Configure for real-time with no queuing
        socket.setsockopt(zmq.SNDHWM, 1)  # Minimal send queue
        socket.setsockopt(zmq.CONFLATE, 1)  # Only keep most recent message
        socket.setsockopt(zmq.LINGER, 0)  # Don't wait when closing
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        logger.info(f"Binding visualization PUB socket to {bind_addr}")
        socket.bind(bind_addr)
        
        # Make this port highly visible in logs for debugging
        print(f"\n==================================================")
        print(f"VISUALIZATION SOCKET BOUND TO: tcp://{self.host}:{self.port}")
        print(f"==================================================\n")
        
        # Send an immediate test message to verify the socket is working
        try:
            # Need to run in event loop
            if asyncio.get_event_loop().is_running():
                asyncio.create_task(self._send_test_message(socket))
            else:
                # Can't use async here, so we'll skip this test
                print("Cannot send test message: no running event loop")
        except Exception as e:
            print(f"Error setting up test message: {e}")
        
        # Start task to send periodic welcome messages to help clients
        if asyncio.get_event_loop().is_running():
            asyncio.create_task(self._send_periodic_welcome())
        
        return socket
        
    async def _send_test_message(self, socket):
        """Send a test message to verify the socket is working."""
        try:
            # Give clients a moment to connect
            await asyncio.sleep(1)
            
            # Create a simple test message
            test_msg = f"TEST_MESSAGE:{time.time()}".encode('utf-8')
            
            # Send the test message on the test topic
            await socket.send_multipart([
                b"test",  # Topic
                test_msg  # Message
            ])
            
            print(f"\n==================================================")
            print(f"TEST MESSAGE SENT TO ZMQ VISUALIZATION SOCKET")
            print(f"Message: {test_msg.decode('utf-8')}")
            print(f"==================================================\n")
            
            # Schedule additional test messages every 5 seconds
            asyncio.create_task(self._send_periodic_test_messages())
            
        except Exception as e:
            print(f"Error sending test message: {e}")
            import traceback
            print(traceback.format_exc())
            
    async def _send_periodic_test_messages(self):
        """Send periodic test messages to help debug ZMQ connections."""
        while self.running and self.socket:
            try:
                # Send a test message every 5 seconds
                test_msg = f"PERIODIC_TEST:{time.time()}".encode('utf-8')
                await self.socket.send_multipart([
                    b"test",  # Topic
                    test_msg  # Message
                ])
                print(f"PERIODIC TEST MESSAGE SENT: {test_msg.decode('utf-8')}")
            except Exception as e:
                print(f"Error sending periodic test: {e}")
            
            # Wait 5 seconds before sending next message
            await asyncio.sleep(5)

    async def _send_periodic_welcome(self):
        """Send periodic welcome messages to help clients detect the server."""
        while self.running and self.socket:
            try:
                # Send a welcome message every 5 seconds
                welcome_msg = f"FEAGI_WELCOME:{time.time()}"
                await self.socket.send_multipart([
                    b"system",
                    welcome_msg.encode('utf-8')
                ])
                logger.info(f"Sent welcome message: {welcome_msg}")
            except Exception as e:
                logger.error(f"Error sending welcome message: {e}")
            
            # Wait 5 seconds before sending next message
            await asyncio.sleep(5)

    def _update_active_mode(self):
        """Update active mode based on genome availability."""
        old_mode = self._active_mode
        
        # Safely check genome loaded state with defensive programming
        try:
            self._active_mode = self.core_api.genome_is_loaded() if self.core_api else False
        except Exception as e:
            # If there's any error accessing genome state, default to standby mode
            logger.warning(f"Error checking genome state: {e}, defaulting to standby mode")
            self._active_mode = False
        
        if old_mode != self._active_mode:
            if self._active_mode:
                logger.info("VisualizationStream entering ACTIVE mode (genome loaded)")
            else:
                logger.info("VisualizationStream entering STANDBY mode (no genome loaded)")

    def _on_genome_state_change(self, old_state, new_state):
        """Handle genome state changes.
        
        Args:
            old_state: Previous genome state
            new_state: New genome state
        """
        logger.debug(f"Received genome state change: {old_state} → {new_state}")
        
        try:
            # Only care about LOADED vs other states
            if new_state == GenomeState.LOADED:
                # Transition to active mode when genome is loaded
                self._active_mode = True
                if self.running:
                    logger.info("VisualizationStream entering ACTIVE mode (genome loaded)")
                    # Create the broadcast task without awaiting it
                    # The asyncio event loop will run it when available
                    if asyncio.get_event_loop().is_running():
                        asyncio.create_task(self._broadcast_system_message("FEAGI_STATE_CHANGE:active"))
                    else:
                        logger.error("Cannot broadcast state change: no running event loop")
            else:
                # Any other state means genome not fully loaded
                self._active_mode = False 
                if self.running:
                    logger.info("VisualizationStream entering STANDBY mode (no genome loaded)")
                    # Create the broadcast task without awaiting it
                    # The asyncio event loop will run it when available
                    if asyncio.get_event_loop().is_running():
                        asyncio.create_task(self._broadcast_system_message("FEAGI_STATE_CHANGE:standby"))
                    else:
                        logger.error("Cannot broadcast state change: no running event loop")
        except Exception as e:
            logger.error(f"Error handling genome state change: {e}")
            # Default to standby mode on error
            self._active_mode = False

    async def _broadcast_system_message(self, message: str):
        """Broadcast a system message to all connected clients.
        
        Args:
            message: System message to broadcast
        """
        try:
            if not self.running or not self.socket:
                return
                
            # Add connected client count to system messages
            connected_clients = self.get_connected_client_count()
            if not message.startswith("FEAGI_CLIENTS_COUNT:"):
                # Send client count update every 10 seconds
                if time.time() % 10 < 1:
                    await self.socket.send_multipart([
                        b"system",
                        f"FEAGI_CLIENTS_COUNT:{connected_clients}".encode('utf-8')
                    ])
                
            # Send on system channel
            await self.socket.send_multipart([
                b"system",                # System channel
                message.encode('utf-8')   # Message
            ])
            
            logger.debug(f"Broadcast system message: {message}")
            
        except Exception as e:
            logger.error(f"Error broadcasting system message: {e}")

    async def start(self) -> None:
        """Start the visualization stream server."""
        if self.running:
            return
            
        logger.info(f"Starting Visualization Stream server on {self.host}:{self.port}")
        self.running = True
        
        # Start FCL processing tasks if FCL sampler is available
        if self.fcl_sampler_queue:
            self.tasks.append(asyncio.create_task(self._process_fcl_data()))
            
        # Start client tracking task
        self.tasks.append(asyncio.create_task(self._cleanup_disconnected_clients()))
        
        # If test mode is enabled, start sending test data periodically
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if state_manager.get_test_visualization_mode():
                logger.info("Starting periodic test data in test visualization mode")
                self.tasks.append(asyncio.create_task(self._send_periodic_test_data()))
        except Exception as e:
            logger.debug(f"Could not check test visualization mode: {e}")
            
        logger.info("Visualization Stream server started")

    async def stop(self) -> None:
        """Stop the visualization stream server."""
        if not self.running:
            return
            
        logger.info("Stopping Visualization Stream server")
        self.running = False
        
        # Cancel all tasks
        for task in self.tasks:
            task.cancel()
            
        # Wait for tasks to complete
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.tasks = []
        
        # Close the socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("Visualization Stream server stopped")

    async def _process_fcl_data(self) -> None:
        """Process FCL data from the sampler queue."""
        if not self.fcl_sampler_queue:
            logger.debug("No FCL sampler queue available for visualization data")
            return
            
        # Check test visualization mode once at the start
        test_viz_mode = False
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            test_viz_mode = state_manager.get_test_visualization_mode()
            
            # Print this prominently if in test mode
            if test_viz_mode:
                print("\n==================================================")
                print("VISUALIZATION STREAM RUNNING IN TEST MODE")
                print(f"FCL Queue type: {type(self.fcl_sampler_queue)}")
                print(f"FCL Queue empty: {self.fcl_sampler_queue.empty() if hasattr(self.fcl_sampler_queue, 'empty') else 'unknown'}")
                print(f"Connected clients: {self.get_connected_client_count()}")
                print("==================================================\n")
        except Exception as e:
            logger.debug(f"Could not check test visualization mode: {e}")
            
        logger.info(f"Starting FCL data processing task. Test mode: {test_viz_mode}")
        
        # For test mode, register a fake client to ensure data flows
        if test_viz_mode:
            # Register a fake client to ensure data is processed
            self.client_last_heartbeat["test_visualization_client"] = time.time()
            logger.info("Registered test visualization client")
            
        while self.running:
            try:  # Outer try for the main loop
                # Wait for data with a short timeout
                fcl_data = None
                
                # Always use non-blocking get() for queue access
                try: # Inner try for queue access
                    if hasattr(self.fcl_sampler_queue, 'get'):
                        try:
                            fcl_data = self.fcl_sampler_queue.get(block=False)
                            logger.info(f"Got data from queue: type={type(fcl_data)} content={fcl_data}")
                        except Exception: # Catches queue.Empty and other queue related errors
                            await asyncio.sleep(0.01)
                            continue 
                    elif hasattr(self.fcl_sampler_queue, '_queue') and len(self.fcl_sampler_queue._queue) > 0:
                        fcl_data = self.fcl_sampler_queue._queue.pop(0) # Get and remove
                        logger.info(f"Directly accessed queue data: type={type(fcl_data)}")
                    else:
                        await asyncio.sleep(0.01)
                        continue
                except Exception as queue_err: 
                    logger.error(f"Error accessing queue data: {queue_err}")
                    await asyncio.sleep(0.01)
                    continue 
                
                if fcl_data is None:
                    await asyncio.sleep(0.01)
                    continue
                    
                if isinstance(fcl_data, bytes):
                    if self.get_connected_client_count() > 0 or test_viz_mode:
                        try:
                            await self.socket.send_multipart([b"activity", fcl_data])
                            logger.info(f"Sent {len(fcl_data)} bytes of binary data")
                            continue  
                        except Exception as e:
                            logger.error(f"Error sending binary data: {e}")
                            import traceback; logger.error(traceback.format_exc())
                
                elif isinstance(fcl_data, tuple):
                    if len(fcl_data) == 2:
                        try:
                            cortical_id, area_fcl = fcl_data
                            neuron_ids = list(area_fcl) if hasattr(area_fcl, '__iter__') else []
                            if not neuron_ids:
                                continue 
                            if self.get_connected_client_count() > 0 or test_viz_mode:
                                x_values, y_values, z_values, potentials = [], [], [], []
                                for neuron_id in neuron_ids:
                                    x, y, z = neuron_id % 100, (neuron_id // 100) % 100, neuron_id // 10000
                                    x_values.append(x); y_values.append(y); z_values.append(z); potentials.append(1.0)
                                cortical_ids_list = [cortical_id] * len(neuron_ids)
                                try:
                                    from feagi_bytes import ByteStructureEncoder
                                    encoder = ByteStructureEncoder()
                                    binary_data = encoder.encode_neuron_flat(
                                        cortical_ids=cortical_ids_list, x_coords=x_values,
                                        y_coords=y_values, z_coords=z_values, potentials=potentials
                                    )
                                    await self.socket.send_multipart([b"activity", binary_data])
                                    print(f"✅ SENT {len(binary_data)} BYTES OF ACTIVITY DATA (2-tuple)")
                                    continue 
                                except Exception as e:
                                    logger.error(f"Error encoding 2-tuple with ByteStructureEncoder: {e}")
                                    import traceback; logger.error(traceback.format_exc())
                        except Exception as e:
                            logger.error(f"Error processing (cortical_id, area_fcl) 2-tuple: {e}")
                            import traceback; logger.error(traceback.format_exc())
                    
                    elif len(fcl_data) >= 5:
                        try:
                            cortical_ids_in, x_coords_in, y_coords_in, z_coords_in, potentials_in = fcl_data[:5]
                            cortical_ids, x_coords, y_coords, z_coords, potentials_list = (
                                [list(el) if not isinstance(el, list) else el for el in 
                                [cortical_ids_in, x_coords_in, y_coords_in, z_coords_in, potentials_in]]
                            )
                            neuron_count = min(len(x_coords), len(y_coords), len(z_coords), len(potentials_list))
                            active_count = sum(1 for p in potentials_list[:neuron_count] if p > 0.0)
                            if len(cortical_ids) < neuron_count:
                                padding_id = cortical_ids[-1] if cortical_ids else "TEST"
                                cortical_ids.extend([padding_id] * (neuron_count - len(cortical_ids)))
                            cortical_ids,x_coords,y_coords,z_coords,potentials_list = (arr[:neuron_count] for arr in [cortical_ids,x_coords,y_coords,z_coords,potentials_list])
                            if neuron_count == 0:
                                continue
                            if self.get_connected_client_count() > 0 or test_viz_mode:
                                try:
                                    from feagi_bytes import ByteStructureEncoder
                                    encoder = ByteStructureEncoder()
                                    binary_data = encoder.encode_neuron_flat(
                                        cortical_ids=cortical_ids, x_coords=x_coords, y_coords=y_coords,
                                        z_coords=z_coords, potentials=potentials_list)
                                    await self.socket.send_multipart([b"activity", binary_data])
                                    print(f"✅ SENT {len(binary_data)} BYTES OF ACTIVITY DATA (5-tuple)")
                                    continue 
                                except Exception as e:
                                    logger.error(f"Error encoding 5-tuple with ByteStructureEncoder: {e}")
                                    # Manual encoding fallback
                                    try:
                                        import struct
                                        binary_data = struct.pack("!BB", 4, 1) + struct.pack("!I", neuron_count)
                                        cid_bytes = b''.join(str(cid)[:6].ljust(6).encode('utf-8') for cid in cortical_ids)
                                        binary_data += struct.pack("!I",len(cid_bytes)) + cid_bytes
                                        for arr_v in [x_coords,y_coords,z_coords]:
                                            for v_ in arr_v: binary_data += struct.pack("!i",int(v_))
                                        for v_ in potentials_list: binary_data += struct.pack("!f",float(v_))
                                        await self.socket.send_multipart([b"activity", binary_data])
                                        print(f"✅ SENT {len(binary_data)} BYTES OF MANUAL ACTIVITY DATA (5-tuple)")
                                        continue 
                                    except Exception as e2:
                                        logger.error(f"Manual 5-tuple encoding also failed: {e2}")
                                        import traceback; logger.error(traceback.format_exc())
                        except Exception as e:
                            logger.error(f"Error processing 5-tuple data elements: {e}")
                            import traceback; logger.error(traceback.format_exc())

                elif isinstance(fcl_data, str) and fcl_data == "STOP":
                    logger.info("Received STOP signal in FCL data queue")
                    break 
                
                elif fcl_data is not None: 
                    if self.get_connected_client_count() > 0 or test_viz_mode:
                        await self.send_visualization_data(fcl_data) 
                    else:
                        logger.debug("No clients connected, skipping visualization data send (default path)")
                
            except asyncio.CancelledError:
                logger.info("FCL data processing task cancelled.")
                break 
                
            except Exception as e: 
                logger.error(f"Error in _process_fcl_data's main loop: {e}")
                import traceback
                logger.error(traceback.format_exc())
                await asyncio.sleep(0.1) 

    async def _cleanup_disconnected_clients(self) -> None:
        """Periodically clean up disconnected clients based on heartbeat timeout."""
        while self.running:
            try:
                current_time = time.time()
                # Create a copy of the keys to avoid modifying during iteration
                client_ids = list(self.client_last_heartbeat.keys())
                
                for client_id in client_ids:
                    last_heartbeat = self.client_last_heartbeat[client_id]
                    if current_time - last_heartbeat > self.client_heartbeat_timeout:
                        # Client considered disconnected
                        logger.info(f"Visualization client {client_id} considered disconnected (no heartbeat in {self.client_heartbeat_timeout}s)")
                        del self.client_last_heartbeat[client_id]
            except Exception as e:
                logger.error(f"Error cleaning up disconnected clients: {e}")
                
            # Check every 10 seconds
            await asyncio.sleep(10)

    def get_connected_client_count(self) -> int:
        """Get the estimated number of connected visualization clients."""
        return len(self.client_last_heartbeat)

    async def record_client_heartbeat(self, client_id: str) -> None:
        """Record a heartbeat from a client.
        
        Args:
            client_id: ID of the client
        """
        now = time.time()
        old_time = self.client_last_heartbeat.get(client_id, 0)
        self.client_last_heartbeat[client_id] = now
        
        # Print new client connections or reconnections prominently
        if old_time == 0:
            # New client
            print(f"\n==================================================")
            print(f"NEW VISUALIZATION CLIENT CONNECTED: {client_id}")
            print(f"Total clients: {len(self.client_last_heartbeat)}")
            print(f"==================================================\n")
            logger.info(f"New visualization client connected: {client_id}")
            
            # Also broadcast a welcome message specifically for this client
            try:
                welcome_msg = f"FEAGI_CLIENT_WELCOME:{client_id}:{time.time()}"
                await self.socket.send_multipart([
                    b"system",
                    welcome_msg.encode('utf-8')
                ])
                logger.info(f"Sent welcome message to new client: {client_id}")
            except Exception as e:
                logger.error(f"Error sending welcome to new client: {e}")
                
        elif now - old_time > 10:
            # Reconnection after a while
            print(f"\n==================================================")
            print(f"VISUALIZATION CLIENT RECONNECTED: {client_id}")
            print(f"After {now - old_time:.1f} seconds without heartbeats")
            print(f"==================================================\n")
            logger.info(f"Visualization client reconnected: {client_id}")
        
        # In test mode, always print client activity for debugging
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if state_manager.get_test_visualization_mode():
                print(f"TEST MODE: Received heartbeat from client {client_id}")
                print(f"Active clients: {list(self.client_last_heartbeat.keys())}")
                
                # Force a direct test message to this client for confirmation
                try:
                    test_message = f"DIRECT_TEST:{time.time()}:{client_id}".encode('utf-8')
                    await self.socket.send_multipart([
                        b"test",
                        test_message
                    ])
                    print(f"Sent direct test message to client {client_id}")
                except Exception as e:
                    print(f"Error sending direct test: {e}")
        except Exception:
            pass

    async def send_visualization_data(self, data) -> None:
        """
        Send visualization data to clients.
        
        Args:
            data: Visualization data (can be bytes, tuple, or other formats)
        """
        if not self.running or not self.socket:
            print("\n!!! VISUALIZATION DATA NOT SENT: server not running or no socket !!!\n")
            logger.debug("Cannot send visualization data: server not running")
            return
            
        # Check if test visualization mode is enabled first
        test_viz_mode = False
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            test_viz_mode = state_manager.get_test_visualization_mode()
        except Exception as e:
            logger.debug(f"Could not check test visualization mode: {e}")
            
        # Skip if in standby mode - BUT NOT IN TEST MODE
        if not self._active_mode and not test_viz_mode:
            print("\n!!! VISUALIZATION DATA NOT SENT: server in standby mode !!!\n")
            logger.debug("Suppressing visualization data in standby mode")
            return
            
        # Count active clients - log here for debugging
        connected_clients = self.get_connected_client_count()
        if connected_clients == 0 and not test_viz_mode:
            logger.debug("No clients connected, skipping visualization data send")
            return
            
        # Log info about clients
        logger.info(f"Preparing to send visualization data to {connected_clients} clients")
        if test_viz_mode:
            print(f"\n==== PREPARING TO SEND VISUALIZATION DATA ====")
            print(f"Connected clients: {connected_clients}")
            print(f"Data type: {type(data)}")
            if isinstance(data, tuple):
                print(f"Tuple length: {len(data)}")
            elif isinstance(data, bytes):
                print(f"Bytes length: {len(data)}")
            print(f"Test mode: {test_viz_mode}")
            print(f"Active mode: {self._active_mode}")
        
        try:
            # Process the data into binary format for sending
            binary_data = None
            
            # Handle different data types
            if isinstance(data, bytes):
                # Already in binary format
                binary_data = data
                logger.info(f"Using direct bytes data: {len(binary_data)} bytes")
            elif isinstance(data, tuple):
                # Handle tuple format - different processing based on tuple length
                logger.info(f"Converting tuple data with {len(data)} elements to binary")
                
                # Handle (cortical_id, area_fcl) tuple format from FCLSampler
                if len(data) == 2:
                    try:
                        # Extract the cortical ID and area FCL
                        cortical_id, area_fcl = data
                        logger.info(f"Processing FCL data for area {cortical_id}")
                        
                        # Extract neuron IDs from the area_fcl (which is likely a set or bitmap)
                        neuron_ids = list(area_fcl) if hasattr(area_fcl, '__iter__') else []
                        
                        # Skip if no neurons are active
                        if not neuron_ids:
                            logger.debug(f"No active neurons in area {cortical_id}, skipping")
                            return
                            
                        # We need to get coordinates for these neurons
                        x_values = []
                        y_values = []
                        z_values = []
                        potentials = []
                        
                        # Simple estimation: use neuron ID for position
                        for neuron_id in neuron_ids:
                            # Simple mapping for visualization purposes
                            x = neuron_id % 100  # Arbitrary grid size
                            y = (neuron_id // 100) % 100
                            z = neuron_id // 10000
                            
                            x_values.append(x)
                            y_values.append(y)
                            z_values.append(z)
                            potentials.append(1.0)  # Default potential for active neurons
                        
                        # Create a list of cortical IDs (one per neuron)
                        cortical_ids = [cortical_id] * len(neuron_ids)
                        
                        # Try using feagi_bytes encoder
                        try:
                            from feagi_bytes import ByteStructureEncoder
                            
                            # Create binary data
                            binary_data = encoder.encode_neuron_flat(
                                cortical_ids=cortical_ids,
                                x_coords=x_values,
                                y_coords=y_values,
                                z_coords=z_values,
                                potentials=potentials
                            )
                            logger.info(f"Encoded {len(neuron_ids)} neurons for area {cortical_id}")
                        except Exception as e:
                            logger.error(f"Error encoding FCL data with ByteStructureEncoder: {e}")
                            import traceback
                            logger.error(traceback.format_exc())

                            # Manual encoding as fallback
                            try:
                                import struct

                                # Structure ID 4 (NeuronFlat), version 1
                                binary_data = struct.pack("!BB", 4, 1)

                                # Pack neuron data as floats [x, y, z, potential] for each neuron
                                data_bytes = b''
                                for i in range(len(neuron_ids)):
                                    neuron_bytes = struct.pack("!ffff",
                                        float(x_values[i]),
                                        float(y_values[i]),
                                        float(z_values[i]),
                                        float(potentials[i])
                                    )
                                    data_bytes += neuron_bytes

                                # Combine all parts
                                binary_data = header + count_bytes + data_bytes

                                active_count = sum(1 for p in potentials if p > 0)
                                print(f"MANUALLY ENCODED {len(neuron_ids)} NEURONS WITH {active_count} ACTIVE")
                                print(f"MANUAL BYTES DATA: {len(binary_data)} bytes")
                                hex_dump = ' '.join([f'{b:02x}' for b in binary_data[:50]])
                                print(f"First 50 bytes: {hex_dump}")
                            except Exception as e2:
                                logger.error(f"Error creating manual test activity data: {e2}")
                                import traceback
                                logger.error(traceback.format_exc())

                                # Y coordinates (4-byte integers)
                                for y in y_values:
                                    binary_data += struct.pack("!i", int(y))

                                # Z coordinates (4-byte integers)
                                for z in z_values:
                                    binary_data += struct.pack("!i", int(z))

                                # Potentials (4-byte floats)
                                for p in potentials:
                                    binary_data += struct.pack("!f", float(p))

                                logger.info(f"Manually encoded {neuron_count} neurons for area {cortical_id}")
                            except Exception as e2:
                                logger.error(f"Manual encoding failed: {e2}")
                                import traceback
                                logger.error(traceback.format_exc())

                        # Also send a system message to confirm communication
                        try:
                            system_message = f"TEST_DATA_SENT:{time.time()}".encode('utf-8')
                            await self.socket.send_multipart([
                                b"system",        # Topic
                                system_message    # Message
                            ])
                            logger.info(f"Sent test system message: {system_message}")
                        except Exception as e:
                            logger.error(f"Error sending test system message: {e}")

                    except Exception as e:
                        logger.error(f"Error processing (cortical_id, area_fcl) tuple: {e}")
                        import traceback
                        logger.error(traceback.format_exc())

                # Handle 5-element tuple format (the legacy or pre-processed format)
                elif len(data) >= 5:
                    # IMPORTANT: This code path should not normally be reached because we directly
                    # send the data in _process_fcl_data, but we include this as a fallback

                    try:
                        # Extract and validate tuple data
                        cortical_ids, x_coords, y_coords, z_coords, potentials = data[:5]

                        # Convert all elements to lists if they aren't already
                        cortical_ids = list(cortical_ids) if not isinstance(cortical_ids, list) else cortical_ids
                        x_coords = list(x_coords) if not isinstance(x_coords, list) else x_coords
                        y_coords = list(y_coords) if not isinstance(y_coords, list) else y_coords
                        z_coords = list(z_coords) if not isinstance(z_coords, list) else z_coords
                        potentials = list(potentials) if not isinstance(potentials, list) else potentials

                        # Find the smallest array length for consistency
                        neuron_count = min(len(x_coords), len(y_coords), len(z_coords), len(potentials))

                        # Ensure cortical_ids is long enough
                        if len(cortical_ids) < neuron_count:
                            # Pad with copies of the last ID or with "TEST"
                            padding_id = cortical_ids[-1] if cortical_ids else "TEST"
                            cortical_ids.extend([padding_id] * (neuron_count - len(cortical_ids)))

                        # Truncate all arrays to the same length
                        cortical_ids = cortical_ids[:neuron_count]
                        x_coords = x_coords[:neuron_count]
                        y_coords = y_coords[:neuron_count]
                        z_coords = z_coords[:neuron_count]
                        potentials = potentials[:neuron_count]

                        # Count active neurons
                        active_count = sum(1 for p in potentials if p > 0)
                        logger.info(f"Processing tuple with {neuron_count} neurons, {active_count} active")

                        # Use the encoder to create binary data
                        try:
                            from feagi_bytes import ByteStructureEncoder
                            encoder = ByteStructureEncoder()

                            if hasattr(encoder, 'encode_neuron_flat'):
                                binary_data = encoder.encode_neuron_flat(
                                    cortical_ids=cortical_ids,
                                    x_coords=x_coords,
                                    y_coords=y_coords,
                                    z_coords=z_coords,
                                    potentials=potentials
                                )
                                logger.info(f"Encoded tuple with ByteStructureEncoder: {len(binary_data)} bytes")
                            else:
                                logger.error("ByteStructureEncoder missing encode_neuron_flat method")
                        except Exception as e:
                            logger.error(f"Error encoding with ByteStructureEncoder: {e}")
                            import traceback
                            logger.error(traceback.format_exc())

                            # Manual encoding as fallback
                            try:
                                import struct

                                # Structure ID 4 (NeuronFlat), version 1
                                binary_data = struct.pack("!BB", 4, 1)

                                # Neuron count (4 bytes)
                                binary_data += struct.pack("!I", neuron_count)

                                # Encode cortical IDs section
                                # Format cortical ID (6 characters)
                                cort_id_6 = str(cortical_ids[0])[:6].ljust(6)
                                cortical_id_bytes = cort_id_6.encode('utf-8') * neuron_count

                                # Section length
                                binary_data += struct.pack("!I", len(cortical_id_bytes))

                                # Add cortical IDs
                                binary_data += cortical_id_bytes

                                # Add coordinates and potentials in blocks
                                # X coordinates (4-byte integers)
                                for x in x_coords:
                                    binary_data += struct.pack("!i", int(x))

                                # Y coordinates (4-byte integers)
                                for y in y_coords:
                                    binary_data += struct.pack("!i", int(y))

                                # Z coordinates (4-byte integers)
                                for z in z_coords:
                                    binary_data += struct.pack("!i", int(z))

                                # Potentials (4-byte floats)
                                for p in potentials:
                                    binary_data += struct.pack("!f", float(p))

                                logger.info(f"Manually encoded tuple: {len(binary_data)} bytes")
                            except Exception as e2:
                                logger.error(f"Manual encoding failed: {e2}")
                                import traceback
                                logger.error(traceback.format_exc())
                    except Exception as e:
                        logger.error(f"Error processing tuple data: {e}")
                        import traceback
                        logger.error(traceback.format_exc())
                else:
                    logger.error(f"Tuple doesn't have enough elements: {len(data)}")
            elif isinstance(data, str):
                # String data - convert to UTF-8 bytes
                logger.warning(f"Converting string data to bytes: {data[:100]}...")
                binary_data = data.encode('utf-8')
            else:
                # For other types, try to convert to JSON
                logger.warning(f"Received unsupported data type: {type(data)}")
                try:
                    logger.info(f"Attempting to serialize as JSON and encode")
                    import json
                    binary_data = json.dumps(data).encode('utf-8')
                except Exception as e:
                    logger.error(f"Cannot convert data of type {type(data)} to bytes: {e}")
                    return

            # If we still don't have binary data, use test data as a last resort
            if binary_data is None:
                if test_viz_mode:
                    logger.warning("Failed to convert data to binary, using test data")
                    import struct
                    import random

                    # Create minimal test data (1 neuron)
                    neuron_count = 1
                    binary_data = struct.pack("!BBI", 10, 1, neuron_count)  # TestPublisher format
                    binary_data += struct.pack("!ffff", 0.0, 0.0, 0.0, 1.0)  # Single active neuron
                    logger.info(f"Created minimal test data: {len(binary_data)} bytes")
                else:
                    logger.error("Failed to convert data to binary format, cannot send")
                    return

            # Log sending details
            logger.info(f"Sending {len(binary_data)} bytes of visualization data to {connected_clients} clients")

            # Send the data to clients
            if test_viz_mode:
                # In test mode, log more details and don't rate limit
                print(f"\n==== SENDING {len(binary_data)} BYTES TO CLIENTS ====")
                hex_dump = ' '.join([f'{b:02x}' for b in binary_data[:50]])
                print(f"First 50 bytes: {hex_dump}")

                try:
                    # Direct send without rate limiting
                    await self.socket.send_multipart([
                        b"activity",  # Topic
                        binary_data   # Binary data
                    ])
                    print(f"✅ SENT {len(binary_data)} BYTES TO {connected_clients} CLIENTS")
                    print(f"ZMQ PUB SOCKET: {self.host}:{self.port}")
                    print(f"==== VISUALIZATION DATA SENT SUCCESSFULLY ====\n")
                except Exception as e:
                    print(f"❌ ERROR SENDING DATA: {e}")
                    import traceback
                    print(traceback.format_exc())
            else:
                # Regular mode with rate limiting
                # Apply rate limiting to avoid overwhelming clients
                if not self.rate_limiter.check_rate("visualization", 0.05):  # Max 20Hz
                    logger.debug("Rate limiting visualization data")
                    return

                # Send multipart message with topic and data
                try:
                    await self.socket.send_multipart([
                        b"activity",  # Topic
                        binary_data   # Binary data
                    ])
                    logger.info(f"Successfully sent {len(binary_data)} bytes of visualization data")
                except Exception as e:
                    logger.error(f"Error sending visualization data: {e}")
                    import traceback
                    logger.error(traceback.format_exc())

        except Exception as e:
            logger.error(f"Error in send_visualization_data: {e}")
            import traceback
            logger.error(traceback.format_exc())

    async def broadcast_update(self, data_type: str, data: bytes) -> None:
        """
        Broadcast an update to all connected agents.

        Args:
            data_type: Type of data ("activity", "structure", "system")
            data: Binary data
        """
        if not self.running or not self.socket:
            logger.debug(f"Cannot broadcast {data_type} update: server not running")
            return

        # Skip if in standby mode (except for system messages)
        if not self._active_mode and data_type != "system":
            logger.debug(f"Suppressing {data_type} update in standby mode")
            return

        try:
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate(f"broadcast_{data_type}", 0.05):  # Max 20Hz
                logger.debug(f"Rate limiting {data_type} broadcast")
                return

            # Send multipart message with topic and data
            await self.socket.send_multipart([
                data_type.encode('utf-8'),  # Topic
                data                         # Binary data
            ])

            logger.debug(f"Broadcast {len(data)} bytes of {data_type} data")

        except Exception as e:
            logger.error(f"Error broadcasting {data_type} data: {e}")

    async def process_system_message(self, message: str) -> None:
        """Process system messages from clients.

        Args:
            message: System message string
        """
        try:
            # Check for all possible heartbeat message formats
            if "HEARTBEAT:" in message or "SIMPLE_HEARTBEAT:" in message:
                # Split on : to get client ID - handle different formats
                parts = message.split(":")
                if len(parts) >= 2:
                    client_id = parts[1].strip()
                    await self.record_client_heartbeat(client_id)
                    logger.debug(f"Processed heartbeat from client: {client_id}")

                    # Send a confirmation back to the client
                    try:
                        confirm_msg = f"HEARTBEAT_CONFIRM:{client_id}:{time.time()}"
                        await self.socket.send_multipart([
                            b"system",
                            confirm_msg.encode('utf-8')
                        ])
                    except Exception as e:
                        logger.error(f"Error sending heartbeat confirmation: {e}")

                    return

            # Log other system messages for debugging
            logger.debug(f"Received system message: {message}")

        except Exception as e:
            logger.error(f"Error processing system message: {e}")

    async def receive_control_message(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """Process control messages from clients.

        Args:
            message: JSON message from client

        Returns:
            JSON response to client
        """
        try:
            message_type = message.get("message_type", "unknown")

            # Handle heartbeat messages
            if message_type == "heartbeat":
                client_id = message.get("agent_id", f"unknown_{time.time()}")
                await self.record_client_heartbeat(client_id)
                return {
                    "status": "ok",
                    "message": "Heartbeat received",
                    "timestamp": time.time()
                }

            # Handle other message types as needed

            return {
                "status": "error",
                "message": f"Unknown message type: {message_type}",
                "timestamp": time.time()
            }

        except Exception as e:
            logger.error(f"Error processing control message: {e}")
            return {
                "status": "error",
                "message": str(e),
                "timestamp": time.time()
            }

    async def _send_periodic_test_data(self) -> None:
        """Send periodic test activity data in test mode."""
        import random
        import struct

        logger.info("Starting periodic test activity data task")
        print("\n==================================================")
        print("STARTING PERIODIC TEST ACTIVITY DATA GENERATOR")
        print("==================================================\n")

        # Wait a bit for clients to connect before sending data
        await asyncio.sleep(5)

        # Keep track of sequence number for the test data
        sequence = 0

        # Create a simple counter wave that cycles through values
        counter = 0

        while self.running:
            try:
                # Only send if we have connected clients
                connected_clients = self.get_connected_client_count()
                if connected_clients > 0:
                    print(f"\n==================================================")
                    print(f"GENERATING TEST ACTIVITY DATA FOR {connected_clients} CLIENTS")
                    print(f"Sequence #{sequence}")
                    print(f"==================================================\n")

                    # Create binary test data using either NeuronFlat (ID=4) or TestPublisher (ID=10) structure
                    # Try both formats alternating to see which one works better

                    # Create test data
                    neuron_count = 100  # Fixed number of neurons for test

                    # Generate coordinates in a grid pattern for better visual testing
                    grid_size = int(neuron_count ** 0.5)  # Square grid
                    x_values = []
                    y_values = []
                    z_values = []

                    for i in range(neuron_count):
                        row = i // grid_size
                        col = i % grid_size
                        x_values.append(col * 2)
                        y_values.append(row * 2)
                        z_values.append(0)  # All in same z-plane for simplicity

                    # Generate potentials with oscillating pattern
                    potentials = [0.0] * neuron_count
                    # Make a moving wave of activity
                    wave_center = counter % neuron_count
                    wave_width = 10

                    for i in range(neuron_count):
                        distance = min(abs(i - wave_center), abs(i - wave_center + neuron_count))
                        if distance < wave_width:
                            # Create a gaussian-like falloff
                            potentials[i] = max(0.0, 1.0 - (distance / wave_width) ** 2)

                    # Create cortical IDs (all "TEST_A" for simplicity)
                    cortical_ids = ["TEST_A"] * neuron_count

                    # IMPORTANT: Try both encoding methods and use the first one that works
                    binary_data = None

                    # Method 1: Use feagi_bytes encoder
                    try:
                        from feagi_bytes import ByteStructureEncoder

                        encoder = ByteStructureEncoder()
                        if hasattr(encoder, 'encode_neuron_flat'):
                            # Use the encoder to generate data as NeuronFlat structure (ID 4)
                            binary_data = encoder.encode_neuron_flat(
                                cortical_ids=cortical_ids,
                                x_coords=x_values,
                                y_coords=y_values,
                                z_coords=z_values,
                                potentials=potentials
                            )

                            # Log success
                            active_count = sum(1 for p in potentials if p > 0)
                            print(f"ENCODED {neuron_count} NEURONS WITH {active_count} ACTIVE")
                            print(f"ENCODED BYTES DATA FOR ZMQ TRANSPORT: {len(binary_data)} bytes")
                            hex_dump = ' '.join([f'{b:02x}' for b in binary_data[:50]])
                            print(f"First 50 bytes: {hex_dump}")
                    except Exception as e:
                        logger.error(f"Error creating test activity data with encoder: {e}")
                        import traceback
                        logger.error(traceback.format_exc())
                    
                    # Method 2: If encoder failed, create data manually
                    if binary_data is None:
                        try:
                            # Use TestPublisher format (ID 10) which is simpler
                            header = struct.pack("!BB", 10, 1)  # Structure 10, version 1
                            
                            # Neuron count
                            count_bytes = struct.pack("!I", neuron_count)
                            
                            # Pack neuron data as floats [x, y, z, potential] for each neuron
                            data_bytes = b''
                            for i in range(neuron_count):
                                neuron_bytes = struct.pack("!ffff", 
                                    float(x_values[i]), 
                                    float(y_values[i]), 
                                    float(z_values[i]), 
                                    float(potentials[i])
                                )
                                data_bytes += neuron_bytes
                            
                            # Combine all parts
                            binary_data = header + count_bytes + data_bytes
                            
                            active_count = sum(1 for p in potentials if p > 0)
                            print(f"MANUALLY ENCODED {neuron_count} NEURONS WITH {active_count} ACTIVE")
                            print(f"MANUAL BYTES DATA: {len(binary_data)} bytes")
                            hex_dump = ' '.join([f'{b:02x}' for b in binary_data[:50]])
                            print(f"First 50 bytes: {hex_dump}")
                        except Exception as e2:
                            logger.error(f"Error creating manual test activity data: {e2}")
                            import traceback
                            logger.error(traceback.format_exc())
                            
                            # Last resort: Create minimal test data
                            try:
                                neuron_count = 1
                                binary_data = struct.pack("!BBI", 10, 1, neuron_count)
                                binary_data += struct.pack("!ffff", 0.0, 0.0, 0.0, 1.0)
                                print("CREATED MINIMAL TEST DATA (1 neuron)")
                            except Exception:
                                logger.error("Even minimal test data creation failed")
                                binary_data = None
                    
                    # Send the data if we have it
                    if binary_data is not None:
                        try:
                            # Send the data directly
                            await self.socket.send_multipart([
                                b"activity",  # Topic
                                binary_data   # Binary data
                            ])
                            
                            print(f"✅ SENT TEST ACTIVITY DATA: {len(binary_data)} bytes on 'activity' topic")
                            logger.info(f"Sent test activity data: {len(binary_data)} bytes")
                        except Exception as e:
                            logger.error(f"Error sending test activity data: {e}")
                            import traceback
                            logger.error(traceback.format_exc())
                    
                    # Also send a system message to confirm communication
                    try:
                        system_message = f"TEST_DATA_SENT:{sequence}:{time.time()}".encode('utf-8')
                        await self.socket.send_multipart([
                            b"system",        # Topic
                            system_message    # Message
                        ])
                        logger.info(f"Sent test system message: {system_message}")
                    except Exception as e:
                        logger.error(f"Error sending test system message: {e}")
                    
                    # Update counters
                    sequence += 1
                    counter += 5  # Move the activity wave
                
                # Wait before sending next batch of test data
                await asyncio.sleep(2)  # Faster update rate (2s) for better testing
                
            except asyncio.CancelledError:
                # Task was cancelled, exit gracefully
                break
            except Exception as e:
                logger.error(f"Error in test data generator: {e}")
                import traceback
                logger.error(traceback.format_exc())
                await asyncio.sleep(5)  # Wait and try again 