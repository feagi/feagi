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
        return socket
        
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
            logger.error("FCL sampler queue not available")
            return
            
        # Check if test visualization mode is enabled
        test_viz_mode = False
        try:
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            test_viz_mode = state_manager.get_test_visualization_mode()
            if test_viz_mode:
                logger.info("Processing FCL data in test visualization mode")
        except Exception as e:
            logger.debug(f"Could not check test visualization mode: {e}")
            
        try:
            while self.running:
                try:
                    # Get data from the queue with a timeout
                    try:
                        # Use a non-async method to get data from the queue to avoid the "await tuple" error
                        # This is a workaround for the visualization test mode
                        fcl_data = None
                        try:
                            # Try non-blocking get first
                            if hasattr(self.fcl_sampler_queue, 'get_nowait'):
                                # Standard Queue
                                fcl_data = self.fcl_sampler_queue.get_nowait()
                            else:
                                # Asyncio Queue
                                fcl_data = self.fcl_sampler_queue.get_nowait()
                        except (asyncio.QueueEmpty, Exception) as e:
                            # If empty or error, wait with timeout
                            await asyncio.sleep(0.01)
                            continue
                        
                        # Process the FCL data - handle both tuple data (area_id, bitmap) and direct bitmap data
                        if isinstance(fcl_data, tuple) and len(fcl_data) == 2:
                            # Format: (cortical_id, bitmap)
                            cortical_id, bitmap = fcl_data
                            # In test mode, log the data but don't try to send it
                            if test_viz_mode:
                                logger.info(f"Test visualization data for area {cortical_id}: {len(bitmap) if bitmap else 0} neurons")
                            else:
                                logger.debug(f"Visualization data for area {cortical_id}: {len(bitmap) if bitmap else 0} neurons")
                            # In a real agent connection, we would encode and send this data
                        elif fcl_data is not None:
                            # Binary data format - just send as is
                            if test_viz_mode:
                                # Print raw bytes for debugging in test mode
                                print("\n============ RAW FCL DATA FROM QUEUE ============")
                                print(f"Type: {type(fcl_data)}, Length: {len(fcl_data) if hasattr(fcl_data, '__len__') else 'unknown'}")
                                if isinstance(fcl_data, bytes):
                                    # Print first 50 bytes as hex
                                    hex_dump = ' '.join([f'{b:02x}' for b in fcl_data[:50]])
                                    print(f"First 50 bytes: {hex_dump}")
                                    
                                    # Try to interpret as JSON if possible
                                    try:
                                        json_data = json.loads(fcl_data.decode('utf-8', errors='ignore'))
                                        print("Appears to be JSON data:")
                                        print(json.dumps(json_data, indent=2)[:1000] + "..." if len(json.dumps(json_data)) > 1000 else json.dumps(json_data, indent=2))
                                    except:
                                        pass
                                print("================================================\n")
                            
                            await self.send_visualization_data(fcl_data)
                        
                    except asyncio.TimeoutError:
                        # No data available, continue
                        await asyncio.sleep(0.01)
                        continue
                        
                except asyncio.CancelledError:
                    # Task was cancelled, exit gracefully
                    break
                    
                except Exception as e:
                    if test_viz_mode:
                        logger.info(f"Error processing FCL data in test mode: {e}")
                        import traceback
                        logger.info(f"FCL data error traceback in test mode: {traceback.format_exc()}")
                    else:
                        logger.error(f"Error processing FCL data: {e}")
                        import traceback
                        logger.debug(f"FCL data error traceback: {traceback.format_exc()}")
                    await asyncio.sleep(0.1)  # Throttle on error
                    
        except asyncio.CancelledError:
            # Task was cancelled, exit gracefully
            pass
        except Exception as e:
            if test_viz_mode:
                logger.info(f"Fatal error in FCL data processor (test mode): {e}")
                import traceback
                logger.info(f"Fatal FCL processor error traceback (test mode): {traceback.format_exc()}")
            else:
                logger.error(f"Fatal error in FCL data processor: {e}")
                import traceback
                logger.debug(f"Fatal FCL processor error traceback: {traceback.format_exc()}")

    async def send_visualization_data(self, data: bytes) -> None:
        """
        Send visualization data to clients.
        
        Args:
            data: Binary visualization data
        """
        if not self.running or not self.socket:
            logger.debug("Cannot send visualization data: server not running")
            return
            
        # Skip if in standby mode
        if not self._active_mode:
            logger.debug("Suppressing visualization data in standby mode")
            return
        
        try:
            # Check if test visualization mode is enabled
            test_viz_mode = False
            try:
                from feagi.core.state_manager import FeagiStateManager
                state_manager = FeagiStateManager.instance()
                test_viz_mode = state_manager.get_test_visualization_mode()
            except Exception as e:
                logger.debug(f"Could not check test visualization mode: {e}")
            
            # If in test visualization mode, log detailed visualization data
            if test_viz_mode:
                logger.info(f"Visualization data in test mode: {len(data)} bytes")
                
                # Try to decode and log the visualization data structure
                try:
                    import json
                    from feagi_bytes import ByteStructureDecoder
                    from feagi_bytes.utils import get_structure_info
                    
                    # First try to get structure info
                    structure_id, version = get_structure_info(data)
                    logger.error(f"TEST VIZ: Structure ID={structure_id}, version={version}")
                    
                    # IMPORTANT: Dump the first 50 bytes for debugging (as hex)
                    hex_dump = ' '.join([f'{b:02x}' for b in data[:50]])
                    print(f"DEBUG: First 50 bytes: {hex_dump}")
                    
                    # Use the decoder to get the raw data
                    decoder = ByteStructureDecoder()
                    formatted_data = {}
                    
                    if hasattr(decoder, 'decode_neuron_flat'):
                        # Try to decode as neuron flat format
                        try:
                            decoded_data = decoder.decode_neuron_flat(data)
                            
                            # Convert to the desired dictionary format
                            cortical_ids = decoded_data.get('cortical_ids', [])
                            x_coords = decoded_data.get('x_coords', [])
                            y_coords = decoded_data.get('y_coords', [])
                            z_coords = decoded_data.get('z_coords', [])
                            potentials = decoded_data.get('potentials', [])
                            
                            # Group by cortical ID (first 6 letters)
                            for i in range(len(cortical_ids)):
                                cortical_id = cortical_ids[i][:6].ljust(6)
                                if cortical_id not in formatted_data:
                                    formatted_data[cortical_id] = [[], [], [], []]
                                
                                formatted_data[cortical_id][0].append(x_coords[i])
                                formatted_data[cortical_id][1].append(y_coords[i])
                                formatted_data[cortical_id][2].append(z_coords[i])
                                formatted_data[cortical_id][3].append(potentials[i])
                        except Exception as e:
                            logger.info(f"Failed to use decoder.decode_neuron_flat: {e}, trying fallback method")
                    else:
                        logger.info("Cannot decode neuron data: decoder.decode_neuron_flat not available, using fallback method")
                    
                    # If the regular decoder failed, try our custom extraction function
                    if not formatted_data:
                        logger.info("Using fallback byte extraction method")
                        formatted_data = extract_neuron_data_from_bytes(data)
                        
                    # If still no formatted data, try one last approach - raw hex dump
                    if not formatted_data:
                        logger.error("All decoding methods failed, trying direct raw dump")
                        
                        # Print the raw bytes for analysis
                        print("\n============= DIRECT RAW BYTES DUMP (FIRST 100) =============")
                        raw_hex = ' '.join(f'{b:02x}' for b in data[:100])
                        print(raw_hex)
                        print("===============================================================\n")
                        
                        # Also try to interpret as JSON directly
                        try:
                            raw_json = json.loads(data.decode('utf-8', errors='ignore'))
                            print("RAW JSON CONTENT:")
                            print(json.dumps(raw_json, indent=2))
                            formatted_data = raw_json
                        except:
                            pass
                            
                    # Log the formatted data exactly as requested
                    if formatted_data:
                        logger.info("Raw neuron activation data:")
                        # Format without pretty-printing for exact requested format
                        raw_data_str = json.dumps(formatted_data, separators=(',', ':'))
                        
                        # Use ERROR level to ensure it's visible in logs
                        logger.error("TEST VISUALIZATION RAW DATA:")
                        logger.error(raw_data_str)
                        
                        # Also use direct print for maximum visibility
                        print("\n=============== TEST VISUALIZATION RAW DATA ===============")
                        print(raw_data_str)
                        print("===========================================================\n")
                        
                        # Also log a summary for easier reading
                        total_neurons = 0
                        area_summary = []
                        for area_id, coords in formatted_data.items():
                            neuron_count = len(coords[0])
                            total_neurons += neuron_count
                            area_summary.append(f"{area_id}({neuron_count})")
                        
                        logger.info(f"Test visualization summary: {total_neurons} neurons across {len(formatted_data)} areas: {', '.join(area_summary)}")
                    else:
                        logger.info("No neuron data could be extracted from the byte stream")
                        
                except Exception as e:
                    logger.info(f"Could not decode visualization data for test mode: {e}")
                    import traceback
                    logger.debug(traceback.format_exc())
                
            # Apply rate limiting if needed
            if not self.rate_limiter.check_rate("visualization", 0.05):  # Max 20Hz
                logger.debug("Rate limiting visualization data")
                return
                
            # Send multipart message with topic and data
            await self.socket.send_multipart([
                b"activity",  # Topic
                data          # Binary data
            ])
            
            logger.debug(f"Sent {len(data)} bytes of visualization data")
            
        except Exception as e:
            logger.error(f"Error sending visualization data: {e}")
            
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