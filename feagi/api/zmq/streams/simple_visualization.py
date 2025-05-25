"""
Simplified ZMQ Visualization Stream for FEAGI

Pure PUB/SUB pattern - no registration, no heartbeats, no complex state management.
Just publishes neural activity data when available, letting subscribers connect/disconnect freely.

Design principles:
- High performance: Minimal overhead, no artificial bottlenecks
- High reliability: No complex state dependencies 
- Easy debugging: Simple data flow, clear logging
- Easy maintenance: Minimal code, clear responsibilities
"""

import time
import threading
from typing import Dict, Any, Optional
from queue import Queue, Empty
import zmq  # Import standard synchronous ZMQ (not zmq.asyncio)

from feagi.utils.logger import setup_logger

# DO NOT import zmq.asyncio or any async ZMQ functionality
# from feagi.utils.zmq_debug import log_zmq_multipart_outbound  # Causes async issues

logger = setup_logger(__name__)


class SimpleVisualizationStream:
    """
    Simplified ZMQ Visualization Stream using pure PUB/SUB pattern.
    
    Key simplifications:
    - No client registration required
    - No heartbeat mechanisms  
    - No STANDBY mode blocking
    - No complex state dependencies
    - Just publishes data when available
    """
    
    def __init__(
        self, 
        host: str = "*", 
        port: int = 5562,
        context: Optional[zmq.Context] = None,
        fq_sampler_queue: Optional[Any] = None
    ):
        """Initialize the simplified visualization stream."""
        self.host = host
        self.port = port
        self.running = False
        # ALWAYS create a NEW sync context - NEVER use shared contexts that might be async
        self.context = zmq.Context()
        self.fq_sampler_queue = fq_sampler_queue
        
        # Simple socket setup
        self.socket = None
        self._setup_socket()
        
        # Single worker thread for data processing
        self.worker_thread = None
        self._stop_event = threading.Event()
        
        # Simple statistics
        self.stats = {
            'data_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time()
        }

    def _setup_socket(self) -> None:
        """Set up the ZMQ PUB socket with optimal settings."""
        # Create a PURELY SYNCHRONOUS PUB socket 
        self.socket = self.context.socket(zmq.PUB)
        
        # Ensure this is synchronous by checking the socket type
        logger.info(f"🔧 Created ZMQ socket type: {type(self.socket)}")
        
        # Optimize for real-time streaming but ALLOW queuing when no subscribers
        self.socket.setsockopt(zmq.SNDHWM, 1000)   # Higher send buffer to prevent drops
        self.socket.setsockopt(zmq.LINGER, 1000)   # Wait briefly on close to send pending messages
        # REMOVED: self.socket.setsockopt(zmq.IMMEDIATE, 1) - This drops messages when no subscribers!
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        self.socket.bind(bind_addr)
        logger.info(f"📡 Simple visualization stream bound to {bind_addr}")
        logger.info(f"🔧 Socket will queue messages even when no subscribers are connected")

    def start(self) -> None:
        """Start the visualization stream."""
        if self.running:
            return
            
        logger.info("🚀 Starting simple visualization stream")
        self.running = True
        self._stop_event.clear()
        
        # Single worker thread for processing FQ data
        if self.fq_sampler_queue:
            self.worker_thread = threading.Thread(
                target=self._data_worker,
                name="SimpleVisualization",
                daemon=True
            )
            self.worker_thread.start()
            logger.info("✅ Simple visualization stream started")
        else:
            logger.warning("⚠️ No FQ sampler queue provided - no data will be processed")

    def stop(self) -> None:
        """Stop the visualization stream."""
        if not self.running:
            return
            
        logger.info("🛑 Stopping simple visualization stream")
        self.running = False
        self._stop_event.set()
        
        # Wait for worker thread
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        
        # Close socket
        if self.socket:
            self.socket.close()
            self.socket = None
            
        logger.info("✅ Simple visualization stream stopped")

    def _data_worker(self) -> None:
        """
        Simple data processing worker.
        Just pulls data from queue and publishes it - no complex logic.
        """
        logger.info("🎬 Simple visualization data worker started")
        
        while self.running and not self._stop_event.is_set():
            try:
                # Try to get data from queue (non-blocking)
                fq_data = None
                try:
                    if hasattr(self.fq_sampler_queue, 'get'):
                        fq_data = self.fq_sampler_queue.get(timeout=0.1)
                    elif hasattr(self.fq_sampler_queue, '_queue') and len(self.fq_sampler_queue._queue) > 0:
                        fq_data = self.fq_sampler_queue._queue.pop(0)
                except Empty:
                    continue
                except Exception as e:
                    logger.debug(f"Queue access error: {e}")
                    time.sleep(0.01)
                    continue
                
                if fq_data is None:
                    continue
                
                # Process and send data based on type
                if isinstance(fq_data, bytes):
                    # Already serialized data
                    self._publish_data(fq_data)
                    
                elif isinstance(fq_data, dict) and 'target' in fq_data:
                    # Tagged format from enhanced FQ sampler
                    if fq_data.get('target') == 'visualization':
                        data = fq_data.get('data')
                        if isinstance(data, bytes):
                            self._publish_data(data)
                        
                elif isinstance(fq_data, tuple) and len(fq_data) == 2:
                    # Legacy (cortical_id, fire_data) tuple format
                    logger.info(f"📤 Processing neural data for: {fq_data[0]}")
                    self._process_tuple_data(fq_data)
                    
                elif isinstance(fq_data, dict):
                    # Legacy fire queue dict format
                    self._process_dict_data(fq_data)
                    
                else:
                    logger.debug(f"Ignoring unsupported data type: {type(fq_data)}")
                
            except Exception as e:
                logger.error(f"Error in data worker: {e}")
                time.sleep(0.1)  # Brief pause on error
                
        logger.info("🛑 Simple visualization data worker stopped")

    def _publish_data(self, data: bytes) -> None:
        """
        Publish data on the 'activity' topic.
        Use the most basic synchronous ZMQ operations.
        """
        try:
            # Use the most basic synchronous send operations
            # Send topic first
            self.socket.send(b"activity", zmq.SNDMORE)
            # Send data
            self.socket.send(data)
            
            # Update simple statistics
            self.stats['data_sent'] += 1
            self.stats['bytes_sent'] += len(data)
            
            # More frequent debug logging to verify publishing
            if self.stats['data_sent'] % 10 == 0:  # Every 10 messages instead of 100
                logger.info(f"📊 Published {self.stats['data_sent']} messages, {self.stats['bytes_sent']} bytes total")
            
            # Log first few messages to confirm publishing is working
            if self.stats['data_sent'] <= 5:
                logger.info(f"🚀 Successfully published message #{self.stats['data_sent']} ({len(data)} bytes)")
            
        except Exception as e:
            logger.error(f"Error publishing data: {e}")
            import traceback
            logger.error(f"Publishing traceback: {traceback.format_exc()}")

    def _process_tuple_data(self, fq_data) -> None:
        """Process legacy tuple format data."""
        try:
            cortical_id, fire_data = fq_data
            
            # LOG RAW DATA FOR DEBUGGING
            logger.info(f"🔬 RAW FIRE_DATA for {cortical_id}:")
            logger.info(f"   📋 Keys: {list(fire_data.keys()) if fire_data else 'None'}")
            if fire_data:
                for key, value in fire_data.items():
                    if isinstance(value, list):
                        logger.info(f"   🔑 {key}: {len(value)} items - FULL DATA: {value}")
                    else:
                        logger.info(f"   🔑 {key}: {value}")
            
            if fire_data and 'neuron_ids' in fire_data:
                neuron_ids = fire_data.get('neuron_ids', [])
                neuron_count = len(neuron_ids)
                
                # Handle coordinates - NO FALLBACKS, fail if missing
                coordinates = fire_data.get('coordinates', [])
                x_coords = []
                y_coords = []
                z_coords = []
                
                if isinstance(coordinates, list) and len(coordinates) > 0:
                    # If coordinates is a list of [x, y, z] triplets
                    if isinstance(coordinates[0], (list, tuple)) and len(coordinates[0]) >= 3:
                        x_coords = [coord[0] for coord in coordinates]
                        y_coords = [coord[1] for coord in coordinates]  
                        z_coords = [coord[2] for coord in coordinates]
                        logger.info(f"✅ Using provided coordinates (triplet format)")
                    else:
                        # If coordinates is a flat list, assume it's organized as [x1,y1,z1,x2,y2,z2,...]
                        coords_per_neuron = 3
                        x_coords = coordinates[0::coords_per_neuron]
                        y_coords = coordinates[1::coords_per_neuron]
                        z_coords = coordinates[2::coords_per_neuron]
                        logger.info(f"✅ Using provided coordinates (flat format)")
                elif isinstance(coordinates, dict):
                    # If coordinates is a dict with x, y, z keys
                    x_coords = coordinates.get('x', [])
                    y_coords = coordinates.get('y', [])
                    z_coords = coordinates.get('z', [])
                    logger.info(f"✅ Using provided coordinates (dict format)")
                else:
                    # NO FALLBACK - FAIL if coordinates are missing
                    logger.error(f"❌ MISSING COORDINATES for {cortical_id} - fire_data has no valid coordinates!")
                    logger.error(f"❌ Coordinates field: {coordinates}")
                    return
                
                # Handle membrane potentials - NO FALLBACKS, fail if missing
                membrane_potentials = fire_data.get('membrane_potentials', [])
                
                if not membrane_potentials:
                    logger.error(f"❌ MISSING MEMBRANE POTENTIALS for {cortical_id} - fire_data has no membrane_potentials field!")
                    return
                
                if len(membrane_potentials) != neuron_count:
                    logger.error(f"❌ MEMBRANE POTENTIAL COUNT MISMATCH for {cortical_id}:")
                    logger.error(f"   🧠 Neuron count: {neuron_count}")
                    logger.error(f"   ⚡ Membrane potential count: {len(membrane_potentials)}")
                    return
                
                # Validate coordinate arrays
                if len(x_coords) != neuron_count or len(y_coords) != neuron_count or len(z_coords) != neuron_count:
                    logger.error(f"❌ COORDINATE COUNT MISMATCH for {cortical_id}:")
                    logger.error(f"   🧠 Neuron count: {neuron_count}")
                    logger.error(f"   📍 X coords: {len(x_coords)}")
                    logger.error(f"   📍 Y coords: {len(y_coords)}")
                    logger.error(f"   📍 Z coords: {len(z_coords)}")
                    return
                
                # LOG FINAL DATA BEFORE ENCODING - SHOW ALL DATA
                logger.info(f"🎯 VALIDATED DATA for {cortical_id} ({neuron_count} neurons):")
                logger.info(f"   🔢 ALL Neuron IDs: {neuron_ids}")
                logger.info(f"   📍 ALL X coords: {x_coords}")
                logger.info(f"   📍 ALL Y coords: {y_coords}")
                logger.info(f"   📍 ALL Z coords: {z_coords}")
                logger.info(f"   ⚡ ALL Potentials: {membrane_potentials}")
                
                # Create cortical IDs list (same cortical ID for all neurons)
                cortical_ids = [cortical_id] * neuron_count
                
                # Encode using feagi_bytes binary format
                try:
                    from feagi_bytes import ByteStructureEncoder
                    encoder = ByteStructureEncoder()
                    
                    binary_data = encoder.encode_neuron_flat(
                        cortical_ids=cortical_ids,
                        x_coords=x_coords,
                        y_coords=y_coords,
                        z_coords=z_coords,
                        potentials=membrane_potentials
                    )
                    
                    # Publish the binary data
                    self._publish_data(binary_data)
                    logger.info(f"✅ Published {cortical_id}: {neuron_count} neurons, {len(binary_data)} bytes (BINARY)")
                    
                except ImportError:
                    logger.error("❌ feagi_bytes library not available - cannot encode binary data")
                except Exception as e:
                    logger.error(f"❌ Error encoding binary data: {e}")
            else:
                logger.error(f"❌ INVALID FIRE_DATA for {cortical_id} - missing neuron_ids or fire_data is None")
                
        except Exception as e:
            logger.error(f"❌ Error processing {fq_data[0] if len(fq_data) > 0 else 'unknown'}: {e}")
            import traceback
            logger.error(f"❌ Traceback: {traceback.format_exc()}")

    def _process_dict_data(self, fire_data) -> None:
        """Process legacy dict format data."""
        try:
            if fire_data and 'neuron_ids' in fire_data:
                neuron_count = len(fire_data.get('neuron_ids', []))
                logger.debug(f"Received dict data: {neuron_count} neurons")
                # TODO: Implement proper binary serialization
                
        except Exception as e:
            logger.error(f"Error processing dict data: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """Get simple statistics."""
        runtime = time.time() - self.stats['start_time']
        return {
            'running': self.running,
            'data_sent': self.stats['data_sent'],
            'bytes_sent': self.stats['bytes_sent'],
            'runtime_seconds': runtime,
            'messages_per_second': self.stats['data_sent'] / max(runtime, 1)
        }

    # COMPATIBILITY METHODS (so existing code doesn't break)
    
    def register_visualization_client(self, client_id: str) -> None:
        """Compatibility method - does nothing in simple mode."""
        logger.debug(f"Simple mode: ignoring client registration for {client_id}")
        
    def unregister_visualization_client(self, client_id: str) -> None:
        """Compatibility method - does nothing in simple mode."""
        logger.debug(f"Simple mode: ignoring client unregistration for {client_id}")
        
    def heartbeat_visualization_client(self, client_id: str) -> None:
        """Handle visualization client heartbeat and enable FQ sampler if needed."""
        logger.info(f"🔧 DEBUG: SimpleVisualization heartbeat from client: {client_id}")
        
        # Get the process manager to enable FQ sampler
        try:
            from feagi.process_manager import get_process_manager
            process_manager = get_process_manager()
            
            if process_manager:
                logger.info(f"🔧 DEBUG: Got process manager, looking for FQ sampler...")
                
                # Get the FQ sampler from the process manager
                if hasattr(process_manager, '_fq_sampler') and process_manager._fq_sampler:
                    fq_sampler = process_manager._fq_sampler
                    logger.info(f"🔧 DEBUG: Found FQ sampler: {type(fq_sampler)}")
                    
                    # Enable visualization subscribers
                    if hasattr(fq_sampler, 'set_visualization_subscribers'):
                        logger.info(f"🔧 DEBUG: Enabling visualization subscribers for {client_id}")
                        fq_sampler.set_visualization_subscribers(True)
                        logger.info(f"✅ FQ sampler visualization subscribers enabled for client: {client_id}")
                    else:
                        logger.warning(f"⚠️ FQ sampler doesn't have set_visualization_subscribers method")
                        logger.info(f"FQ sampler methods: {[m for m in dir(fq_sampler) if not m.startswith('_')]}")
                else:
                    logger.warning(f"⚠️ Process manager has no _fq_sampler or it's None")
            else:
                logger.warning(f"⚠️ No process manager available for FQ sampler control")
                
        except Exception as e:
            logger.error(f"❌ Error enabling FQ sampler for {client_id}: {e}")
            import traceback
            logger.error(f"❌ Traceback: {traceback.format_exc()}")
            
        # Could also track heartbeats here if needed for SimpleVisualizationStream
        # For now, just log that we received it
        logger.debug(f"💗 Heartbeat received from visualization client: {client_id}")
        
    def send_visualization_data(self, data) -> None:
        """Compatibility method for external data sending."""
        if isinstance(data, bytes):
            self._publish_data(data) 