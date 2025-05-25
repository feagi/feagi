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
import zmq

from feagi.utils.logger import setup_logger
from feagi.utils.zmq_debug import log_zmq_multipart_outbound

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
        self.context = context or zmq.Context.instance()
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
        self.socket = self.context.socket(zmq.PUB)
        
        # Optimize for real-time streaming
        self.socket.setsockopt(zmq.SNDHWM, 100)    # Reasonable send buffer
        self.socket.setsockopt(zmq.LINGER, 0)      # Don't wait on close
        self.socket.setsockopt(zmq.IMMEDIATE, 1)   # Don't queue if no subscribers
        
        bind_addr = f"tcp://{self.host}:{self.port}"
        self.socket.bind(bind_addr)
        logger.info(f"📡 Simple visualization stream bound to {bind_addr}")

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
        Simple, no barriers, no complex checks.
        """
        try:
            # Just send the data - let ZMQ handle subscribers
            self.socket.send_multipart([
                b"activity",
                data
            ])
            
            # Update simple statistics
            self.stats['data_sent'] += 1
            self.stats['bytes_sent'] += len(data)
            
            # Occasional debug logging
            if self.stats['data_sent'] % 100 == 0:
                logger.debug(f"📊 Published {self.stats['data_sent']} messages, {self.stats['bytes_sent']} bytes total")
                
            # ZMQ debugging - log outbound visualization data
            endpoint = f"tcp://{self.host}:{self.port}"
            log_zmq_multipart_outbound(
                endpoint=endpoint,
                multipart_data=[b"activity", data],
                context=f"Simple visualization stream message #{self.stats['data_sent']}",
                message_type="visualization_activity"
            )
            
        except Exception as e:
            logger.error(f"Error publishing data: {e}")

    def _process_tuple_data(self, fq_data) -> None:
        """Process legacy tuple format data."""
        try:
            cortical_id, fire_data = fq_data
            if fire_data and 'neuron_ids' in fire_data:
                # Convert to simple binary format (this would need proper serialization)
                # For now, just log that we received it
                neuron_count = len(fire_data.get('neuron_ids', []))
                logger.debug(f"Received tuple data for {cortical_id}: {neuron_count} neurons")
                # TODO: Implement proper binary serialization
                
        except Exception as e:
            logger.error(f"Error processing tuple data: {e}")

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
        """Compatibility method - does nothing in simple mode."""
        pass  # Silent ignore for heartbeats
        
    def send_visualization_data(self, data) -> None:
        """Compatibility method for external data sending."""
        if isinstance(data, bytes):
            self._publish_data(data) 