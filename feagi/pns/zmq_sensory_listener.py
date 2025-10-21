"""
ZMQ Sensory Data Listener for Python FEAGI.

Listens for sensory data from agents (like video_agent_rust) via ZMQ PULL socket
and injects it into the NPU.
"""

import json
import logging
import threading
from typing import Optional

import zmq

logger = logging.getLogger(__name__)


class ZmqSensoryListener:
    """
    Listens for ZMQ sensory data from agents and injects into NPU.
    
    Agents use ZMQ PUSH to send sensory data, this uses PULL to receive.
    Format: JSON with { "neuron_id_potential_pairs": [[id, potential], ...] }
    """

    def __init__(self, npu_interface, host: str = "*", port: int = 5555):
        self.npu_interface = npu_interface
        self.host = host
        self.port = port
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # ZMQ context and socket (created in start())
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        
        # Statistics
        self.total_messages_received = 0
        self.total_neurons_injected = 0
    
    def start(self):
        """Start the ZMQ sensory listener in a background thread"""
        if self.running:
            logger.warning("ZMQ sensory listener already running")
            return
        
        self.running = True
        self.thread = threading.Thread(
            target=self._listener_loop,
            name="ZMQ-Sensory-Listener",
            daemon=True
        )
        self.thread.start()
        logger.info(f"🦀 ZMQ sensory listener thread started for tcp://{self.host}:{self.port}")
    
    def stop(self):
        """Stop the ZMQ sensory listener"""
        if not self.running:
            return
        
        logger.info("Stopping ZMQ sensory listener...")
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        logger.info(f"ZMQ sensory listener stopped. Total messages: {self.total_messages_received}, Total neurons: {self.total_neurons_injected}")
    
    def _listener_loop(self):
        """Main loop for listening to ZMQ sensory data."""
        try:
            # Create ZMQ context and PULL socket
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PULL)
            
            # Bind to address
            bind_address = f"tcp://{self.host}:{self.port}"
            self.socket.bind(bind_address)
            
            # Set receive timeout so we can check self.running periodically
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            
            logger.info(f"✅ ZMQ sensory listener SUCCESSFULLY bound to {bind_address}")
            
            first_10_logged = False
            message_count = 0
            
            while self.running:
                try:
                    # Receive sensory data (with timeout)
                    message_bytes = self.socket.recv()
                    self.total_messages_received += 1
                    message_count += 1
                    
                    # Parse JSON message
                    try:
                        data = json.loads(message_bytes)
                        
                        # Extract neuron_id and potential pairs
                        # Format: { "neuron_id_potential_pairs": [[id, potential], ...] }
                        if "neuron_id_potential_pairs" in data:
                            pairs = data["neuron_id_potential_pairs"]
                            
                            if pairs:
                                # Convert to list of tuples for NPU injection
                                injection_data = [(int(pair[0]), float(pair[1])) for pair in pairs]
                                
                                # Log first 10 data points
                                if not first_10_logged and injection_data:
                                    logger.info(f"📥 [SENSORY-DATA] Received {len(injection_data)} neurons from agent")
                                    log_count = min(10, len(injection_data))
                                    for i in range(log_count):
                                        neuron_id, potential = injection_data[i]
                                        logger.info(f"  Neuron[{i}]: ID={neuron_id}, Potential={potential:.2f}")
                                    if len(injection_data) > 10:
                                        logger.info(f"  ... and {len(injection_data) - 10} more neurons")
                                    first_10_logged = True
                                
                                # Inject into NPU
                                try:
                                    self.npu_interface.inject_sensory_with_potentials(injection_data)
                                    self.total_neurons_injected += len(injection_data)
                                    
                                    # Log periodically
                                    if message_count % 100 == 0:
                                        logger.debug(f"📊 Sensory stats: {self.total_messages_received} messages, {self.total_neurons_injected} neurons total")
                                
                                except Exception as e:
                                    logger.error(f"❌ Failed to inject sensory data into NPU: {e}", exc_info=True)
                        
                        else:
                            logger.warning(f"⚠️ Sensory message missing 'neuron_id_potential_pairs' field: {data}")
                    
                    except json.JSONDecodeError as e:
                        logger.error(f"❌ Failed to parse sensory data JSON: {e}")
                        logger.debug(f"Raw message: {message_bytes[:100]}")  # Log first 100 bytes
                
                except zmq.Again:
                    # Timeout, no message received, continue loop to check self.running
                    pass
                except Exception as e:
                    logger.error(f"❌ Error processing ZMQ sensory message: {e}", exc_info=True)
        
        except zmq.ZMQError as e:
            error_msg = str(e)
            if "Address already in use" in error_msg:
                logger.critical(
                    f"❌ CRITICAL: Cannot bind to tcp://{self.host}:{self.port} - Address already in use! "
                    f"Another process is using port {self.port}."
                )
            else:
                logger.error(f"Fatal error in ZMQ sensory listener: {e}", exc_info=True)
            # Re-raise to ensure calling code knows about the failure
            raise
        finally:
            logger.info("ZMQ sensory listener loop exited")

