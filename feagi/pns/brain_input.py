"""
FEAGI Brain Input Manager

Global manager for all FEAGI inputs (sensory data sources).
Uses Rust IOCache for high-performance encoding.
"""

from typing import List, Optional, TYPE_CHECKING
import logging

if TYPE_CHECKING:
    from feagi.pns.inputs.base import BaseInput

logger = logging.getLogger("feagi.pns.brain_input")


class BrainInput:
    """
    Global brain input manager.
    
    Manages all registered inputs and handles automatic encoding
    and transmission to FEAGI. Uses Rust IOCache for performance.
    
    This is a singleton - use the module-level `brain_input` instance.
    
    Example:
        from feagi.pns.inputs import Camera
        from feagi.pns import brain_input
        
        # Register inputs
        camera = Camera.register(resolution=(1920, 1080))
        
        # Configure and connect
        brain_input.configure(feagi_host="localhost")
        brain_input.connect()
        
        # Main loop
        while True:
            camera.set_frame(frame)
            brain_input.send()  # Encodes and sends all inputs
    """
    
    def __init__(self):
        # Rust IOCache (lazy-initialized)
        self._cache = None
        self._cache_available = False
        
        # Registry of all inputs
        self._inputs: List['BaseInput'] = []
        
        # Transport (ZMQ/WebSocket)
        self._transport = None
        self._connected = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Configuration
        self._feagi_host = "localhost"
        self._feagi_port = 5558
        self._transport_type = "zmq"
    
    def _init_cache(self):
        """Initialize Rust IOCache (lazy)"""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            self._cache = frpl.io_processing.cache.IOCache()
            self._cache_available = True
            logger.info("✅ Rust IOCache initialized")
        except ImportError as e:
            logger.error(f"❌ Failed to initialize Rust IOCache: {e}")
            logger.error("   Install with: pip install feagi_rust_py_libs")
            raise ImportError(
                "Rust SDK (feagi_rust_py_libs) is required for brain_input.\n"
                "Install with: pip install feagi_rust_py_libs"
            ) from e
    
    def _allocate_group_id(self) -> int:
        """Allocate next cortical group ID"""
        group_id = self._next_group_id
        self._next_group_id += 1
        return group_id
    
    def configure(
        self,
        feagi_host: str = "localhost",
        feagi_port: int = 5558,
        transport: str = "zmq"
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            feagi_host: FEAGI server hostname or IP
            feagi_port: Sensory input port (default: 5558)
            transport: Transport type - "zmq" or "websocket"
        """
        self._init_cache()
        self._feagi_host = feagi_host
        self._feagi_port = feagi_port
        self._transport_type = transport
        
        logger.info(f"📡 Configured: {transport}://{feagi_host}:{feagi_port}")
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Initializes transport and establishes connection.
        """
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        # TODO: Initialize actual transport (ZMQ/WebSocket)
        # For now, just mark as connected
        self._connected = True
        logger.info(f"🔗 Connected to FEAGI at {self._feagi_host}:{self._feagi_port}")
    
    def disconnect(self):
        """Disconnect from FEAGI"""
        if self._transport:
            # TODO: Close transport
            pass
        self._connected = False
        logger.info("🔌 Disconnected from FEAGI")
    
    def register_input(self, input_instance: 'BaseInput'):
        """
        Register an input (called internally by input classes).
        
        Args:
            input_instance: Input instance to register
        """
        self._init_cache()
        
        # Allocate group ID
        group_id = self._allocate_group_id()
        
        # Register with Rust cache
        input_instance._register_with_cache(self._cache, group_id)
        input_instance._mark_registered(group_id)
        
        # Add to registry
        self._inputs.append(input_instance)
        
        logger.debug(f"✅ Registered input: {input_instance.__class__.__name__} (group={group_id})")
    
    def send(self):
        """
        Send all input data to FEAGI.
        
        This is the main loop method:
        1. Updates all inputs to cache
        2. Encodes to neurons (Rust - fast!)
        3. Serializes to bytes
        4. Sends via transport
        
        Call this in your main loop after updating all input values.
        """
        if not self._connected:
            raise RuntimeError(
                "Not connected to FEAGI. Call brain_input.connect() first."
            )
        
        # Update all inputs to cache
        for input_instance in self._inputs:
            try:
                input_instance._write_to_cache(self._cache)
            except Exception as e:
                logger.error(f"Error writing {input_instance.__class__.__name__} to cache: {e}")
                raise
        
        # Encode all sensors to neurons (Rust)
        try:
            neuron_data = self._cache.sensor_encode_to_neurons()
        except Exception as e:
            logger.error(f"Error encoding sensors to neurons: {e}")
            raise
        
        # Serialize to bytes
        try:
            byte_struct = neuron_data.as_new_feagi_byte_structure()
            serialized = byte_struct.copy_out_as_byte_vector()
        except Exception as e:
            logger.error(f"Error serializing neuron data: {e}")
            raise
        
        # Send via transport
        if self._transport:
            try:
                self._transport.send(serialized)
            except Exception as e:
                logger.error(f"Error sending data: {e}")
                raise
        else:
            # TODO: For now, just log the size
            logger.debug(f"📤 Encoded {len(self._inputs)} inputs → {len(serialized)} bytes")
    
    def get_input_count(self) -> int:
        """Get number of registered inputs"""
        return len(self._inputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_input = BrainInput()

