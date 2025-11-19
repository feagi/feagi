"""
FEAGI Brain Output Manager

Global manager for all FEAGI outputs (motor/action targets).
Uses Rust IOCache for high-performance decoding.
"""

from typing import List, Optional, TYPE_CHECKING, Dict, Any
import logging

if TYPE_CHECKING:
    from feagi.pns.outputs.base import BaseOutput

logger = logging.getLogger("feagi.pns.brain_output")


class BrainOutput:
    """
    Global brain output manager.
    
    Manages all registered outputs and handles automatic receiving
    and decoding of motor commands from FEAGI. Uses Rust IOCache.
    
    This is a singleton - use the module-level `brain_output` instance.
    
    Example:
        from feagi.pns.outputs import ServoMotor
        from feagi.pns import brain_output
        
        # Register outputs
        servo = ServoMotor.register(range=(0, 180))
        
        # Configure and connect
        brain_output.configure(feagi_host="localhost")
        brain_output.connect()
        
        # Main loop
        while True:
            brain_output.receive()  # Receives and decodes all outputs
            angle = servo.get_angle()
    """
    
    def __init__(self):
        # Rust IOCache (lazy-initialized)
        self._cache = None
        self._cache_available = False
        
        # Registry of all outputs
        self._outputs: List['BaseOutput'] = []
        
        # Transport (ZMQ/WebSocket)
        self._transport = None
        self._connected = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Latest motor data (for debugging)
        self._motor_data: Dict[str, Any] = {}
        
        # Configuration
        self._feagi_host = "localhost"
        self._feagi_port = 5564
        self._transport_type = "zmq"
    
    def _init_cache(self):
        """Initialize Rust IOCache (lazy)"""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            self._cache = frpl.connector_core.caching.IOCache()
            self._cache_available = True
            logger.info("✅ Rust IOCache initialized")
        except ImportError as e:
            logger.error(f"❌ Failed to initialize Rust IOCache: {e}")
            logger.error("   Install with: pip install feagi_rust_py_libs")
            raise ImportError(
                "Rust SDK (feagi_rust_py_libs) is required for brain_output.\n"
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
        feagi_port: int = 5564,
        transport: str = "zmq"
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            feagi_host: FEAGI server hostname or IP
            feagi_port: Motor output port (default: 5564)
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
    
    def register_output(self, output_instance: 'BaseOutput'):
        """
        Register an output (called internally by output classes).
        
        Args:
            output_instance: Output instance to register
        """
        self._init_cache()
        
        # Allocate group ID
        group_id = self._allocate_group_id()
        
        # Register with Rust cache
        output_instance._register_with_cache(self._cache, group_id)
        output_instance._mark_registered(group_id)
        
        # Add to registry
        self._outputs.append(output_instance)
        
        logger.debug(f"✅ Registered output: {output_instance.__class__.__name__} (group={group_id})")
    
    def receive(self):
        """
        Receive motor commands from FEAGI.
        
        This is the main loop method:
        1. Receives bytes from transport
        2. Decodes bytes to neurons (Rust)
        3. Updates motor cache
        4. Reads values from cache to outputs
        
        Call this in your main loop to update all output values.
        """
        if not self._connected:
            raise RuntimeError(
                "Not connected to FEAGI. Call brain_output.connect() first."
            )
        
        # Receive from transport
        if self._transport:
            try:
                motor_bytes = self._transport.receive()
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                raise
        else:
            # TODO: For now, no actual receiving
            motor_bytes = b""
        
        # Decode (TODO: implement motor decoding in Rust)
        if motor_bytes:
            try:
                # TODO: Decode motor_bytes to cache
                # motor_data = self._cache.motor_decode_from_bytes(motor_bytes)
                pass
            except Exception as e:
                logger.error(f"Error decoding motor data: {e}")
                raise
        
        # Read all outputs from cache
        for output_instance in self._outputs:
            try:
                output_instance._read_from_cache(self._cache)
            except Exception as e:
                logger.error(f"Error reading {output_instance.__class__.__name__} from cache: {e}")
                raise
        
        logger.debug(f"📥 Updated {len(self._outputs)} outputs")
    
    def get_output_count(self) -> int:
        """Get number of registered outputs"""
        return len(self._outputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_output = BrainOutput()

