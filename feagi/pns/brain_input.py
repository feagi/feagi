"""
FEAGI Brain Input Manager

Global manager for all FEAGI inputs (sensory data sources).
Uses Rust IOCache for high-performance encoding.
"""

from typing import List, Optional, TYPE_CHECKING, Any, Dict
import logging
import time
from datetime import datetime

if TYPE_CHECKING:
    from feagi.pns.inputs.base import BaseInput
    from feagi.pns.observability.monitor import Monitor

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
        
        # Observability monitors
        self._monitors: List['Monitor'] = []
    
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
    
    def attach_monitor(self, monitor: 'Monitor'):
        """
        Attach an observability monitor.
        
        Args:
            monitor: Monitor instance to attach
        """
        self._monitors.append(monitor)
        logger.debug(f"Attached monitor: {monitor.__class__.__name__}")
    
    def detach_monitor(self, monitor: 'Monitor'):
        """
        Detach an observability monitor.
        
        Args:
            monitor: Monitor instance to detach
        """
        if monitor in self._monitors:
            self._monitors.remove(monitor)
            logger.debug(f"Detached monitor: {monitor.__class__.__name__}")
    
    def _notify_monitors_send_start(self, data: Dict[str, Any]):
        """Notify all monitors of send start."""
        for monitor in self._monitors:
            try:
                monitor.on_send_start(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def _notify_monitors_send_complete(self, data: Dict[str, Any]):
        """Notify all monitors of send complete."""
        for monitor in self._monitors:
            try:
                monitor.on_send_complete(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
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
        
        # Start timing
        start_time = time.perf_counter()
        
        # Notify monitors of send start
        cortical_areas = [inp._cortical_area for inp in self._inputs if hasattr(inp, '_cortical_area')]
        self._notify_monitors_send_start({
            'timestamp': datetime.now(),
            'input_count': len(self._inputs),
            'cortical_areas': cortical_areas
        })
        
        try:
            # Update all inputs to cache
            for input_instance in self._inputs:
                try:
                    input_instance._write_to_cache(self._cache)
                except Exception as e:
                    logger.error(f"Error writing {input_instance.__class__.__name__} to cache: {e}")
                    raise
            
            # Get encoded byte container (Rust)
            try:
                serialized = self._cache.sensor_get_byte_container()
            except Exception as e:
                logger.error(f"Error getting sensor byte container: {e}")
                raise
            
            # Check if we have data
            if not serialized:
                logger.warning("No sensor data to send")
                return
            
            # Send via transport
            if self._transport:
                try:
                    self._transport.send(serialized)
                except Exception as e:
                    logger.error(f"Error sending data: {e}")
                    raise
            else:
                # No transport configured yet (shouldn't happen in normal use)
                logger.warning(f"No transport configured - cannot send {len(self._inputs)} inputs")
            
            # Calculate metrics
            duration_ms = (time.perf_counter() - start_time) * 1000.0
            packet_size = len(serialized) if serialized else 0
            
            # Estimate neuron count (this is approximate)
            neuron_count = packet_size // 4  # Rough estimate
            
            # Notify monitors of send complete
            self._notify_monitors_send_complete({
                'timestamp': datetime.now(),
                'packet_size_bytes': packet_size,
                'neuron_count': neuron_count,
                'duration_ms': duration_ms,
                'cortical_areas': cortical_areas
            })
            
        except Exception as e:
            # Notify monitors of error
            for monitor in self._monitors:
                try:
                    monitor.on_error(e, {'operation': 'send', 'stage': 'processing'})
                except Exception:
                    pass
            raise
    
    def get_input_count(self) -> int:
        """Get number of registered inputs"""
        return len(self._inputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_input = BrainInput()

