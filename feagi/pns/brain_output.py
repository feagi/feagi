"""
FEAGI Brain Output Manager

Global manager for all FEAGI outputs (motor/action targets).
Uses Rust MotorDeviceCache for high-performance decoding.
"""

from typing import List, Optional, TYPE_CHECKING, Dict, Any
import logging
import sys
import time
from datetime import datetime

if TYPE_CHECKING:
    from feagi.pns.outputs.base import BaseOutput
    from feagi.pns.observability.monitor import Monitor

# Configure SDK logger with console handler (ensures output is visible)
logger = logging.getLogger("feagi.pns.brain_output")
if not logger.handlers:
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(asctime)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False


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
        # Rust MotorDeviceCache (lazy-initialized)
        self._cache = None
        self._cache_available = False
        
        # Registry of all outputs
        self._outputs: List['BaseOutput'] = []
        
        # Transport (ZMQ/WebSocket)
        self._zmq_context = None  # MUST keep context alive!
        self._transport = None
        self._connected = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Latest motor data (for debugging)
        self._motor_data: Dict[str, Any] = {}
        
        # Configuration
        self._agent_id = None
        self._feagi_host = "localhost"
        self._feagi_port = 5564
        self._feagi_api_port = 8000
        self._transport_type = "zmq"
        
        # Observability monitors
        self._monitors: List['Monitor'] = []
    
    def _init_cache(self):
        """Initialize Rust MotorDeviceCache (lazy)"""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            self._cache = frpl.connector_core.caching.MotorDeviceCache()
            self._cache_available = True
            logger.info("✅ Rust MotorDeviceCache initialized")
        except ImportError as e:
            logger.error(f"❌ Failed to initialize Rust MotorDeviceCache: {e}")
            logger.error("   Install with: pip install feagi_rust_py_libs")
            raise ImportError(
                "Rust SDK (feagi_rust_py_libs) is required for brain_output.\n"
                "Install with: pip install feagi_rust_py_libs"
            ) from e
    
    def _register_with_feagi(self):
        """Register agent with FEAGI HTTP API including motor subscriptions"""
        import requests
        import base64
        
        # Collect all unique cortical IDs from registered outputs
        cortical_ids = set()
        for output in self._outputs:
            if hasattr(output, '_get_cortical_id'):
                cortical_ids.add(output._get_cortical_id())
            else:
                # Default: use "opse" for servos (encoded as base64)
                from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
                if isinstance(output, ServoMotor):
                    # "opse" padded to 8 bytes
                    cid_bytes = b"opse\x00\x00\x00\x00"
                    cortical_ids.add(base64.b64encode(cid_bytes).decode())
                elif isinstance(output, RotaryMotor):
                    # "omot" padded to 8 bytes
                    cid_bytes = b"omot\x00\x00\x00\x00"
                    cortical_ids.add(base64.b64encode(cid_bytes).decode())
        
        # If no specific IDs, subscribe to common positional servo area
        if not cortical_ids:
            cortical_ids = {base64.b64encode(b"opse\x00\x00\x00\x00").decode()}
        
        # Build registration request matching /v1/agent/register schema
        request_payload = {
            "agent_id": self._agent_id,
            "agent_type": "motor",  # Motor-only agent
            "agent_data_port": 0,  # Not applicable for motor-only agents
            "agent_version": "2.0.1",
            "controller_version": "2.0.1",
            "capabilities": {
                "motor": {
                    "modality": "generic",
                    "output_count": len(self._outputs),
                    "source_cortical_areas": list(cortical_ids)
                }
            },
            "chosen_transport": "zmq"
        }
        
        # Call registration endpoint
        try:
            url = f"http://{self._feagi_host}:{self._feagi_api_port}/v1/agent/register"
            logger.info(f"📝 POST {url}")
            logger.info(f"📝 Registering with cortical IDs: {list(cortical_ids)}")
            
            response = requests.post(url, json=request_payload, timeout=5)
            response.raise_for_status()
            
            result = response.json()
            logger.info(f"✅ Agent registered: {result.get('message', 'OK')}")
            
            # Log subscription confirmation
            logger.info(f"✅ Motor subscriptions registered for {len(cortical_ids)} cortical areas")
            
        except Exception as e:
            logger.error(f"❌ Registration failed: {e}")
            raise RuntimeError(f"Failed to register with FEAGI: {e}") from e
    
    def _allocate_group_id(self) -> int:
        """Allocate next cortical group ID"""
        group_id = self._next_group_id
        self._next_group_id += 1
        return group_id
    
    def configure(
        self,
        agent_id: str,
        feagi_host: str = "localhost",
        feagi_port: int = 5564,
        feagi_api_port: int = 8000,
        transport: str = "zmq"
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            agent_id: Unique agent identifier (required for registration)
            feagi_host: FEAGI server hostname or IP
            feagi_port: Motor output port (default: 5564)
            feagi_api_port: FEAGI API port (default: 8000)
            transport: Transport type - "zmq" or "websocket"
        """
        self._init_cache()
        self._agent_id = agent_id
        self._feagi_host = feagi_host
        self._feagi_port = feagi_port
        self._feagi_api_port = feagi_api_port
        self._transport_type = transport
        
        logger.info(f"📡 Configured: agent={agent_id}, {transport}://{feagi_host}:{feagi_port}")
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Registers agent with FEAGI PNS and establishes transport connection.
        """
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        if not self._agent_id:
            raise RuntimeError("Agent ID not set. Call configure(agent_id='...') first.")
        
        # Step 1: Register with FEAGI PNS
        logger.info(f"📝 Registering agent '{self._agent_id}' with FEAGI...")
        self._register_with_feagi()
        
        # Step 2: Initialize transport
        if self._transport_type == "zmq":
            import zmq
            self._zmq_context = zmq.Context()  # Store as instance variable to prevent garbage collection!
            self._transport = self._zmq_context.socket(zmq.SUB)
            self._transport.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to ALL messages (motor data not prefixed with agent_id)
            self._transport.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout for non-blocking
            endpoint = f"tcp://{self._feagi_host}:{self._feagi_port}"
            self._transport.connect(endpoint)
            logger.info(f"🔗 Connected to FEAGI ZMQ at {endpoint}")
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")
        
        self._connected = True
        logger.info(f"✅ Agent '{self._agent_id}' connected successfully")
    
    def disconnect(self):
        """Disconnect from FEAGI"""
        if self._transport:
            try:
                self._transport.close()
            except Exception as e:
                logger.warning(f"Error closing transport: {e}")
            self._transport = None
        if self._zmq_context:
            try:
                self._zmq_context.term()
            except Exception as e:
                logger.warning(f"Error terminating ZMQ context: {e}")
            self._zmq_context = None
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
    
    def _notify_monitors_receive_start(self, data: Dict[str, Any]):
        """Notify all monitors of receive start."""
        for monitor in self._monitors:
            try:
                monitor.on_receive_start(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
    def _notify_monitors_receive_complete(self, data: Dict[str, Any]):
        """Notify all monitors of receive complete."""
        for monitor in self._monitors:
            try:
                monitor.on_receive_complete(data)
            except Exception as e:
                logger.error(f"Error in monitor {monitor.__class__.__name__}: {e}")
    
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
        
        # Start timing
        start_time = time.perf_counter()
        
        # Heartbeat removed - was causing performance issues
        
        # Notify monitors of receive start
        self._notify_monitors_receive_start({
            'timestamp': datetime.now(),
            'output_count': len(self._outputs)
        })
        
        try:
            # Receive from transport
            motor_bytes = b""
            command_count = 0
            
            if self._transport:
                try:
                    # Non-blocking receive (will raise zmq.Again if no data)
                    import zmq
                    motor_bytes = self._transport.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    # No data available right now - this is normal for non-blocking recv
                    pass
                except Exception as e:
                    logger.error(f"Error receiving data: {e}")
                    raise
            
            # Decode and process motor bytes
            if motor_bytes:
                # CRITICAL: Check if this is motor data (version 2) or something else
                if len(motor_bytes) > 0 and motor_bytes[0] != 0x02:
                    # Skip non-motor messages (e.g., agent_id, heartbeat, etc.)
                    pass
                else:
                    # Process valid motor data
                    self._cache.process_neurons(list(motor_bytes))
                    command_count = 1
            
            # Note: _read_from_cache() is no longer needed as callbacks handle updates
            # The motor values are already updated via callbacks during process_neurons()
            
            logger.debug(f"📥 Updated {len(self._outputs)} outputs")
            
            # Calculate metrics
            duration_ms = (time.perf_counter() - start_time) * 1000.0
            
            # Notify monitors of receive complete
            self._notify_monitors_receive_complete({
                'timestamp': datetime.now(),
                'command_count': command_count,
                'duration_ms': duration_ms
            })
            
        except Exception as e:
            # Notify monitors of error
            for monitor in self._monitors:
                try:
                    monitor.on_error(e, {'operation': 'receive', 'stage': 'processing'})
                except Exception:
                    pass
            raise
    
    def get_output_count(self) -> int:
        """Get number of registered outputs"""
        return len(self._outputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_output = BrainOutput()

