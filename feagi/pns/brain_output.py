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
        
        # Channel index counter for motors (all motors use group=0, differentiated by channel)
        self._next_motor_channel = 0
        
        # Motor decoder tracking
        self._motor_total_channels = 0
        self._motor_decoder_registered = False
        
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
                # Construct cortical ID matching Rust cache registration
                from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
                if isinstance(output, ServoMotor):
                    # PositionalServo with SignedPercentage, Absolute, Linear, group=0
                    # Bytes: [111, 112, 115, 101, 4, 0, 0, 0]
                    # Byte 4 = 4 (SignedPercentage), Byte 5 = 0, Byte 6 = 0, Byte 7 = 0 (group)
                    cid_bytes = bytes([111, 112, 115, 101, 4, 0, 0, 0])  # "opse" + data_type_config=4 + group=0
                    cortical_ids.add(base64.b64encode(cid_bytes).decode())
                elif isinstance(output, RotaryMotor):
                    # RotaryMotor: construct similarly (may need adjustment based on actual requirements)
                    # For now, use default "omot" format
                    cid_bytes = b"omot\x00\x00\x00\x00"
                    cortical_ids.add(base64.b64encode(cid_bytes).decode())
        
        # If no specific IDs, subscribe to common positional servo area with SignedPercentage
        if not cortical_ids:
            # Default: PositionalServo with SignedPercentage (data_type_config=4)
            cid_bytes = bytes([111, 112, 115, 101, 4, 0, 0, 0])
            cortical_ids = {base64.b64encode(cid_bytes).decode()}
        
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
    
    def _register_motor_decoder(self):
        """Register motor decoder with Rust cache (called once with total channel count)"""
        import feagi_rust_py_libs as frpl
        from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
        
        # Determine motor type from first motor (assume all motors are same type for now)
        motor_type = None
        encoding = "absolute"
        for output in self._outputs:
            if isinstance(output, (ServoMotor, RotaryMotor)):
                if isinstance(output, ServoMotor):
                    motor_type = frpl.data_structures.genomic.MotorCorticalType.PositionalServo
                else:
                    motor_type = frpl.data_structures.genomic.MotorCorticalType.RotaryMotor
                encoding = output.encoding
                break
        
        if motor_type is None:
            return  # No motors registered
        
        # Register decoder with total channel count
        self._cache.register(
            motor_unit=motor_type,
            group=0,  # All motors share group=0
            channels=self._motor_total_channels,
            z_resolution=100,
            frame_change_handling=0 if encoding == "absolute" else 1,
            percentage_positioning=0  # Linear
        )
        
        # Now register all motor callbacks
        for output in self._outputs:
            if isinstance(output, (ServoMotor, RotaryMotor)):
                # Re-call _register_with_cache with decoder_registered=True
                output._register_with_cache(self._cache, output.group_id, output.channel, decoder_registered=True)
    
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
        
        # Step 0: Register motor decoder with total channel count (if motors were registered)
        if self._motor_total_channels > 0 and not self._motor_decoder_registered:
            self._register_motor_decoder()
            self._motor_decoder_registered = True
        
        # Step 1: Register with FEAGI PNS
        self._register_with_feagi()
        
        # Step 2: Initialize transport
        if self._transport_type == "zmq":
            import zmq
            self._zmq_context = zmq.Context()  # Store as instance variable to prevent garbage collection!
            self._transport = self._zmq_context.socket(zmq.SUB)
            # Subscribe to agent-specific messages (FEAGI sends multipart: [agent_id, data])
            self._transport.setsockopt(zmq.SUBSCRIBE, self._agent_id.encode())
            # Also subscribe to empty prefix to catch any messages (backup)
            self._transport.setsockopt(zmq.SUBSCRIBE, b"")
            # RCVTIMEO removed - using NOBLOCK flag instead
            endpoint = f"tcp://{self._feagi_host}:{self._feagi_port}"
            self._transport.connect(endpoint)
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")
        
        self._connected = True
    
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
    
    def register_output(self, output_instance: 'BaseOutput'):
        """
        Register an output (called internally by output classes).
        
        Args:
            output_instance: Output instance to register
        """
        self._init_cache()
        
        # Check if this is a motor output (ServoMotor or RotaryMotor)
        from feagi.pns.outputs.motor import ServoMotor, RotaryMotor
        is_motor = isinstance(output_instance, (ServoMotor, RotaryMotor))
        
        if is_motor:
            # Motors: Use group=0, assign unique channel index
            group_id = 0
            channel_index = self._next_motor_channel
            self._next_motor_channel += 1
            output_instance.channel = channel_index
            
            # Track total motor count for decoder registration
            self._motor_total_channels += 1
        else:
            # Other outputs: Use auto-incrementing group IDs
            group_id = self._allocate_group_id()
            channel_index = 0
        
        # Add to registry first (needed for connect() to know total count)
        self._outputs.append(output_instance)
        
        # Register with Rust cache (motors will defer decoder registration until connect())
        output_instance._register_with_cache(self._cache, group_id, channel_index, self._motor_decoder_registered)
        output_instance._mark_registered(group_id)
        
        if is_motor:
            logger.debug(f"✅ Registered output: {output_instance.__class__.__name__} (group={group_id}, channel={channel_index})")
        else:
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
        
        # All timing and monitoring removed for performance
        
        try:
            # Receive from transport
            motor_bytes = b""
            command_count = 0
            
            if self._transport:
                try:
                    # Non-blocking receive (will raise zmq.Again if no data)
                    # FEAGI sends multipart messages: [agent_id, data]
                    import zmq
                    parts = self._transport.recv_multipart(flags=zmq.NOBLOCK)
                    if len(parts) >= 2:
                        # Part 0: agent_id (topic), Part 1: motor data
                        agent_id_received = parts[0].decode('utf-8', errors='ignore')
                        motor_bytes = parts[1]
                        print(f"📥 RECEIVED multipart: agent_id='{agent_id_received}', data={len(motor_bytes)} bytes (first byte: 0x{motor_bytes[0]:02x})", flush=True)
                    elif len(parts) == 1:
                        # Fallback: single part message (old format?)
                        motor_bytes = parts[0]
                        print(f"📥 RECEIVED single part: {len(motor_bytes)} bytes (first byte: 0x{motor_bytes[0]:02x})", flush=True)
                    else:
                        # Empty message, skip
                        motor_bytes = b""
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
                    print(f"⏭️  SKIP: First byte is 0x{motor_bytes[0]:02x} (not 0x02)", flush=True)
                else:
                    # Process valid motor data (with error handling to prevent blocking)
                    try:
                        print(f"📥 Processing {len(motor_bytes)} bytes of motor data", flush=True)
                        self._cache.process_neurons(list(motor_bytes))
                        print(f"✅ process_neurons completed", flush=True)
                        command_count = 1
                    except Exception as e:
                        # Log error but don't crash - prevents blocking
                        print(f"❌ Error processing neurons: {e}", flush=True)
                        logger.debug(f"Error processing neurons: {e}")
                        pass
            else:
                # Log occasionally when no data is received (every 100 calls to avoid spam)
                if not hasattr(self, '_receive_call_count'):
                    self._receive_call_count = 0
                self._receive_call_count += 1
                if self._receive_call_count % 100 == 0:
                    print(f"💤 No data received (call #{self._receive_call_count})", flush=True)
            
            # Note: _read_from_cache() is no longer needed as callbacks handle updates
            # The motor values are already updated via callbacks during process_neurons()
            
        except Exception as e:
            # Monitor error notifications disabled - was causing performance issues
            raise
    
    def get_output_count(self) -> int:
        """Get number of registered outputs"""
        return len(self._outputs)
    
    def is_connected(self) -> bool:
        """Check if connected to FEAGI"""
        return self._connected


# Global singleton instance
brain_output = BrainOutput()

