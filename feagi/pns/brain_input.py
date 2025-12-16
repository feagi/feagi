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
        self._zmq_context = None  # MUST keep context alive!
        self._connected = False
        
        # Auto-incrementing group IDs
        self._next_group_id = 0
        
        # Configuration
        self._feagi_host = "localhost"
        self._feagi_port = 5558
        self._transport_type = "zmq"
        self._api_port = 8000  # FEAGI API port for registration
        
        # Agent registration (REQUIRED in FEAGI 2.0)
        self._agent_registered = False
        self._agent_id = None
        self._registration_response = None
        self._registration_manager = None
        
        # Observability monitors
        self._monitors: List['Monitor'] = []
    
    def _init_cache(self):
        """Initialize Rust ConnectorAgent (lazy)"""
        if self._cache is not None:
            return
        
        try:
            import feagi_rust_py_libs as frpl
            
            # Initialize Rust tracing logging (if available)
            try:
                # Try top-level function first (available in latest version)
                if hasattr(frpl, 'init_rust_logging'):
                    frpl.init_rust_logging()
                    logger.debug("✅ Rust tracing logging initialized")
                # Fallback to connector_core module
                elif hasattr(frpl, 'connector_core') and hasattr(frpl.connector_core, 'init_rust_logging'):
                    frpl.connector_core.init_rust_logging()
                    logger.debug("✅ Rust tracing logging initialized via connector_core")
                # Fallback to feagi_agent_sdk (may be commented out)
                elif hasattr(frpl, 'feagi_agent_sdk') and hasattr(frpl.feagi_agent_sdk, 'init_rust_logging'):
                    frpl.feagi_agent_sdk.init_rust_logging()
                    logger.debug("✅ Rust tracing logging initialized via feagi_agent_sdk")
            except Exception as log_init_err:
                logger.debug(f"Could not initialize Rust logging (non-fatal): {log_init_err}")
            
            # Use ConnectorAgent which provides sensor methods
            # NOTE: frpl.connector_core.caching.IOCache() does not exist in current API
            # Using ConnectorAgent instead - encoding methods still need to be added to rust-py-libs
            self._cache = frpl.connector_core.ConnectorAgent()
            self._cache_available = True
            logger.info("✅ Rust ConnectorAgent initialized")
        except (ImportError, AttributeError) as e:
            logger.error(f"❌ Failed to initialize Rust ConnectorAgent: {e}")
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
        transport: str = "zmq",
        api_port: int = 8000
    ):
        """
        Configure connection to FEAGI.
        
        Args:
            feagi_host: FEAGI server hostname or IP
            feagi_port: Sensory input port (default: 5558)
            transport: Transport type - "zmq" or "websocket"
            api_port: FEAGI API port for registration (default: 8000)
        """
        self._init_cache()
        self._feagi_host = feagi_host
        self._feagi_port = feagi_port
        self._transport_type = transport
        self._api_port = api_port
        
        logger.info(f"📡 Configured: {transport}://{feagi_host}:{feagi_port} (API: {feagi_host}:{api_port})")
    
    def register_agent(
        self,
        agent_id: str,
        agent_type: str = "sensory",
        capabilities: Optional[Dict[str, Any]] = None,
        agent_version: Optional[str] = None,
        controller_version: Optional[str] = None,
    ) -> bool:
        """
        Register as an agent with FEAGI (REQUIRED in FEAGI 2.0).
        
        This MUST be called before connect() or any other operations.
        Registration triggers auto-creation of missing IPU/OPU cortical areas.
        
        Args:
            agent_id: Unique agent identifier
            agent_type: Agent type ("sensory", "motor", "both", "visualization", "infrastructure")
            capabilities: Agent capabilities dict (e.g., {"sensory": {"rate_hz": 30.0}})
            agent_version: Optional agent version string
            controller_version: Optional controller version string
            
        Returns:
            True if registration succeeded, False otherwise
            
        Raises:
            RuntimeError: If registration fails (FEAGI 2.0 requires successful registration)
        """
        if self._agent_registered:
            logger.warning(f"Agent '{self._agent_id}' already registered. Re-registering...")
        
        # Initialize registration manager
        if self._registration_manager is None:
            try:
                from feagi.pns.registration_manager import RegistrationManager, AgentRegistrationRequest
                self._registration_manager = RegistrationManager()
            except ImportError as e:
                logger.error(f"Failed to import RegistrationManager: {e}")
                raise RuntimeError(
                    "RegistrationManager not available. Agent registration is required in FEAGI 2.0."
                ) from e
        
        # Build capabilities from registered inputs if not provided
        if capabilities is None:
            # Check if we have segmented vision inputs (which use Absolute encoding)
            has_segmented_vision = any(
                hasattr(inp, 'encoding') and inp.encoding == "absolute" and 
                hasattr(inp, '__class__') and 'Camera' in inp.__class__.__name__
                for inp in self._inputs
            )
            
            capabilities = {
                "sensory": {
                    "rate_hz": 30.0,  # Default rate
                    "input_count": len(self._inputs),
                    "cortical_areas": [
                        getattr(inp, '_cortical_area', 'unknown') 
                        for inp in self._inputs 
                        if hasattr(inp, '_cortical_area')
                    ],
                    "encoding": "absolute",  # Explicitly specify Absolute encoding for segmented vision
                    "cortical_unit_type": "segmented_vision" if has_segmented_vision else None
                }
            }
        
        # Create registration request with API URL in metadata
        from feagi.pns.registration_manager import AgentRegistrationRequest
        metadata = {
            "feagi_api_url": f"http://{self._feagi_host}:{self._api_port}",
            "feagi_api_port": self._api_port,
        }
        request = AgentRegistrationRequest(
            agent_id=agent_id,
            agent_type=agent_type,
            agent_ip=self._feagi_host,
            capabilities=capabilities,
            agent_version=agent_version,
            controller_version=controller_version,
            metadata=metadata,
        )
        
        # Register with FEAGI
        try:
            response = self._registration_manager.register_agent(request)
            
            if not response.success:
                error_msg = (
                    f"Agent registration FAILED: {response.message}\n"
                    f"Error code: {response.error_code}\n"
                    f"FEAGI 2.0 requires successful agent registration before any operations.\n"
                    f"Please ensure FEAGI is running and the cortical areas exist (or auto-create is enabled)."
                )
                logger.error(error_msg)
                raise RuntimeError(error_msg)
            
            # Registration succeeded
            self._agent_registered = True
            self._agent_id = agent_id
            self._registration_response = response
            
            # Log cortical area status
            if response.cortical_areas:
                ipu_areas = response.cortical_areas.get("required_ipu_areas", [])
                opu_areas = response.cortical_areas.get("required_opu_areas", [])
                
                logger.info(f"✅ Agent '{agent_id}' registered successfully")
                logger.info(f"   IPU areas: {len(ipu_areas)} required")
                logger.info(f"   OPU areas: {len(opu_areas)} required")
                
                # Log status of each area
                for area in ipu_areas:
                    status = area.get("status", "unknown")
                    area_name = area.get("area_name", "unknown")
                    if status == "Created":
                        logger.info(f"   ✅ IPU area '{area_name}' was auto-created")
                    elif status == "Existing":
                        logger.info(f"   ✅ IPU area '{area_name}' exists")
                    elif status == "Missing":
                        logger.warning(f"   ⚠️ IPU area '{area_name}' is missing (auto-create disabled?)")
                    elif status == "Error":
                        logger.error(f"   ❌ IPU area '{area_name}' creation failed: {area.get('message', 'unknown error')}")
            
            return True
            
        except Exception as e:
            error_msg = (
                f"Agent registration FAILED with exception: {e}\n"
                f"FEAGI 2.0 requires successful agent registration before any operations."
            )
            logger.error(error_msg, exc_info=True)
            raise RuntimeError(error_msg) from e
    
    def connect(self):
        """
        Connect to FEAGI.
        
        Initializes transport and establishes connection.
        
        REQUIRES: Agent registration must succeed before calling this.
        """
        if not self._agent_registered:
            raise RuntimeError(
                "Agent registration required before connecting.\n"
                "Call brain_input.register_agent(agent_id='...', ...) first.\n"
                "FEAGI 2.0 requires successful agent registration before any operations."
            )
        
        if not self._cache_available:
            raise RuntimeError("Cache not initialized. Call configure() first.")
        
        # Initialize transport
        if self._transport_type == "zmq":
            import zmq
            self._zmq_context = zmq.Context()  # Store as instance variable to prevent garbage collection!
            self._transport = self._zmq_context.socket(zmq.PUSH)  # PUSH socket to send data to FEAGI
            
            # Set socket options for better debugging
            self._transport.setsockopt(zmq.LINGER, 0)  # Don't wait on close
            self._transport.setsockopt(zmq.SNDHWM, 1000)  # Send high water mark
            
            # Use configured port (should be ZMQ sensory port, not API port)
            endpoint = f"tcp://{self._feagi_host}:{self._feagi_port}"
            logger.info(f"🔗 [ZMQ] Attempting to connect to FEAGI at {endpoint}...")
            logger.info(f"🔗 [ZMQ] Using port {self._feagi_port} (API port is {self._api_port})")
            
            # Warn if port matches API port (likely misconfiguration)
            if self._feagi_port == self._api_port:
                logger.warning(f"🔗 [ZMQ] ⚠️ WARNING: ZMQ port ({self._feagi_port}) matches API port! This is likely wrong. ZMQ sensory port should be 5558, not {self._api_port}")
            
            try:
                self._transport.connect(endpoint)
                logger.info(f"🔗 [ZMQ] ✅ Successfully connected to FEAGI at {endpoint}")
                logger.info(f"🔗 [ZMQ] Socket type: PUSH, State: {self._transport.get(zmq.TYPE)}")
            except Exception as e:
                logger.error(f"🔗 [ZMQ] ❌ Failed to connect to {endpoint}: {e}", exc_info=True)
                raise
        else:
            raise NotImplementedError(f"Transport type '{self._transport_type}' not yet implemented")
        
        self._connected = True
        
        # Start heartbeat via RegistrationManager (proper module for heartbeat logic)
        if self._registration_manager and self._agent_id:
            self._registration_manager.start_heartbeat(self._agent_id, self._feagi_host, self._api_port)
    
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
    
    def register_input(self, input_instance: 'BaseInput', group_id: Optional[int] = None):
        """
        Register an input (called internally by input classes).
        
        NOTE: In FEAGI 2.0, you must call register_agent() AFTER registering all inputs
        so that the registration includes the correct cortical area information.
        
        Args:
            input_instance: Input instance to register
            group_id: Optional group ID to use. If None, auto-allocates next available ID.
        """
        self._init_cache()
        
        # Use provided group_id or allocate new one
        if group_id is None:
            group_id = self._allocate_group_id()
        else:
            logger.debug(f"Using provided group_id={group_id} for {input_instance.__class__.__name__}")
        
        # Register with Rust cache
        input_instance._register_with_cache(self._cache, group_id)
        input_instance._mark_registered(group_id)
        
        # Add to registry
        self._inputs.append(input_instance)
        
        logger.debug(f"✅ Registered input: {input_instance.__class__.__name__} (group={group_id})")
        
        # Note: Agent registration should be called after all inputs are registered
        # so that the registration includes complete capability information
    
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
        
        REQUIRES: Agent registration must succeed before calling this.
        """
        if not self._agent_registered:
            raise RuntimeError(
                "Agent registration required before sending data.\n"
                "Call brain_input.register_agent(agent_id='...', ...) first.\n"
                "FEAGI 2.0 requires successful agent registration before any operations."
            )
        
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
        
        t_send_total_start = time.perf_counter()
        try:
            # Update all inputs to cache
            for input_instance in self._inputs:
                try:
                    input_instance._write_to_cache(self._cache)
                except Exception as e:
                    logger.error(f"❌ [BRAIN-INPUT] Error writing {input_instance.__class__.__name__} to cache: {e}", exc_info=True)
                    raise
            
            # Encode cached data to bytes
            # Note: Method name was changed to sensors_encode_cached_sensor_data_to_bytes in recent refactor
            try:
                if not hasattr(self._cache, 'sensors_encode_cached_sensor_data_to_bytes'):
                    raise AttributeError(
                        "ConnectorAgent missing required method: sensors_encode_cached_sensor_data_to_bytes()\n"
                        "This method needs to be added to rust-py-libs ConnectorAgent.\n"
                        "See MISSING_RUST_PY_LIBS_API.md for details."
                    )
                
                self._cache.sensors_encode_cached_sensor_data_to_bytes()
            except AttributeError:
                raise
            except Exception as e:
                logger.error(f"❌ [BRAIN-INPUT] Error encoding cached data to bytes: {e}", exc_info=True)
                raise
            
            # Get encoded bytes (Rust)
            # Note: API changed to sensors_read_bytes() which directly returns Vec<u8>
            try:
                if not hasattr(self._cache, 'sensors_read_bytes'):
                    raise AttributeError(
                        "ConnectorAgent missing required method: sensors_read_bytes()\n"
                        "This method needs to be added to rust-py-libs ConnectorAgent.\n"
                        "See MISSING_RUST_PY_LIBS_API.md for details."
                    )
                py_bytes = self._cache.sensors_read_bytes()
                serialized = bytes(py_bytes)
                
                # Logging removed for hot path
            except AttributeError:
                raise
            except Exception as e:
                logger.error(f"❌ [BRAIN-INPUT] Error reading sensor bytes: {e}", exc_info=True)
                raise
            
            # Check if we have data
            if not serialized:
                logger.warning("⚠️ [BRAIN-INPUT] No sensor data to send (serialized data is empty)")
                return
            
            # Send via transport
            if self._transport:
                try:
                    import zmq
                    # Send with NOBLOCK to detect immediate failures
                    try:
                        self._transport.send(serialized, zmq.NOBLOCK)
                        # Logging removed for hot path
                    except zmq.Again:
                        # Socket buffer full - this shouldn't happen with PUSH/PULL but log it
                        logger.warning(f"📤 [BRAIN-INPUT] ⚠️ [ZMQ] Socket buffer full (EAGAIN), retrying with blocking send...")
                        self._transport.send(serialized, 0)  # Blocking send
                        logger.info(f"📤 [BRAIN-INPUT] ✅ Successfully sent {len(serialized)} bytes to FEAGI via ZMQ (blocking)")
                    except zmq.ZMQError as zmq_err:
                        logger.error(f"📤 [BRAIN-INPUT] ❌ [ZMQ] ZMQ error during send: {zmq_err} (errno: {zmq_err.errno})", exc_info=True)
                        raise
                except zmq.ZMQError as zmq_err:
                    logger.error(f"📤 [BRAIN-INPUT] ❌ [ZMQ] ZMQ error sending data: {zmq_err} (errno: {zmq_err.errno})", exc_info=True)
                    raise
                except Exception as e:
                    logger.error(f"📤 [BRAIN-INPUT] ❌ Error sending data to FEAGI: {e}", exc_info=True)
                    raise
            else:
                # No transport configured yet (shouldn't happen in normal use)
                logger.error(f"No transport configured - cannot send {len(self._inputs)} inputs")
                raise RuntimeError("Transport not configured")
            
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
    
    def is_registered(self) -> bool:
        """Check if agent is registered with FEAGI (REQUIRED in FEAGI 2.0)"""
        return self._agent_registered
    
    def get_registration_info(self) -> Optional[Dict[str, Any]]:
        """Get registration response information"""
        if not self._agent_registered or self._registration_response is None:
            return None
        
        return {
            "agent_id": self._agent_id,
            "success": self._registration_response.success,
            "message": self._registration_response.message,
            "cortical_areas": self._registration_response.cortical_areas,
            "transport_info": self._registration_response.transport_info,
        }


# Global singleton instance
brain_input = BrainInput()

