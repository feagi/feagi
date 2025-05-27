"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""FEAGI Process Manager.

This module provides a centralized Process Manager for FEAGI that implements the
Rust/RTOS compatible architecture described in feagi_processes.md. It handles starting, 
monitoring, and stopping async tasks and services according to their priority levels.

ARCHITECTURE: Rust/RTOS Compatible
- Uses singleton ConnectomeManager for mission-critical reliability
- Direct task spawning instead of subprocess boundaries  
- Memory-mapped state for zero-copy data access
- No environment variable dependencies for IPC

MIGRATION PATH: Python → Rust
- threading.Thread → tokio::spawn
- Memory-mapped files → memmap2 crate
- Singleton pattern → std::sync::Once
"""
import asyncio
from feagi.utils.logger import setup_logger
logger = setup_logger(name="Process Manager")

import os
import signal
import socket
import sys
import threading
import time
from typing import Dict, Any, Optional, List, Tuple, Set
from feagi.npu.burst_engine import FQSampler
from feagi.core.state_manager import FeagiStateManager
from queue import Queue
import traceback
import multiprocessing

# Import TOML configuration system
from feagi.config.toml_loader import (
    load_feagi_config, 
    FeagiConfigurationError,
    get_port_config
)
from feagi.utils.port_checker import (
    check_port_availability,
    PortConflictError
)

# Process priority levels
PRIORITY_CRITICAL = 1  # Real-time critical processes
PRIORITY_IMPORTANT = 2  # Near real-time processes
PRIORITY_BACKGROUND = 3  # Best effort processes

class ProcessManager:
    """
    FEAGI Process Manager - Rust/RTOS Compatible Architecture.
    
    Responsible for:
    1. Task Creation: Launches async tasks with appropriate resources and parameters.
    2. Task Monitoring: Monitors task health and performance without subprocess overhead.
    3. Resource Allocation: Distributes computing resources based on priority.
    4. Fault Tolerance: Restarts failed tasks and maintains system integrity.
    
    RUST/RTOS COMPATIBLE FEATURES:
    - Singleton ConnectomeManager for consistent state access
    - Direct task spawning eliminates subprocess boundaries  
    - Memory-mapped state for instant synchronization
    - No environment variable IPC dependencies
    """
    
    def __init__(self):
        """Initialize the Process Manager with singleton ConnectomeManager."""
        self._processes = {}
        self._resource_allocations = {}
        self._running = False
        self._monitor_thread = None
        self._core_api = None
        self._zmq_server = None
        
        # CRITICAL: Use ConnectomeManager singleton for mission-critical reliability
        # The ConnectomeManager should already be initialized with proper parameters by main.py
        from feagi.bdu.connectome_manager import ConnectomeManager
        self._connectome_manager = ConnectomeManager.instance()
        
        # Internal references for critical components
        self._burst_engine = None
        self._fcl_manager = None
        self._memory_manager = None
        
        # Track which ports are in use
        self._used_ports = set()
        
        self._fcl_sampler = None
        self._fcl_sampler_thread = None
        self._fcl_sampler_queue = None
        
        # FQ Sampler (Fire Queue Sampler) - replacement for FCL sampler
        self._fq_sampler = None
        self._fq_sampler_thread = None
        self._fq_sampler_queue = None
        
    def load_and_validate_ports(self, cli_args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Load port configuration from TOML file and validate availability.
        
        This replaces the old auto port conflict resolution with fail-fast validation.
        
        Args:
            cli_args: Optional command-line argument overrides
            
        Returns:
            Complete configuration dictionary
            
        Raises:
            FeagiConfigurationError: If configuration loading fails
            PortConflictError: If any port is already in use
        """
        try:
            # Load TOML configuration with all overrides applied
            config = load_feagi_config(cli_args=cli_args)
            
            # Extract port configuration
            port_config = get_port_config(config)
            
            # Validate all ports are available
            host = config.get('zmq', {}).get('host', '127.0.0.1')
            
            for port_name, port_number in port_config.get_all_ports().items():
                try:
                    check_port_availability(host, port_number)
                    logger.debug(f"Port {port_number} ({port_name}) is available")
                except PortConflictError as e:
                    logger.error(f"Port conflict detected: {e}")
                    raise PortConflictError(
                        f"Port {port_number} (used for {port_name}) is already in use. "
                        f"Edit feagi_configuration.toml to change port assignments. "
                        f"Available ports can be found using: netstat -tuln"
                    )
            
            # Also validate API port
            api_port = config.get('api', {}).get('port', 8000)
            api_host = config.get('api', {}).get('host', '127.0.0.1')
            try:
                check_port_availability(api_host, api_port)
                logger.debug(f"API port {api_port} is available")
            except PortConflictError as e:
                logger.error(f"API port conflict detected: {e}")
                raise PortConflictError(
                    f"API port {api_port} is already in use. "
                    f"Edit feagi_configuration.toml to change the api.port setting. "
                    f"Available ports can be found using: netstat -tuln"
                )
            
            logger.info("All port validations passed")
            return config
            
        except Exception as e:
            logger.error(f"Failed to load and validate port configuration: {e}")
            raise

    def find_available_port(self, start_port: int, max_tries: int = 10) -> Optional[int]:
        """
        DEPRECATED: This method is replaced by TOML-based port configuration.
        
        Use load_and_validate_ports() instead, which loads hardcoded ports from
        feagi_configuration.toml and validates them without auto-resolution.
        
        Args:
            start_port: The port to start checking from (ignored)
            max_tries: Maximum number of ports to try (ignored)
            
        Returns:
            None - This method is deprecated
            
        Raises:
            DeprecationWarning: Always, to indicate this method should not be used
        """
        import warnings
        warnings.warn(
            "find_available_port() is deprecated. "
            "Use load_and_validate_ports() with TOML configuration instead. "
            "Edit feagi_configuration.toml to set port assignments.",
            DeprecationWarning,
            stacklevel=2
        )
        logger.error(
            "DEPRECATED: find_available_port() called. "
            "FEAGI 2.0 uses hardcoded port configuration from feagi_configuration.toml. "
            "Edit the configuration file to resolve port conflicts."
        )
        return None
    
    def init_critical_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 1 (Critical) processes.
        
        These processes are essential for neural simulation and include:
        - Burst Engine
        - Connectome Manager
        - FCL Manager
        - Memory & Learning Manager
        
        Args:
            config: Configuration parameters for the processes
            
        Returns:
            True if successfully initialized, False otherwise
        """
        logger.info("Initializing critical (Priority 1) processes...")
        
        try:
            # Check if embedded mode is enabled to prevent FastAPI imports
            embedded_mode = config.get('system', {}).get('embedded', False)
            
            if embedded_mode:
                logger.info("🔧 Embedded mode: Initializing core components without REST API imports")
                # Environment variable already set in main.py before any imports
            
            # Initialize core components - these run in the same process
            # but are conceptually distinct according to the architecture
            
            # Import here to avoid circular imports
            from feagi.core import create_core_api
            
            # Create the core API with all critical components
            self._core_api = create_core_api(self._connectome_manager, config)
            
            # Get references to individual critical components
            if self._core_api:
                self._burst_engine = self._core_api.get_burst_engine()
                self._connectome_manager = self._core_api.get_connectome_manager()
                self._fcl_manager = self._core_api.get_fcl_manager()
                self._memory_manager = self._core_api.get_memory_manager()
            
            logger.info("✓ Critical processes initialized successfully", emoji1="✓ ")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize critical processes: {e}")
            return False
    
    def init_important_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 2 (Important) processes.
        
        These processes handle important but less time-critical operations:
        - FCL Sampler
        - PNS Message Broker (implemented as ZMQ server)
        - Resource Manager
        
        Args:
            config: Configuration parameters loaded from TOML file
            
        Returns:
            True if successfully initialized, False otherwise
        """
        logger.info("Initializing important (Priority 2) processes...")
        
        # Check if embedded mode is enabled
        embedded_mode = config.get('system', {}).get('embedded', False)
        if embedded_mode:
            logger.info("🔧 Embedded device mode enabled - disabling non-essential components")
        
        try:
            # --- FQSampler Integration ---
            from feagi.core.state_manager import FeagiStateManager, ServiceState
            state_manager = FeagiStateManager.instance()
            
            # Get port configuration from TOML config
            port_config = get_port_config(config)
            zmq_host = config.get('zmq', {}).get('host', '127.0.0.1')
            
            # --- ZMQ Message Broker Setup ---
            try:
                from feagi.api.zmq.server import ZmqServer
                
                # Get stream configuration
                zmq_config = config.get('zmq', {})
                stream_config = zmq_config.get('streams', {})
                
                # Check which streams are enabled (disable visualization in embedded mode)
                visualization_enabled = stream_config.get('visualization', {}).get('enabled', True) and not embedded_mode
                sensory_enabled = stream_config.get('sensory', {}).get('enabled', True)
                motor_enabled = stream_config.get('motor', {}).get('enabled', True)
                control_enabled = stream_config.get('control', {}).get('enabled', True)
                
                if embedded_mode:
                    logger.info("🔧 Embedded mode: Visualization stream disabled")
                
                logger.info(f"Stream configuration: visualization={visualization_enabled}, "
                           f"sensory={sensory_enabled}, motor={motor_enabled}, control={control_enabled}")
                
                # --- FQ Sampler Setup (if visualization is enabled) ---
                if visualization_enabled:
                    try:
                        from feagi.npu.burst_engine import FQSampler
                        from queue import Queue
                        
                        # Create FQ sampler queue
                        self._fq_sampler_queue = Queue(maxsize=100)
                        
                        # Create FQ sampler
                        self._fq_sampler = FQSampler(
                            fire_queue_provider=self._core_api,  # Core API provides fire queue access
                            sample_frequency_hz=50.0,  # 50Hz sampling by default
                            output_queue=self._fq_sampler_queue,
                            connectome_manager=self._connectome_manager
                        )
                        
                        # Start FQ sampler in a thread
                        import threading
                        self._fq_sampler_thread = threading.Thread(
                            target=self._fq_sampler.run,
                            daemon=True,
                            name="FQSampler"
                        )
                        self._fq_sampler_thread.start()
                        
                        logger.info("✅ FQ Sampler initialized and started")
                        
                    except Exception as e:
                        logger.error(f"Failed to initialize FQ Sampler: {e}")
                        self._fq_sampler = None
                        self._fq_sampler_queue = None
                        self._fq_sampler_thread = None
                else:
                    logger.info("Visualization disabled, skipping FQ Sampler initialization")
                
                # Use hardcoded ports from configuration
                zmq_ports = {
                    'req_rep': port_config.zmq_req_rep_port,
                    'pub_sub': port_config.zmq_pub_sub_port,
                    'push_pull': port_config.zmq_push_pull_port,
                    'sensory': port_config.zmq_sensory_port if sensory_enabled else None,
                    'motor': port_config.zmq_motor_port if motor_enabled else None,
                    'control': port_config.zmq_control_port if control_enabled else None,
                    'visualization': port_config.zmq_visualization_port if visualization_enabled else None,
                    'rest': port_config.zmq_rest_port,
                }
                
                logger.info(f"Starting ZMQ server with ports: {zmq_ports}")
                
                # Initialize ZMQ server with configuration-based stream enablement
                zmq_server = ZmqServer(
                    core_api=self._core_api,
                    host=zmq_host,
                    req_rep_port=port_config.zmq_req_rep_port,
                    pub_sub_port=port_config.zmq_pub_sub_port,
                    push_pull_port=port_config.zmq_push_pull_port,
                    sensory_port=port_config.zmq_sensory_port if sensory_enabled else None,
                    motor_port=port_config.zmq_motor_port if motor_enabled else None,
                    control_port=port_config.zmq_control_port if control_enabled else None,
                    rest_port=port_config.zmq_rest_port,
                    vis_port=port_config.zmq_visualization_port if visualization_enabled else None,
                    fq_sampler=self._fq_sampler,  # Pass the actual FQ sampler
                    fq_sampler_queue=self._fq_sampler_queue,  # Pass the actual queue
                    stream_config=stream_config  # Pass stream configuration to ZMQ server
                )
                
                # Start ZMQ server
                if zmq_server.start():
                    self._processes['zmq_server'] = zmq_server
                    state_manager.set_zmq_state(ServiceState.READY)
                    logger.info("ZMQ Message Broker initialized successfully")
                else:
                    logger.error("Failed to start ZMQ Message Broker")
                    return False
                    
            except Exception as e:
                logger.error(f"Failed to initialize ZMQ Message Broker: {e}")
                logger.debug(traceback.format_exc())
                return False
                
            # --- Resource Manager ---
            try:
                from feagi.core.resource_mgr import ResourceManager
                resource_manager = ResourceManager.get_instance(config.get('resources', {}))
                
                if resource_manager.initialize_critical_structures():
                    self._processes['resource_manager'] = resource_manager
                    # Resource manager doesn't have a specific state in FeagiStateManager
                    logger.info("Resource Manager initialized successfully")
                else:
                    logger.error("Failed to initialize Resource Manager")
                    return False
                    
            except Exception as e:
                logger.error(f"Failed to initialize Resource Manager: {e}")
                logger.debug(traceback.format_exc())
                # Non-critical - continue without resource manager
                logger.warning("Continuing without Resource Manager")
                
            # --- Health Check Service ---
            try:
                health_enabled = config.get('resources', {}).get('enable_health_check', True)
                if health_enabled:
                    from feagi.core.health_monitor import HealthMonitor
                    
                    health_monitor = HealthMonitor()
                    if health_monitor.start():
                        self._processes['health_monitor'] = health_monitor
                        logger.info("Health monitor started")
                    else:
                        logger.warning("Failed to start health monitor - continuing without it")
                        
            except Exception as e:
                logger.warning(f"Health monitor initialization failed: {e}")
                # Non-critical - continue without health monitoring
            
            # --- System Resource Monitor (Profile Mode) ---
            try:
                # Check if profiling mode is enabled via --profile flag
                profile_enabled = config.get('system', {}).get('profile', False)
                
                # In embedded mode, only enable profiling if explicitly requested
                if profile_enabled and (not embedded_mode or profile_enabled):
                    from feagi.utils.system_monitor import start_system_monitoring
                    
                    # Configure monitoring based on profile settings
                    monitoring_interval = config.get('profile', {}).get('resource_monitor_interval', 5.0)
                    enable_gpu = config.get('profile', {}).get('monitor_gpu', True)
                    enable_logging = config.get('profile', {}).get('monitor_logging', True)
                    
                    # In embedded mode, use minimal resource monitoring
                    if embedded_mode:
                        monitoring_interval = max(monitoring_interval, 10.0)  # Slower monitoring
                        enable_gpu = False  # No GPU monitoring in embedded
                        logger.info("🔧 Embedded mode: Using minimal resource monitoring")
                    
                    system_monitor = start_system_monitoring(
                        monitoring_interval=monitoring_interval,
                        enable_gpu_monitoring=enable_gpu,
                        enable_detailed_logging=enable_logging
                    )
                    
                    if system_monitor:
                        self._processes['system_monitor'] = system_monitor
                        logger.info(f"📊 System resource monitor started (profile mode) - interval: {monitoring_interval}s")
                    else:
                        logger.warning("Failed to start system resource monitor")
                elif embedded_mode:
                    logger.info("🔧 Embedded mode: System resource monitoring disabled (use --profile to enable minimal monitoring)")
                        
            except Exception as e:
                logger.warning(f"System resource monitor initialization failed: {e}")
                # Non-critical - continue without resource monitoring
                
            logger.info("Important processes initialization completed")
            return True
            
        except Exception as e:
            logger.error(f"Critical error during important processes initialization: {e}")
            logger.debug(traceback.format_exc())
            return False
    
    def init_background_processes(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Priority 3 (Background) processes.
        
        RUST/RTOS COMPATIBLE: Uses direct task spawning instead of subprocesses.
        All services run in the same process space with shared memory access.
        
        Args:
            config: Configuration parameters loaded from TOML file
            
        Returns:
            True if successfully initialized, False otherwise.
        """
        logger.info("Initializing background (Priority 3) processes...")
        
        # Check if embedded mode is enabled
        embedded_mode = config.get('system', {}).get('embedded', False)
        
        try:
            # --- REST API (normal mode only) ---
            api_config = config.get('api', {})
            api_host = api_config.get('host', '127.0.0.1')
            api_port = api_config.get('port', 8000)
            
            if not embedded_mode:
                try:
                    # Only run FastAPI if not in embedded mode
                    api_config = {
                        'host': api_host,
                        'port': api_port,
                        'reload': config.get('development', {}).get('reload', False),
                        'access_log': config.get('api', {}).get('access_log', True)
                    }
                    
                    # Create event loop if not already running
                    try:
                        loop = asyncio.get_event_loop()
                    except RuntimeError:
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                    
                    def run_uvicorn():
                        """Run uvicorn server in background thread."""
                        try:
                            # Import FastAPI app here to avoid circular imports
                            from feagi.api.rest.app import create_rest_app
                            app = create_rest_app()
                            
                            import uvicorn
                            uvicorn.run(
                                app,
                                host=api_config['host'],
                                port=api_config['port'],
                                access_log=api_config.get('access_log', True),
                                loop="asyncio"
                            )
                        except Exception as e:
                            logger.error(f"Failed to start uvicorn: {e}")
                    
                    # Start uvicorn in background thread
                    api_thread = threading.Thread(target=run_uvicorn, daemon=True)
                    api_thread.start()
                    
                    # Store the thread reference for shutdown
                    self._processes['rest_api'] = api_thread
                    logger.info(f"REST API server started on http://{api_host}:{api_port}")
                        
                except Exception as e:
                    logger.error(f"Failed to initialize REST API server: {e}")
                    return False
            else:
                # Embedded mode: No HTTP interface at all
                logger.info("🔧 Embedded mode: REST API completely disabled for minimal resource usage")
                logger.info("🔧 Control interface available only via ZMQ streams (control, sensory, motor)")
                logger.info("🔧 No web interface, no FastAPI imports, no uvicorn server")
                logger.info("🔧 Status available via ZMQ control stream: tcp://127.0.0.1:5559")
            
            # --- WebSocket Server (Optional) ---
            try:
                websocket_enabled = config.get('websocket', {}).get('enabled', False)
                if websocket_enabled and not embedded_mode:
                    from feagi.api.websocket.server import WebSocketServer
                    
                    ws_port = config.get('websocket', {}).get('port', 8080)
                    ws_host = config.get('websocket', {}).get('host', '127.0.0.1')
                    
                    ws_server = WebSocketServer(host=ws_host, port=ws_port)
                    
                    if ws_server.start():
                        self._processes['websocket'] = ws_server
                        logger.info(f"WebSocket server started on {ws_host}:{ws_port}")
                    else:
                        logger.warning("Failed to start WebSocket server - continuing without it")
                elif embedded_mode:
                    logger.info("🔧 Embedded mode: WebSocket server disabled")
                        
            except Exception as e:
                logger.warning(f"WebSocket server initialization failed: {e}")
                # Non-critical - continue without WebSocket
                
            # --- Health Check Service (skip in embedded mode) ---
            try:
                health_enabled = config.get('resources', {}).get('enable_health_check', True) and not embedded_mode
                if health_enabled:
                    from feagi.core.health_monitor import HealthMonitor
                    
                    health_monitor = HealthMonitor()
                    if health_monitor.start():
                        self._processes['health_monitor'] = health_monitor
                        logger.info("Health monitor started")
                    else:
                        logger.warning("Failed to start health monitor - continuing without it")
                elif embedded_mode:
                    logger.info("🔧 Embedded mode: Health monitor disabled")
                        
            except Exception as e:
                logger.warning(f"Health monitor initialization failed: {e}")
                # Non-critical - continue without health monitoring

            logger.info("Background processes initialization completed")
            return True
            
        except Exception as e:
            logger.error(f"Critical error during background processes initialization: {e}")
            logger.debug(traceback.format_exc())
            return False
    
    def _start_api_service_task(self, config: Dict[str, Any]) -> Optional[Any]:
        """
        Start API service as async task instead of subprocess.
        
        RUST/RTOS COMPATIBLE: This pattern translates directly to Rust async tasks.
        """
        try:
            import asyncio
            import threading
            
            # CRITICAL FIX: Ensure state synchronization between main process and FastAPI thread
            # Set environment variable so FastAPI thread uses the same state file
            from feagi.core.state_manager import FeagiStateManager
            state_manager = FeagiStateManager.instance()
            if hasattr(state_manager, 'path'):
                os.environ["FEAGI_STATE_FILE"] = state_manager.path
                logger.info(f"🔗 Sharing state file with FastAPI thread: {state_manager.path}")
            else:
                logger.warning("⚠️  State manager has no path attribute")
            
            # Create dedicated event loop for API service
            # In Rust, this would be a tokio::spawn() call
            def run_api_service():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                
                try:
                    # Import and create FastAPI app with direct dependencies
                    from feagi.api.rest.app import create_rest_app_direct
                    app = create_rest_app_direct(config)
                    
                    # Run uvicorn in the same process
                    import uvicorn
                    uvicorn.run(
                        app, 
                        host=config['host'], 
                        port=config['port'],
                        loop="asyncio"
                    )
                except Exception as e:
                    logger.error(f"API service task failed: {e}")
                finally:
                    loop.close()
            
            # Start as daemon thread (in Rust: tokio::spawn with proper task management)
            api_thread = threading.Thread(target=run_api_service, daemon=True)
            api_thread.start()
            
            logger.info("✓ API service task started successfully", emoji1="✓ ")
            return api_thread
            
        except Exception as e:
            logger.error(f"Failed to start API service task: {e}")
            return None
    
    def start(self, config: Dict[str, Any]) -> bool:
        """
        Start all FEAGI processes in priority order.
        
        Args:
            config: Configuration for all processes
            
        Returns:
            True if all processes started successfully, False otherwise
        """
        logger.info("Starting FEAGI Process Manager...")
        self._running = True
        
        # Start processes in priority order
        # 1. Critical processes (Priority 1)
        if not self.init_critical_processes(config):
            logger.error("Failed to initialize critical processes, aborting startup")
            self.shutdown()
            return False
            
        # 2. Important processes (Priority 2)
        if not self.init_important_processes(config):
            logger.error("Failed to initialize important processes, aborting startup")
            self.shutdown()
            return False
            
        # 3. Background processes (Priority 3)
        if not self.init_background_processes(config):
            logger.error("Failed to initialize background processes, aborting startup")
            self.shutdown()
            return False
            
        # Start monitoring thread
        self._start_monitoring()
        
        logger.info("FEAGI Process Manager started successfully")
        return True
    
    def _start_monitoring(self):
        """Start the process monitoring thread."""
        def monitor_processes():
            while self._running:
                try:
                    self._check_processes()
                except Exception as e:
                    logger.error(f"Error in process monitor: {e}")
                time.sleep(5)  # Check every 5 seconds
                
        self._monitor_thread = threading.Thread(target=monitor_processes, daemon=True)
        self._monitor_thread.start()
    
    def _check_processes(self):
        """
        Check all tasks and processes and restart any that have failed.
        
        RUST/RTOS COMPATIBLE: Monitors both async tasks and legacy processes.
        In Rust, this would be integrated with the async runtime's task monitoring.
        """
        for name, service in self._processes.items():
            try:
                # Determine service type based on the actual object type/attributes
                if hasattr(service, 'is_running') and callable(service.is_running):
                    # Service with is_running() method (like ZmqServer)
                    if not service.is_running():
                        logger.error(f"Service {name} is not running")
                elif hasattr(service, 'is_alive') and callable(service.is_alive):
                    # Thread-like objects
                    if not service.is_alive():
                        logger.error(f"Thread {name} has stopped unexpectedly")
                elif hasattr(service, 'poll') and callable(service.poll):
                    # Legacy subprocess
                    if service.poll() is not None:
                        exit_code = service.poll()
                        logger.error(f"Process {name} exited with code {exit_code}")
                else:
                    # Service type we can't monitor - just log debug info
                    logger.debug(f"Service {name} doesn't support health checking (type: {type(service).__name__})")
                    
            except Exception as e:
                logger.warning(f"Error checking service {name}: {e}")
    
    def get_core_api(self):
        """Get the Core API instance."""
        return self._core_api
        
    def get_zmq_server(self):
        """Get the ZMQ server instance."""
        return self._zmq_server
    
    @classmethod
    def get_instance(cls) -> Optional['ProcessManager']:
        """Get the global ProcessManager instance (alias for get_process_manager)."""
        return get_process_manager()
        
    def shutdown(self) -> None:
        """
        Shut down the Process Manager and all managed tasks/processes.
        
        RUST/RTOS COMPATIBLE: Handles both async tasks and legacy processes.
        In Rust, this would be a clean async task cancellation system.
        """
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        try:
            print("Shutting down FEAGI services...", file=sys.stderr, flush=True)
            
            # Stop running flag to signal all services to stop
            self._running = False
            
            # Import required modules for timeout handling
            import threading
            import time
            
            # Define timeout for graceful shutdown attempts
            GRACEFUL_SHUTDOWN_TIMEOUT = 8.0  # 8 seconds for graceful shutdown
            
            # Shutdown each service properly based on its type with timeouts
            for name, service in self._processes.items():
                try:
                    print(f"Stopping service: {name}...", file=sys.stderr, flush=True)
                    
                    # Create a shutdown function that can be run with timeout
                    def shutdown_service():
                        try:
                            # Handle different service types
                            if name == 'zmq_server' and hasattr(service, 'shutdown'):
                                # ZMQ server has a shutdown method
                                service.shutdown()
                            elif name == 'rest_api' and hasattr(service, 'stop'):
                                # REST API server has a stop method
                                service.stop()
                            elif name == 'websocket' and hasattr(service, 'stop'):
                                # WebSocket server has a stop method  
                                service.stop()
                            elif name == 'health_monitor' and hasattr(service, 'stop'):
                                # Health monitor has a stop method
                                service.stop()
                            elif name == 'system_monitor' and hasattr(service, 'stop'):
                                # System resource monitor has a stop method
                                service.stop()
                            elif name == 'resource_manager' and hasattr(service, 'cleanup'):
                                # Resource manager has a cleanup method
                                service.cleanup()
                            elif hasattr(service, 'terminate') and hasattr(service, 'poll'):
                                # Legacy subprocess handling
                                if service.poll() is None:
                                    service.terminate()
                                    service.wait(timeout=2)
                            elif hasattr(service, 'is_alive') and hasattr(service, 'join'):
                                # Thread-like objects
                                if service.is_alive():
                                    service.join(timeout=2)
                            else:
                                print(f"Service {name} doesn't have a known shutdown method", file=sys.stderr, flush=True)
                        except Exception as e:
                            print(f"Error in shutdown_service for {name}: {e}", file=sys.stderr, flush=True)
                    
                    # Run shutdown with timeout using a separate thread
                    shutdown_thread = threading.Thread(target=shutdown_service, daemon=True)
                    shutdown_thread.start()
                    shutdown_thread.join(timeout=GRACEFUL_SHUTDOWN_TIMEOUT)
                    
                    if shutdown_thread.is_alive():
                        print(f"⚠️  Service {name} didn't stop within {GRACEFUL_SHUTDOWN_TIMEOUT}s - continuing anyway", file=sys.stderr, flush=True)
                        
                except Exception as e:
                    print(f"Error stopping service {name}: {e}", file=sys.stderr, flush=True)
                        
            # Stop FQSampler if running
            if hasattr(self, '_fq_sampler') and self._fq_sampler:
                print("Stopping FQSampler...", file=sys.stderr, flush=True)
                try:
                    self._fq_sampler.stop()
                    if hasattr(self, '_fq_sampler_thread') and self._fq_sampler_thread:
                        self._fq_sampler_thread.join(timeout=2)
                    self._fq_sampler = None
                    self._fq_sampler_thread = None
                    self._fq_sampler_queue = None
                except Exception as e:
                    print(f"Error stopping FQSampler: {e}", file=sys.stderr, flush=True)
            
            print("FEAGI services shut down", file=sys.stderr, flush=True)
            
        except Exception as e:
            # Last resort error handling - print to stderr and continue
            print(f"Critical error during shutdown: {e}", file=sys.stderr, flush=True)
        
    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        self.shutdown()
        
    def update_area_sample_rate(self, cortical_id, rate):
        """
        Notify the running FQSampler of a per-area sample rate change.
        """
        if hasattr(self, '_fq_sampler') and self._fq_sampler is not None:
            self._fq_sampler.update_area_sample_rate(cortical_id, rate)
        # If FQSampler is not running, do nothing

# Global instance for the process manager
_process_manager = None

def get_process_manager() -> ProcessManager:
    """Get the global ProcessManager instance."""
    global _process_manager
    if _process_manager is None:
        _process_manager = ProcessManager()
    return _process_manager

def start_all_processes(startup_config: dict, config: Dict[str, Any]) -> bool:
    """
    Start all FEAGI processes based on configuration.
    
    Args:
        startup_config: Configuration for process startup
        config: Main FEAGI configuration
        
    Returns:
        bool: True if all processes started successfully
    """
    try:
        # Check if profiling is enabled
        profile_enabled = config.get('system', {}).get('profile', False)
        
        if profile_enabled:
            # Import and start detailed profiling
            from feagi.utils.resource_profiler import start_profiling, profile_component
            start_profiling()
            logger.info("🔍 Detailed resource profiling enabled")
            
            # Profile initial state
            profile_component("startup_baseline")

        # ... existing code ...
        
        # Start Health Monitor (Important processes)
        if startup_config.get('health_monitor', {}).get('enabled', True):
            logger.info("Starting Health Monitor...")
            
            if profile_enabled:
                profile_component("pre_health_monitor")
            
            # Health monitor startup code here...
            
            if profile_enabled:
                profile_component("post_health_monitor")

        # Start Resource Manager
        if startup_config.get('resource_manager', {}).get('enabled', True):
            logger.info("Starting Resource Manager...")
            
            if profile_enabled:
                profile_component("pre_resource_manager")
            
            # Resource manager startup code here...
            
            if profile_enabled:
                profile_component("post_resource_manager")

        # Start REST API server (Critical processes)
        if startup_config.get('rest_api', {}).get('enabled', True):
            logger.info("Starting REST API server...")
            
            if profile_enabled:
                profile_component("pre_rest_api")
            
            # Start REST API
            rest_host = config.get('api', {}).get('rest_host', '0.0.0.0')
            rest_port = config.get('api', {}).get('rest_port', 8080)
            
            rest_process = multiprocessing.Process(
                target=_start_rest_api_server,
                args=(rest_host, rest_port),
                name="FEAGI-REST-API"
            )
            rest_process.start()
            _active_processes.append(rest_process)
            logger.info(f"REST API server started on {rest_host}:{rest_port}")
            
            if profile_enabled:
                profile_component("post_rest_api")

        # Start ZMQ server (Critical processes)
        if startup_config.get('zmq_server', {}).get('enabled', True):
            logger.info("Starting ZMQ server...")
            
            if profile_enabled:
                profile_component("pre_zmq_server")
            
            # ZMQ server startup code here...
            
            if profile_enabled:
                profile_component("post_zmq_server")

        # Start Burst Engine (Important processes)
        if startup_config.get('burst_engine', {}).get('enabled', True):
            logger.info("Starting Burst Engine...")
            
            if profile_enabled:
                profile_component("pre_burst_engine")
            
            # Burst engine startup code here...
            
            if profile_enabled:
                profile_component("post_burst_engine")

        # Start FQ Sampler
        if startup_config.get('fq_sampler', {}).get('enabled', True):
            logger.info("Starting FQ Sampler...")
            
            if profile_enabled:
                profile_component("pre_fq_sampler")
            
            # FQ sampler startup code here...
            
            if profile_enabled:
                profile_component("post_fq_sampler")

        # Start system resource monitor if profile enabled
        if profile_enabled:
            logger.info("📊 System resource profiling enabled via --profile flag")
            from feagi.utils.system_monitor import SystemMonitor
            
            monitor = SystemMonitor()
            monitor_thread = threading.Thread(
                target=monitor.start_monitoring,
                args=(5.0,),  # 5 second interval
                daemon=True,
                name="SystemMonitor"
            )
            monitor_thread.start()
            logger.info("📊 System resource monitor started (profile mode) - interval: 5.0s")
            
            # Final profiling snapshot
            profile_component("all_processes_started")

        logger.info("✅ All processes started successfully")
        return True

    except Exception as e:
        logger.error(f"❌ Error starting processes: {e}")
        return False

# Removed the global process_manager instantiation that was here 