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

logger = setup_logger(name="feagi.process_manager")

import os
import sys
import threading
import time
import traceback
from typing import Any, Dict, Optional

# Import TOML configuration system
from feagi.config.toml_loader import (
    get_host_config,
    get_port_config,
    get_timeout_config,
    load_feagi_config,
)
from feagi.utils.port_checker import PortConflictError, check_port_availability

# Process priority levels
PRIORITY_CRITICAL = 1  # Real-time critical processes
PRIORITY_IMPORTANT = 2  # Near real-time processes
PRIORITY_BACKGROUND = 3  # Best effort processes


class ProcessManager:
    """
    FEAGI Process Manager - Orchestrates all FEAGI services with OS/platform agnostic design.

    Manages the lifecycle of critical services using a three-tier priority system:
    - Priority 1 (Critical): Core brain services that must start first
    - Priority 2 (Important): Network services that depend on Priority 1
    - Priority 3 (Background): Optional services for monitoring and management

    RUST/RTOS COMPATIBLE: Uses direct imports and shared memory instead of subprocesses.
    """

    def __init__(self):
        """Initialize ProcessManager with empty state."""
        self._processes = {}
        self._core_api = None
        self._burst_engine = None
        self._viz_fq_sampler = None
        self._motor_fq_sampler = None
        self._viz_fq_thread = None
        self._motor_fq_thread = None
        self._fq_sampler_config = {}
        self._monitoring_active = False
        self._zmq_server = None  # Add missing _zmq_server attribute

        # Add startup phase tracking
        self._startup_phase = True  # True during initial startup, False during runtime

        logger.info("[SINGLETON] ProcessManager initialized")

        # CRITICAL: Use ConnectomeManager singleton for mission-critical reliability
        from feagi.bdu.connectome_manager import ConnectomeManager

        self._connectome_manager = ConnectomeManager.instance()

        # Internal references for critical components
        self._fcl_manager = None
        self._memory_manager = None

        # Track which ports are in use
        self._used_ports = set()

        # FQ Sampler references
        self._fq_sampler = None
        self._motor_fq_sampler = None
        self._viz_fq_sampler = None

        # Initialize event system for genome load coordination
        self._setup_genome_load_event_handling()

    def load_and_validate_ports(
        self, cli_args: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Load and validate port configuration from TOML configuration.

        This method enforces the principle that NO hardcoded defaults should exist for
        network configuration. All hosts and ports must come from explicit configuration.

        Args:
            cli_args: Optional command-line arguments to override config

        Returns:
            Complete configuration dictionary with validated ports and hosts

        Raises:
            PortConflictError: If any configured port is already in use
            ValueError: If required host/port configuration is missing
            FeagiConfigurationError: If configuration loading fails
        """
        try:
            # Load TOML configuration with all overrides applied
            config = load_feagi_config(cli_args=cli_args)

            # Extract and validate host configuration (will fail if hosts not set)
            host_config = get_host_config(config)

            # Extract port configuration
            port_config = get_port_config(config)

            # Validate all ports are available on configured hosts
            for port_name, port_number in port_config.get_all_ports().items():
                try:
                    check_port_availability(host_config.zmq_host, port_number)
                    logger.debug(
                        f"Port {port_number} ({port_name}) is available on {host_config.zmq_host}"
                    )
                except PortConflictError as e:
                    logger.error(f"Port conflict detected: {e}")
                    raise PortConflictError(
                        f"Port {port_number} (used for {port_name}) is already in use on {host_config.zmq_host}. "
                        f"Edit feagi_configuration.toml to change port assignments. "
                        f"Available ports can be found using: netstat -tuln"
                    )

            # Also validate API port
            api_port = config.get("api", {}).get("port", 0)
            if api_port <= 0:
                raise ValueError("API port must be configured and greater than 0")

            try:
                check_port_availability(host_config.api_host, api_port)
                logger.debug(
                    f"API port {api_port} is available on {host_config.api_host}"
                )
            except PortConflictError as e:
                logger.error(f"API port conflict detected: {e}")
                raise PortConflictError(
                    f"API port {api_port} is already in use on {host_config.api_host}. "
                    f"Edit feagi_configuration.toml to change the api.port setting. "
                    f"Available ports can be found using: netstat -tuln"
                )

            logger.info("All port validations passed")
            return config

        except Exception as e:
            logger.error(f"Failed to load and validate port configuration: {e}")
            raise

    def find_available_port(
        self, start_port: int, max_tries: int = 10
    ) -> Optional[int]:
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
            stacklevel=2,
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

        These processes are essential for FEAGI's core operation:
        - Connectome Manager (neural structure and arrays)
        - FeagiStateManager (system state coordination)
        - Core API Services
        - Burst Engine (neural processing - initialized in STANDBY mode)

        Args:
            config: Configuration parameters loaded from TOML file

        Returns:
            True if successfully initialized, False otherwise
        """
        logger.info("Initializing critical (Priority 1) processes...")

        try:
            # --- Configure Backend ---
            # Get backend from config with fallback to 'auto'
            backend_name = config.get("npu", {}).get("backend", "auto")
            logger.info(f"Using backend: {backend_name}")

            # --- Initialize State Manager Early ---
            from feagi.core.state_manager import FeagiStateManager, ServiceState

            state_manager = FeagiStateManager.instance()
            logger.info("State Manager initialized")

            # --- Initialize Connectome Manager ---
            logger.info("Initializing connectome manager...")
            from feagi.bdu.connectome_manager import ConnectomeManager
            from feagi.core.state_manager import ConnectomeState, GenomeState

            self._connectome_manager = ConnectomeManager.instance(
                config, backend=backend_name
            )

            # Set connectome state to READY - it's operational even without genome
            state_manager.set_connectome_state(ConnectomeState.READY)
            logger.info("Connectome Manager initialized and ready")

            # CORRECT: Initialize genome state to MISSING since no genome is loaded at startup
            # The genome service will update this to LOADED when actual genomes are loaded
            state_manager.set_genome_state(GenomeState.MISSING)
            logger.info(
                "Genome state initialized as MISSING (no genome loaded at startup)"
            )

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

                # Initialize burst engine in STANDBY mode for early FQ sampler registration
                burst_engine = self._core_api.get_burst_engine()
                if burst_engine:
                    # CORRECT: Set burst engine state to UNAVAILABLE since no genome is loaded
                    # It will transition to READY when a genome is loaded and auto-start is triggered
                    state_manager.set_burst_engine_state(ServiceState.UNAVAILABLE)

                    logger.info(
                        "🔥 BURST ENGINE: UNAVAILABLE state initialized - will start when genome loads"
                    )
                else:
                    logger.error(
                        "❌ Failed to get burst engine instance - FQ sampler registration will fail!"
                    )
                    return False

            logger.info(
                "[OK] Critical processes initialized successfully", status="[OK] "
            )

            # ===== CRITICAL SERVICE READINESS GATE =====
            # ZMQ services can start unless there are actual ERROR states
            # MISSING genome and UNAVAILABLE burst engine are CORRECT states for fresh startup
            try:
                from feagi.core.state_manager import (
                    GenomeState,
                    ServiceState,
                    get_state_manager,
                )

                state_manager = get_state_manager()

                # Check for actual ERROR states that would block ZMQ services
                critical_status = state_manager.get_critical_services_status()

                # Only block on actual ERROR states, not on correct initial states
                error_states = []
                for service, state in critical_status.items():
                    if service in [
                        "burst_engine",
                        "connectome",
                        "genome",
                        "state_manager",
                    ]:
                        if state.value == "ERROR":
                            error_states.append(f"{service}: {state.value}")

                if error_states:
                    logger.error(
                        "[ERR] BLOCKED: Cannot start ZMQ services - critical services in ERROR state"
                    )
                    for error in error_states:
                        logger.error(f"[ERR]   {error}")
                    return False

                # Log the current states (these are correct for fresh startup)
                logger.info(
                    "[OK] Core critical services healthy - proceeding with ZMQ server initialization"
                )
                for service, state in critical_status.items():
                    if service in [
                        "burst_engine",
                        "connectome",
                        "genome",
                        "state_manager",
                    ]:
                        logger.info(f"[OK]   {service}: {state.value}")

            except Exception as e:
                logger.error(f"[ERR] Failed to check critical service readiness: {e}")
                return False
            # ===== END CRITICAL SERVICE READINESS GATE =====

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

        # ===== CRITICAL SERVICE READINESS GATE =====
        # BLOCK ZMQ server (and agent connections) until ALL core critical services are ready
        # Note: We don't check zmq_server and api_server since those are what we're starting here
        try:
            from feagi.core.state_manager import ServiceState, get_state_manager

            state_manager = get_state_manager()

            # Check only CORE critical services (not the ones we're about to start)
            core_critical_services = [
                "burst_engine",
                "connectome",
                "genome",
                "state_manager",
            ]
            critical_status = state_manager.get_critical_services_status()

            # Filter to only check core services
            core_status = {
                k: v for k, v in critical_status.items() if k in core_critical_services
            }

            # Only block on actual ERROR states, not on correct initial states
            # MISSING genome and UNAVAILABLE burst engine are CORRECT for fresh startup
            error_states = []
            for service, state in core_status.items():
                if state.value == "ERROR":
                    error_states.append(f"{service}: {state.value}")

            if error_states:
                logger.error(
                    "[ERR] BLOCKED: Cannot start ZMQ services - critical services in ERROR state"
                )
                for error in error_states:
                    logger.error(f"[ERR]   {error}")
                return False

            # Log current states (MISSING genome and UNAVAILABLE burst engine are correct for fresh startup)
            logger.info(
                "[OK] Core critical services healthy - proceeding with ZMQ server initialization"
            )
            for service, state in core_status.items():
                logger.info(f"[OK]   {service}: {state.value}")

        except Exception as e:
            logger.error(f"[ERR] Failed to check critical service readiness: {e}")
            return False
        # ===== END CRITICAL SERVICE READINESS GATE =====

        # Check if embedded mode is enabled
        embedded_mode = config.get("system", {}).get("embedded", False)
        if embedded_mode:
            logger.info(
                "[CONFIG] Embedded device mode enabled - disabling non-essential components"
            )

        try:
            # --- Get Stream Configuration Early ---
            zmq_config = config.get("zmq", {})
            stream_config = zmq_config.get("streams", {})

            # Check which streams are enabled (disable visualization in embedded mode)
            visualization_enabled = (
                stream_config.get("visualization", {}).get("enabled", True)
                and not embedded_mode
            )
            sensory_enabled = stream_config.get("sensory", {}).get("enabled", True)
            motor_enabled = stream_config.get("motor", {}).get("enabled", True)
            rest_enabled = stream_config.get("rest", {}).get(
                "enabled", True
            )  # REST API always enabled by default

            if embedded_mode:
                logger.info("[CONFIG] Embedded mode: Visualization stream disabled")

            logger.info(
                f"Stream configuration: visualization={visualization_enabled}, "
                f"sensory={sensory_enabled}, motor={motor_enabled}, rest={rest_enabled}"
            )

            # --- FQ Sampler Setup: On-Demand Creation Only ---
            # FQ samplers are NOT created at startup. They are created on-demand by the
            # Registration Manager when agents with specific capabilities connect.
            self._motor_fq_sampler = None
            self._viz_fq_sampler = None
            self._motor_fq_thread = None
            self._viz_fq_thread = None

            # Store configuration for on-demand creation
            self._fq_sampler_config = {
                "motor_enabled": motor_enabled,
                "visualization_enabled": visualization_enabled,
                "motor_frequency": 100.0,
                "visualization_frequency": 30.0,
            }

            logger.info(
                "🔥 FQ Samplers initialized for on-demand creation - no instances created at startup"
            )

            # --- Registration Manager Setup (Central Agent Coordination) ---
            try:
                from feagi.core.state_manager import FeagiStateManager
                from feagi.pns.registration_manager import create_registration_manager

                # Get State Manager instance
                state_manager = FeagiStateManager.instance()

                # Create Registration Manager with references to State Manager and Process Manager
                registration_manager = create_registration_manager(
                    state_manager=state_manager,
                    process_manager=self,  # Pass self reference for FQ sampler control
                )

                if registration_manager:
                    logger.info(
                        "🏛️ Registration Manager initialized - central agent coordination ready"
                    )
                else:
                    logger.error("❌ Failed to initialize Registration Manager")

            except Exception as e:
                logger.error(f"Failed to initialize Registration Manager: {e}")
                # Non-critical error - system can continue without Registration Manager

            # --- ZMQ Message Broker Setup ---
            try:
                from feagi.api.zmq.server import ZmqServer
                from feagi.core.state_manager import FeagiStateManager, ServiceState

                state_manager = FeagiStateManager.instance()

                # Get port configuration from TOML config
                port_config = get_port_config(config)

                # Get host configuration with validation (no hardcoded fallbacks)
                host_config = get_host_config(config)
                zmq_host = host_config.zmq_host

                # Windows-specific ZMQ binding fix: normalize host for binding
                # On Windows, binding to 127.0.0.1 can cause permission issues
                # Use "*" (all interfaces) for binding when host is loopback on Windows
                import platform

                if platform.system() == "Windows" and zmq_host in [
                    "127.0.0.1",
                    "localhost",
                ]:  # @architecture:acceptable - Windows compatibility fix
                    logger.info(
                        f"🪟 Windows detected: Converting ZMQ host '{zmq_host}' to '*' for proper binding"
                    )
                    zmq_bind_host = "*"
                else:
                    # On non-Windows platforms, use the configured host directly
                    # ZMQ will handle 0.0.0.0 appropriately on each platform
                    zmq_bind_host = zmq_host

                logger.info(
                    f"ZMQ server will bind to: {zmq_bind_host} (configured host: {zmq_host})"
                )

                # Use hardcoded ports from configuration
                zmq_ports = {
                    "req_rep": port_config.zmq_req_rep_port,
                    "pub_sub": port_config.zmq_pub_sub_port,
                    "push_pull": port_config.zmq_push_pull_port,
                    "sensory": (
                        port_config.zmq_sensory_port if sensory_enabled else None
                    ),
                    "motor": port_config.zmq_motor_port if motor_enabled else None,
                    "visualization": (
                        port_config.zmq_visualization_port
                        if visualization_enabled
                        else None
                    ),
                    "rest": port_config.zmq_rest_port if rest_enabled else None,
                }

                logger.info(f"Starting ZMQ server with ports: {zmq_ports}")

                # Initialize ZMQ server with configuration-based stream enablement
                zmq_server = ZmqServer(
                    core_api=self._core_api,
                    host=zmq_bind_host,
                    req_rep_port=port_config.zmq_req_rep_port,
                    pub_sub_port=port_config.zmq_pub_sub_port,
                    push_pull_port=port_config.zmq_push_pull_port,
                    sensory_port=(
                        port_config.zmq_sensory_port if sensory_enabled else None
                    ),
                    motor_port=port_config.zmq_motor_port if motor_enabled else None,
                    rest_port=port_config.zmq_rest_port if rest_enabled else None,
                    vis_port=(
                        port_config.zmq_visualization_port
                        if visualization_enabled
                        else None
                    ),
                    fq_sampler=None,  # No FQ sampler at startup - will be created on-demand
                    fire_queue_provider=self._core_api,  # Use core_api as fire_queue_provider
                    stream_config=stream_config,  # Pass stream configuration to ZMQ server
                    process_manager=self,  # Pass process manager reference for on-demand FQ sampler creation
                )

                # Start ZMQ server
                if zmq_server.start():
                    self._processes["zmq_server"] = zmq_server
                    self._zmq_server = (
                        zmq_server  # Store for registration manager access
                    )
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
            # CRITICAL SAFETY: Skip ResourceManager during brain development
            # ResourceManager uses multiprocessing which causes resource leaks and heap corruption
            # during neurogenesis when the heap is under stress
            skip_resource_manager = os.environ.get(
                "FEAGI_SKIP_RESOURCE_MANAGER", ""
            ).lower() in ("true", "1", "yes")

            if skip_resource_manager:
                logger.warning(
                    "Skipping Resource Manager initialization (FEAGI_SKIP_RESOURCE_MANAGER=true)"
                )
                logger.warning(
                    "This is a safety measure to prevent multiprocessing resource leaks during brain development"
                )
            else:
                try:
                    from feagi.core.resource_mgr import ResourceManager

                    resource_manager = ResourceManager.get_instance(
                        config.get("resources", {})
                    )

                    if resource_manager.initialize_critical_structures():
                        self._processes["resource_manager"] = resource_manager
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
                health_enabled = config.get("resources", {}).get(
                    "enable_health_check", True
                )
                if health_enabled:
                    from feagi.core.health_monitor import HealthMonitor

                    health_monitor = HealthMonitor()
                    health_monitor.start_monitoring()
                    self._processes["health_monitor"] = health_monitor
                    logger.info("Health monitor started")

            except Exception as e:
                logger.warning(f"Health monitor initialization failed: {e}")
                # Non-critical - continue without health monitoring

            # --- System Resource Monitor (Profile Mode) ---
            try:
                # Check if profiling mode is enabled via --profile flag
                profile_enabled = config.get("system", {}).get("profile", False)

                # In embedded mode, only enable profiling if explicitly requested
                if profile_enabled and (not embedded_mode or profile_enabled):
                    from feagi.utils.system_monitor import start_system_monitoring

                    # Configure monitoring based on profile settings
                    monitoring_interval = config.get("profile", {}).get(
                        "resource_monitor_interval", 5.0
                    )
                    enable_gpu = config.get("profile", {}).get("monitor_gpu", True)
                    enable_logging = config.get("profile", {}).get(
                        "monitor_logging", True
                    )

                    # In embedded mode, use minimal resource monitoring
                    if embedded_mode:
                        monitoring_interval = max(
                            monitoring_interval, 10.0
                        )  # Slower monitoring
                        enable_gpu = False  # No GPU monitoring in embedded
                        logger.info(
                            "[CONFIG] Embedded mode: Using minimal resource monitoring"
                        )

                    system_monitor = start_system_monitoring(
                        monitoring_interval=monitoring_interval,
                        enable_gpu_monitoring=enable_gpu,
                        enable_detailed_logging=enable_logging,
                    )

                    if system_monitor:
                        self._processes["system_monitor"] = system_monitor
                        logger.info(
                            f"[STATS] System resource monitor started (profile mode) - interval: {monitoring_interval}s"
                        )
                    else:
                        logger.warning("Failed to start system resource monitor")
                elif embedded_mode:
                    logger.info(
                        "[CONFIG] Embedded mode: System resource monitoring disabled (use --profile to enable minimal monitoring)"
                    )

            except Exception as e:
                logger.warning(f"System resource monitor initialization failed: {e}")
                # Non-critical - continue without resource monitoring

            logger.info("Important processes initialization completed")
            return True

        except Exception as e:
            logger.error(
                f"Critical error during important processes initialization: {e}"
            )
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
        embedded_mode = config.get("system", {}).get("embedded", False)

        try:
            # --- REST API (normal mode only) ---
            api_config = config.get("api", {})

            # Get host configuration with validation (no hardcoded fallbacks)
            host_config = get_host_config(config)
            api_host = host_config.api_host
            api_port = api_config.get("port", 8000)

            # Also get ZMQ host for embedded mode logging
            # zmq_host = host_config.zmq_host  # Unused variable removed

            # Get port configuration for embedded mode logging
            # port_config = get_port_config(config)  # Unused variable removed

            if not embedded_mode:
                try:
                    # Only run FastAPI if not in embedded mode
                    api_config = {
                        "host": api_host,
                        "port": api_port,
                        "reload": config.get("development", {}).get("reload", False),
                        "access_log": config.get("api", {}).get("access_log", True),
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
                                host=api_config["host"],
                                port=api_config["port"],
                                access_log=api_config.get("access_log", True),
                                loop="asyncio",
                            )
                        except Exception as e:
                            logger.error(f"Failed to start uvicorn: {e}")
                            logger.error(f"Full traceback: {traceback.format_exc()}")
                            # Also log the exception type and context for debugging
                            logger.error(f"Exception type: {type(e).__name__}")
                            logger.error(
                                f"Host: {api_config['host']}, Port: {api_config['port']}"
                            )
                            # Re-raise to ensure the thread actually exits with failure
                            raise

                    # Start uvicorn in background thread
                    api_thread = threading.Thread(target=run_uvicorn, daemon=True)
                    api_thread.start()

                    # Store the thread reference for shutdown
                    self._processes["rest_api"] = api_thread
                    logger.info(
                        f"REST API server started on http://{api_host}:{api_port}"
                    )

                except Exception as e:
                    logger.error(f"Failed to initialize REST API server: {e}")
                    return False
            else:
                # Embedded mode: No HTTP interface at all
                logger.info(
                    "[CONFIG] Embedded mode: REST API completely disabled for minimal resource usage"
                )
                logger.info(
                    "[CONFIG] Control interface available only via ZMQ REST stream (port 5563)"
                )
                logger.info(
                    "[CONFIG] No web interface, no FastAPI imports, no uvicorn server"
                )

            # --- WebSocket Server (Optional) ---
            try:
                websocket_enabled = config.get("websocket", {}).get("enabled", False)
                if websocket_enabled and not embedded_mode:
                    from feagi.api.websocket.server import WebSocketServer

                    ws_port = config.get("websocket", {}).get("port", 8080)
                    # Use validated host configuration (no hardcoded fallbacks)
                    ws_host = host_config.api_host  # WebSocket uses same host as API

                    ws_server = WebSocketServer(host=ws_host, port=ws_port)

                    if ws_server.start():
                        self._processes["websocket"] = ws_server
                        logger.info(f"WebSocket server started on {ws_host}:{ws_port}")
                    else:
                        logger.warning(
                            "Failed to start WebSocket server - continuing without it"
                        )
                elif embedded_mode:
                    logger.info("[CONFIG] Embedded mode: WebSocket server disabled")

            except Exception as e:
                logger.warning(f"WebSocket server initialization failed: {e}")
                # Non-critical - continue without WebSocket

            # --- Health Check Service (skip in embedded mode) ---
            try:
                health_enabled = (
                    config.get("resources", {}).get("enable_health_check", True)
                    and not embedded_mode
                )
                if health_enabled:
                    from feagi.core.health_monitor import HealthMonitor

                    health_monitor = HealthMonitor()
                    health_monitor.start_monitoring()
                    self._processes["health_monitor"] = health_monitor
                    logger.info("Health monitor started")
                elif embedded_mode:
                    logger.info("[CONFIG] Embedded mode: Health monitor disabled")

            except Exception as e:
                logger.warning(f"Health monitor initialization failed: {e}")
                # Non-critical - continue without health monitoring

            logger.info("Background processes initialization completed")
            return True

        except Exception as e:
            logger.error(
                f"Critical error during background processes initialization: {e}"
            )
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
            if hasattr(state_manager, "path"):
                os.environ["FEAGI_STATE_FILE"] = state_manager.path
                logger.info(
                    f"[LINK] Sharing state file with FastAPI thread: {state_manager.path}"
                )
            else:
                logger.warning("[WARN]  State manager has no path attribute")

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
                        app, host=config["host"], port=config["port"], loop="asyncio"
                    )
                except Exception as e:
                    logger.error(f"API service task failed: {e}")
                finally:
                    loop.close()

            # Start as daemon thread (in Rust: tokio::spawn with proper task management)
            api_thread = threading.Thread(target=run_api_service, daemon=True)
            api_thread.start()

            logger.info("[OK] API service task started successfully", status="[OK] ")
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

        # Transition from startup phase to runtime phase
        # This allows FQ samplers to be created without critical service checks during runtime
        self._startup_phase = False
        logger.info("🚀 FEAGI startup phase completed - transitioning to runtime phase")
        logger.info(
            "🔄 FQ samplers can now be created on-demand during agent registration"
        )

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

                # Use configurable timeout instead of hardcoded value
                try:
                    config = load_feagi_config()
                    timeout_config = get_timeout_config(config)
                    monitor_interval = (
                        timeout_config.polling_timeout / 1000.0
                    )  # Convert ms to seconds
                except Exception:
                    # Fallback if config unavailable
                    monitor_interval = 5.0

                time.sleep(monitor_interval)  # Use configurable interval

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
                if hasattr(service, "is_running") and callable(service.is_running):
                    # Service with is_running() method (like ZmqServer)
                    if not service.is_running():
                        logger.error(f"Service {name} is not running")
                elif hasattr(service, "is_alive") and callable(service.is_alive):
                    # Thread-like objects
                    if not service.is_alive():
                        logger.error(f"Thread {name} has stopped unexpectedly")
                        if name == "rest_api":
                            logger.error(
                                "REST API thread failure - this usually indicates:"
                            )
                            logger.error("  1. Unicode/encoding issues in log messages")
                            logger.error("  2. Import errors or missing dependencies")
                            logger.error("  3. Port conflicts or network issues")
                            logger.error("  4. FastAPI/uvicorn startup failures")
                            logger.error(
                                "Check the detailed traceback above for the root cause"
                            )
                elif hasattr(service, "poll") and callable(service.poll):
                    # Legacy subprocess
                    if service.poll() is not None:
                        exit_code = service.poll()
                        logger.error(f"Process {name} exited with code {exit_code}")
                else:
                    # Service type we can't monitor - silently skip
                    # (Expected for HealthMonitor, ZmqServer, ResourceManager etc.)
                    pass

            except Exception as e:
                logger.warning(f"Error checking service {name}: {e}")

    def get_core_api(self):
        """Get the Core API instance."""
        return self._core_api

    def get_zmq_server(self):
        """Get the ZMQ server instance."""
        return self._zmq_server

    def shutdown(self) -> None:
        """
        Gracefully shutdown all FEAGI processes and services.

        Uses configurable timeout values from TOML configuration instead of hardcoded values.
        """
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        try:
            print("Shutting down FEAGI services...", file=sys.stderr, flush=True)

            # Stop running flag to signal all services to stop
            self._running = False

            # Import required modules for timeout handling
            import threading

            # Load timeout configuration from TOML (use defaults if config unavailable during shutdown)
            try:
                config = load_feagi_config()
                timeout_config = get_timeout_config(config)
                graceful_shutdown_timeout = timeout_config.graceful_shutdown
                thread_join_timeout = timeout_config.thread_join
                process_join_timeout = timeout_config.process_join
                fq_sampler_timeout = timeout_config.fq_sampler_shutdown
            except Exception as e:
                print(
                    f"Warning: Could not load timeout config during shutdown, using fallback values: {e}",
                    file=sys.stderr,
                    flush=True,
                )
                # Emergency fallback values only used if configuration is completely unavailable
                graceful_shutdown_timeout = (
                    8.0  # @architecture:acceptable - emergency fallback
                )
                thread_join_timeout = (
                    2.0  # @architecture:acceptable - emergency fallback
                )
                process_join_timeout = (
                    2.0  # @architecture:acceptable - emergency fallback
                )
                # fq_sampler_timeout = (
                #     2.0  # @architecture:acceptable - emergency fallback
                # )  # Unused variable removed

            # Shutdown each service properly based on its type with configurable timeouts
            for name, service in self._processes.items():
                try:
                    print(f"Stopping service: {name}...", file=sys.stderr, flush=True)

                    # Create a shutdown function that can be run with timeout
                    def shutdown_service():
                        try:
                            # Handle different service types
                            if name == "zmq_server" and hasattr(service, "shutdown"):
                                # ZMQ server has a shutdown method
                                service.shutdown()
                            elif name == "rest_api" and hasattr(service, "stop"):
                                # REST API server has a stop method
                                service.stop()
                            elif name == "websocket" and hasattr(service, "stop"):
                                # WebSocket server has a stop method
                                service.stop()
                            elif name == "health_monitor" and hasattr(service, "stop"):
                                # Health monitor has a stop method
                                service.stop()
                            elif name == "system_monitor" and hasattr(service, "stop"):
                                # System resource monitor has a stop method
                                service.stop()
                            elif name == "resource_manager" and hasattr(
                                service, "cleanup"
                            ):
                                # Resource manager has a cleanup method
                                service.cleanup()
                            elif hasattr(service, "terminate") and hasattr(
                                service, "poll"
                            ):
                                # Legacy subprocess handling
                                if service.poll() is None:
                                    service.terminate()
                                    service.wait(timeout=process_join_timeout)
                            elif hasattr(service, "is_alive") and hasattr(
                                service, "join"
                            ):
                                # Thread-like objects
                                if service.is_alive():
                                    service.join(timeout=thread_join_timeout)
                            else:
                                print(
                                    f"Service {name} doesn't have a known shutdown method",
                                    file=sys.stderr,
                                    flush=True,
                                )
                        except Exception as e:
                            print(
                                f"Error in shutdown_service for {name}: {e}",
                                file=sys.stderr,
                                flush=True,
                            )

                    # Run shutdown with timeout using a separate thread
                    shutdown_thread = threading.Thread(
                        target=shutdown_service, daemon=True
                    )
                    shutdown_thread.start()
                    shutdown_thread.join(timeout=graceful_shutdown_timeout)

                    if shutdown_thread.is_alive():
                        print(
                            f"[WARN]  Service {name} didn't stop within {graceful_shutdown_timeout}s - continuing anyway",
                            file=sys.stderr,
                            flush=True,
                        )

                except Exception as e:
                    print(
                        f"Error stopping service {name}: {e}",
                        file=sys.stderr,
                        flush=True,
                    )

            # Stop Motor FQSampler if running
            if hasattr(self, "_motor_fq_sampler") and self._motor_fq_sampler:
                print("Stopping Motor FQSampler...", file=sys.stderr, flush=True)
                try:
                    self._motor_fq_sampler.stop()
                    # Wait for thread to finish
                    if (
                        hasattr(self, "_motor_fq_thread")
                        and self._motor_fq_thread
                        and self._motor_fq_thread.is_alive()
                    ):
                        self._motor_fq_thread.join(timeout=2.0)
                        if self._motor_fq_thread.is_alive():
                            print(
                                "Motor FQ Sampler thread did not stop within timeout",
                                file=sys.stderr,
                                flush=True,
                            )
                    self._motor_fq_sampler = None
                    self._motor_fq_thread = None
                except Exception as e:
                    print(
                        f"Error stopping Motor FQSampler: {e}",
                        file=sys.stderr,
                        flush=True,
                    )

            # Stop Visualization FQSampler if running
            if hasattr(self, "_viz_fq_sampler") and self._viz_fq_sampler:
                print(
                    "Stopping Visualization FQSampler...", file=sys.stderr, flush=True
                )
                try:
                    self._viz_fq_sampler.stop()
                    # Wait for thread to finish
                    if (
                        hasattr(self, "_viz_fq_thread")
                        and self._viz_fq_thread
                        and self._viz_fq_thread.is_alive()
                    ):
                        self._viz_fq_thread.join(timeout=2.0)
                        if self._viz_fq_thread.is_alive():
                            print(
                                "Visualization FQ Sampler thread did not stop within timeout",
                                file=sys.stderr,
                                flush=True,
                            )
                    self._viz_fq_sampler = None
                    self._viz_fq_thread = None
                except Exception as e:
                    print(
                        f"Error stopping Visualization FQSampler: {e}",
                        file=sys.stderr,
                        flush=True,
                    )

            print("FEAGI services shut down", file=sys.stderr, flush=True)

        except Exception as e:
            # Last resort error handling - print to stderr and continue
            print(f"Critical error during shutdown: {e}", file=sys.stderr, flush=True)

    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        self.shutdown()

    def update_area_sample_rate(self, cortical_id, rate):
        """
        Update sample rate for specific cortical areas in the appropriate FQ sampler.

        Args:
            cortical_id: ID of the cortical area
            rate: New sampling rate in Hz
        """
        # Check if it's an OPU area (motor) or other area (visualization)
        is_motor_area = False

        # Try to determine if this is a motor area
        try:
            if self._connectome_manager:
                area_info = self._connectome_manager.get_area_info(cortical_id)
                if area_info and area_info.get("type") == "OPU":
                    is_motor_area = True
        except Exception:
            # If we can't determine area type, assume visualization
            pass

        # Update the appropriate sampler
        if (
            is_motor_area
            and hasattr(self, "_motor_fq_sampler")
            and self._motor_fq_sampler is not None
        ):
            logger.info(
                f"Updating motor sampler frequency for area {cortical_id}: {rate}Hz"
            )
            self._motor_fq_sampler.set_sample_frequency(rate)
        elif (
            not is_motor_area
            and hasattr(self, "_viz_fq_sampler")
            and self._viz_fq_sampler is not None
        ):
            logger.info(
                f"Updating visualization sampler frequency for area {cortical_id}: {rate}Hz"
            )
            self._viz_fq_sampler.set_sample_frequency(rate)
        else:
            logger.warning(
                f"Could not update sample rate for area {cortical_id}: appropriate sampler not available"
            )

    def get_motor_fq_sampler(self):
        """Get the motor FQ sampler instance."""
        return getattr(self, "_motor_fq_sampler", None)

    def get_viz_fq_sampler(self):
        """Get the visualization FQ sampler instance."""
        return getattr(self, "_viz_fq_sampler", None)

    def get_fq_sampler_performance_stats(self):
        """Get performance statistics from both samplers."""
        stats = {}

        if hasattr(self, "_motor_fq_sampler") and self._motor_fq_sampler:
            stats["motor"] = self._motor_fq_sampler.get_performance_stats()

        if hasattr(self, "_viz_fq_sampler") and self._viz_fq_sampler:
            stats["visualization"] = self._viz_fq_sampler.get_performance_stats()

        return stats

    def create_fq_sampler(self, mode: str, frequency: float) -> bool:
        """
        Create and register FQ sampler of the specified mode.

        Args:
            mode: Sampling mode ('visualization', 'opu')
            frequency: Sampling frequency in Hz

        Returns:
            True if sampler was created and registered successfully, False otherwise
        """
        logger.info(f"🔥 Creating FQ Sampler: mode={mode}, frequency={frequency}Hz")

        # ===== CRITICAL SERVICE READINESS GATE (STARTUP ONLY) =====
        # ONLY block FQ sampler creation during initial startup phase
        # During runtime, agents should be able to register immediately
        if self._startup_phase:
            logger.debug(
                "🔄 Startup phase: Checking critical service readiness for FQ sampler creation"
            )
            try:
                from feagi.core.state_manager import get_state_manager

                state_manager = get_state_manager()

                # Check if system is ready for FQ samplers
                if not state_manager.is_system_ready_for_fq_samplers():
                    logger.warning(
                        f"🚨 BLOCKED: FQ sampler creation for mode '{mode}' - critical services not ready"
                    )

                    # Get detailed status for debugging
                    critical_status = state_manager.get_critical_services_status()
                    not_ready = [
                        service
                        for service, status in critical_status.items()
                        if status.value != "READY"
                    ]

                    if not_ready:
                        logger.warning(f"🚨 Services not ready: {', '.join(not_ready)}")
                        for service, status in critical_status.items():
                            if status.value != "READY":
                                logger.warning(f"🚨   {service}: {status.value}")

                    # Wait briefly for services to become ready (non-blocking timeout)
                    timeout_seconds = 10.0
                    logger.info(
                        f"🔄 Waiting up to {timeout_seconds} seconds for critical services..."
                    )
                    
                    if state_manager.wait_for_critical_services(
                        timeout_seconds=timeout_seconds, check_interval=0.5
                    ):
                        logger.info(
                            "✅ Critical services now ready - proceeding with FQ sampler creation"
                        )
                    else:
                        logger.error(
                            "❌ TIMEOUT: Critical services still not ready after 10 seconds"
                        )
                        logger.error(f"❌ FQ sampler creation DENIED for mode '{mode}'")
                        return False
                else:
                    logger.debug(
                        f"✅ Critical services ready - proceeding with FQ sampler creation for mode '{mode}'"
                    )

            except Exception as gate_error:
                logger.error(
                    f"🚨 Error in critical service readiness gate: {gate_error}"
                )
                logger.error(
                    "🚨 Proceeding with FQ sampler creation anyway to maintain backward compatibility"
                )
        else:
            logger.debug(
                "🔄 Runtime phase: Skipping critical service check for FQ sampler creation (services already verified)"
            )

        # ===== END CRITICAL SERVICE READINESS GATE =====

        try:
            # Check if sampler already exists for this mode
            if mode == "visualization" and self._viz_fq_sampler is not None:
                logger.warning(
                    f"🔥 Visualization FQ Sampler already exists: {self._viz_fq_sampler.instance_id}"
                )
                return True
            elif mode == "opu" and self._motor_fq_sampler is not None:
                logger.warning(
                    f"🔥 Motor FQ Sampler already exists: {self._motor_fq_sampler.instance_id}"
                )
                return True

            # Import FQ sampler
            from feagi.npu.fq_sampler import UnifiedFQSampler

            # Create the sampler
            fq_sampler = UnifiedFQSampler(
                fire_queue_provider=self._core_api,
                sample_frequency_hz=frequency,
                sampling_mode=mode,
                connectome_manager=(
                    self._core_api.get_connectome_manager() if self._core_api else None
                ),
                state_manager=(
                    self._core_api.get_state_manager() if self._core_api else None
                ),
            )

            # Store the sampler based on mode
            if mode == "visualization":
                self._viz_fq_sampler = fq_sampler
                logger.info(
                    f"🎨 Visualization FQ Sampler created: {fq_sampler.instance_id}"
                )
            elif mode == "opu":
                self._motor_fq_sampler = fq_sampler
                logger.info(f"🚗 Motor FQ Sampler created: {fq_sampler.instance_id}")
            else:
                logger.warning(f"Unknown FQ sampler mode: {mode}")
                return False

            # [ARCHITECTURE FIX] Register with burst engine for data flow coordination
            # The burst engine should always be available in STANDBY mode for registration
            logger.debug(
                f"🔥 [DEBUG] Attempting to register FQ sampler [{fq_sampler.instance_id}] with burst engine"
            )
            logger.debug(
                f"🔥 [DEBUG] Burst engine available: {self._burst_engine is not None}"
            )
            if self._burst_engine:
                logger.debug(
                    f"🔥 [DEBUG] Burst engine type: {type(self._burst_engine)}"
                )
                logger.debug(
                    f"🔥 [DEBUG] Burst engine has register method: {hasattr(self._burst_engine, 'register_fq_sampler')}"
                )
                try:
                    self._burst_engine.register_fq_sampler(fq_sampler)
                    logger.info(
                        f"🔥 FQ sampler [{fq_sampler.instance_id}] registered with burst engine successfully"
                    )
                    logger.debug(
                        "🔥 [DEBUG] Registration completed - checking burst engine FQ sampler count"
                    )
                    if hasattr(self._burst_engine, "_fq_samplers"):
                        sampler_count = len(self._burst_engine._fq_samplers)
                        logger.debug(
                            f"🔥 [DEBUG] Burst engine now has {sampler_count} registered FQ samplers"
                        )
                    else:
                        logger.warning(
                            "🔥 [DEBUG] Burst engine has no _fq_samplers attribute - registration may have failed"
                        )
                except Exception as reg_error:
                    logger.error(
                        f"🔥 Failed to register FQ sampler [{fq_sampler.instance_id}] with burst engine: {reg_error}"
                    )
                    import traceback

                    logger.error(
                        f"🔥 Registration error traceback: {traceback.format_exc()}"
                    )
                    # Continue anyway - FQ sampler can still function without registration
            else:
                logger.error(
                    f"🔥 Could not register FQ sampler [{fq_sampler.instance_id}] - burst engine not available"
                )
                logger.error(
                    "🔥 This should not happen if burst engine is properly initialized in STANDBY mode"
                )
                logger.error(
                    f"🔥 [DEBUG] Process manager state: _burst_engine={self._burst_engine}"
                )
                logger.error(f"🔥 [DEBUG] Core API state: {self._core_api is not None}")
                if self._core_api:
                    try:
                        be_from_api = self._core_api.get_burst_engine()
                        logger.error(
                            f"🔥 [DEBUG] Burst engine from core API: {be_from_api is not None}"
                        )
                    except Exception as api_error:
                        logger.error(
                            f"🔥 [DEBUG] Error getting burst engine from core API: {api_error}"
                        )
                # Continue anyway - the system should still function

            # CRITICAL FIX: DO NOT start internal run() thread for stream-based samplers
            # Streams (visualization/motor) call sample() directly, so running internal thread
            # causes double execution and double logging
            logger.info(
                f"🔥 FQ Sampler [{fq_sampler.instance_id}] created for {mode} mode"
            )
            logger.info(
                f"🔥 Internal run() thread NOT started - {mode} stream will call sample() directly"
            )
            logger.info(
                "🔥 This prevents double logging from both thread and stream calling sample()"
            )

            # Store reference without thread for streams to use
            if mode == "visualization":
                self._viz_fq_thread = None  # No thread needed
                logger.info("[RENDER] Visualization FQ Sampler ready for stream usage")
            elif mode == "opu":
                self._motor_fq_thread = None  # No thread needed
                logger.info("[MOTOR] Motor FQ Sampler ready for stream usage")

            logger.info(
                f"[DEBUG] FQ Sampler created successfully: mode={mode} (stream-based, no internal thread)"
            )

            return True

        except Exception as e:
            logger.error(f"🔥 Failed to create FQ sampler (mode={mode}): {e}")
            return False

    def disable_fq_sampler(self, mode: str):
        """
        Disable and destroy FQ sampler of the specified mode.

        Args:
            mode: Sampling mode to disable ('visualization', 'opu')
        """
        logger.info(f"🔥 Disabling FQ Sampler: mode={mode}")

        try:
            if mode == "visualization":
                if self._viz_fq_sampler is not None:
                    # Unregister from burst engine first
                    if self._burst_engine:
                        self._burst_engine.unregister_fq_sampler(self._viz_fq_sampler)
                        logger.info(
                            f"🔥 Visualization FQ sampler [{getattr(self._viz_fq_sampler, 'instance_id', 'unknown')}] unregistered from burst engine"
                        )

                    if hasattr(self._viz_fq_sampler, "stop"):
                        self._viz_fq_sampler.stop()

                    if self._viz_fq_thread and self._viz_fq_thread.is_alive():
                        self._viz_fq_thread.join(timeout=2.0)

                    self._viz_fq_sampler = None
                    self._viz_fq_thread = None
                    logger.info("🎨 Visualization FQ sampler disabled and destroyed")

            elif mode == "opu" or mode == "motor":
                if self._motor_fq_sampler is not None:
                    # Unregister from burst engine first
                    if self._burst_engine:
                        self._burst_engine.unregister_fq_sampler(self._motor_fq_sampler)
                        logger.info(
                            f"🔥 Motor FQ sampler [{getattr(self._motor_fq_sampler, 'instance_id', 'unknown')}] unregistered from burst engine"
                        )

                    if hasattr(self._motor_fq_sampler, "stop"):
                        self._motor_fq_sampler.stop()

                    if self._motor_fq_thread and self._motor_fq_thread.is_alive():
                        self._motor_fq_thread.join(timeout=2.0)

                    self._motor_fq_sampler = None
                    self._motor_fq_thread = None
                    logger.info("🚗 Motor FQ sampler disabled and destroyed")
            else:
                logger.warning(f"Unknown FQ sampler mode to disable: {mode}")

        except Exception as e:
            logger.error(f"Error disabling FQ sampler ({mode}): {e}")

    def _setup_genome_load_event_handling(self):
        """
        Set up event handling for genome load events.

        When a genome is successfully loaded, this will automatically start the burst engine.
        This implements the design requirement: "upon genome load, burst engine transitions to running"
        """
        try:
            # Import event system here to avoid circular imports
            from feagi.api.shared_memory.events import (
                EventNotificationSystem,
                EventType,
            )

            # Create event system for this process manager
            self._event_system = EventNotificationSystem("process_manager")

            # Register handler for genome loaded events
            self._event_system.register_handler(
                EventType.GENOME_LOADED, self._handle_genome_loaded_event
            )

            # Start the event system
            self._event_system.start()

            logger.info(
                "📡 Event system initialized - listening for GENOME_LOADED events"
            )

        except Exception as e:
            logger.warning(f"Failed to initialize genome load event handling: {e}")
            logger.warning(
                "Process manager will need to monitor state changes manually"
            )

    def _handle_genome_loaded_event(self, event):
        """
        Handle genome loaded event by starting the burst engine.

        Args:
            event: The genome loaded event containing filename and cortical area count
        """
        try:
            logger.info(
                f"🧬 GENOME_LOADED event received: {event.data.get('filename', 'unknown')} "
                f"with {event.data.get('cortical_areas', 0)} cortical areas"
            )

            # Check if burst engine is available and not already running
            from feagi.core.state_manager import FeagiStateManager, ServiceState

            state_manager = FeagiStateManager.instance()

            current_state = state_manager.get_burst_engine_state()

            if current_state == ServiceState.READY:
                logger.info("⚡ Burst engine already running - no action needed")
                return

            # Start burst engine through the SAME exact method as the REST API
            logger.info("⚡ Starting burst engine after genome load...")

            try:
                # Use the CoreAPIService start method (same as REST API) instead of brain service directly
                # This ensures identical behavior between manual start and automatic genome start
                core_api = self.get_core_api()
                if not core_api:
                    logger.error(
                        "❌ Core API service not available for burst engine start"
                    )
                    return

                success = core_api.start_burst_engine()

                if success:
                    logger.info(
                        "✅ Burst engine started successfully after genome load"
                    )
                else:
                    logger.error("❌ Failed to start burst engine after genome load")

            except Exception as start_error:
                logger.error(
                    f"❌ Error starting burst engine after genome load: {start_error}"
                )

        except Exception as e:
            logger.error(f"Error handling genome loaded event: {e}")
            # Don't re-raise - event handling should not crash the process manager


# Global instance for the process manager
_process_manager = None


def get_process_manager() -> ProcessManager:
    """Get the global ProcessManager instance."""
    global _process_manager
    if _process_manager is None:
        logger.info("[SINGLETON] Creating new ProcessManager instance")
        _process_manager = ProcessManager()
    else:
        logger.debug("[SINGLETON] Reusing existing ProcessManager instance")
    return _process_manager


def reset_process_manager() -> None:
    """Reset the global ProcessManager instance. USE WITH CAUTION - only for testing or emergency cleanup."""
    global _process_manager
    if _process_manager is not None:
        logger.warning("[SINGLETON] Resetting ProcessManager instance")
        try:
            _process_manager.shutdown()
        except Exception:
            pass
        _process_manager = None
    else:
        logger.debug("[SINGLETON] ProcessManager already None - no reset needed")


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
        profile_enabled = config.get("system", {}).get("profile", False)

        if profile_enabled:
            # Import and start detailed profiling
            from feagi.utils.resource_profiler import profile_component, start_profiling

            start_profiling()
            logger.info("[SEARCH] Detailed resource profiling enabled")

            # Profile initial state
            profile_component("startup_baseline")

        # ... existing code ...

        # Start Health Monitor (Important processes)
        if startup_config.get("health_monitor", {}).get("enabled", True):
            logger.info("Starting Health Monitor...")

            if profile_enabled:
                profile_component("pre_health_monitor")

            # Health monitor startup code here...

            if profile_enabled:
                profile_component("post_health_monitor")

        # Start Resource Manager
        if startup_config.get("resource_manager", {}).get("enabled", True):
            logger.info("Starting Resource Manager...")

            if profile_enabled:
                profile_component("pre_resource_manager")

            # Resource manager startup code here...

            if profile_enabled:
                profile_component("post_resource_manager")

        # Start REST API server (Critical processes)
        if startup_config.get("rest_api", {}).get("enabled", True):
            logger.info("Starting REST API server...")

            if profile_enabled:
                profile_component("pre_rest_api")

            # Start REST API
            rest_host = config.get("api", {}).get("rest_host", "0.0.0.0")
            rest_port = config.get("api", {}).get("rest_port", 8080)

            # TODO: Complete REST API server implementation
            # rest_process = multiprocessing.Process(
            #     target=_start_rest_api_server,
            #     args=(rest_host, rest_port),
            #     name="FEAGI-REST-API",
            # )
            # rest_process.start()
            # _active_processes.append(rest_process)
            logger.info(f"REST API server started on {rest_host}:{rest_port}")

            if profile_enabled:
                profile_component("post_rest_api")

        # Start ZMQ server (Critical processes)
        if startup_config.get("zmq_server", {}).get("enabled", True):
            logger.info("Starting ZMQ server...")

            if profile_enabled:
                profile_component("pre_zmq_server")

            # ZMQ server startup code here...

            if profile_enabled:
                profile_component("post_zmq_server")

        # Start Burst Engine (Important processes)
        if startup_config.get("burst_engine", {}).get("enabled", True):
            logger.info("Starting Burst Engine...")

            if profile_enabled:
                profile_component("pre_burst_engine")

            # Burst engine startup code here...

            if profile_enabled:
                profile_component("post_burst_engine")

        # Start FQ Sampler
        if startup_config.get("fq_sampler", {}).get("enabled", True):
            logger.info("Starting FQ Sampler...")

            if profile_enabled:
                profile_component("pre_fq_sampler")

            # FQ sampler startup code here...

            if profile_enabled:
                profile_component("post_fq_sampler")

        # Start system resource monitor if profile enabled
        if profile_enabled:
            logger.info("[STATS] System resource profiling enabled via --profile flag")
            from feagi.utils.system_monitor import SystemMonitor

            monitor = SystemMonitor()
            monitor_thread = threading.Thread(
                target=monitor.start_monitoring,
                args=(5.0,),  # 5 second interval
                daemon=True,
                name="SystemMonitor",
            )
            monitor_thread.start()
            logger.info(
                "[STATS] System resource monitor started (profile mode) - interval: 5.0s"
            )

            # Final profiling snapshot
            profile_component("all_processes_started")

        logger.info("[OK] All processes started successfully")
        return True

    except Exception as e:
        logger.error(f"[ERR] Error starting processes: {e}")
        return False


# Removed the global process_manager instantiation that was here
