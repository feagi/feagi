"""Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at
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
from typing import Any, Dict, List, Optional

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
        self._zmq_server = None  # Legacy attribute (now points to Rust PNS)
        self._pns = None  # Rust PNS (Peripheral Nervous System)
        self._sleep_manager = None

        # Add startup phase tracking
        self._startup_phase = (
            True  # True during initial startup, False during runtime
        )

        logger.info("[SINGLETON] ProcessManager initialized")

        #  CRITICAL: ConnectomeManager will be initialized in init_critical_processes
        #  with proper configuration and backend parameters
        self._connectome_manager = None

        # Internal references for critical components
        self._fcl_manager = None
        self._memory_manager = None
        
        # 🦀 Rust NPU Integration (will be set during init_critical_processes)
        self.rust_npu_integration = None

        # Track which ports are in use
        self._used_ports = set()

        # FQ Sampler references
        self._fq_sampler = None
        self._motor_fq_sampler = None
        self._viz_fq_sampler = None

        # Event system for genome load coordination (lazy initialization)
        self._event_system = None
        self._event_system_available = None  # None = not checked, True/False = availability status

    def load_and_validate_ports(
        self,
        cli_args: Optional[Dict[str, Any]] = None,
        explicit_config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Load and validate port configuration from TOML configuration.

        This method enforces the principle that NO hardcoded defaults should exist for
        network configuration. All hosts and ports must come from explicit configuration.

        Args:
            cli_args: Optional command-line arguments to override config
            explicit_config: Optional explicit configuration (for testing), bypasses file loading

        Returns:
            Complete configuration dictionary with validated ports and hosts

        Raises:
            PortConflictError: If any configured port is already in use
            ValueError: If required host/port configuration is missing
            FeagiConfigurationError: If configuration loading fails
        """
        try:
            #  Load TOML configuration with all overrides applied, or use
            #  explicit config for testing
            if explicit_config is not None:
                config = explicit_config
            else:
                config = load_feagi_config(cli_args=cli_args)

            #  Extract and validate host configuration (will fail if hosts not
            #  set)
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
                raise ValueError(
                    "API port must be configured and greater than 0"
                )

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
            logger.error(
                f"Failed to load and validate port configuration: {e}"
            )
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
        """Initialize Priority 1 (Critical) processes.

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
            from feagi.core.state_manager import (
                FeagiStateManager,
                ServiceState,
            )

            state_manager = FeagiStateManager.instance()
            logger.info("State Manager initialized")

            # --- Initialize Connectome Manager ---
            logger.info("Initializing connectome manager...")
            from feagi.bdu.connectome_manager import ConnectomeManager
            from feagi.core.state_manager import ConnectomeState, GenomeState

            self._connectome_manager = ConnectomeManager.instance(
                config, backend=backend_name
            )

            #  Set connectome state to READY - it's operational even without
            #  genome
            state_manager.set_connectome_state(ConnectomeState.READY)
            logger.info("Connectome Manager initialized and ready")

            #  CORRECT: Initialize genome state to MISSING since no genome is
            #  loaded at startup
            #  The genome service will update this to LOADED when actual
            #  genomes are loaded
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
                self._connectome_manager = (
                    self._core_api.get_connectome_manager()
                )
                self._fcl_manager = self._core_api.get_fcl_manager()
                self._memory_manager = self._core_api.get_memory_manager()
                
                # 🦀 RUST: Get Rust NPU integration from ConnectomeManager's NPU Interface
                if hasattr(self._connectome_manager, '_npu_interface'):
                    npu_interface = self._connectome_manager._npu_interface
                    if hasattr(npu_interface, '_rust_npu_integration'):
                        self.rust_npu_integration = npu_interface._rust_npu_integration
                        logger.info("🦀 [RUST-NPU] Integration attached to ProcessManager")
                    else:
                        logger.warning("🦀 [RUST-NPU] NPU Interface has no _rust_npu_integration")
                else:
                    logger.warning("🦀 [RUST-NPU] ConnectomeManager has no _npu_interface")

                #  Burst engine is now pure Rust - Python wrapper is deprecated
                #  Set initial burst engine state
                state_manager.set_burst_engine_state(
                    ServiceState.UNAVAILABLE
                )

                logger.info(
                    "🦀 BURST ENGINE: Pure Rust burst loop - will start when genome loads"
                )

            logger.info(
                "[OK] Critical processes initialized successfully",
                status="[OK] ",
            )

            # ===== CRITICAL SERVICE READINESS GATE =====
            # ZMQ services can start unless there are actual ERROR states
            #  MISSING genome and UNAVAILABLE burst engine are CORRECT states
            #  for fresh startup
            try:
                from feagi.core.state_manager import (
                    GenomeState,
                    ServiceState,
                    get_state_manager,
                )

                state_manager = get_state_manager()

                # Check for actual ERROR states that would block ZMQ services
                critical_status = state_manager.get_critical_services_status()

                #  Only block on actual ERROR states, not on correct initial
                #  states
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
                logger.error(
                    f"[ERR] Failed to check critical service readiness: {e}"
                )
                return False
            # ===== END CRITICAL SERVICE READINESS GATE =====

            return True

        except Exception as e:
            logger.error(f"Failed to initialize critical processes: {e}")
            return False

    def init_important_processes(self, config: Dict[str, Any]) -> bool:
        """Initialize Priority 2 (Important) processes.

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
        #  BLOCK ZMQ server (and agent connections) until ALL core critical
        #  services are ready
        #  Note: We don't check zmq_server and api_server since those are what
        #  we're starting here
        try:
            from feagi.core.state_manager import (
                ServiceState,
                get_state_manager,
            )

            state_manager = get_state_manager()

            #  Check only CORE critical services (not the ones we're about to
            #  start)
            core_critical_services = [
                "burst_engine",
                "connectome",
                "genome",
                "state_manager",
            ]
            critical_status = state_manager.get_critical_services_status()

            # Filter to only check core services
            core_status = {
                k: v
                for k, v in critical_status.items()
                if k in core_critical_services
            }

            # Only block on actual ERROR states, not on correct initial states
            #  MISSING genome and UNAVAILABLE burst engine are CORRECT for
            #  fresh startup
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

            #  Log current states (MISSING genome and UNAVAILABLE burst engine
            #  are correct for fresh startup)
            logger.info(
                "[OK] Core critical services healthy - proceeding with ZMQ server initialization"
            )
            for service, state in core_status.items():
                logger.info(f"[OK]   {service}: {state.value}")

        except Exception as e:
            logger.error(
                f"[ERR] Failed to check critical service readiness: {e}"
            )
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

            #  Check which streams are enabled (disable visualization in
            #  embedded mode)
            visualization_enabled = (
                stream_config.get("visualization", {}).get("enabled", True)
                and not embedded_mode
            )
            sensory_enabled = stream_config.get("sensory", {}).get(
                "enabled", True
            )
            motor_enabled = stream_config.get("motor", {}).get("enabled", True)
            rest_enabled = stream_config.get("rest", {}).get(
                "enabled", True
            )  # REST API always enabled by default

            if embedded_mode:
                logger.info(
                    "[CONFIG] Embedded mode: Visualization stream disabled"
                )

            logger.info(
                f"Stream configuration: visualization={visualization_enabled}, "
                f"sensory={sensory_enabled}, motor={motor_enabled}, rest={rest_enabled}"
            )

            # --- FQ Sampler Setup: On-Demand Creation Only ---
            #  FQ samplers are NOT created at startup. They are created
            #  on-demand by the
            #  Registration Manager when agents with specific capabilities
            #  connect.
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
                from feagi.pns.registration_manager import (
                    create_registration_manager,
                )

                # Get State Manager instance
                state_manager = FeagiStateManager.instance()

                #  Create Registration Manager with references to State Manager
                #  and Process Manager
                registration_manager = create_registration_manager(
                    state_manager=state_manager,
                    process_manager=self,  # Pass self reference for FQ sampler control
                )

                if registration_manager:
                    logger.info(
                        "🏛️ Registration Manager initialized - central agent coordination ready"
                    )
                    
                    # Start ZMQ registration listener
                    try:
                        from feagi.pns.zmq_registration_listener import ZmqRegistrationListener
                        
                        # Get registration port from config (default 5000)
                        registration_port = 5000
                        try:
                            config = load_feagi_config()
                            registration_port = config.get("agent", {}).get("registration_port", 5000)
                        except Exception:
                            pass
                        
                        zmq_reg_listener = ZmqRegistrationListener(
                            registration_manager=registration_manager,
                            host="*",
                            port=registration_port
                        )
                        zmq_reg_listener.start()
                        
                        # Store for later access
                        self._zmq_registration_listener = zmq_reg_listener
                        logger.info(f"🦀 ZMQ registration listener started on port {registration_port}")
                        
                    except Exception as e:
                        logger.warning(f"Failed to start ZMQ registration listener: {e}")
                        # Non-fatal - REST API registration still works
                else:
                    logger.error(
                        "❌ Failed to initialize Registration Manager"
                    )

            except Exception as e:
                logger.error(f"Failed to initialize Registration Manager: {e}")
                #  Non-critical error - system can continue without
                #  Registration Manager

            # --- ZMQ Message Broker Setup (Rust PNS) ---
            try:
                import feagi_rust
                from feagi.core.state_manager import (
                    FeagiStateManager,
                    ServiceState,
                )

                state_manager = FeagiStateManager.instance()
                logger.info("🦀 Initializing Rust PNS (Peripheral Nervous System) for ZMQ services")

                # Create Rust PNS
                pns = feagi_rust.PyPNS()
                
                # Connect PNS to burst engine for SHM I/O coordination
                if self._core_api and hasattr(self._core_api, '_rust_npu_integration'):
                    rust_npu_integration = self._core_api._rust_npu_integration
                    if rust_npu_integration and rust_npu_integration._rust_npu:
                        pns.connect_to_burst_engine(rust_npu_integration._rust_npu)
                        logger.info("🦀 PNS connected to burst engine's sensory agent manager")
                    else:
                        logger.warning("🦀 PNS created but burst engine not yet initialized - will connect later")
                else:
                    logger.warning("🦀 PNS created but core API not yet initialized - will connect later")

                # Register Python callbacks for agent lifecycle events
                logger.info("🦀 Registering Python callbacks with Rust PNS")
                pns.set_on_agent_registered(self._on_agent_registered)
                pns.set_on_agent_deregistered(self._on_agent_deregistered)
                logger.info("🦀 Callbacks registered successfully")

                # Start PNS ZMQ streams
                pns.start()
                logger.info("🦀 PNS ZMQ streams started successfully")
                
                # Store PNS for access by registration manager and other components
                self._processes["zmq_server"] = pns  # Use same key for compatibility
                self._zmq_server = pns  # Store for registration manager access
                self._pns = pns  # Also store as _pns for clarity
                
                state_manager.set_zmq_state(ServiceState.READY)
                logger.info("🦀 Rust PNS initialized successfully - all ZMQ services are now 100% Rust")

            except Exception as e:
                logger.error(f"Failed to initialize Rust PNS: {e}")
                logger.debug(traceback.format_exc())
                return False

            # --- Resource Manager ---
            # CRITICAL SAFETY: Skip ResourceManager during brain development
            #  ResourceManager uses multiprocessing which causes resource leaks
            #  and heap corruption
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
                        #  Resource manager doesn't have a specific state in
                        #  FeagiStateManager
                        logger.info(
                            "Resource Manager initialized successfully"
                        )
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
                profile_enabled = config.get("system", {}).get(
                    "profile", False
                )

                #  In embedded mode, only enable profiling if explicitly
                #  requested
                if profile_enabled and (not embedded_mode or profile_enabled):
                    from feagi.utils.system_monitor import (
                        start_system_monitoring,
                    )

                    # Configure monitoring based on profile settings
                    monitoring_interval = config.get("profile", {}).get(
                        "resource_monitor_interval", 5.0
                    )
                    enable_gpu = config.get("profile", {}).get(
                        "monitor_gpu", True
                    )
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
                        logger.warning(
                            "Failed to start system resource monitor"
                        )
                elif embedded_mode:
                    logger.info(
                        "[CONFIG] Embedded mode: System resource monitoring disabled (use --profile to enable minimal monitoring)"
                    )

            except Exception as e:
                logger.warning(
                    f"System resource monitor initialization failed: {e}"
                )
                # Non-critical - continue without resource monitoring

            logger.info("Important processes initialization completed")

            # --- Sleep Manager (Memory GC/Consolidation) ---
            try:
                # Strict config gating: require explicit section and keys
                if (
                    "memory_processing" in config
                    and isinstance(config["memory_processing"], dict)
                    and "sleep_manager" in config["memory_processing"]
                    and isinstance(
                        config["memory_processing"]["sleep_manager"], dict
                    )
                ):
                    sm_cfg = config["memory_processing"]["sleep_manager"]
                    enabled = sm_cfg.get("enabled", False)
                    # Required numeric keys must exist to enable
                    required_keys = [
                        "fcl_low_activity_window_bursts",
                        "fcl_low_activity_threshold",
                        "monitor_interval_seconds",
                        "gc_prune_inactive_after_bursts",
                    ]
                    has_required = all(k in sm_cfg for k in required_keys)
                    if enabled and has_required:
                        try:
                            self._sleep_manager = SleepManager(
                                fcl_manager=self._fcl_manager,
                                connectome_manager=self._connectome_manager,
                                memory_processor=(
                                    self._burst_engine.memory_processor
                                    if self._burst_engine
                                    else None
                                ),
                                window_bursts=int(
                                    sm_cfg["fcl_low_activity_window_bursts"]
                                ),
                                activity_threshold=int(
                                    sm_cfg["fcl_low_activity_threshold"]
                                ),
                                monitor_interval=float(
                                    sm_cfg["monitor_interval_seconds"]
                                ),
                                gc_prune_after_bursts=int(
                                    sm_cfg["gc_prune_inactive_after_bursts"]
                                ),
                            )
                            self._sleep_manager.start()
                            self._processes["sleep_manager"] = (
                                self._sleep_manager
                            )
                            logger.info(
                                "Sleep Manager initialized and monitoring FCL activity"
                            )
                        except Exception as sm_err:
                            logger.error(
                                f"Failed to initialize Sleep Manager: {sm_err}"
                            )
                    else:
                        logger.info(
                            "Sleep Manager disabled or missing required config; skipping initialization"
                        )
                else:
                    logger.info(
                        "Sleep Manager config section not found; skipping initialization"
                    )
            except Exception as e:
                logger.error(f"Error during Sleep Manager setup: {e}")
            return True

        except Exception as e:
            logger.error(
                f"Critical error during important processes initialization: {e}"
            )
            logger.debug(traceback.format_exc())
            return False

    def init_background_processes(self, config: Dict[str, Any]) -> bool:
        """Initialize Priority 3 (Background) processes.

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
                        "reload": config.get("development", {}).get(
                            "reload", False
                        ),
                        "access_log": config.get("api", {}).get(
                            "access_log", True
                        ),
                    }

                    def run_uvicorn():
                        """Run uvicorn server in background thread."""
                        import sys
                        try:
                            print("🔵 REST API thread started - creating FastAPI app...", file=sys.stderr, flush=True)
                            logger.info("🔵 REST API thread started - creating FastAPI app...")
                            # Import FastAPI app here to avoid circular imports
                            from feagi.api.rest.app import create_rest_app

                            print("🔵 About to call create_rest_app()...", file=sys.stderr, flush=True)
                            app = create_rest_app()
                            print("🔵 FastAPI app created successfully - starting uvicorn...", file=sys.stderr, flush=True)
                            logger.info("🔵 FastAPI app created successfully - starting uvicorn...")

                            import uvicorn

                            # Determine Uvicorn log level based on debug flags
                            import os
                            
                            # Check if API debugging is enabled
                            debug_api_enabled = os.environ.get("FEAGI_DEBUG_API", "0") == "1"
                            
                            if debug_api_enabled:
                                # API debug mode: show INFO level logs
                                uvicorn_log_level = "info"
                            else:
                                # Use global log level from environment or config
                                global_level = os.environ.get("FEAGI_CLI_LOG_LEVEL", "WARNING")
                                uvicorn_log_level = global_level.lower()
                            
                            uvicorn.run(
                                app,
                                host=api_config["host"],
                                port=api_config["port"],
                                access_log=api_config.get("access_log", True),
                                log_level=uvicorn_log_level,
                                loop="asyncio",
                                timeout_keep_alive=1,
                                limit_concurrency=256,
                            )
                        except Exception as e:
                            logger.error(f"Failed to start uvicorn: {e}")
                            logger.error(
                                f"Full traceback: {traceback.format_exc()}"
                            )
                            #  Also log the exception type and context for
                            #  debugging
                            logger.error(f"Exception type: {type(e).__name__}")
                            logger.error(
                                f"Host: {api_config['host']}, Port: {api_config['port']}"
                            )
                            #  Re-raise to ensure the thread actually exits
                            #  with failure
                            raise

                    # Start uvicorn in background thread
                    api_thread = threading.Thread(
                        target=run_uvicorn, daemon=True, name="REST-API"
                    )
                    api_thread.start()
                    
                    import sys
                    print(f"🔵 Main thread: REST API thread started, continuing...", file=sys.stderr, flush=True)

                    # Store the thread reference for shutdown
                    self._processes["rest_api"] = api_thread
                    logger.info(
                        f"REST API server starting on http://{api_host}:{api_port}"
                    )
                    print(f"🔵 Main thread: Logged REST API message, continuing to WebSocket check...", file=sys.stderr, flush=True)

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
                websocket_enabled = config.get("websocket", {}).get(
                    "enabled", False
                )
                if websocket_enabled and not embedded_mode:
                    from feagi.api.websocket.server import WebSocketServer

                    ws_port = config.get("websocket", {}).get("port", 8080)
                    # Use validated host configuration (no hardcoded fallbacks)
                    ws_host = (
                        host_config.api_host
                    )  # WebSocket uses same host as API

                    ws_server = WebSocketServer(host=ws_host, port=ws_port)

                    if ws_server.start():
                        self._processes["websocket"] = ws_server
                        logger.info(
                            f"WebSocket server started on {ws_host}:{ws_port}"
                        )
                    else:
                        logger.warning(
                            "Failed to start WebSocket server - continuing without it"
                        )
                elif embedded_mode:
                    logger.info(
                        "[CONFIG] Embedded mode: WebSocket server disabled"
                    )

            except Exception as e:
                logger.warning(f"WebSocket server initialization failed: {e}")
                # Non-critical - continue without WebSocket

            # --- Health Check Service (skip in embedded mode) ---
            try:
                health_enabled = (
                    config.get("resources", {}).get(
                        "enable_health_check", True
                    )
                    and not embedded_mode
                )
                if health_enabled:
                    from feagi.core.health_monitor import HealthMonitor

                    health_monitor = HealthMonitor()
                    health_monitor.start_monitoring()
                    self._processes["health_monitor"] = health_monitor
                    logger.info("Health monitor started")
                elif embedded_mode:
                    logger.info(
                        "[CONFIG] Embedded mode: Health monitor disabled"
                    )

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
        """Start API service as async task instead of subprocess.

        RUST/RTOS COMPATIBLE: This pattern translates directly to Rust async
        tasks.
        """
        try:
            import asyncio
            import threading

            #  CRITICAL FIX: Ensure state synchronization between main process
            #  and FastAPI thread
            #  Set environment variable so FastAPI thread uses the same state
            #  file
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
                    import os
                    
                    # Determine Uvicorn log level based on debug flags
                    debug_api_enabled = os.environ.get("FEAGI_DEBUG_API", "0") == "1"
                    
                    if debug_api_enabled:
                        # API debug mode: show INFO level logs
                        uvicorn_log_level = "info"
                    else:
                        # Use global log level from environment or config
                        global_level = os.environ.get("FEAGI_CLI_LOG_LEVEL", "WARNING")
                        uvicorn_log_level = global_level.lower()

                    uvicorn.run(
                        app,
                        host=config["host"],
                        port=config["port"],
                        log_level=uvicorn_log_level,
                        loop="asyncio",
                        timeout_keep_alive=1,
                        limit_concurrency=256,
                    )
                except Exception as e:
                    logger.error(f"API service task failed: {e}")
                finally:
                    loop.close()

            #  Start as daemon thread (in Rust: tokio::spawn with proper task
            #  management)
            api_thread = threading.Thread(target=run_api_service, daemon=True)
            api_thread.start()

            logger.info(
                "[OK] API service task started successfully", status="[OK] "
            )
            return api_thread

        except Exception as e:
            logger.error(f"Failed to start API service task: {e}")
            return None

    def start(self, config: Dict[str, Any]) -> bool:
        """Start all FEAGI processes in priority order.

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
            logger.error(
                "Failed to initialize critical processes, aborting startup"
            )
            self.shutdown()
            return False

        # 2. Important processes (Priority 2)
        if not self.init_important_processes(config):
            logger.error(
                "Failed to initialize important processes, aborting startup"
            )
            self.shutdown()
            return False

        # 3. Background processes (Priority 3)
        if not self.init_background_processes(config):
            logger.error(
                "Failed to initialize background processes, aborting startup"
            )
            self.shutdown()
            return False

        # Start monitoring thread
        self._start_monitoring()

        # Initialize event system for genome load coordination
        # This is safe on all platforms - will fail gracefully on Windows
        try:
            self._setup_genome_load_event_handling()
        except Exception as e:
            logger.debug(f"Event system not available (this is OK on Windows): {e}")

        # Initialize and start the heartbeat coordinator
        try:
            from feagi.api.v1.agent_heartbeat_coordinator import get_heartbeat_coordinator
            heartbeat_coordinator = get_heartbeat_coordinator()
            heartbeat_coordinator.start_monitoring()
            logger.info("💗 Agent Heartbeat Coordinator started successfully")
        except Exception as e:
            logger.error(f"Failed to start heartbeat coordinator: {e}")

        # Transition from startup phase to runtime phase
        #  This allows FQ samplers to be created without critical service
        #  checks during runtime
        self._startup_phase = False
        logger.info(
            "🚀 FEAGI startup phase completed - transitioning to runtime phase"
        )
        logger.info(
            "🔄 FQ samplers can now be created on-demand during agent registration"
        )

        logger.info("FEAGI Process Manager started successfully")
        return True
    
    def _init_registration_manager(self) -> bool:
        """Initialize RegistrationManager separately.
        
        This method extracts RegistrationManager initialization from init_important_processes()
        to allow it to be called independently without starting ZMQ server.
        
        Returns:
            True if RegistrationManager initialized successfully, False otherwise
        """
        try:
            from feagi.core.state_manager import FeagiStateManager
            from feagi.pns.registration_manager import create_registration_manager
            
            logger.info("🏛️ Initializing Registration Manager...")
            
            # Get State Manager instance
            state_manager = FeagiStateManager.instance()

            #  Create Registration Manager with references to State Manager
            #  and Process Manager
            registration_manager = create_registration_manager(
                state_manager=state_manager,
                process_manager=self,
            )

            if registration_manager:
                logger.info(
                    "🏛️ Registration Manager initialized - central agent coordination ready"
                )
                # Note: ZMQ registration listener is started in init_important_processes()
                return True
            else:
                logger.error(
                    "❌ Failed to initialize Registration Manager"
                )
                return False

        except Exception as e:
            logger.error(f"Failed to initialize Registration Manager: {e}")
            return False

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

        self._monitor_thread = threading.Thread(
            target=monitor_processes, daemon=True
        )
        self._monitor_thread.start()

    def _check_processes(self):
        """Check all tasks and processes and restart any that have failed.

        RUST/RTOS COMPATIBLE: Monitors both async tasks and legacy processes.
        In Rust, this would be integrated with the async runtime's task
        monitoring.
        """
        for name, service in self._processes.items():
            try:
                #  Determine service type based on the actual object
                #  type/attributes
                if hasattr(service, "is_running") and callable(
                    service.is_running
                ):
                    # Service with is_running() method (like ZmqServer)
                    if not service.is_running():
                        logger.error(f"Service {name} is not running")
                elif hasattr(service, "is_alive") and callable(
                    service.is_alive
                ):
                    # Thread-like objects
                    if not service.is_alive():
                        logger.error(f"Thread {name} has stopped unexpectedly")
                        if name == "rest_api":
                            logger.error(
                                "REST API thread failure - this usually indicates:"
                            )
                            logger.error(
                                "  1. Unicode/encoding issues in log messages"
                            )
                            logger.error(
                                "  2. Import errors or missing dependencies"
                            )
                            logger.error(
                                "  3. Port conflicts or network issues"
                            )
                            logger.error(
                                "  4. FastAPI/uvicorn startup failures"
                            )
                            logger.error(
                                "Check the detailed traceback above for the root cause"
                            )
                elif hasattr(service, "poll") and callable(service.poll):
                    # Legacy subprocess
                    if service.poll() is not None:
                        exit_code = service.poll()
                        logger.error(
                            f"Process {name} exited with code {exit_code}"
                        )
                else:
                    # Service type we can't monitor - silently skip
                    #  (Expected for HealthMonitor, ZmqServer, ResourceManager
                    #  etc.)
                    pass

            except Exception as e:
                logger.warning(f"Error checking service {name}: {e}")

    def get_core_api(self):
        """Get the Core API instance."""
        return self._core_api

    def get_zmq_server(self):
        """Get the ZMQ server instance."""
        return self._zmq_server

    # ═══════════════════════════════════════════════════════════════════════
    # FQ Sampler Management (for HTTP registration path)
    # ═══════════════════════════════════════════════════════════════════════

    def create_fq_sampler(self, mode: str, frequency_hz: float) -> bool:
        """Attach SHM writer for the given mode.
        
        This is called by RegistrationManager when agents register via HTTP.
        For visualization, it attaches the Rust NPU's SHM writer (no Python FQ sampler needed).
        
        NOTE: If called before Rust NPU is initialized (early startup), returns True anyway.
        The Rust burst loop will automatically write to SHM once it starts.
        
        Args:
            mode: Either 'visualization' or 'motor'
            frequency_hz: Sampling frequency in Hz - sets the FQ sampler rate in Rust NPU
            
        Returns:
            True if SHM writer was attached successfully (or will be attached later)
        """
        try:
            if mode == 'visualization':
                logger.info(f"🎨 [FQ-CREATE] Setting up visualization at {frequency_hz}Hz (Rust burst loop handles FQ sampling + SHM write)")
                
                # Try to attach Rust NPU visualization SHM writer
                # The Rust burst loop samples FQ every burst and writes to SHM
                # Per-agent rate limiting happens in capability rate manager
                if self.rust_npu_integration and self.rust_npu_integration._rust_npu:
                    from feagi.core.state_manager import FeagiStateManager
                    sm = FeagiStateManager.instance()
                    shm_registry = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
                    viz_shm_path = shm_registry.get("visualization_stream", "/tmp/feagi-shared-mem-visualization_stream.bin")
                    
                    self.rust_npu_integration._rust_npu.attach_viz_shm_writer(viz_shm_path)
                    logger.info(f"🎨 [FQ-CREATE] ✅ Rust NPU visualization SHM writer attached: {viz_shm_path}")
                    logger.info(f"🎨 [FQ-CREATE] ℹ️  FQ Sampler runs at burst frequency, per-agent throttling by capability manager")
                    return True
                
                # If Rust NPU not ready yet (early startup), that's OK
                # The burst loop will start writing to SHM automatically once initialized
                logger.info(f"🎨 [FQ-CREATE] ℹ️  Rust NPU not ready yet - SHM writer will attach when burst loop starts")
                return True
            
            elif mode == 'motor':
                logger.info(f"🚗 [FQ-CREATE] Motor output handled by Rust burst engine")
                # Motor is handled by Rust burst engine directly
                return True
            
            else:
                logger.warning(f"⚠️ [FQ-CREATE] Unknown mode: {mode}")
                return False
                
        except Exception as e:
            logger.error(f"❌ [FQ-CREATE] Failed to attach {mode} SHM writer: {e}")
            logger.debug(traceback.format_exc())
            return False

    # ═══════════════════════════════════════════════════════════════════════
    # Rust PNS Agent Lifecycle Callbacks
    # ═══════════════════════════════════════════════════════════════════════

    def _on_agent_registered(self, agent_id: str, agent_type: str, capabilities_json: str):
        """Callback invoked by Rust PNS when an agent registers.
        
        This creates the necessary Python-side resources (FQ samplers, SHM writers).
        
        Args:
            agent_id: Unique agent identifier
            agent_type: Type of agent (visualizer, external, etc.)
            capabilities_json: JSON string of agent capabilities
        """
        import json
        
        try:
            logger.info(f"🦀 [CALLBACK] Agent registered: {agent_id} (type: {agent_type})")
            capabilities = json.loads(capabilities_json)
            
            # Check for visualization capability
            if 'visualization' in capabilities:
                viz_caps = capabilities['visualization']
                if isinstance(viz_caps, dict) and viz_caps.get('enabled', False):
                    rate_hz = viz_caps.get('rate_hz', 30.0)
                    logger.info(f"🦀 [CALLBACK] Creating visualization FQ sampler at {rate_hz}Hz")
                    
                    # Create visualization FQ sampler if not already created
                    if not hasattr(self, '_viz_fq_sampler') or self._viz_fq_sampler is None:
                        try:
                            from feagi.npu.fire_queue.unified_fq_sampler import UnifiedFQSampler
                            
                            self._viz_fq_sampler = UnifiedFQSampler(
                                fire_queue_provider=self._processes.get("brain_service"),
                                mode='visualization',
                                sample_frequency_hz=rate_hz
                            )
                            
                            # Attach to burst engine
                            if self._burst_engine:
                                self._burst_engine.register_fq_sampler(
                                    self._viz_fq_sampler, 
                                    name='rust_fq_sampler_wrapper'
                                )
                                logger.info(f"🦀 [CALLBACK] ✅ Visualization FQ sampler created and attached")
                            else:
                                logger.warning(f"🦀 [CALLBACK] ⚠️  Burst engine not available for FQ sampler registration")
                                
                            # Attach Rust NPU visualization SHM writer AND set frequency
                            if self._core_api and hasattr(self._core_api, '_rust_npu_integration'):
                                rust_npu_integration = self._core_api._rust_npu_integration
                                if rust_npu_integration and rust_npu_integration._rust_npu:
                                    from feagi.core.state_manager import FeagiStateManager
                                    sm = FeagiStateManager.instance()
                                    shm_registry = sm.get_shared_memory_registry() if hasattr(sm, "get_shared_memory_registry") else {}
                                    viz_shm_path = shm_registry.get("visualization_stream", "/tmp/feagi-shared-mem-visualization_stream.bin")
                                    
                                    # CRITICAL FIX: Set FQ sampler frequency to agent's requested rate
                                    rust_npu_integration.set_fq_sampler_frequency(rate_hz)
                                    logger.info(f"🦀 [CALLBACK] ✅ FQ Sampler frequency set to {rate_hz}Hz")
                                    
                                    rust_npu_integration._rust_npu.attach_viz_shm_writer(viz_shm_path)
                                    logger.info(f"🦀 [CALLBACK] ✅ Rust NPU visualization SHM writer attached: {viz_shm_path}")
                        except Exception as e:
                            logger.error(f"🦀 [CALLBACK] Failed to create visualization FQ sampler: {e}")
                    else:
                        logger.info(f"🦀 [CALLBACK] Visualization FQ sampler already exists, reusing")
            
            # Check for motor capability
            if 'motor' in capabilities:
                motor_caps = capabilities['motor']
                if isinstance(motor_caps, dict) and motor_caps.get('enabled', False):
                    rate_hz = motor_caps.get('rate_hz', 20.0)
                    logger.info(f"🦀 [CALLBACK] Agent {agent_id} has motor capability at {rate_hz}Hz")
                    # Motor FQ sampler creation happens on-demand via motor stream
            
            # Check for sensory capability
            if 'sensory' in capabilities:
                sensory_caps = capabilities['sensory']
                if isinstance(sensory_caps, dict):
                    rate_hz = sensory_caps.get('rate_hz', 30.0)
                    logger.info(f"🦀 [CALLBACK] Agent {agent_id} has sensory capability at {rate_hz}Hz")
                    # Sensory is handled by Rust burst engine directly
            
        except Exception as e:
            logger.error(f"🦀 [CALLBACK] Error handling agent registration for {agent_id}: {e}")
            logger.debug(traceback.format_exc())

    def _on_agent_deregistered(self, agent_id: str):
        """Callback invoked by Rust PNS when an agent deregisters.
        
        This cleans up Python-side resources.
        
        Args:
            agent_id: Unique agent identifier
        """
        try:
            logger.info(f"🦀 [CALLBACK] Agent deregistered: {agent_id}")
            
            # TODO: Implement cleanup logic
            # - Check if this was the last visualization agent -> stop viz FQ sampler
            # - Check if this was the last motor agent -> stop motor FQ sampler
            # - For now, we keep FQ samplers running (they're cheap when no data flows)
            
        except Exception as e:
            logger.error(f"🦀 [CALLBACK] Error handling agent deregistration for {agent_id}: {e}")
            logger.debug(traceback.format_exc())

    # ═══════════════════════════════════════════════════════════════════════

    def shutdown(self) -> None:
        """Gracefully shutdown all FEAGI processes and services.

        Uses configurable timeout values from TOML configuration instead of
        hardcoded values.
        """
        # @cursor:critical-path - Signal-safe shutdown should minimize logging
        try:
            print(
                "Shutting down FEAGI services...", file=sys.stderr, flush=True
            )

            # Stop running flag to signal all services to stop
            self._running = False

            # Shutdown heartbeat coordinator first
            try:
                from feagi.api.v1.agent_heartbeat_coordinator import get_heartbeat_coordinator
                heartbeat_coordinator = get_heartbeat_coordinator()
                heartbeat_coordinator._running = False  # Stop the monitoring flag
                print("💔 Heartbeat coordinator shutdown initiated", file=sys.stderr, flush=True)
            except Exception as e:
                print(f"Error stopping heartbeat coordinator: {e}", file=sys.stderr, flush=True)

            # Shutdown ZMQ registration listener
            try:
                if hasattr(self, '_zmq_registration_listener') and self._zmq_registration_listener:
                    self._zmq_registration_listener.stop()
                    print("🦀 ZMQ registration listener stopped", file=sys.stderr, flush=True)
            except Exception as e:
                print(f"Error stopping ZMQ registration listener: {e}", file=sys.stderr, flush=True)

            # Import required modules for timeout handling
            import threading

            #  Load timeout configuration from TOML (use defaults if config
            #  unavailable during shutdown)
            try:
                config = load_feagi_config()
                timeout_config = get_timeout_config(config)
                graceful_shutdown_timeout = timeout_config.graceful_shutdown
                thread_join_timeout = timeout_config.thread_join
                process_join_timeout = timeout_config.process_join
                # fq_sampler_timeout = timeout_config.fq_sampler_shutdown  # Unused for now
            except Exception as e:
                print(
                    f"Warning: Could not load timeout config during shutdown, using fallback values: {e}",
                    file=sys.stderr,
                    flush=True,
                )
                #  Emergency fallback values only used if configuration is
                #  completely unavailable
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

            #  Shutdown each service properly based on its type with
            #  configurable timeouts
            for name, service in self._processes.items():
                try:
                    print(
                        f"Stopping service: {name}...",
                        file=sys.stderr,
                        flush=True,
                    )

                    # Create a shutdown function that can be run with timeout
                    def shutdown_service():
                        try:
                            # Handle different service types
                            if name == "zmq_server" and hasattr(
                                service, "stop"
                            ):
                                # ZMQ server (Rust PNS) has a stop method
                                service.stop()
                            elif name == "rest_api" and hasattr(
                                service, "stop"
                            ):
                                # REST API server has a stop method
                                service.stop()
                            elif name == "websocket" and hasattr(
                                service, "stop"
                            ):
                                # WebSocket server has a stop method
                                service.stop()
                            elif name == "health_monitor" and hasattr(
                                service, "stop"
                            ):
                                # Health monitor has a stop method
                                service.stop()
                            elif name == "system_monitor" and hasattr(
                                service, "stop"
                            ):
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
                print(
                    "Stopping Motor FQSampler...", file=sys.stderr, flush=True
                )
                try:
                    self._motor_fq_sampler.stop()
                    # Wait for thread to finish
                    if (
                        hasattr(self, "_motor_fq_thread")
                        and self._motor_fq_thread
                        and self._motor_fq_thread.is_alive()
                    ):
                        self._motor_fq_thread.join(timeout=thread_join_timeout)
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
                    "Stopping Visualization FQSampler...",
                    file=sys.stderr,
                    flush=True,
                )
                try:
                    self._viz_fq_sampler.stop()
                    # Wait for thread to finish
                    if (
                        hasattr(self, "_viz_fq_thread")
                        and self._viz_fq_thread
                        and self._viz_fq_thread.is_alive()
                    ):
                        self._viz_fq_thread.join(timeout=thread_join_timeout)
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

            # Stop Sleep Manager if running
            try:
                if hasattr(self, "_sleep_manager") and self._sleep_manager:
                    print(
                        "Stopping Sleep Manager...",
                        file=sys.stderr,
                        flush=True,
                    )
                    self._sleep_manager.stop()
            except Exception as e:
                print(
                    f"Error stopping Sleep Manager: {e}",
                    file=sys.stderr,
                    flush=True,
                )

            print("FEAGI services shut down", file=sys.stderr, flush=True)

        except Exception as e:
            # Last resort error handling - print to stderr and continue
            print(
                f"Critical error during shutdown: {e}",
                file=sys.stderr,
                flush=True,
            )

    def signal_handler(self, sig, frame):
        """Handle termination signals."""
        self.shutdown()

    def update_area_sample_rate(self, cortical_id, rate):
        """Update sample rate for specific cortical areas in the appropriate FQ
        sampler.

        Args:
            cortical_id: ID of the cortical area
            rate: New sampling rate in Hz
        """
        # Check if it's an OPU area (motor) or other area (visualization)
        is_motor_area = False

        # Try to determine if this is a motor area using strict helper
        try:
            from feagi.api.core.services.core_api_service import CoreAPIService
            if hasattr(self, "_core_api") and isinstance(getattr(self, "_core_api"), CoreAPIService):
                is_motor_area = bool(self._core_api.is_opu_area(cortical_id))
            elif self._connectome_manager and hasattr(self._connectome_manager, "is_opu"):
                is_motor_area = bool(self._connectome_manager.is_opu(cortical_id))
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
        """Get the visualization FQ sampler instance.
        
        RUST/RTOS COMPATIBLE: Direct reference retrieval with no fallbacks.
        Returns None if no sampler exists (normal during startup before agents connect).
        """
        sampler = getattr(self, "_viz_fq_sampler", None)
        if sampler:
            logger.debug(f"✅ get_viz_fq_sampler: Found sampler {sampler.instance_id} in ProcessManager id={id(self)}")
        else:
            logger.debug(
                f"ℹ️ get_viz_fq_sampler: No visualization sampler in ProcessManager id={id(self)} "
                "(normal if no visualization agents have connected yet)"
            )
        return sampler

    def get_fq_sampler_performance_stats(self):
        """Get performance statistics from both samplers."""
        stats = {}

        if hasattr(self, "_motor_fq_sampler") and self._motor_fq_sampler:
            stats["motor"] = self._motor_fq_sampler.get_performance_stats()

        if hasattr(self, "_viz_fq_sampler") and self._viz_fq_sampler:
            stats["visualization"] = (
                self._viz_fq_sampler.get_performance_stats()
            )

        return stats

    def update_visualization_stream_frequency(
        self, new_frequency: float
    ) -> int:
        """Update visualization stream frequency to sync with FQ sampler
        frequency.

        CRITICAL FIX: Visualization streams use their own timing (sample_rate),
        independent of FQ sampler frequency. This method updates both.

        Args:
            new_frequency: New frequency in Hz

        Returns:
            Number of streams updated
        """
        updated_count = 0

        try:
            # Update via ZMQ server reference if available
            if hasattr(self, "_zmq_server") and self._zmq_server:
                zmq_server = self._zmq_server
                if (
                    hasattr(zmq_server, "_visualization")
                    and zmq_server._visualization
                ):
                    viz_stream = zmq_server._visualization
                    if hasattr(viz_stream, "sample_rate"):
                        old_rate = viz_stream.sample_rate
                        viz_stream.sample_rate = new_frequency
                        logger.info(
                            f"🎬 [FREQ-SYNC] Visualization stream updated: {old_rate}Hz → {new_frequency}Hz"
                        )
                        updated_count += 1
                    else:
                        logger.warning(
                            "🎬 [FREQ-SYNC] Visualization stream found but no sample_rate attribute"
                        )
                else:
                    logger.debug(
                        "🎬 [FREQ-SYNC] No visualization stream found in ZMQ server"
                    )
            else:
                logger.debug(
                    "🎬 [FREQ-SYNC] No ZMQ server reference available"
                )

            #  TODO: Add support for other visualization stream instances if
            #  needed
            # (e.g., standalone streams, additional servers, etc.)

        except Exception as e:
            logger.error(
                f"🎬 [FREQ-SYNC] Error updating visualization stream frequency: {e}"
            )

        return updated_count


    def disable_fq_sampler(self, mode: str):
        """Disable and destroy FQ sampler of the specified mode.

        Args:
            mode: Sampling mode to disable ('visualization', 'opu')
        """
        logger.info(f"🔥 Disabling FQ Sampler: mode={mode}")

        # Load timeout configuration
        try:
            config = load_feagi_config()
            timeout_config = get_timeout_config(config)
            thread_join_timeout = timeout_config.thread_join
        except Exception as e:
            logger.warning(
                f"Could not load timeout config, using fallback: {e}"
            )
            thread_join_timeout = (
                2.0  # @architecture:acceptable - emergency fallback
            )

        try:
            if mode == "visualization":
                if self._viz_fq_sampler is not None:
                    # 🦀 RUST: No unregister needed - Rust FQ sampler is self-contained
                    
                    if hasattr(self._viz_fq_sampler, "stop"):
                        self._viz_fq_sampler.stop()

                    if self._viz_fq_thread and self._viz_fq_thread.is_alive():
                        self._viz_fq_thread.join(timeout=thread_join_timeout)

                    self._viz_fq_sampler = None
                    self._viz_fq_thread = None
                    logger.info(
                        "🎨 Visualization FQ sampler disabled and destroyed"
                    )

            elif mode == "opu" or mode == "motor":
                if self._motor_fq_sampler is not None:
                    # 🦀 RUST: No unregister needed - Rust FQ sampler is self-contained
                    
                    if hasattr(self._motor_fq_sampler, "stop"):
                        self._motor_fq_sampler.stop()

                    if (
                        self._motor_fq_thread
                        and self._motor_fq_thread.is_alive()
                    ):
                        self._motor_fq_thread.join(timeout=thread_join_timeout)

                    self._motor_fq_sampler = None
                    self._motor_fq_thread = None
                    logger.info("🚗 Motor FQ sampler disabled and destroyed")
            else:
                logger.warning(f"Unknown FQ sampler mode to disable: {mode}")

        except Exception as e:
            logger.error(f"Error disabling FQ sampler ({mode}): {e}")

    def _setup_genome_load_event_handling(self) -> bool:
        """Set up event handling for genome load events (lazy initialization).

        When a genome is successfully loaded, this will automatically start the burst engine.
        This implements the design requirement: "upon genome load, burst engine transitions to running"
        
        This is only available when:
        - Shared memory mode is enabled
        - Platform supports named pipes (Unix/Linux/macOS)
        
        Returns:
            True if event system was initialized successfully, False otherwise
        """
        # Check if already initialized or already determined to be unavailable
        if self._event_system_available is not None:
            return self._event_system_available
            
        try:
            # Import event system here to avoid circular imports
            from feagi.api.shared_memory.events import (
                EventNotificationSystem,
                EventType,
            )

            # Try to create event system for this process manager
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
            
            self._event_system_available = True
            return True

        except RuntimeError as e:
            # Platform doesn't support event system (e.g., Windows)
            logger.debug(
                f"Event system not available on this platform: {e}"
            )
            logger.debug(
                "Genome load events will not trigger automatic burst engine start"
            )
            self._event_system_available = False
            return False
            
        except Exception as e:
            logger.warning(
                f"Failed to initialize genome load event handling: {e}"
            )
            logger.warning(
                "Process manager will need to monitor state changes manually"
            )
            self._event_system_available = False
            return False

    def _handle_genome_loaded_event(self, event):
        """Handle genome loaded event by starting the burst engine.

        Args:
            event: The genome loaded event containing filename and cortical area count
        """
        try:
            logger.info(
                f"🧬 GENOME_LOADED event received: {event.data.get('filename', 'unknown')} "
                f"with {event.data.get('cortical_areas', 0)} cortical areas"
            )

            # Check if burst engine is available and not already running
            from feagi.core.state_manager import (
                FeagiStateManager,
                ServiceState,
            )

            state_manager = FeagiStateManager.instance()

            current_state = state_manager.get_burst_engine_state()

            if current_state == ServiceState.READY:
                logger.info(
                    "⚡ Burst engine already running - no action needed"
                )
                return

            # Start burst engine through the SAME exact method as the REST API
            logger.info("⚡ Starting burst engine after genome load...")

            try:
                #  Use the CoreAPIService start method (same as REST API)
                #  instead of brain service directly
                #  This ensures identical behavior between manual start and
                #  automatic genome start
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
                    logger.error(
                        "❌ Failed to start burst engine after genome load"
                    )

            except Exception as start_error:
                logger.error(
                    f"❌ Error starting burst engine after genome load: {start_error}"
                )

        except Exception as e:
            logger.error(f"Error handling genome loaded event: {e}")
            #  Don't re-raise - event handling should not crash the process
            #  manager


# Global instance for the process manager
_process_manager = None


def get_process_manager() -> ProcessManager:
    """Get the global ProcessManager instance."""
    global _process_manager
    if _process_manager is None:
        logger.warning("🏭🏭🏭 [SINGLETON] Creating new ProcessManager instance")
        _process_manager = ProcessManager()
        logger.warning(f"🏭🏭🏭 [SINGLETON] ProcessManager created with id={id(_process_manager)}")
    else:
        logger.warning(f"🏭🏭🏭 [SINGLETON] Reusing existing ProcessManager instance id={id(_process_manager)}")
    return _process_manager


def reset_process_manager() -> None:
    """Reset the global ProcessManager instance.

    USE WITH CAUTION - only for testing or emergency cleanup.
    """
    global _process_manager
    if _process_manager is not None:
        logger.warning("[SINGLETON] Resetting ProcessManager instance")
        try:
            _process_manager.shutdown()
        except Exception:
            pass
        _process_manager = None
    else:
        logger.debug(
            "[SINGLETON] ProcessManager already None - no reset needed"
        )


def start_all_processes(startup_config: dict, config: Dict[str, Any]) -> bool:
    """Start all FEAGI processes based on configuration.

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
            from feagi.utils.resource_profiler import (
                profile_component,
                start_profiling,
            )

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
            logger.info(
                "[STATS] System resource profiling enabled via --profile flag"
            )
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


class SleepManager:
    """Background task that detects periods of low FCL activity and triggers
    memory maintenance tasks such as pattern-map GC and consolidation.

    Configuration is provided via TOML and must include:
      - memory_processing.sleep_manager.enabled = true
      - memory_processing.sleep_manager.fcl_low_activity_window_bursts
      - memory_processing.sleep_manager.fcl_low_activity_threshold
      - memory_processing.sleep_manager.monitor_interval_seconds
      - memory_processing.sleep_manager.gc_prune_inactive_after_bursts
    """

    def __init__(
        self,
        fcl_manager,
        connectome_manager,
        memory_processor,
        window_bursts: int,
        activity_threshold: int,
        monitor_interval: float,
        gc_prune_after_bursts: int,
    ) -> None:
        self._fcl = fcl_manager
        self._cm = connectome_manager
        self._mp = memory_processor
        self._window = int(window_bursts)
        self._threshold = int(activity_threshold)
        self._interval = float(monitor_interval)
        self._gc_prune_after = int(gc_prune_after_bursts)
        self._running = False
        self._thread = None
        # Genome physiology overrides
        self._use_genome_trigger = True
        
        # Global brain locking coordination
        self._state_manager = None
        self._component_name = "SleepManager"

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(
                timeout=2.0
            )  # @architecture:acceptable - shutdown cleanup
        
        # Clean up any locked areas during shutdown
        self._cleanup_locked_areas()

    def is_running(self) -> bool:
        return self._running and self._thread and self._thread.is_alive()

    def _run(self) -> None:
        try:
            while self._running:
                try:
                    if not self._fcl or not hasattr(
                        self._fcl, "current_timestep"
                    ):
                        time.sleep(self._interval)
                        continue
                    current_ts = int(self._fcl.current_timestep)

                    # Prefer genome physiology thresholds if available
                    window_bursts = self._window
                    activity_max = self._threshold
                    try:
                        physiology = None
                        if hasattr(self._cm, "genome") and isinstance(
                            self._cm.genome, dict
                        ):
                            physiology = self._cm.genome.get("physiology", {})
                        if not physiology:
                            #  Attempt to get via state manager genome cache if
                            #  available in future
                            pass
                        if physiology:
                            window_bursts = int(
                                physiology.get(
                                    "sleep_trigger_inactivity_window",
                                    window_bursts,
                                )
                            )
                            activity_max = int(
                                physiology.get(
                                    "sleep_trigger_neural_activity_max",
                                    activity_max,
                                )
                            )
                    except Exception:
                        # Use configured defaults
                        pass

                    # Use cumulative counters from StateManager
                    try:
                        from feagi.core.state_manager import FeagiStateManager

                        sm = FeagiStateManager.instance()
                        counters = sm.get_cumulative_activity()
                        bursts = int(counters.get("bursts", 0))
                        neurons = int(counters.get("neurons", 0))
                        if bursts >= window_bursts and neurons <= activity_max:
                            self._run_memory_maintenance(current_ts)
                            # Reset counters after a maintenance pass
                            sm.reset_cumulative_activity()
                        else:
                            # Optional debug log to understand gating
                            if bursts > 0 and (
                                bursts % max(1, window_bursts // 4) == 0
                            ):
                                logger.debug(
                                    f"[SLEEP] Not triggering: bursts={bursts}/{window_bursts}, neurons={neurons} (max {activity_max})"
                                )
                    except Exception as counter_err:
                        logger.debug(
                            f"Sleep Manager counter read error: {counter_err}"
                        )
                except Exception as loop_err:
                    logger.debug(f"Sleep Manager loop error: {loop_err}")
                time.sleep(self._interval)
        except Exception as e:
            logger.error(f"Sleep Manager terminated with error: {e}")

    def _get_state_manager(self):
        """Get State Manager instance for cortical locking."""
        if self._state_manager is None:
            try:
                from feagi.core.state_manager import FeagiStateManager
                self._state_manager = FeagiStateManager.instance()
            except Exception as e:
                logger.error(f"Failed to get State Manager for cortical locking: {e}")
                self._state_manager = None
        return self._state_manager

    def _get_memory_cortical_areas(self) -> List[int]:
        """Get list of memory cortical area indices that need maintenance.
        
        Returns:
            List of cortical_idx values for memory areas
        """
        memory_areas = []
        try:
            if self._mp and hasattr(self._mp, "memory_area_properties"):
                # Get memory areas from memory processor
                for area_id, props in self._mp.memory_area_properties.items():
                    # Convert area_id (6-letter string) to cortical_idx (integer)
                    # TODO: Need proper mapping from ConnectomeManager
                    # For now, use a simple hash-based approach
                    cortical_idx = hash(area_id) % 10000  # Simple mapping
                    if cortical_idx >= 0:
                        memory_areas.append(cortical_idx)
            
            # Fallback: if no memory processor, assume memory areas exist
            # This is a temporary approach until proper integration
            if not memory_areas and self._cm:
                # Check if ConnectomeManager has memory areas
                if hasattr(self._cm, 'cortical_areas'):
                    for area_id, area_info in self._cm.cortical_areas.items():
                        if area_info.get('type') == 'memory':
                            cortical_idx = hash(area_id) % 10000
                            if cortical_idx >= 0:
                                memory_areas.append(cortical_idx)
        except Exception as e:
            logger.debug(f"Error getting memory cortical areas: {e}")
        
        return memory_areas

    def _lock_memory_areas_for_maintenance(self) -> bool:
        """Lock the entire brain for global memory maintenance operations.
        
        This uses the efficient global brain lock instead of locking individual areas.
        
        Returns:
            True if global brain lock was successful, False otherwise
        """
        state_manager = self._get_state_manager()
        if not state_manager:
            logger.warning("No State Manager available for global brain locking")
            return True  # Proceed without locking (backward compatibility)
        
        # Use global brain lock for efficient maintenance
        success = state_manager.lock_global_brain(
            locked_by=self._component_name,
            operation="global_memory_maintenance"
        )
        
        if success:
            logger.debug(f"🌍 Sleep Manager acquired global brain lock for maintenance")
        else:
            logger.warning(f"Failed to acquire global brain lock for maintenance")
        
        return success

    def _unlock_memory_areas_after_maintenance(self) -> None:
        """Unlock the global brain after maintenance completion."""
        state_manager = self._get_state_manager()
        if not state_manager:
            return
        
        # Unlock global brain
        success = state_manager.unlock_global_brain(locked_by=self._component_name)
        
        if success:
            logger.debug(f"🌍 Sleep Manager released global brain lock after maintenance")
        else:
            logger.warning(f"Failed to release global brain lock")

    def _cleanup_locked_areas(self) -> None:
        """Emergency cleanup of global brain lock (called during shutdown)."""
        state_manager = self._get_state_manager()
        if not state_manager:
            return
        
        # Force unlock global brain
        unlocked = state_manager.force_unlock_global_brain(locked_by=self._component_name)
        
        if unlocked:
            logger.info(f"🚨 Sleep Manager emergency cleanup: released global brain lock")

    def _run_memory_maintenance(self, current_ts: int) -> None:
        """Run memory maintenance operations with cortical area locking coordination.
        
        This method now coordinates with the NPU by locking memory cortical areas
        during maintenance operations, ensuring BDU operations take precedence.
        """
        try:
            if not self._cm or not hasattr(self._cm, "memory_neuron_array"):
                return
            
            # Lock memory areas before maintenance (BDU operations take precedence)
            if not self._lock_memory_areas_for_maintenance():
                logger.warning("Failed to lock memory areas, skipping maintenance cycle")
                return
            
            try:
                mna = self._cm.memory_neuron_array
                logger.debug(f"🔧 Sleep Manager starting memory maintenance (ts={current_ts})")
                
                # Aging: decrement lifespans by elapsed bursts since last run
                try:
                    if not hasattr(self, "_last_aging_burst"):
                        self._last_aging_burst = current_ts
                    delta = int(
                        current_ts - getattr(self, "_last_aging_burst", current_ts)
                    )
                    if delta > 0:
                        died = []
                        # Prefer vectorized aging if available
                        if hasattr(mna, "age_by_bursts") and callable(
                            mna.age_by_bursts
                        ):
                            died = mna.age_by_bursts(delta)
                        else:
                            # Fallback to per-burst aging loop (should be rare)
                            for _ in range(delta):
                                died.extend(
                                    mna.age_memory_neurons(
                                        current_burst=current_ts
                                    )
                                )
                        if died:
                            #  Update global counts via MemoryProcessor helper if
                            #  available
                            try:
                                if self._mp and hasattr(
                                    self._mp, "_update_state_manager_neuron_count"
                                ):
                                    self._mp._update_state_manager_neuron_count(
                                        increment=-len(died)
                                    )
                            except Exception:
                                pass
                            logger.info(
                                f"[MEMORY-DEATH] aged_by={delta} died={len(died)} sample={died[:10]}"
                            )
                    self._last_aging_burst = current_ts
                except Exception as age_err:
                    logger.debug(f"Sleep Manager aging error: {age_err}")
                
                #  Consolidation: re-check long-term conversion under current
                #  thresholds
                try:
                    #  Derive thresholds from registered memory areas if memory
                    #  processor exists
                    if self._mp and hasattr(self._mp, "memory_area_properties"):
                        thresholds = set()
                        for props in self._mp.memory_area_properties.values():
                            try:
                                thresholds.add(
                                    int(props.get("longterm_threshold", 100))
                                )
                            except Exception:
                                pass
                        converted_total = 0
                        converted_sample = []
                        for t in thresholds:
                            converted = mna.check_longterm_conversion(
                                longterm_threshold=t
                            )
                            if converted:
                                converted_total += len(converted)
                                if len(converted_sample) < 10:
                                    converted_sample.extend(
                                        converted[
                                            : max(0, 10 - len(converted_sample))
                                        ]
                                    )
                        if converted_total:
                            logger.info(
                                f"[MEMORY-LTM] converted={converted_total} sample={converted_sample}"
                            )
                except Exception as conv_err:
                    logger.debug(
                        f"Sleep Manager conversion pass error: {conv_err}"
                    )
                
                # GC: prune stale inactive pattern mappings
                try:
                    pruned = mna.collect_garbage(
                        current_burst=current_ts,
                        prune_inactive_after_bursts=self._gc_prune_after,
                    )
                    if (
                        pruned
                        and self._mp
                        and hasattr(self._mp, "_remove_neuron_from_cache")
                    ):
                        # Best effort: ensure caches drop any pruned indices
                        #  Note: indexes removed from mappings may still be present
                        #  in cache if inactive; flush conservatively
                        pass  # Cache uses pattern keys; pruning mappings already reduces memory footprint
                    if pruned:
                        logger.info(
                            f"[SLEEP] Memory GC completed: pruned {pruned} mappings (ts={current_ts})"
                        )
                except Exception as gc_err:
                    logger.debug(f"Sleep Manager GC error: {gc_err}")
                
                logger.debug(f"🔧 Sleep Manager completed memory maintenance (ts={current_ts})")
                
            finally:
                # Always unlock memory areas after maintenance, regardless of success/failure
                self._unlock_memory_areas_after_maintenance()
                
        except Exception as e:
            logger.debug(f"Sleep Manager maintenance error: {e}")
            # Ensure areas are unlocked even if maintenance fails
            self._unlock_memory_areas_after_maintenance()
