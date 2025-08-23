#!/usr/bin/env python3
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

FEAGI Main Entry Point.

This module provides the single entry point for starting the complete FEAGI system.
It uses the ProcessManager to handle process creation, monitoring, and shutdown
according to the architecture described in feagi_processes.md.
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path

from feagi.core.state_manager import FeagiStateManager
from feagi.logging_config import setup_feagi_logging
from feagi.process_manager import get_process_manager
from feagi.utils.logger import setup_logger

# CRITICAL: Check for embedded mode BEFORE any other imports
# This prevents FastAPI modules from being imported in embedded mode
if "--embedded" in sys.argv:
    os.environ["FEAGI_EMBEDDED_MODE"] = "1"
    print("[CONFIG] Embedded mode detected - FastAPI imports disabled")

setup_feagi_logging()

# # Configure logging
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(message)s'
# )
logger = setup_logger("feagi.main")


def _update_all_logger_levels(log_level_str: str):
    """Update all existing logger levels to the new level.

    This is necessary because loggers created before CLI override
    retain their original level and don't automatically update.

    Args:
        log_level_str: Log level string (DEBUG, INFO, WARNING, ERROR)
    """
    import logging

    # Convert string to logging level
    level = getattr(logging, log_level_str.upper(), logging.INFO)

    # Update root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)

    # Update all existing loggers in the logger registry
    logger_dict = logging.Logger.manager.loggerDict
    for _logger_name, logger_obj in logger_dict.items():
        if isinstance(logger_obj, logging.Logger):
            logger_obj.setLevel(level)
        # PlaceHolder objects don't need updating


def _set_global_level_from_config():
    """Load log level from config file and set it globally.
    
    This function ensures that the global log level from the config file
    is properly set before module imports happen, fixing the caching issue.
    """
    try:
        from feagi.config.toml_loader import load_feagi_config
        
        config = load_feagi_config()
        config_log_level = config.get("system", {}).get("log_level", "INFO")
        
        # Set environment variable so setup_logger() will use this level
        os.environ["FEAGI_CLI_LOG_LEVEL"] = config_log_level
        
        # Update all existing loggers
        _update_all_logger_levels(config_log_level)
        
        logger.info(f"Global log level set from config: {config_log_level}")
        
    except Exception as e:
        # Fallback to INFO if config loading fails
        os.environ["FEAGI_CLI_LOG_LEVEL"] = "INFO"
        _update_all_logger_levels("INFO")
        logger.warning(f"Failed to load config for log level, using INFO: {e}")


def check_dependencies():
    """Check if installed dependencies match required versions.

    This function verifies that all required packages are installed with the correct versions.
    If not, it displays warnings or errors as appropriate.

    Returns:
        bool: True if all dependencies are compatible, False otherwise.
    """
    logger.info("Checking dependency versions...", status="[CHECK]")
    try:
        # Check if FEAGI_SKIP_VERSION_CHECK environment variable is set
        skip_check = os.environ.get(
            "FEAGI_SKIP_VERSION_CHECK", ""
        ).lower() in (
            "1",
            "true",
            "yes",
        )
        if skip_check:
            logger.info(
                "Dependency version check skipped (FEAGI_SKIP_VERSION_CHECK is set)",
                status="[OK]",
            )
            return True

        #  Try to import the version checker - if it doesn't exist, just
        #  continue
        try:
            from feagi.utils.version_checker import verify_dependencies

            # Get the path to requirements.txt
            requirements_path = (
                Path(__file__).parent.parent / "requirements.txt"
            )

            #  Verify dependencies, don't raise an exception but return False
            #  if there's a mismatch
            is_compatible = verify_dependencies(
                requirements_path, raise_exception=False
            )

            if is_compatible:
                logger.info(
                    "All dependencies are compatible with requirements",
                    status="[OK]",
                )
            else:
                logger.warning(
                    "Some dependencies have version mismatches. Set FEAGI_SKIP_VERSION_CHECK=1 to bypass this check.",
                    status="[WARN]",
                )

            return is_compatible

        except ImportError:
            # Version checker not available - this is okay, just continue
            logger.info(
                "Version checker not available, skipping dependency check",
                status="[OK]",
            )
            return True

    except Exception as e:
        logger.warning(f"Dependency check failed: {e}")
        # Don't fail startup due to dependency check issues
        logger.info("Continuing without dependency check", status="[OK]")
        return True


def main():
    """Main entry point for FEAGI.

    Parses command-line arguments, loads TOML configuration with overrides,
    validates port availability, and starts all FEAGI processes.

    Returns:
        int: Exit code (0 for success, non-zero for failure)
    """
    # CRITICAL: Windows asyncio compatibility fix for ZMQ
    # Must be done BEFORE any ZMQ/asyncio operations
    import platform

    if platform.system() == "Windows":
        import asyncio

        try:
            #  Set the event loop policy to WindowsSelectorEventLoopPolicy for
            #  ZMQ compatibility
            asyncio.set_event_loop_policy(
                asyncio.WindowsSelectorEventLoopPolicy()
            )
            print(
                "Windows detected: Set SelectorEventLoopPolicy for ZMQ compatibility"
            )
        except AttributeError:
            # Fallback for older Python versions
            print(
                "[WARN]  Warning: WindowsSelectorEventLoopPolicy not available - ZMQ may have issues"
            )

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="FEAGI - Framework for Evolutionary Artificial General Intelligence"
    )

    # API server arguments (maintained for backwards compatibility)
    parser.add_argument(
        "--api-host",
        type=str,
        help="Host for the API server (overrides config)",
    )
    parser.add_argument(
        "--api-port",
        type=int,
        help="Port for the API server (overrides config)",
    )
    parser.add_argument(
        "--api-reload",
        action="store_true",
        help="Enable auto-reload for the API server",
    )

    # ZMQ server arguments (maintained for backwards compatibility)
    parser.add_argument(
        "--zmq-host",
        type=str,
        help="Host for the ZMQ server (overrides config)",
    )
    parser.add_argument(
        "--zmq-req-port",
        type=int,
        help="Port for REQ/REP ZMQ pattern (overrides config)",
    )
    parser.add_argument(
        "--zmq-pub-port",
        type=int,
        help="Port for PUB/SUB ZMQ pattern (overrides config)",
    )
    parser.add_argument(
        "--zmq-push-port",
        type=int,
        help="Port for PUSH/PULL ZMQ pattern (overrides config)",
    )
    parser.add_argument(
        "--zmq-sensory-port",
        type=int,
        help="Port for sensory ZMQ stream (overrides config)",
    )
    parser.add_argument(
        "--zmq-motor-port",
        type=int,
        help="Port for motor ZMQ stream (overrides config)",
    )
    parser.add_argument(
        "--zmq-rest-port",
        type=int,
        help="Port for REST ZMQ stream (overrides config)",
    )
    parser.add_argument(
        "--zmq-visualization-port",
        type=int,
        help="Port for visualization ZMQ stream (overrides config)",
    )

    # Configuration file argument
    parser.add_argument(
        "--config",
        type=str,
        help="Path to TOML configuration file (default: auto-discover)",
    )

    # Core configuration
    parser.add_argument(
        "--gpu", action="store_true", help="Use GPU acceleration if available"
    )
    parser.add_argument(
        "--cpu-cores",
        type=int,
        default=None,
        help="Number of CPU cores to use (default: all)",
    )
    parser.add_argument(
        "--memory-limit",
        type=int,
        default=None,
        help="Memory limit in MB (default: no limit)",
    )
    parser.add_argument(
        "--genome-path",
        type=str,
        default=None,
        help="Path to genome file to load on startup",
    )
    parser.add_argument(
        "--genome",
        type=str,
        default=None,
        help="Path to genome file to load on startup (alias for --genome-path)",
    )

    # Test mode arguments

    parser.add_argument(
        "--test",
        action="store_true",
        help="Run FEAGI in test mode (defaults to mode 1)",
    )
    parser.add_argument(
        "--test-mode-1",
        action="store_true",
        help="Run FEAGI in test mode 1 (JSON-based predictable activations)",
    )
    parser.add_argument(
        "--test-mode-2",
        action="store_true",
        help="Run FEAGI in test mode 2 (numpy-based scalable random generation)",
    )
    parser.add_argument(
        "--test-duration",
        type=int,
        default=10,
        help="Duration of the test in seconds",
    )
    parser.add_argument(
        "--test-frequency",
        type=int,
        default=10,
        help="Frequency of sensory input generation in Hz",
    )

    # Debug arguments
    parser.add_argument(
        "--debug", action="store_true", help="Enable debug mode"
    )
    parser.add_argument(
        "--log-level",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Set log level",
    )
    parser.add_argument(
        "--debug-api",
        action="store_true",
        help="Enable detailed API request/response logging",
    )
    parser.add_argument(
        "--debug-npu",
        action="store_true",
        help="Enable fire queue debugging - shows neuron firing data every burst",
    )
    parser.add_argument(
        "--debug-zmq-outbound",
        action="store_true",
        help="Log all outbound ZMQ traffic with decoded data",
    )
    parser.add_argument(
        "--debug-zmq-inbound",
        action="store_true",
        help="Log all inbound ZMQ traffic with decoded data",
    )
    parser.add_argument(
        "--debug-bdu",
        action="store_true",
        help="Enable detailed BDU (Brain Development Unit) debugging - shows synapse creation and candidate neighbors",
    )
    parser.add_argument(
        "--debug-mem",
        action="store_true",
        help="Enable detailed memory system debugging - shows memory neuron creation, pattern detection, and long-term conversion",
    )

    # Performance profiling arguments
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Enable system resource monitoring (CPU, memory, GPU usage)",
    )

    # Embedded device mode
    parser.add_argument(
        "--embedded",
        action="store_true",
        help="Enable embedded device mode (disables REST API, uvicorn, visualization, and non-essential monitoring)",
    )

    args = parser.parse_args()

    # CRITICAL: Set global log level IMMEDIATELY after parsing CLI args
    # This must happen before any logging occurs to respect --log-level
    if args.log_level is not None:
        # Set environment variable for future logger creation
        os.environ["FEAGI_CLI_LOG_LEVEL"] = args.log_level
        # Update all existing loggers immediately
        _update_all_logger_levels(args.log_level)
    else:
        # No CLI override - ensure config file level is loaded and set globally
        # This fixes the caching issue by explicitly setting the global level early
        _set_global_level_from_config()

        # If any module-specific debug flag is set, elevate global level to INFO
        # so gated debug logs (typically INFO-level) are emitted without requiring
        # an explicit --log-level override.
        if (
            getattr(args, "debug_npu", False)
            or getattr(args, "debug_api", False)
            or getattr(args, "debug_bdu", False)
            or getattr(args, "debug_zmq_outbound", False)
            or getattr(args, "debug_zmq_inbound", False)
            or getattr(args, "debug_mem", False)
        ):
            os.environ["FEAGI_CLI_LOG_LEVEL"] = "INFO"
            _update_all_logger_levels("INFO")

    # Show deferred logger setup info now that CLI override is applied
    from feagi.utils.logger import show_deferred_setup_info

    show_deferred_setup_info()

    try:
        # Load TOML configuration with command-line overrides
        from feagi.config.toml_loader import (
            FeagiConfigurationError,
            load_feagi_config,
        )
        from feagi.utils.port_checker import PortConflictError

        # Convert argparse Namespace to dict for CLI overrides
        cli_overrides = {}

        # Map command-line arguments to configuration keys
        if args.api_host is not None:
            cli_overrides["api_host"] = args.api_host
        if args.api_port is not None:
            cli_overrides["api_port"] = args.api_port
        if args.api_reload:
            cli_overrides["api_reload"] = args.api_reload

        if args.zmq_host is not None:
            cli_overrides["zmq_host"] = args.zmq_host
        if args.zmq_req_port is not None:
            cli_overrides["zmq_req_port"] = args.zmq_req_port
        if args.zmq_pub_port is not None:
            cli_overrides["zmq_pub_port"] = args.zmq_pub_port
        if args.zmq_push_port is not None:
            cli_overrides["zmq_push_port"] = args.zmq_push_port
        if args.zmq_sensory_port is not None:
            cli_overrides["zmq_sensory_port"] = args.zmq_sensory_port
        if args.zmq_motor_port is not None:
            cli_overrides["zmq_motor_port"] = args.zmq_motor_port
        if args.zmq_rest_port is not None:
            cli_overrides["zmq_rest_port"] = args.zmq_rest_port
        if args.zmq_visualization_port is not None:
            cli_overrides["zmq_visualization_port"] = (
                args.zmq_visualization_port
            )

        if args.debug:
            cli_overrides["debug"] = True
        if args.log_level is not None:
            cli_overrides["log_level"] = args.log_level
        if args.debug_api:
            cli_overrides["debug_api"] = True
            # Set environment variable for middleware detection
            os.environ["FEAGI_DEBUG_API"] = "1"
            logger.info("API debug logging enabled via --debug-api flag")

        if args.debug_npu:
            cli_overrides["debug_npu"] = True
            os.environ["FEAGI_DEBUG_NPU"] = "1"
            logger.info(
                "[DEBUG] NPU fire queue debugging enabled via --debug-npu flag"
            )
            # Ensure StateManager debug flag reflects CLI immediately
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                sm._debug_config["debug_npu"] = True
            except Exception:
                pass

        if args.debug_zmq_outbound or args.debug_zmq_inbound:
            if args.debug_zmq_outbound:
                cli_overrides["debug_zmq_outbound"] = True
                logger.info(
                    "ZMQ outbound traffic debugging enabled via --debug-zmq-outbound flag"
                )
            if args.debug_zmq_inbound:
                cli_overrides["debug_zmq_inbound"] = True
                logger.info(
                    "ZMQ inbound traffic debugging enabled via --debug-zmq-inbound flag"
                )
            # Set common ZMQ debug environment variable for both
            os.environ["FEAGI_DEBUG_ZMQ"] = "1"
            # Reflect in StateManager
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                if args.debug_zmq_outbound:
                    sm._debug_config["debug_zmq_outbound"] = True
                if args.debug_zmq_inbound:
                    sm._debug_config["debug_zmq_inbound"] = True
            except Exception:
                pass

        if args.debug_bdu:
            cli_overrides["debug_bdu"] = True
            os.environ["FEAGI_DEBUG_BDU"] = "1"
            logger.info("BDU debugging enabled via --debug-bdu flag")
            try:
                from feagi.core.state_manager import FeagiStateManager
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                sm._debug_config["debug_bdu"] = True
            except Exception:
                pass

        if args.debug_mem:
            cli_overrides["debug_mem"] = True
            os.environ["FEAGI_DEBUG_MEM"] = "1"
            logger.info("Memory debugging enabled via --debug-mem flag")

        # Module-specific debug levels are now handled automatically by setup_logger()
        # when each module creates its logger - no additional processing needed here

        if args.profile:
            cli_overrides["profile"] = True
            logger.info(
                "[STATS] System resource profiling enabled via --profile flag"
            )

        if args.embedded:
            cli_overrides["embedded"] = True
            logger.info(
                "[CONFIG] Embedded device mode enabled via --embedded flag"
            )

        # Handle genome path (support both --genome and --genome-path)
        genome_path = args.genome or args.genome_path
        if genome_path:
            cli_overrides["genome_path"] = genome_path
            logger.info(f"Genome file specified: {genome_path}")

        # Load configuration with CLI overrides
        logger.info("Loading FEAGI configuration...")
        config = load_feagi_config(cli_args=cli_overrides)

        # Logger levels already updated immediately after CLI parsing

        # Log the final configuration being used
        api_config = config.get("api", {})
        port_config = config.get("ports", {})
        logger.info("Configuration loaded successfully:")
        logger.info(
            f"  API: {api_config.get('host')}:{api_config.get('port')}"
        )
        logger.info(
            f"  ZMQ Ports: REQ/REP={port_config.get('zmq_req_rep_port')}, "
            f"PUB/SUB={port_config.get('zmq_pub_sub_port')}, "
            f"Sensory={port_config.get('zmq_sensory_port')}, "
            f"Motor={port_config.get('zmq_motor_port')}"
        )

    except FeagiConfigurationError as e:
        logger.error("[ERR] CONFIGURATION ERROR [ERR]")
        logger.error(str(e))
        logger.error("\nTo fix this:")
        logger.error(
            "1. Check that feagi_configuration.toml exists and is valid"
        )
        logger.error(
            "2. Verify all port numbers are unique and within range 1024-65535"
        )
        logger.error(
            "3. Ensure no other processes are using the configured ports"
        )
        return 1

    except PortConflictError as e:
        logger.error("[ERR] PORT CONFLICT ERROR [ERR]")
        logger.error(str(e))
        logger.error("\nTo resolve port conflicts:")
        logger.error("1. Stop the process using the conflicting port, OR")
        logger.error("2. Edit feagi_configuration.toml to use different ports")
        logger.error("3. Check available ports with: netstat -tuln")
        return 1

    except Exception as e:
        logger.error(f"[ERR] STARTUP ERROR: {e}")
        import traceback

        logger.debug(f"Full error details: {traceback.format_exc()}")
        return 1

    # Check dependencies
    if not check_dependencies():
        logger.error(
            "Dependency check failed. Please install required dependencies."
        )
        return 1

    # Initialize state manager and set debug configuration

    state_manager = FeagiStateManager.instance()
    state_manager.set_debug_config(config)
    # Re-apply CLI debug flags to state manager to ensure they are not overwritten by config
    try:
        if not hasattr(state_manager, "_debug_config"):
            state_manager._debug_config = {}
        if args.debug_npu:
            state_manager._debug_config["debug_npu"] = True
        if args.debug_api:
            state_manager._debug_config["debug_api"] = True
        if args.debug_bdu:
            state_manager._debug_config["debug_bdu"] = True
        if args.debug_zmq_outbound:
            state_manager._debug_config["debug_zmq_outbound"] = True
        if args.debug_zmq_inbound:
            state_manager._debug_config["debug_zmq_inbound"] = True
        if args.debug_mem:
            state_manager._debug_config["mem_debug"] = True
    except Exception:
        pass

    # Initialize the ProcessManager (which will create ConnectomeManager with proper config)
    process_manager = get_process_manager()
    
    # Initialize critical processes with proper configuration
    if not process_manager.init_critical_processes(config):
        logger.error("Failed to initialize critical processes")
        return 1
    
    # Get the properly configured connectome instance from ProcessManager
    connectome = process_manager._connectome_manager

    #  Set the connectome instance for FastAPI dependency injection (only in
    #  normal mode)
    embedded_mode = config.get("system", {}).get("embedded", False)
    if not embedded_mode:
        from feagi.api.rest.dependencies import set_connectome_instance

        set_connectome_instance(connectome)
    else:
        logger.info(
            "[CONFIG] Embedded mode: Skipping FastAPI dependency injection setup"
        )

    # Set up signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        #  @cursor:critical-path - Signal handlers must be minimal and avoid
        #  logging/locking operations
        # Print directly to stderr instead of using logger to avoid deadlocks
        print("\nShutting down FEAGI servers...", file=sys.stderr, flush=True)

        # Set a flag to prevent recursive shutdown calls
        if hasattr(signal_handler, "_shutdown_in_progress"):
            print(
                "Shutdown already in progress, ignoring signal",
                file=sys.stderr,
                flush=True,
            )
            return
        signal_handler._shutdown_in_progress = True

        import threading
        import time

        def force_exit():
            """Force exit after timeout if graceful shutdown hangs."""
            try:
                from feagi.config.toml_loader import (
                    get_timeout_config,
                    load_feagi_config,
                )

                cfg = load_feagi_config()
                to = get_timeout_config(cfg)
                timeout_seconds = float(
                    getattr(to, "api_service_shutdown", 10.0)
                )
            except Exception:
                timeout_seconds = (
                    15.0  # @architecture:acceptable - emergency fallback
                )

            time.sleep(timeout_seconds)
            print(
                "[WARN]  Force exiting FEAGI after timeout - some services may not have shut down cleanly",
                file=sys.stderr,
                flush=True,
            )
            os._exit(1)  # Force exit without cleanup

        # Start force exit timer in daemon thread
        force_exit_thread = threading.Thread(target=force_exit, daemon=True)
        force_exit_thread.start()

        try:
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
        except Exception as e:
            print(f"Error during shutdown: {e}", file=sys.stderr, flush=True)
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Add legacy configuration mapping for backwards compatibility
    legacy_config = {
        "core": {
            "use_gpu": args.gpu,
            "cpu_cores": args.cpu_cores,
            "memory_limit": args.memory_limit,
            "genome_path": args.genome_path,
        },
        "test": {
            "enabled": args.test or args.test_mode_1 or args.test_mode_2,
            "mode_1": args.test_mode_1,
            "mode_2": args.test_mode_2,
            "duration": args.test_duration,
            "frequency": args.test_frequency,
        },
        "debug": {
            "debug_npu": args.debug_npu,
            "debug_api": args.debug_api,
            "debug_bdu": args.debug_bdu,
            "debug_zmq_inbound": args.debug_zmq_inbound,
            "debug_zmq_outbound": args.debug_zmq_outbound,
            "log_level": args.log_level or "INFO",
            "verbose": args.debug,
        },
    }

    # Merge TOML config with legacy config for backwards compatibility
    config.update(legacy_config)

    # Start all FEAGI processes with the loaded configuration
    if not process_manager.start(config):
        logger.error("[ERR] Failed to start FEAGI. See logs for details.")
        return 1

    # Log startup summary with all state information
    try:
        from feagi.core.state_manager import get_state_manager

        state_manager = get_state_manager()
        state_manager.log_startup_summary()
    except Exception as e:
        logger.warning(f"Could not log startup summary: {e}")

    # CLI GENOME LOADING: Load genome if specified via --genome flag
    #  Use existing genome service infrastructure instead of duplicating
    #  functionality
    genome_path = args.genome or args.genome_path
    if genome_path:
        logger.info(f"🧬 CLI genome specified: {genome_path}")
        try:
            # Use existing genome service through core API
            core_api = process_manager.get_core_api()
            if core_api:
                # Check if file exists
                import json
                from pathlib import Path

                genome_file = Path(genome_path)
                if not genome_file.exists():
                    logger.error(f"❌ Genome file not found: {genome_path}")
                    process_manager.shutdown()
                    FeagiStateManager.instance().cleanup()
                    return 1

                # Load genome data
                try:
                    logger.info(f"🧬 Loading genome from CLI: {genome_path}")
                    with open(genome_file, "r") as f:
                        genome_data = json.load(f)

                    # Use existing genome service (same as REST API upload)
                    result = core_api.load_genome(
                        genome_data, filename=genome_file.name
                    )

                    if result.get("success", False):
                        logger.info(
                            f"✅ CLI genome loaded successfully: {genome_file.name}"
                        )
                        logger.info(
                            f"   🧠 Cortical areas: {result.get('cortical_area_count', 0)}"
                        )
                    else:
                        logger.error(
                            f"❌ Failed to load CLI genome: {result.get('error', 'Unknown error')}"
                        )
                        process_manager.shutdown()
                        FeagiStateManager.instance().cleanup()
                        return 1

                except json.JSONDecodeError as e:
                    logger.error(
                        f"❌ Invalid JSON in genome file {genome_path}: {e}"
                    )
                    process_manager.shutdown()
                    FeagiStateManager.instance().cleanup()
                    return 1
                except Exception as e:
                    logger.error(
                        f"❌ Error reading genome file {genome_path}: {e}"
                    )
                    process_manager.shutdown()
                    FeagiStateManager.instance().cleanup()
                    return 1

            else:
                logger.error(
                    "❌ Core API not available for CLI genome loading"
                )
                process_manager.shutdown()
                FeagiStateManager.instance().cleanup()
                return 1

        except Exception as e:
            logger.error(f"❌ Error during CLI genome loading: {e}")
            import traceback

            logger.debug(
                f"CLI genome loading error details: {traceback.format_exc()}"
            )
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 1

    # If in test mode, run tests AFTER processes are started
    if args.test or args.test_mode_1 or args.test_mode_2:
        logger.info("Starting FEAGI in test mode")

        # Determine test mode
        if args.test_mode_1:
            test_mode = "mode_1"
            logger.info("Using test mode 1: JSON-based predictable testing")
        elif args.test_mode_2:
            test_mode = "mode_2"
            logger.info(
                "Starting FEAGI in test mode 2 (numpy-based scalable random generation)"
            )
        else:
            test_mode = "mode_1"
            logger.info(
                "Starting FEAGI in test mode 1 (JSON-based predictable activations)"
            )

        # Import test module
        from feagi.utils.test_mode import run_test_mode

        # Get the core API from the process manager
        core_api = process_manager.get_core_api()

        # Run tests
        test_result = run_test_mode(
            core_api_service=core_api,
            test_mode=test_mode,
            genome_path=args.genome_path,
            test_duration=args.test_duration,
            frequency_hz=args.test_frequency,
        )

        # Exit with appropriate exit code
        if test_result:
            logger.info("[OK] Tests passed successfully")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 0
        else:
            logger.error("[ERR] Tests failed")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 1

    logger.info("[OK] FEAGI started successfully! All services are running.")
    logger.info("Press Ctrl+C to stop FEAGI")

    # Keep the main thread alive to handle signals
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        logger.error(f"Error in main process: {e}")
        process_manager.shutdown()
        FeagiStateManager.instance().cleanup()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
