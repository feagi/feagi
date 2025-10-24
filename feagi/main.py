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

            # Get the path to pyproject.toml
            pyproject_path = (
                Path(__file__).parent.parent / "pyproject.toml"
            )

            #  Verify dependencies, don't raise an exception but return False
            #  if there's a mismatch
            is_compatible = verify_dependencies(
                pyproject_path, raise_exception=False
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
        "--shared-mem",
        action="store_true",
        help="Enable shared-memory data transport as default (streams prefer SHM)",
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
    parser.add_argument(
        "--debug-shm",
        action="store_true",
        help="Enable detailed shared-memory debugging (attach/readers/payload/injection summaries)",
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
    
    # DIAGNOSTIC: Show all parsed arguments
    print(f"🔍 [ARGS-PRINT] All parsed arguments: {vars(args)}")
    print(f"🔍 [ARGS-PRINT] Specifically debug_mem: {getattr(args, 'debug_mem', 'NOT_FOUND')}")

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

        # IMPORTANT: Do NOT elevate global level for --debug-api.
        # Only elevate globally for other module-specific debug flags as needed.
        if (
            getattr(args, "debug_npu", False)
            or getattr(args, "debug_bdu", False)
            or getattr(args, "debug_zmq_outbound", False)
            or getattr(args, "debug_zmq_inbound", False)
            or getattr(args, "debug_mem", False)
        ):
            os.environ["FEAGI_CLI_LOG_LEVEL"] = "INFO"
            _update_all_logger_levels("INFO")

    # Apply subsystem levels (logger + handlers) using centralized mapping
    try:
        from feagi.utils.logger import apply_subsystem_log_levels
        import logging

        debug_cfg = {
            # legacy aggregate API flag
            "debug_api": bool(getattr(args, "debug_api", False)),
            # granular API flags
            "debug_api_core": bool(getattr(args, "debug_api_core", False)),
            "debug_api_rest": bool(getattr(args, "debug_api_rest", False)),
            "debug_api_zmq": bool(getattr(args, "debug_api_zmq", False)),
            "debug_npu": bool(getattr(args, "debug_npu", False)),
            "debug_bdu": bool(getattr(args, "debug_bdu", False)),
            "debug_zmq_inbound": bool(getattr(args, "debug_zmq_inbound", False)),
            "debug_zmq_outbound": bool(getattr(args, "debug_zmq_outbound", False)),
            "mem_debug": bool(getattr(args, "debug_mem", False)),
            "debug_shm": bool(getattr(args, "debug_shm", False)),
        }

        # Keep env vars for newly created loggers
        if debug_cfg["debug_api"]:
            os.environ["FEAGI_DEBUG_API"] = "1"
        if debug_cfg["debug_api_core"]:
            os.environ["FEAGI_DEBUG_API_CORE"] = "1"
        if debug_cfg["debug_api_rest"]:
            os.environ["FEAGI_DEBUG_API_REST"] = "1"
        if debug_cfg["debug_api_zmq"]:
            os.environ["FEAGI_DEBUG_API_ZMQ"] = "1"
        if debug_cfg["debug_npu"]:
            os.environ["FEAGI_DEBUG_NPU"] = "1"
        if debug_cfg["debug_bdu"]:
            os.environ["FEAGI_DEBUG_BDU"] = "1"
        if debug_cfg["debug_zmq_inbound"] or debug_cfg["debug_zmq_outbound"]:
            os.environ["FEAGI_DEBUG_ZMQ"] = "1"
        if debug_cfg["mem_debug"]:
            os.environ["FEAGI_DEBUG_MEM"] = "1"
        if debug_cfg["debug_shm"]:
            os.environ["FEAGI_DEBUG_SHM"] = "1"

        baseline = os.environ.get("FEAGI_CLI_LOG_LEVEL", "INFO")
        baseline_level = getattr(logging, baseline.upper(), logging.INFO)
        apply_subsystem_log_levels(debug_cfg, baseline_level)
    except Exception:
        pass

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
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                sm._debug_config["debug_npu"] = True
            except Exception:
                pass

        # SHM debug flag: make available via overrides, env, and StateManager immediately
        if getattr(args, "debug_shm", False):
            cli_overrides["debug_shm"] = True
            os.environ["FEAGI_DEBUG_SHM"] = "1"
            logger.info("SHM debugging enabled via --debug-shm flag")
            try:
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                sm._debug_config["debug_shm"] = True
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
                sm = FeagiStateManager.instance()
                if not hasattr(sm, "_debug_config"):
                    sm._debug_config = {}
                sm._debug_config["debug_bdu"] = True
            except Exception:
                pass

        if args.debug_mem:
            cli_overrides["debug_mem"] = True
        if args.debug_shm:
            cli_overrides["debug_shm"] = True
            os.environ["FEAGI_DEBUG_MEM"] = "1"
            print("🔍 [MAIN-PRINT] --debug-mem flag detected! (using print to bypass logging)")
            logger.info("🔍 [MAIN-DEBUG] Memory debugging enabled via --debug-mem flag")
            logger.info(f"🔍 [MAIN-DEBUG] cli_overrides now contains: {cli_overrides}")
        else:
            print("🔍 [MAIN-PRINT] --debug-mem flag NOT detected")

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
        
        # Apply CPU affinity if configured (cross-platform)
        try:
            system_config = config.get("system", {})
            cpu_affinity = system_config.get("cpu_affinity", [])
            if cpu_affinity:  # Non-empty list means user wants to pin cores
                import psutil
                current_process = psutil.Process()
                
                # Try to set CPU affinity (works on Linux, Windows; limited/unavailable on macOS)
                try:
                    current_process.cpu_affinity(cpu_affinity)
                    logger.info(f"CPU affinity set to cores: {cpu_affinity}")
                except (AttributeError, OSError) as e:
                    # macOS doesn't support CPU affinity - use nice/ionice as alternative
                    logger.warning(f"CPU affinity not supported on {platform.system()}: {e}")
                    if platform.system() == "Darwin":  # macOS
                        logger.info("Alternative: Use 'sudo renice' or taskpolicy to control FEAGI's CPU priority")
                        logger.info(f"  Example: sudo renice -n -10 -p {os.getpid()}")
            else:
                logger.info("CPU affinity: Using all available cores (default)")
        except Exception as e:
            logger.warning(f"Failed to configure CPU settings: {e}")

        # Apply process priority if configured (cross-platform)
        try:
            system_config = config.get("system", {})
            priority_config = system_config.get("priority", 0)
            
            if priority_config != 0:  # Non-zero means user wants to change priority
                import psutil
                current_process = psutil.Process()
                
                if platform.system() == "Windows":
                    # Windows uses priority class strings
                    priority_map = {
                        "realtime": psutil.REALTIME_PRIORITY_CLASS,
                        "high": psutil.HIGH_PRIORITY_CLASS,
                        "above_normal": psutil.ABOVE_NORMAL_PRIORITY_CLASS,
                        "normal": psutil.NORMAL_PRIORITY_CLASS,
                        "below_normal": psutil.BELOW_NORMAL_PRIORITY_CLASS,
                        "idle": psutil.IDLE_PRIORITY_CLASS,
                    }
                    
                    if isinstance(priority_config, str):
                        priority_class = priority_map.get(priority_config.lower())
                        if priority_class:
                            try:
                                current_process.nice(priority_class)
                                logger.info(f"Process priority set to: {priority_config.upper()}")
                            except (PermissionError, OSError) as e:
                                logger.warning(f"Failed to set priority to '{priority_config}': {e}")
                                logger.info("Run as Administrator to set high/realtime priority on Windows")
                        else:
                            logger.warning(f"Invalid Windows priority: '{priority_config}'. Use: realtime, high, above_normal, normal, below_normal, idle")
                    else:
                        logger.warning(f"Windows requires string priority (e.g., 'high'), got: {priority_config}")
                
                else:  # Linux/macOS use nice values
                    if isinstance(priority_config, int):
                        if -20 <= priority_config <= 19:
                            try:
                                # Get current nice value first
                                current_nice = os.nice(0)
                                # Calculate increment needed
                                increment = priority_config - current_nice
                                if increment != 0:
                                    os.nice(increment)
                                    logger.info(f"Process priority (nice value) set to: {priority_config}")
                            except (PermissionError, OSError) as e:
                                logger.warning(f"Failed to set nice value to {priority_config}: {e}")
                                if priority_config < 0:
                                    logger.info(f"Run with sudo to set negative nice values (higher priority)")
                                    logger.info(f"  Example: sudo python -m feagi.main")
                        else:
                            logger.warning(f"Nice value must be between -20 and 19, got: {priority_config}")
                    else:
                        logger.warning(f"Linux/macOS requires integer nice value, got: {priority_config}")
            else:
                logger.info("Process priority: Using normal priority (default)")
        except Exception as e:
            logger.warning(f"Failed to configure process priority: {e}")

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
    # Ensure CLI debug flags like --debug-shm are reflected in config.debug
    try:
        dbg_section = config.get("debug", {}) if isinstance(config, dict) else {}
        dbg_section = dict(dbg_section)
        if getattr(args, "debug_shm", False):
            dbg_section["debug_shm"] = True
        config["debug"] = dbg_section
    except Exception:
        pass
    print(f"🔍 [CONFIG-PRINT] Config being passed to set_debug_config: {config.get('debug', 'NO_DEBUG_SECTION')}")
    
    state_manager = FeagiStateManager.instance()
    state_manager.set_debug_config(config)
    # If shared memory mode enabled, create core stream SHM files
    if getattr(args, "shared_mem", False):
        try:
            # Visualization, motor, sensory core streams
            # Registry lives in the State Manager; creation is delegated to its manager
            if hasattr(state_manager, "_shm_manager") and state_manager._shm_manager:
                # Use consistent underscore naming for stream keys and filenames
                viz_path = state_manager._shm_manager.create_stream_file("visualization_stream")
                motor_path = state_manager._shm_manager.create_stream_file("motor_stream")
                sensory_path = state_manager._shm_manager.create_stream_file("sensory_stream")
                # Register each path individually in the StateManager registry
                try:
                    state_manager.register_core_shared_memory_path("visualization_stream", viz_path)
                    state_manager.register_core_shared_memory_path("motor_stream", motor_path)
                    state_manager.register_core_shared_memory_path("sensory_stream", sensory_path)
                    logger.info(
                        f"[SHM] Core streams initialized: viz={viz_path}, motor={motor_path}, sensory={sensory_path}"
                    )
                except Exception as e:
                    logger.warning(f"[SHM] Failed to register core SHM paths: {e}")
        except Exception as e:
            logger.warning(f"[SHM] Failed to initialize core stream SHM files: {e}")
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
            logger.info(f"🔍 [MAIN-DEBUG] Set StateManager mem_debug = True")
            logger.info(f"🔍 [MAIN-DEBUG] StateManager debug config now: {state_manager._debug_config}")
    except Exception:
        pass

    # Initialize the ProcessManager (which will create ConnectomeManager with proper config)
    process_manager = get_process_manager()
    
    # NOTE: Do NOT call init_critical_processes() or _init_registration_manager() here
    # The process_manager.start() method below will call all initialization methods
    # in the correct order: init_critical_processes → init_important_processes → init_background_processes
    # Calling them early causes Registration Manager to be created twice, breaking agent list functionality
    
    # NOTE: Connectome instance will be created during process_manager.start()
    # The REST API will get it via dependency injection after startup completes
    # We don't set it here because process_manager hasn't initialized it yet
    
    embedded_mode = config.get("system", {}).get("embedded", False)

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
        force_exit_thread = threading.Thread(target=force_exit, daemon=True, name="Force-Exit-Timer")
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
    print("🚀 Starting FEAGI processes...", flush=True)
    if not process_manager.start(config):
        logger.error("[ERR] Failed to start FEAGI. See logs for details.")
        return 1
    
    print("✅ FEAGI processes started, checking for CLI genome...", flush=True)

    # Load CLI genome after services are ready
    genome_path = args.genome or args.genome_path
    print(f"🔍 genome_path={genome_path}, args.genome={args.genome}, args.genome_path={args.genome_path}", flush=True)
    if genome_path:
        print(f"🧬 CLI genome specified: {genome_path}", flush=True)
        print(f"⏳ Waiting for services to be fully ready...", flush=True)
        
        # Wait for services to be fully ready
        try:
            from feagi.config.toml_loader import get_timeout_config
            timeout_cfg = get_timeout_config(config)
            startup_wait = float(getattr(timeout_cfg, "api_service_startup", 1.0))
        except Exception:
            startup_wait = 1.0  # @architecture:acceptable - emergency fallback
        
        time.sleep(startup_wait)
        
        try:
            import json
            from pathlib import Path

            genome_file = Path(genome_path)
            if not genome_file.exists():
                logger.error(f"❌ Genome file not found: {genome_path}")
                process_manager.shutdown()
                FeagiStateManager.instance().cleanup()
                return 1

            print(f"📖 Reading genome file...", flush=True)
            with open(genome_file, "r") as f:
                genome_data = json.load(f)
            print(f"✅ Genome JSON parsed", flush=True)

            core_api = process_manager.get_core_api()
            if not core_api:
                logger.error("❌ Core API not available")
                process_manager.shutdown()
                FeagiStateManager.instance().cleanup()
                return 1

            print(f"🔄 Loading genome via core API...", flush=True)
            result = core_api.load_genome(genome_data, filename=genome_file.name)
            print(f"📊 Load result: success={result.get('success')}, areas={result.get('cortical_area_count')}, error={result.get('error')}", flush=True)
            
            if result.get("success"):
                print(f"✅ CLI genome loaded: {result.get('cortical_area_count', 0)} cortical areas", flush=True)
                
                # Debug: Check if blueprint exists in genome service
                try:
                    genome_service = core_api._genome_service
                    current_genome = getattr(genome_service, "_current_genome", None)
                    if current_genome:
                        has_blueprint = "blueprint" in current_genome
                        blueprint_type = type(current_genome.get("blueprint")) if has_blueprint else None
                        print(f"🔍 DEBUG: _current_genome exists, has_blueprint={has_blueprint}, type={blueprint_type}", flush=True)
                    else:
                        print(f"🔍 DEBUG: _current_genome is None!", flush=True)
                except Exception as e:
                    print(f"🔍 DEBUG: Error checking genome service: {e}", flush=True)
                
                # Wait for genome to fully initialize
                print("⏳ Waiting for genome to fully initialize...", flush=True)
                from feagi.core.state_manager import GenomeState
                state_manager = FeagiStateManager.instance()
                max_wait = 5.0
                waited = 0.0
                while waited < max_wait:
                    if state_manager.get_genome_state() == GenomeState.LOADED.value:
                        print(f"✅ Genome state confirmed LOADED after {waited:.1f}s", flush=True)
                        break
                    time.sleep(0.1)
                    waited += 0.1
                else:
                    print(f"⚠️ Genome state not LOADED after {max_wait}s", flush=True)
            else:
                logger.error(f"❌ Genome load failed: {result.get('error')}")
                process_manager.shutdown()
                FeagiStateManager.instance().cleanup()
                return 1

        except json.JSONDecodeError as e:
            logger.error(f"❌ Invalid JSON: {e}")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 1
        except Exception as e:
            logger.error(f"❌ Error loading genome: {e}")
            import traceback
            logger.error(f"Traceback: {traceback.format_exc()}")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 1

    # Log startup summary with all state information
    try:
        from feagi.core.state_manager import get_state_manager

        state_manager = get_state_manager()
        state_manager.log_startup_summary()
    except Exception as e:
        logger.warning(f"Could not log startup summary: {e}")

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
