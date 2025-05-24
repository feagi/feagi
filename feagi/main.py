#!/usr/bin/env python
"""FEAGI Main Entry Point.

This module provides the single entry point for starting the complete FEAGI system.
It uses the ProcessManager to handle process creation, monitoring, and shutdown
according to the architecture described in feagi_processes.md.
"""
import argparse
import os
import signal
import sys
import time
from feagi.utils.logger import setup_logger
from feagi.core.state_manager import FeagiStateManager
from feagi.logging_config import setup_feagi_logging

setup_feagi_logging()

# # Configure logging
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(message)s'
# )
logger = setup_logger("feagi.main")

def check_dependencies():
    """
    Check if installed dependencies match required versions.
    
    This function verifies that all required packages are installed with the correct versions.
    If not, it displays warnings or errors as appropriate.
    
    Returns:
        bool: True if all dependencies are compatible, False otherwise.
    """
    logger.info("Checking dependency versions...", emoji1="  ")
    
    try:
        # Import here to avoid circular imports
        from feagi.utils.version_checker import verify_dependencies
        
        # Check if FEAGI_SKIP_VERSION_CHECK environment variable is set
        skip_check = os.environ.get("FEAGI_SKIP_VERSION_CHECK", "").lower() in ("1", "true", "yes")
        if skip_check:
            logger.info("Dependency version check skipped (FEAGI_SKIP_VERSION_CHECK is set)", emoji1="✓")
            return True
            
        # Get the path to requirements.txt
        requirements_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "requirements.txt")
        
        # Verify dependencies, don't raise an exception but return False if there's a mismatch
        is_compatible = verify_dependencies(requirements_path, raise_exception=False)
        
        if is_compatible:
            logger.info("All dependencies are compatible with requirements", emoji1="✓ ")
        else:
            logger.warning("Some dependencies have version mismatches. Set FEAGI_SKIP_VERSION_CHECK=1 to bypass this check.", emoji1="⚠ ")
            
        return is_compatible
        
    except Exception as e:
        logger.error(f"Error checking dependencies: {e}", emoji1="❌")
        return True  # Continue execution despite the error

def main():
    """
    Main entry point for FEAGI.
    
    Parses command-line arguments, loads TOML configuration with overrides,
    validates port availability, and starts all FEAGI processes.
    
    Returns:
        int: Exit code (0 for success, non-zero for failure)
    """
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="FEAGI - Framework for Evolutionary Artificial General Intelligence")
    
    # API server arguments (maintained for backwards compatibility)
    parser.add_argument("--api-host", type=str, help="Host for the API server (overrides config)")
    parser.add_argument("--api-port", type=int, help="Port for the API server (overrides config)")
    parser.add_argument("--api-reload", action="store_true", help="Enable auto-reload for the API server")
    
    # ZMQ server arguments (maintained for backwards compatibility)
    parser.add_argument("--zmq-host", type=str, help="Host for the ZMQ server (overrides config)")
    parser.add_argument("--zmq-req-port", type=int, help="Port for REQ/REP ZMQ pattern (overrides config)")
    parser.add_argument("--zmq-pub-port", type=int, help="Port for PUB/SUB ZMQ pattern (overrides config)")
    parser.add_argument("--zmq-push-port", type=int, help="Port for PUSH/PULL ZMQ pattern (overrides config)")
    parser.add_argument("--zmq-sensory-port", type=int, help="Port for sensory ZMQ stream (overrides config)")
    parser.add_argument("--zmq-motor-port", type=int, help="Port for motor ZMQ stream (overrides config)")
    parser.add_argument("--zmq-control-port", type=int, help="Port for control ZMQ stream (overrides config)")
    parser.add_argument("--zmq-rest-port", type=int, help="Port for REST API ZMQ stream (overrides config)")
    parser.add_argument("--zmq-visualization-port", type=int, help="Port for visualization ZMQ stream (overrides config)")
    
    # Configuration file argument
    parser.add_argument("--config", type=str, help="Path to TOML configuration file (default: auto-discover)")
    
    # Core configuration
    parser.add_argument("--gpu", action="store_true", help="Use GPU acceleration if available")
    parser.add_argument("--cpu-cores", type=int, default=None, help="Number of CPU cores to use (default: all)")
    parser.add_argument("--memory-limit", type=int, default=None, help="Memory limit in MB (default: no limit)")
    parser.add_argument("--genome-path", type=str, default=None, help="Path to genome file to load on startup")
    
    # Test mode arguments
    parser.add_argument("--test", action="store_true", help="Run FEAGI in test mode")
    parser.add_argument("--test-duration", type=int, default=10, help="Duration of the test in seconds")
    parser.add_argument("--test-frequency", type=int, default=10, help="Frequency of sensory input generation in Hz")
    parser.add_argument("--test-visualization", action="store_true", help="Test visualization data flow without using ZMQ")
    
    # Debug arguments
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--log-level", type=str, choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], help="Set log level")
    parser.add_argument("--debug-api", action="store_true", help="Enable detailed API request/response logging")
    
    args = parser.parse_args()
    
    try:
        # Load TOML configuration with command-line overrides
        from feagi.config.toml_loader import load_feagi_config, FeagiConfigurationError
        from feagi.utils.port_checker import PortConflictError
        
        # Convert argparse Namespace to dict for CLI overrides
        cli_overrides = {}
        
        # Map command-line arguments to configuration keys
        if args.api_host is not None:
            cli_overrides['api_host'] = args.api_host
        if args.api_port is not None:
            cli_overrides['api_port'] = args.api_port
        if args.api_reload:
            cli_overrides['api_reload'] = args.api_reload
            
        if args.zmq_host is not None:
            cli_overrides['zmq_host'] = args.zmq_host
        if args.zmq_req_port is not None:
            cli_overrides['zmq_req_port'] = args.zmq_req_port
        if args.zmq_pub_port is not None:
            cli_overrides['zmq_pub_port'] = args.zmq_pub_port
        if args.zmq_push_port is not None:
            cli_overrides['zmq_push_port'] = args.zmq_push_port
        if args.zmq_sensory_port is not None:
            cli_overrides['zmq_sensory_port'] = args.zmq_sensory_port
        if args.zmq_motor_port is not None:
            cli_overrides['zmq_motor_port'] = args.zmq_motor_port
        if args.zmq_control_port is not None:
            cli_overrides['zmq_control_port'] = args.zmq_control_port
        if args.zmq_rest_port is not None:
            cli_overrides['zmq_rest_port'] = args.zmq_rest_port
        if args.zmq_visualization_port is not None:
            cli_overrides['zmq_visualization_port'] = args.zmq_visualization_port
            
        if args.debug:
            cli_overrides['debug'] = True
        if args.log_level is not None:
            cli_overrides['log_level'] = args.log_level
        
        # Load configuration with CLI overrides
        logger.info("Loading FEAGI configuration...")
        config = load_feagi_config(cli_args=cli_overrides)
        
        # Log the final configuration being used
        api_config = config.get('api', {})
        port_config = config.get('ports', {})
        logger.info(f"Configuration loaded successfully:")
        logger.info(f"  API: {api_config.get('host')}:{api_config.get('port')}")
        logger.info(f"  ZMQ Ports: REQ/REP={port_config.get('zmq_req_rep_port')}, "
                   f"PUB/SUB={port_config.get('zmq_pub_sub_port')}, "
                   f"Sensory={port_config.get('zmq_sensory_port')}, "
                   f"Motor={port_config.get('zmq_motor_port')}")
        
    except FeagiConfigurationError as e:
        logger.error("❌ CONFIGURATION ERROR ❌")
        logger.error(str(e))
        logger.error("\nTo fix this:")
        logger.error("1. Check that feagi_configuration.toml exists and is valid")
        logger.error("2. Verify all port numbers are unique and within range 1024-65535")
        logger.error("3. Ensure no other processes are using the configured ports")
        return 1
        
    except PortConflictError as e:
        logger.error("❌ PORT CONFLICT ERROR ❌")
        logger.error(str(e))
        logger.error("\nTo resolve port conflicts:")
        logger.error("1. Stop the process using the conflicting port, OR")
        logger.error("2. Edit feagi_configuration.toml to use different ports")
        logger.error("3. Check available ports with: netstat -tuln")
        return 1
        
    except Exception as e:
        logger.error(f"❌ STARTUP ERROR: {e}")
        logger.debug(f"Full error details: {traceback.format_exc()}")
        return 1
    
    # Check dependencies
    if not check_dependencies():
        logger.error("Dependency check failed. Please install required dependencies.")
        return 1
    
    # Initialize the main connectome instance using singleton pattern
    from feagi.bdu.connectome_manager import ConnectomeManager
    connectome = ConnectomeManager.instance()
    
    # Set the connectome instance for FastAPI dependency injection
    from feagi.api.rest.dependencies import set_connectome_instance
    set_connectome_instance(connectome)
    
    # Initialize the ProcessManager with the connectome  
    from feagi.process_manager import get_process_manager
    process_manager = get_process_manager()
    
    # Set up signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        # @cursor:critical-path - Signal handlers must be minimal and avoid logging/locking operations
        # Print directly to stderr instead of using logger to avoid deadlocks
        print("\nShutting down FEAGI servers...", file=sys.stderr, flush=True)
        
        # Set a flag to prevent recursive shutdown calls
        if hasattr(signal_handler, '_shutdown_in_progress'):
            print("Shutdown already in progress, ignoring signal", file=sys.stderr, flush=True)
            return
        signal_handler._shutdown_in_progress = True
        
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
            "genome_path": args.genome_path
        },
        "test": {
            "enabled": args.test,
            "duration": args.test_duration,
            "frequency": args.test_frequency,
            "visualization": args.test_visualization
        }
    }
    
    # Merge TOML config with legacy config for backwards compatibility
    config.update(legacy_config)
    
    # Start all FEAGI processes with the loaded configuration
    if not process_manager.start(config):
        logger.error("❌ Failed to start FEAGI. See logs for details.")
        return 1
    
    # If in test mode, run tests AFTER processes are started
    if args.test:
        logger.info("Starting FEAGI in test mode")
        
        # Import test module
        from feagi.test_mode import run_test_mode
        
        # Get the core API from the process manager
        core_api = process_manager.get_core_api()
        
        # Run tests
        test_result = run_test_mode(
            core_api_service=core_api,
            genome_path=args.genome_path,
            test_duration=args.test_duration,
            frequency_hz=args.test_frequency,
            test_visualization=args.test_visualization
        )
        
        # Exit with appropriate exit code
        if test_result:
            logger.info("✓ Tests passed successfully")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 0
        else:
            logger.error("❌ Tests failed")
            process_manager.shutdown()
            FeagiStateManager.instance().cleanup()
            return 1
    
    logger.info("✓ FEAGI started successfully! All services are running.")
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