#!/usr/bin/env python
"""FEAGI Main Runner.

This module provides a convenient single entry point for starting both the API and ZMQ
servers of the FEAGI system, while still allowing configuration of each component.
"""
import argparse
import subprocess
import sys
import time
import os
import signal
import logging
import importlib
from typing import Dict, Any, Optional, List

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi.main")

# Global reference to ZMQ server instance
_zmq_server_instance = None

def check_dependencies():
    """
    Check if installed dependencies match required versions.
    
    This function verifies that all required packages are installed with the correct versions.
    If not, it displays warnings or errors as appropriate.
    
    Returns:
        bool: True if all dependencies are compatible, False otherwise.
    """
    logger.info("Checking dependency versions...")
    
    try:
        # Import here to avoid circular imports
        from feagi.utils.version_checker import verify_dependencies
        
        # Check if FEAGI_SKIP_VERSION_CHECK environment variable is set
        skip_check = os.environ.get("FEAGI_SKIP_VERSION_CHECK", "").lower() in ("1", "true", "yes")
        if skip_check:
            logger.info("Dependency version check skipped (FEAGI_SKIP_VERSION_CHECK is set)")
            return True
            
        # Get the path to requirements.txt
        requirements_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "requirements.txt")
        
        # Verify dependencies, don't raise an exception but return False if there's a mismatch
        is_compatible = verify_dependencies(requirements_path, raise_exception=False)
        
        if is_compatible:
            logger.info("✓ All dependencies are compatible with requirements")
        else:
            logger.warning("⚠ Some dependencies have version mismatches. Set FEAGI_SKIP_VERSION_CHECK=1 to bypass this check.")
            
        return is_compatible
        
    except Exception as e:
        logger.error(f"Error checking dependencies: {e}")
        return True  # Continue execution despite the error


def start_api_server(args: Dict[str, Any]) -> Optional[subprocess.Popen]:
    """Start the API server as a separate process.
    
    Args:
        args: Configuration arguments for the API server
        
    Returns:
        The process object if successful, None otherwise
    """
    logger.info(f"Starting API server on {args['host']}:{args['port']}")
    
    cmd = [
        sys.executable, "-m", "feagi.api.server",
        "--host", args["host"],
        "--port", str(args["port"])
    ]
    
    if args.get("reload"):
        cmd.append("--reload")
    
    try:
        process = subprocess.Popen(cmd)
        logger.info(f"API server started with PID {process.pid}")
        return process
    except Exception as e:
        logger.error(f"Failed to start API server: {e}")
        return None


def start_zmq_server(args: Dict[str, Any]) -> bool:
    """Start the ZMQ server in the current process.
    
    This avoids subprocess issues by running the ZMQ server in-process.
    
    Args:
        args: Configuration arguments for the ZMQ server
        
    Returns:
        True if started successfully, False otherwise
    """
    global _zmq_server_instance
    
    logger.info(f"Starting ZMQ server on {args['host']} (pub: {args['pub_port']}, sub: {args['sub_port']})")
    
    # Set relevant environment variables
    os.environ["FEAGI_ZMQ_HOST"] = args["host"]
    os.environ["FEAGI_ZMQ_PUB_PORT"] = str(args["pub_port"])
    os.environ["FEAGI_ZMQ_SUB_PORT"] = str(args["sub_port"])
    os.environ["FEAGI_ZMQ_TOPICS"] = ",".join(args["topics"] if args["topics"] else ["neural", "metrics", "heartbeat"])
    
    try:
        # Import ZMQServer with better diagnostics
        try:
            # Try to import the module without creating a ZMQServer yet
            module = importlib.import_module("feagi.core.zmq.server")
            logger.info(f"Successfully imported ZMQ server module from: {getattr(module, '__file__', 'unknown')}")
            
            # Check if the module has ZMQ_AVAILABLE flag and log it
            if hasattr(module, 'ZMQ_AVAILABLE'):
                logger.info(f"ZMQ_AVAILABLE in module: {module.ZMQ_AVAILABLE}")
            
            # Check if ZMQ has Context in the module's scope
            if hasattr(module, 'zmq') and hasattr(module.zmq, 'Context'):
                logger.info(f"ZMQ Context found in module: {module.zmq.Context}")
            
            # Get ZMQServer class
            ZMQServer = getattr(module, "ZMQServer")
            
            # Create server instance
            logger.info("Creating ZMQServer instance...")
            _zmq_server_instance = ZMQServer(
                host=args["host"],
                pub_port=args["pub_port"],
                sub_port=args["sub_port"],
                topics=args["topics"],
                logger=logger
            )
            
        except (ImportError, AttributeError) as e:
            logger.error(f"Failed to import ZMQServer from feagi.core.zmq.server: {e}")
            logger.info("Trying fallback import from feagi.zmq...")
            
            # Fallback to legacy import path
            try:
                from feagi.zmq import create_zmq_server
                _zmq_server_instance = create_zmq_server(
                    host=args["host"],
                    pub_port=args["pub_port"],
                    sub_port=args["sub_port"],
                    topics=args["topics"]
                )
                
                if _zmq_server_instance is None:
                    logger.error("Failed to create ZMQ server using legacy API")
                    return False
            except Exception as e:
                logger.error(f"Fallback import also failed: {e}")
                return False
        
        # Start the server and return the result
        if _zmq_server_instance is None:
            logger.error("Failed to create ZMQ server instance")
            return False
            
        logger.info("Starting ZMQ server...")
        success = _zmq_server_instance.start()
        if not success:
            logger.error("Failed to start ZMQ server")
            _zmq_server_instance = None
            
        return success
            
    except Exception as e:
        logger.error(f"Error starting ZMQ server: {e}")
        _zmq_server_instance = None
        return False


def get_zmq_client():
    """
    Get the ZMQ client instance.
    
    This provides access to the ZMQ server's functionality from other parts of FEAGI.
    
    Returns:
        The ZMQ server instance or None if not running
    """
    global _zmq_server_instance
    return _zmq_server_instance


def main():
    """Run the complete FEAGI system with API and ZMQ servers."""
    global _zmq_server_instance
    
    parser = argparse.ArgumentParser(description="FEAGI Main Runner")
    
    # API server arguments
    api_group = parser.add_argument_group("API Server Arguments")
    api_group.add_argument("--api-host", default="127.0.0.1", help="API server host")
    api_group.add_argument("--api-port", type=int, default=8000, help="API server port")
    api_group.add_argument("--api-reload", action="store_true", help="Enable API server auto-reload")
    
    # ZMQ server arguments
    zmq_group = parser.add_argument_group("ZMQ Server Arguments")
    zmq_group.add_argument("--zmq-host", default="127.0.0.1", help="ZMQ server host")
    zmq_group.add_argument("--zmq-pub-port", type=int, default=5556, help="ZMQ publisher port")
    zmq_group.add_argument("--zmq-sub-port", type=int, default=5557, help="ZMQ subscriber port")
    zmq_group.add_argument("--zmq-topics", type=str, nargs="+", default=["neural", "metrics", "heartbeat"], 
                          help="ZMQ topics to support")
    
    # General arguments
    parser.add_argument("--config", type=str, help="Path to configuration file")
    parser.add_argument("--api-only", action="store_true", help="Start only the API server")
    parser.add_argument("--zmq-only", action="store_true", help="Start only the ZMQ server")
    parser.add_argument("--skip-version-check", action="store_true", help="Skip dependency version check")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    # Set debug logging if requested
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    # Set environment variables for containerization support
    os.environ["FEAGI_ZMQ_HOST"] = args.zmq_host
    os.environ["FEAGI_ZMQ_PUB_PORT"] = str(args.zmq_pub_port)
    os.environ["FEAGI_ZMQ_SUB_PORT"] = str(args.zmq_sub_port)
    os.environ["FEAGI_ZMQ_TOPICS"] = ",".join(args.zmq_topics)
    
    # Check dependencies unless explicitly skipped
    if args.skip_version_check:
        logger.info("Dependency version check skipped (--skip-version-check flag)")
    else:
        if not check_dependencies():
            # Failed dependency check, but continue with a warning
            logger.warning("⚠ Continuing despite dependency version mismatches")
    
    # Dictionary to store processes
    processes = {}
    
    # Handle Ctrl+C and termination signals
    def signal_handler(sig, frame):
        logger.info("\nShutting down FEAGI servers...")
        
        # Shutdown ZMQ server if running
        if _zmq_server_instance is not None:
            logger.info("Terminating ZMQ server...")
            _zmq_server_instance.shutdown()
            _zmq_server_instance = None
        
        # Terminate child processes
        for name, process in processes.items():
            if process.poll() is None:  # If process is still running
                logger.info(f"Terminating {name} server...")
                process.terminate()
                
        logger.info("FEAGI servers shut down")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Start API server if requested or if neither --api-only nor --zmq-only is specified
        if not args.zmq_only:
            api_args = {
                "host": args.api_host,
                "port": args.api_port,
                "reload": args.api_reload
            }
            api_process = start_api_server(api_args)
            if api_process:
                processes["API"] = api_process
            else:
                logger.error("Failed to start API server")
                return 1
        
        # Start ZMQ server if requested or if neither --api-only nor --zmq-only is specified
        if not args.api_only:
            zmq_args = {
                "host": args.zmq_host,
                "pub_port": args.zmq_pub_port,
                "sub_port": args.zmq_sub_port,
                "topics": args.zmq_topics,
            }
            
            # Start ZMQ server inline
            if not start_zmq_server(zmq_args):
                logger.error("Failed to start ZMQ server")
                return 1
        
        # Keep the main process running and monitor child processes
        while True:
            time.sleep(1)
            
            # Check if any process has terminated
            for name, process in list(processes.items()):
                if process.poll() is not None:
                    return_code = process.returncode
                    logger.error(f"{name} server exited with code {return_code}")
                    del processes[name]
                    
                    # If API server exits, also stop the ZMQ server
                    if name == "API" and _zmq_server_instance is not None:
                        logger.info("API server exited, shutting down ZMQ server")
                        _zmq_server_instance.shutdown()
                        _zmq_server_instance = None
            
            # Exit if all processes have terminated and ZMQ server is not running
            if not processes and _zmq_server_instance is None:
                logger.error("All servers have terminated. Exiting.")
                return 1
    
    except KeyboardInterrupt:
        # This should be caught by the signal handler
        pass
    
    except Exception as e:
        logger.error(f"Error in main process: {e}")
        # Ensure all resources are released
        if _zmq_server_instance is not None:
            _zmq_server_instance.shutdown()
            _zmq_server_instance = None
            
        # Terminate child processes
        for name, process in processes.items():
            if process.poll() is None:
                process.terminate()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 