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
import socket

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


def find_available_port(start_port, max_tries=10):
    """
    Find an available port starting from start_port.
    
    Args:
        start_port: The port to start checking from
        max_tries: Maximum number of ports to try
        
    Returns:
        An available port or None if no port was found
    """
    for port_offset in range(max_tries):
        port = start_port + port_offset
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", port))
                return port
        except OSError:
            logger.warning(f"Port {port} is already in use, trying next port...")
    
    logger.error(f"Could not find an available port after {max_tries} attempts")
    return None


def start_api_server(args: Dict[str, Any]) -> Optional[subprocess.Popen]:
    """
    Start the API server in a separate process.
    
    Args:
        args: Dictionary containing API server arguments
        
    Returns:
        A subprocess.Popen instance for the API server process or None if startup failed
    """
    host = args.get("host", "127.0.0.1")
    port = args.get("port", 8000)
    
    # Check if the specified port is available, otherwise find a free port
    available_port = find_available_port(port)
    if available_port is None:
        logger.error("Could not find an available port for the API server")
        return None
    
    if available_port != port:
        logger.warning(f"Port {port} is already in use, using port {available_port} instead")
        port = available_port
    
    reload = args.get("reload", False)
    
    # Build the command to start the API server
    cmd = [
        sys.executable, "-m", "uvicorn", 
        "feagi.api.rest.main:app", 
        "--host", host,
        "--port", str(port)
    ]
    
    if reload:
        cmd.append("--reload")
    
    # Log the API server startup
    logger.info(f"Starting FEAGI API server on {host}:{port}")
    print(f"Starting FEAGI API server on {host}:{port}")
    
    # Start the process
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        return process
    except Exception as e:
        logger.error(f"Failed to start API server: {e}")
        return None


def start_zmq_server(args: Dict[str, Any]) -> bool:
    """
    Start the ZMQ server.
    
    Args:
        args: Configuration arguments for the ZMQ server
        
    Returns:
        True if successful, False otherwise
    """
    global _zmq_server_instance
    
    try:
        # Check and adjust ports if needed
        req_rep_port = args.get("req_port", 5555)
        pub_sub_port = args.get("pub_port", 5556)
        push_pull_port = args.get("push_port", 5557)
        sensorimotor_port = args.get("sensorimotor_port", 5558)
        vis_base_port = args.get("vis_base_port", 5560)
        
        # Check if the ports are available, otherwise find free ports
        available_req_rep_port = find_available_port(req_rep_port)
        if available_req_rep_port != req_rep_port:
            logger.warning(f"Port {req_rep_port} is already in use, using port {available_req_rep_port} instead")
            req_rep_port = available_req_rep_port
            
        available_pub_sub_port = find_available_port(pub_sub_port)
        if available_pub_sub_port != pub_sub_port:
            logger.warning(f"Port {pub_sub_port} is already in use, using port {available_pub_sub_port} instead")
            pub_sub_port = available_pub_sub_port
            
        available_push_pull_port = find_available_port(push_pull_port)
        if available_push_pull_port != push_pull_port:
            logger.warning(f"Port {push_pull_port} is already in use, using port {available_push_pull_port} instead")
            push_pull_port = available_push_pull_port
            
        available_sensorimotor_port = find_available_port(sensorimotor_port)
        if available_sensorimotor_port != sensorimotor_port:
            logger.warning(f"Port {sensorimotor_port} is already in use, using port {available_sensorimotor_port} instead")
            sensorimotor_port = available_sensorimotor_port
            
        available_vis_base_port = find_available_port(vis_base_port)
        if available_vis_base_port != vis_base_port:
            logger.warning(f"Port {vis_base_port} is already in use, using port {available_vis_base_port} instead")
            vis_base_port = available_vis_base_port
        
        # Update the args dictionary with the available ports
        args["req_port"] = req_rep_port
        args["pub_port"] = pub_sub_port
        args["push_port"] = push_pull_port
        args["sensorimotor_port"] = available_sensorimotor_port
        args["vis_base_port"] = available_vis_base_port
        
        logger.info(f"Starting ZMQ server with ports: req={req_rep_port}, pub={pub_sub_port}, push={push_pull_port}, "
                   f"sensorimotor={sensorimotor_port}, vis_base={vis_base_port}")
        
        # Try to import the ZmqServer class directly
        try:
            from feagi.api.zmq.server import ZmqServer
            
            # Create the ZMQ server instance
            _zmq_server_instance = ZmqServer(
                core_api=None,  # This will be replaced with the actual CoreApiService
                host=args["host"],
                req_rep_port=args["req_port"],
                pub_sub_port=args["pub_port"],
                push_pull_port=args["push_port"],
                sensorimotor_port=args["sensorimotor_port"],
                vis_base_port=args["vis_base_port"]
            )
            
        except (ImportError, AttributeError) as e:
            # If direct import fails, try using the factory function
            logger.error(f"Failed to import ZmqServer from feagi.api.zmq.server: {e}")
            
            from feagi.api.zmq import create_zmq_server
            _zmq_server_instance = create_zmq_server(
                host=args["host"],
                pub_port=args["pub_port"],
                sub_port=args["sub_port"],
                topics=args["topics"]
            )
            
            if _zmq_server_instance is None:
                logger.error("Failed to create ZMQ server")
                return False
        
        # Start the server and return the result
        if _zmq_server_instance is None:
            logger.error("Failed to create ZMQ server instance")
            return False
            
        logger.info("Starting ZMQ server...")
        try:
            success = _zmq_server_instance.start()
            if not success:
                logger.error("Failed to start ZMQ server")
                _zmq_server_instance = None
                return False
            
            logger.info("ZMQ server started successfully")
            return True
            
        except RuntimeWarning as w:
            # Handle the "coroutine was never awaited" warning
            logger.warning(f"Warning during ZMQ server start (this is expected in async mode): {w}")
            # Continue since the server was started in a background thread
            logger.info("ZMQ server starting in background thread")
            return True
            
        except Exception as e:
            logger.error(f"Error starting ZMQ server: {e}")
            _zmq_server_instance = None
            return False
            
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
        global _zmq_server_instance
        logger.info("\nShutting down FEAGI servers...")
        
        try:
            # Shutdown ZMQ server if running
            if '_zmq_server_instance' in globals() and _zmq_server_instance is not None:
                logger.info("Terminating ZMQ server...")
                _zmq_server_instance.shutdown()
                _zmq_server_instance = None
        except Exception as e:
            logger.error(f"Error shutting down ZMQ server: {e}")
        
        try:
            # Terminate child processes
            for name, process in processes.items():
                if process.poll() is None:  # If process is still running
                    logger.info(f"Terminating {name} server...")
                    process.terminate()
        except Exception as e:
            logger.error(f"Error terminating processes: {e}")
                
        logger.info("FEAGI servers shut down")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize to avoid reference errors
    _zmq_server_instance = None
    
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
        try:
            if '_zmq_server_instance' in globals() and _zmq_server_instance is not None:
                _zmq_server_instance.shutdown()
                _zmq_server_instance = None
        except Exception as shutdown_error:
            logger.error(f"Error during ZMQ server shutdown: {shutdown_error}")
            
        # Terminate child processes
        for name, process in processes.items():
            if process.poll() is None:
                process.terminate()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 