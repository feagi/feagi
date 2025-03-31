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

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi.main")

def main():
    """Run the complete FEAGI system with API and ZMQ servers."""
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
    zmq_group.add_argument("--zmq-auth", action="store_true", help="Enable ZMQ authentication")
    zmq_group.add_argument("--zmq-encryption", action="store_true", help="Enable ZMQ encryption")
    
    # General arguments
    parser.add_argument("--config", type=str, help="Path to general configuration file")
    parser.add_argument("--api-only", action="store_true", help="Start only the API server")
    parser.add_argument("--zmq-only", action="store_true", help="Start only the ZMQ server")
    
    args = parser.parse_args()
    
    # Dictionary to store processes
    processes = {}
    
    # Handle Ctrl+C and termination signals
    def signal_handler(sig, frame):
        logger.info("\nShutting down FEAGI servers...")
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
            logger.info(f"Starting API server on {args.api_host}:{args.api_port}")
            api_cmd = [
                sys.executable, "-m", "feagi.api.server",
                "--host", args.api_host,
                "--port", str(args.api_port)
            ]
            if args.api_reload:
                api_cmd.append("--reload")
            
            api_process = subprocess.Popen(api_cmd)
            processes["API"] = api_process
            logger.info(f"API server started with PID {api_process.pid}")
        
        # Start ZMQ server if requested or if neither --api-only nor --zmq-only is specified
        if not args.api_only:
            logger.info(f"Starting ZMQ server on {args.zmq_host} (pub: {args.zmq_pub_port}, sub: {args.zmq_sub_port})")
            zmq_cmd = [
                sys.executable, "-m", "feagi.zmq.server",
                "--host", args.zmq_host,
                "--pub-port", str(args.zmq_pub_port),
                "--sub-port", str(args.zmq_sub_port)
            ]
            
            # Add topics if specified
            if args.zmq_topics:
                zmq_cmd.extend(["--topics"] + args.zmq_topics)
            
            # Add auth and encryption flags if enabled
            if args.zmq_auth:
                zmq_cmd.append("--auth")
            if args.zmq_encryption:
                zmq_cmd.append("--encryption")
            
            # Add config if specified
            if args.config:
                zmq_cmd.extend(["--config", args.config])
            
            zmq_process = subprocess.Popen(zmq_cmd)
            processes["ZMQ"] = zmq_process
            logger.info(f"ZMQ server started with PID {zmq_process.pid}")
        
        # Keep the main process running and monitor child processes
        while True:
            time.sleep(1)
            
            # Check if any process has terminated
            for name, process in list(processes.items()):
                if process.poll() is not None:
                    return_code = process.returncode
                    logger.error(f"{name} server exited with code {return_code}")
                    del processes[name]
            
            # Exit if all processes have terminated
            if not processes:
                logger.error("All servers have terminated. Exiting.")
                return 1
    
    except KeyboardInterrupt:
        # This should be caught by the signal handler
        pass
    
    except Exception as e:
        logger.error(f"Error in main process: {e}")
        # Ensure all child processes are terminated
        for name, process in processes.items():
            if process.poll() is None:
                process.terminate()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 