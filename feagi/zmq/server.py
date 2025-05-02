#!/usr/bin/env python
"""ZMQ Server for FEAGI.

This is a compatibility module that forwards to the new implementation.
"""
import argparse
import logging
import importlib
import signal
import sys
import os
from typing import Dict, Any, Optional, List

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Global ZMQ server instance
zmq_server_instance = None

def main():
    """Run the FEAGI ZMQ server."""
    parser = argparse.ArgumentParser(description="FEAGI ZMQ Server")
    
    # Add arguments
    parser.add_argument("--host", default="*", help="Host address to bind to")
    parser.add_argument("--pub-port", dest="pub_port", type=int, default=5556, help="Publisher port")
    parser.add_argument("--sub-port", dest="sub_port", type=int, default=5557, help="Subscriber port")
    parser.add_argument("--topics", type=str, nargs="+", default=["neural", "metrics", "heartbeat"], 
                       help="Topics to support")
    parser.add_argument("--encryption", action="store_true", help="Enable encryption (deprecated)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    # Set debug logging if requested
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
        
    if args.encryption:
        logger.warning("The '--encryption' parameter is deprecated and will be ignored.")
    
    # Create and start the ZMQ server
    print(f"Starting FEAGI ZMQ server on {args.host} (pub: {args.pub_port}, sub: {args.sub_port})")
    
    global zmq_server_instance
    
    # Import and create the server using the new implementation
    try:
        from feagi.api.zmq import create_zmq_server
        zmq_server_instance = create_zmq_server(
            host=args.host,
            pub_port=args.pub_port,
            sub_port=args.sub_port,
            topics=args.topics
        )
            
        if zmq_server_instance is None:
            logger.error("Failed to create ZMQ server")
            return
            
        # Start the server
        if not zmq_server_instance.start():
            logger.error("Failed to start ZMQ server")
            return
    
    except Exception as e:
        logger.error(f"Error creating or starting ZMQ server: {e}")
        return
    
    # Setup signal handling for graceful shutdown
    def signal_handler(sig, frame):
        print("\nShutting down ZMQ server...")
        if zmq_server_instance is not None:
            zmq_server_instance.shutdown()
        print("ZMQ server stopped")
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Keep the process running
        print("ZMQ server running (press Ctrl+C to quit)")
        signal.pause()
    except KeyboardInterrupt:
        pass
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 