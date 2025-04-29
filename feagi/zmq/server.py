"""ZMQ Server launcher for FEAGI.

This module provides a command-line interface for starting ZMQ servers
in FEAGI, following the same pattern as the REST API server.
"""
import os
import argparse
import threading
import signal
import logging
import importlib
from typing import Dict, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("feagi.zmq.server")

# Global reference to the ZMQ server instance
zmq_server_instance = None

def main():
    """Run the FEAGI ZMQ server."""
    parser = argparse.ArgumentParser(description="FEAGI ZMQ Server")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to run the server on (default: 127.0.0.1)")
    parser.add_argument("--pub-port", type=int, default=5556, help="Publisher port")
    parser.add_argument("--sub-port", type=int, default=5557, help="Subscriber port")
    parser.add_argument("--topics", type=str, nargs="+", default=["neural", "metrics", "heartbeat"], help="Topics to support")
    parser.add_argument("--auth", action="store_true", help="Enable authentication (deprecated)")
    parser.add_argument("--encryption", action="store_true", help="Enable encryption (deprecated)")
    parser.add_argument("--config", type=str, help="Path to configuration file (deprecated)")
    args = parser.parse_args()
    
    # Set environment variables for containerization
    os.environ["FEAGI_ZMQ_HOST"] = args.host
    os.environ["FEAGI_ZMQ_PUB_PORT"] = str(args.pub_port)
    os.environ["FEAGI_ZMQ_SUB_PORT"] = str(args.sub_port)
    os.environ["FEAGI_ZMQ_TOPICS"] = ",".join(args.topics)
    
    # Load configuration if provided
    config = None
    if args.config:
        logger.warning("The '--config' parameter is deprecated and will be ignored.")
        
    if args.auth:
        logger.warning("The '--auth' parameter is deprecated and will be ignored.")
        
    if args.encryption:
        logger.warning("The '--encryption' parameter is deprecated and will be ignored.")
    
    # Create and start the ZMQ server
    print(f"Starting FEAGI ZMQ server on {args.host} (pub: {args.pub_port}, sub: {args.sub_port})")
    
    global zmq_server_instance
    
    # Try to import ZMQServer from core implementation
    try:
        # Try to import from core first
        try:
            core_module = importlib.import_module("feagi.core.zmq.server")
            ZMQServer = getattr(core_module, "ZMQServer")
            
            # Create the server directly instead of using factory
            zmq_server_instance = ZMQServer(
                host=args.host,
                pub_port=args.pub_port,
                sub_port=args.sub_port,
                topics=args.topics,
                logger=logger
            )
        except (ImportError, AttributeError):
            # Fall back to legacy factory function
            from feagi.zmq import create_zmq_server
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
    
    # Keep the main thread alive
    try:
        # Use polling instead of signal.pause() which is not available on all platforms
        while zmq_server_instance.running:
            try:
                import time
                time.sleep(1)
            except KeyboardInterrupt:
                break
    except (KeyboardInterrupt, SystemExit):
        if zmq_server_instance is not None:
            zmq_server_instance.shutdown()


if __name__ == "__main__":
    main() 