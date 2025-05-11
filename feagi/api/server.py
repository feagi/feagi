"""Server script for running the FEAGI API standalone.

This script provides a way to run just the FEAGI API server without the full
FEAGI system. This is useful for development and testing, but the main.py
entry point should be used for running the complete system.
"""
import os
import argparse
import logging
import uvicorn
from feagi.logging_config import setup_feagi_logging

setup_feagi_logging()

logger = logging.getLogger(__name__)

def main():
    """Run the FEAGI API server in standalone mode."""
    parser = argparse.ArgumentParser(description="FEAGI API Server (Standalone)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to run the server on")
    parser.add_argument("--port", type=int, default=8000, help="Port to run the server on")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload")
    parser.add_argument("--zmq", action="store_true", help="Enable ZeroMQ client mode")
    parser.add_argument("--zmq-host", type=str, default="127.0.0.1", help="ZeroMQ host")
    parser.add_argument("--zmq-req-port", type=int, default=5555, help="ZeroMQ Request-Reply port")
    parser.add_argument("--zmq-pub-port", type=int, default=5556, help="ZeroMQ Publish-Subscribe port")
    parser.add_argument("--zmq-push-port", type=int, default=5557, help="ZeroMQ Push-Pull port")
    parser.add_argument("--zmq-stream-port", type=int, default=5558, help="ZeroMQ Stream port")
    args = parser.parse_args()
    
    # Warning about standalone mode
    logger.warning("Running FEAGI API server in standalone mode. For full functionality, use 'python -m feagi.main'")
    
    # Set environment variables for ZMQ configuration
    if args.zmq:
        os.environ["FEAGI_ZMQ_ENABLED"] = "1"
        os.environ["FEAGI_ZMQ_HOST"] = args.zmq_host
        os.environ["FEAGI_ZMQ_REQ_PORT"] = str(args.zmq_req_port)
        os.environ["FEAGI_ZMQ_PUB_PORT"] = str(args.zmq_pub_port)
        os.environ["FEAGI_ZMQ_PUSH_PORT"] = str(args.zmq_push_port)
        os.environ["FEAGI_ZMQ_STREAM_PORT"] = str(args.zmq_stream_port)
    
    logger.info(f"Starting FEAGI API server on {args.host}:{args.port}", emoji="🚀")
    if args.zmq:
        logger.info(f"ZeroMQ client mode enabled, connecting to {args.zmq_host}", emoji="  ")
        logger.info(f"  - Request-Reply port: {args.zmq_req_port}", emoji="  ")
        logger.info(f"  - Publish-Subscribe port: {args.zmq_pub_port}", emoji="  ")
        logger.info(f"  - Push-Pull port: {args.zmq_push_port}", emoji="  ")
        logger.info(f"  - Stream port: {args.zmq_stream_port}", emoji="  ")
    
    # Run the API server
    uvicorn.run(
        "feagi.api.rest.app:create_rest_app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        factory=True,
    )

if __name__ == "__main__":
    main() 