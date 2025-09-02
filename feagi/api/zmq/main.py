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

"""Standalone entry point for the FEAGI ZMQ server.

This script provides a way to run just the ZMQ server without the full FEAGI system.
This is useful for development and testing, but the main.py entry point should be
used for running the complete system.
"""
import argparse

from feagi.utils.logger import setup_logger

logger = setup_logger(__name__)
import logging
import signal
import sys
import time

from feagi.logging_config import setup_feagi_logging

setup_feagi_logging()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("feagi.zmq")


def main():
    """Run the FEAGI ZMQ server in standalone mode."""
    parser = argparse.ArgumentParser(
        description="FEAGI ZMQ Server (Standalone)"
    )
    parser.add_argument(
        "--host", type=str, help="Host to run the server on (required)"
    )
    parser.add_argument(
        "--req-port", type=int, default=5555, help="REQ/REP port"
    )
    parser.add_argument(
        "--pub-port", type=int, default=5556, help="PUB/SUB port"
    )
    parser.add_argument(
        "--push-port", type=int, default=5557, help="PUSH/PULL port"
    )
    parser.add_argument(
        "--sensorimotor-port", type=int, default=5558, help="Sensorimotor port"
    )
    parser.add_argument(
        "--control-port", type=int, default=5559, help="Control protocol port"
    )
    parser.add_argument(
        "--vis-base-port",
        type=int,
        default=5560,
        help="Visualization base port",
    )
    parser.add_argument(
        "--mock", action="store_true", help="Use mock core API"
    )
    args = parser.parse_args()

    # Validate required arguments
    if not args.host:
        parser.error(
            "--host is required. No hardcoded defaults for deployment compatibility."
        )

    # Warning about standalone mode
    logger.warning(
        "Running FEAGI ZMQ server in standalone mode. For full functionality, use 'python -m feagi.main'"
    )

    # Import server here to avoid circular imports
    from feagi.api.zmq.server import ZmqServer

    # Setup mock core API if needed
    core_api = None
    if args.mock:
        from unittest.mock import MagicMock

        core_api = MagicMock()
        logger.info("Using mock core API", status="[DEBUG]")
    else:
        try:
            from feagi.core import create_core_api

            core_api = create_core_api({})
            logger.info("Using real core API", status="[CONFIG]")
        except ImportError:
            logger.error(
                "Failed to import core API, using mock instead",
                status="[WARN]",
            )
            from unittest.mock import MagicMock

            core_api = MagicMock()

    # Setup ZMQ server
    zmq_server = ZmqServer(
        core_api=core_api,
        host=args.host,
        req_rep_port=args.req_port,
        pub_sub_port=args.pub_port,
        push_pull_port=args.push_port,
        sensorimotor_port=args.sensorimotor_port,
        control_port=args.control_port,
        vis_base_port=args.vis_base_port,
    )

    # Setup signal handlers
    def signal_handler(sig, frame):
        logger.info("Shutting down ZMQ server...", status="[HALT]")
        zmq_server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start the server
    logger.info(
        f"Starting ZMQ server on {args.host}:{args.req_port}", status="[START]"
    )
    logger.info(f"    - PUB/SUB port: {args.pub_port}")
    logger.info(f"    - PUSH/PULL port: {args.push_port}")
    logger.info(f"    - Sensorimotor port: {args.sensorimotor_port}")
    logger.info(f"    - Control port: {args.control_port}")
    logger.info(f"    - Visualization base port: {args.vis_base_port}")

    success = zmq_server.start()
    if not success:
        logger.error("Failed to start ZMQ server")
        return 1

    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        logger.error(f"Error in main thread: {e}")
        zmq_server.shutdown()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
