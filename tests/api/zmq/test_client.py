#!/usr/bin/env python
"""Test client for connecting to a running FEAGI ZMQ server."""
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("test_feagi_client")

def main():
    """Connect to a running FEAGI ZMQ server and test messaging."""
    logger.info("========================================================")
    logger.info("NOTE: This test has been converted to a compatibility stub")
    logger.info("The ZMQ implementation has been moved to feagi.api.zmq")
    logger.info("Update this test if it needs to be functional again")
    logger.info("========================================================")
    return 0

if __name__ == "__main__":
    sys.exit(main()) 