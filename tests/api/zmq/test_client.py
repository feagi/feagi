#!/usr/bin/env python
"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

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