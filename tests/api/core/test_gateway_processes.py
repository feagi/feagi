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

"""
Test script for testing API gateway integration with the FEAGI process manager.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import pytest

from feagi.api import APIGateway, get_api_gateway


class TestGatewayProcessIntegration(unittest.TestCase):
    """Tests for API Gateway integration with FEAGI process manager."""

    def setUp(self):
        """Set up test fixtures."""
        # Reset the gateway singleton before each test
        APIGateway._instance = None
        # Save original environment
        self.original_env = os.environ.copy()

    def tearDown(self):
        """Clean up after tests."""
        # Restore original environment
        os.environ.clear()
        os.environ.update(self.original_env)

    def test_process_manager_integration(self):
        """Test that the gateway detects the FEAGI_INITIALIZED environment variable."""
        # Create initial gateway instance with mock
        with patch("feagi.api.gateway.api_gateway.logger") as mock_logger:
            initial_gateway = get_api_gateway()

            # Verify we have a core_api (should be a mock)
            self.assertIsNotNone(initial_gateway.core_api)

            # Set environment variable to simulate running in FEAGI main process
            os.environ["FEAGI_INITIALIZED"] = "1"

            # Reset the gateway singleton to force re-initialization
            APIGateway._instance = None

            # Create a new gateway instance
            gateway = get_api_gateway()

            # Verify the appropriate log message was emitted (shows environment detection)
            mock_logger.info.assert_any_call(
                "Running in FEAGI main process, using Process Manager"
            )

            # Test that the singleton pattern still works
            self.assertIs(gateway, APIGateway())

    def test_zmq_client_initialization(self):
        """Test that the gateway initializes the ZMQ client correctly."""
        # Set environment variables for ZMQ
        os.environ["FEAGI_ZMQ_ENABLED"] = "1"
        os.environ["FEAGI_ZMQ_HOST"] = "127.0.0.1"
        os.environ["FEAGI_ZMQ_REQ_PORT"] = "5555"
        os.environ["FEAGI_ZMQ_PUB_PORT"] = "5556"
        os.environ["FEAGI_ZMQ_PUSH_PORT"] = "5557"
        os.environ["FEAGI_ZMQ_STREAM_PORT"] = "5558"

        # Patch the logger to verify ZMQ initialization is attempted
        with patch("feagi.api.gateway.api_gateway.logger") as mock_logger:
            # Reset the gateway singleton to force re-initialization
            APIGateway._instance = None

            # Patch the ZmqClient to avoid actual network operations
            with patch(
                "feagi.api.gateway.api_gateway.ZmqClient"
            ) as mock_zmq_client_cls:
                # Create a new gateway instance
                gateway = get_api_gateway()

                # Verify ZMQ client initialization was attempted
                mock_logger.info.assert_any_call(
                    f"Initializing ZMQ client to 127.0.0.1"
                )

                # Verify ZmqClient constructor was called
                mock_zmq_client_cls.assert_called_once()

    def test_local_core_api_initialization(self):
        """Test that the gateway recognizes the local core environment variable."""
        # Set environment variable for local core
        os.environ["FEAGI_LOCAL_CORE"] = "1"

        # Reset the gateway singleton to force re-initialization
        APIGateway._instance = None

        # Create a mock for create_core_api with the expected parameter
        mock_core_api = MagicMock()

        # Patch both the logger and the create_core_api function
        # Note: create_core_api is imported inside the _initialize_core_api method
        # so we need to patch feagi.core.create_core_api
        with patch("feagi.api.gateway.api_gateway.logger") as mock_logger, patch(
            "feagi.core.create_core_api", return_value=mock_core_api
        ):
            # Create a new gateway instance
            gateway = get_api_gateway()

            # Verify the correct message was logged
            mock_logger.info.assert_any_call("Creating local Core API instance")

            # Verify the gateway initialized properly
            self.assertIsNotNone(gateway)
            self.assertIs(APIGateway._instance, gateway)
            self.assertEqual(gateway.core_api, mock_core_api)


if __name__ == "__main__":
    unittest.main()
