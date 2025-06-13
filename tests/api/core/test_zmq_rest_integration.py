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
Integration tests for CoreAPIService and ZMQ REST API.

This module tests the integration between CoreAPIService and the ZMQ REST API adapter,
verifying that CoreAPIService can properly handle requests from ZMQ REST API clients.
"""

import asyncio
import threading
import time
from unittest.mock import MagicMock, patch

import pytest
import zmq

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.api.zmq.rest_adapter import ZMQRestAPIAdapter
from feagi.core.state_manager import FeagiStateManager


class MockConnectomeManager:
    """Mock ConnectomeManager for testing."""

    def __init__(self):
        """Initialize the mock connectome manager."""
        # Add fcl_manager attribute to satisfy CoreAPIService initialization check
        self.fcl_manager = MagicMock()

        self.cortical_areas = {
            "test-area-1": {
                "id": "test-area-1",
                "name": "Test Area 1",
                "type": "sensory",
                "position": [0, 0, 0],
                "dimensions": [10, 10, 1],
            },
            "test-area-2": {
                "id": "test-area-2",
                "name": "Test Area 2",
                "type": "association",
                "position": [20, 0, 0],
                "dimensions": [10, 10, 1],
            },
        }

        # Add api_message_queue to satisfy any potential checks
        self.api_message_queue = MagicMock()

    def get_all_cortical_areas(self):
        """Get all cortical areas."""
        return list(self.cortical_areas.values())

    def get_cortical_area(self, cortical_id):
        """Get a specific cortical area."""
        return self.cortical_areas.get(cortical_id, None)


@pytest.fixture
def mock_state_manager():
    """Create a mock FeagiStateManager."""
    mock_manager = MagicMock()
    mock_manager.get_connectome.return_value = MockConnectomeManager()
    mock_manager.is_ready.return_value = True
    mock_manager.get_burst_engine_state.return_value = "running"
    mock_manager.load_genome.return_value = True
    return mock_manager


@pytest.fixture
def core_api_service(mock_state_manager):
    """Create a CoreAPIService with mocked dependencies."""
    connectome_manager = mock_state_manager.get_connectome()

    # Monkey patch FeagiStateManager.instance to return our mock
    original_instance = FeagiStateManager.instance
    FeagiStateManager.instance = lambda: mock_state_manager

    # Patch CoreAPIService to avoid burst engine creation issues
    with patch(
        "feagi.api.core.services.core_api_service.CoreAPIService._create_burst_engine"
    ) as mock_create_burst:
        # Return a mock burst engine
        mock_burst_engine = MagicMock()
        mock_create_burst.return_value = mock_burst_engine

        # Create the service
        service = CoreAPIService(connectome_manager, mock_state_manager)

        # Restore original instance method
        FeagiStateManager.instance = original_instance

        # Set up necessary mocks for tests
        service.get_cortical_areas = MagicMock(
            return_value=connectome_manager.get_all_cortical_areas()
        )
        service.get_cortical_area = MagicMock(
            side_effect=connectome_manager.get_cortical_area
        )

        return service


@pytest.fixture
def rest_adapter(core_api_service):
    """Create a ZMQRestAPIAdapter with the CoreAPIService."""
    return ZMQRestAPIAdapter(core_api_service)


class TestZmqServer:
    """Custom ZMQ server for integration testing."""

    def __init__(self, rest_adapter, host="127.0.0.1", port=5555):
        """Initialize the test server."""
        self.host = host
        self.port = port
        self.rest_adapter = rest_adapter
        self.context = zmq.Context.instance()
        self.socket = None
        self.running = False
        self.thread = None

    def start(self):
        """Start the server in a background thread."""
        self.thread = threading.Thread(target=self._run_server)
        self.thread.daemon = True
        self.thread.start()

        # Wait for server to start
        time.sleep(0.2)

    def _run_server(self):
        """Run the server."""
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.bind(f"tcp://{self.host}:{self.port}")
        self.running = True

        # Process messages until stopped
        while self.running:
            try:
                # Wait for message with timeout to allow clean shutdown
                self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
                message_parts = self.socket.recv_multipart()

                # Basic format checking
                if len(message_parts) < 2:
                    continue

                # Split into identity and payload
                identity = message_parts[0]
                payload = message_parts[1]

                # Process with REST API adapter
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                response_data = loop.run_until_complete(
                    self.rest_adapter.process_message(payload)
                )
                loop.close()

                # Send response back
                self.socket.send_multipart([identity, response_data])

            except zmq.Again:
                # Timeout, check if we should continue
                continue
            except Exception as e:
                print(f"Error in test server: {e}")
                continue

    def stop(self):
        """Stop the server."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()


@pytest.fixture
def zmq_server(rest_adapter):
    """Create and start a test ZMQ server."""
    # Use a high port to avoid conflicts
    server = TestZmqServer(rest_adapter, host="127.0.0.1", port=15556)
    server.start()

    yield server

    # Cleanup
    server.stop()


class ZMQClient:
    """Simple ZMQ client for testing."""

    def __init__(self, host="localhost", port=5555, timeout=30):
        """Initialize the ZMQ client."""
        self.host = host
        self.port = port
        self.timeout = timeout
        self.context = zmq.Context.instance()
        self.socket = None
        self.identity = f"test-client-{time.time()}".encode("utf-8")

    def connect(self):
        """Connect to the ZMQ server."""
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.IDENTITY, self.identity)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def disconnect(self):
        """Disconnect from the ZMQ server."""
        if self.socket:
            self.socket.close()
            self.socket = None

    def send_rest_request(self, method, route, params=None, query=None, body=None):
        """Send a REST API-style request."""
        request = {
            "method": method,
            "route": route,
            "timestamp": int(time.time() * 1000),
        }

        if params:
            request["params"] = params
        if query:
            request["query"] = query
        if body:
            request["body"] = body

        self.socket.send_json(request)

        # Receive response
        response = self.socket.recv_json()
        return response


@pytest.fixture
def zmq_client(zmq_server):
    """Create a ZMQ client connected to the test server."""
    client = ZMQClient(host="127.0.0.1", port=15556, timeout=2)
    client.connect()

    yield client

    # Cleanup
    client.disconnect()


def test_get_cortical_areas(zmq_client, core_api_service):
    """Test getting cortical areas through ZMQ REST API."""
    # Send request
    response = zmq_client.send_rest_request("GET", "/v1/connectome/cortical_areas")

    # Verify response
    assert response["status"] == 200
    assert len(response["body"]) == 2
    assert response["body"][0]["id"] == "test-area-1"
    assert response["body"][0]["name"] == "Test Area 1"
    assert response["body"][1]["id"] == "test-area-2"
    assert response["body"][1]["name"] == "Test Area 2"


def test_get_configuration(zmq_client, core_api_service):
    """Test getting configuration through ZMQ REST API."""
    # Mock the configuration
    core_api_service.get_configuration = MagicMock(
        return_value={"burst_rate": 60, "learning_rate": 0.01}
    )

    # Send request
    response = zmq_client.send_rest_request("GET", "/v1/system/configuration")

    # Verify response
    assert response["status"] == 200
    assert response["body"]["burst_rate"] == 60
    assert response["body"]["learning_rate"] == 0.01


def test_update_configuration(zmq_client, core_api_service):
    """Test updating configuration through ZMQ REST API."""
    # Mock the update_configuration method
    core_api_service.update_configuration = MagicMock(return_value=True)

    # Send request
    new_config = {"burst_rate": 120}
    response = zmq_client.send_rest_request(
        "PUT", "/v1/system/configuration", body=new_config
    )

    # Verify response
    assert response["status"] == 200
    assert response["body"]["status"] == "success"

    # Verify service method was called
    core_api_service.update_configuration.assert_called_once_with(new_config)


def test_get_status(zmq_client, core_api_service, mock_state_manager):
    """Test getting system status through ZMQ REST API."""
    # Mock genome_is_loaded
    core_api_service.genome_is_loaded = MagicMock(return_value=True)

    # Send request
    response = zmq_client.send_rest_request("GET", "/v1/status")

    # Verify response
    assert response["status"] == 200
    assert response["body"]["genome_availability"] is True
    assert response["body"]["brain_readiness"] is True
    assert "burst_engine_status" in response["body"]
    assert "timestamp" in response["body"]


def test_error_handling(zmq_client, core_api_service):
    """Test error handling through ZMQ REST API."""
    # Mock get_configuration to raise an exception
    core_api_service.get_configuration = MagicMock(side_effect=ValueError("Test error"))

    # Send request
    response = zmq_client.send_rest_request("GET", "/v1/system/configuration")

    # Verify error response
    assert response["status"] == 500
    assert response["body"]["type"] == "error"
    assert "Handler error" in response["body"]["message"]
    assert "Test error" in response["body"]["message"]


def test_invalid_route(zmq_client):
    """Test handling an invalid route through ZMQ REST API."""
    # Send request to an invalid route
    response = zmq_client.send_rest_request("GET", "/v1/invalid/endpoint")

    # Verify error response
    assert response["status"] == 404
    assert response["body"]["type"] == "error"
    assert "Endpoint not found" in response["body"]["message"]
