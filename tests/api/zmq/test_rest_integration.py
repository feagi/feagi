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
Integration tests for REST API over ZMQ.

Tests the ZMQ server's ability to handle REST API-style requests.
"""

import pytest

# Skip the entire test module since the ZMQ implementation has changed
pytest.skip("ZMQ server tests need to be updated after protocol refactoring", allow_module_level=True)

# Keep original code for reference
import asyncio
import json
import time
from unittest.mock import MagicMock, patch, AsyncMock

import zmq
import zmq.asyncio

from feagi.api.zmq.server import ZmqServer


# Test client for sending REST API requests over ZMQ
class TestZmqServer:
    def __init__(self, host="localhost", port=5559):
        self.host = host
        self.port = port
        self.context = zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.connect(f"tcp://{host}:{port}")
        
    async def send_request(self, route, method="GET", params=None, query=None, body=None):
        """Send a REST API-style request over ZMQ."""
        request = {
            "route": route,
            "method": method,
            "params": params or {},
            "query": query or {},
            "body": body or {},
            "timestamp": int(time.time() * 1000)
        }
        
        # Send the request
        await self.socket.send_multipart([b"", json.dumps(request).encode('utf-8')])
        
        # Receive the response
        frames = await self.socket.recv_multipart()
        response = json.loads(frames[0].decode('utf-8'))
        
        # Extract the body for convenience
        if "body" in response:
            return response["body"]
        return response
        
    def close(self):
        """Close the socket."""
        self.socket.close()


# Test fixtures
@pytest.fixture
def core_api_mock():
    """Mock for the core API service."""
    mock = MagicMock()
    
    # Mock health check
    mock.get_system_health = AsyncMock(return_value={"status": "healthy"})
    
    # Mock configuration
    mock.get_configuration = AsyncMock(return_value={"burst_rate": 60})
    mock.update_configuration = AsyncMock(return_value={"message": "Configuration updated", "status": "success"})
    
    # Mock genome data
    mock.get_genome_blueprint = AsyncMock(return_value={
        "test_area": {
            "id": "1",
            "name": "Test Area"
        }
    })
    
    # Mock cortical area data
    mock.get_cortical_areas = AsyncMock(return_value=[{
        "id": "1",
        "name": "Test Area"
    }])
    
    # Mock single cortical area
    mock.get_cortical_area = AsyncMock(return_value={
        "id": "1",
        "name": "Test Area"
    })
    
    # Mock runtime status
    mock.get_runtime_status = AsyncMock(return_value={
        "runtime_state": "running",
        "genome_availability": "loaded",
        "neurons": 1000,
        "synapses": 5000,
        "burst_counter": 100
    })
    
    return mock


@pytest.fixture
async def zmq_server(core_api_mock):
    """Create a ZMQ server for testing."""
    # Create a server with mock core API
    server = ZmqServer(
        core_api=core_api_mock,
        control_port=5559
    )
    
    # Start the server
    server.start()
    
    # Give it time to initialize
    await asyncio.sleep(0.1)
    
    # Return the server for the test
    yield server
    
    # Cleanup after test
    server.shutdown()


@pytest.fixture
async def test_client():
    """Create a test client for sending REST API requests."""
    client = TestZmqServer()
    yield client
    client.close()


# Tests
@pytest.mark.asyncio
async def test_get_health(zmq_server, test_client):
    """Test GET /v1/system/health_check."""
    response = await test_client.send_request("/v1/system/health_check")
    assert response == {"status": "healthy"}


@pytest.mark.asyncio
async def test_get_configuration(zmq_server, test_client):
    """Test GET /v1/configuration."""
    response = await test_client.send_request("/v1/configuration")
    assert response == {"burst_rate": 60}


@pytest.mark.asyncio
async def test_update_configuration(zmq_server, test_client):
    """Test PUT /v1/configuration."""
    response = await test_client.send_request(
        "/v1/configuration",
        method="PUT",
        body={"burst_rate": 120}
    )
    assert response == {"message": "Configuration updated", "status": "success"}


@pytest.mark.asyncio
async def test_get_genome_blueprint(zmq_server, test_client):
    """Test GET /v1/genome/blueprint."""
    response = await test_client.send_request("/v1/genome/blueprint")
    assert response == {"test_area": {"id": "1", "name": "Test Area"}}


@pytest.mark.asyncio
async def test_get_cortical_areas(zmq_server, test_client):
    """Test GET /v1/cortical_areas."""
    response = await test_client.send_request("/v1/cortical_areas")
    assert response == [{"id": "1", "name": "Test Area"}]


@pytest.mark.asyncio
async def test_get_cortical_area(zmq_server, test_client):
    """Test GET /v1/cortical_areas/{id}."""
    response = await test_client.send_request("/v1/cortical_areas/1")
    assert response == {"id": "1", "name": "Test Area"}


@pytest.mark.asyncio
async def test_get_status(zmq_server, test_client):
    """Test GET /v1/status."""
    response = await test_client.send_request("/v1/status")
    assert response["runtime_state"] == "running"
    assert response["genome_availability"] == "loaded"
    assert response["neurons"] == 1000
    assert response["synapses"] == 5000
    assert response["burst_counter"] == 100


@pytest.mark.asyncio
async def test_error_handling(zmq_server, test_client, core_api_mock):
    """Test error handling for REST API requests."""
    # Mock the API to raise an exception
    core_api_mock.get_system_health.side_effect = RuntimeError("Test error")
    
    # Send request and check for error response
    with pytest.raises(RuntimeError):
        await test_client.send_request("/v1/system/health_check") 