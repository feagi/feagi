"""
Integration tests for the ZMQ REST API protocol.

This module tests the integration between the ZMQ server and the REST API adapter,
verifying that REST API-style requests can be processed properly over ZMQ.
"""

import pytest
import json
import asyncio
import threading
import time
from unittest.mock import MagicMock, AsyncMock, patch

import zmq

from feagi.api.zmq.server import ZmqServer
from feagi.api.zmq.rest_adapter import ZMQRestAPIAdapter
from feagi.api.zmq.rest_client import ZMQRestClient


class TestZmqServer:
    """Custom ZMQ server for integration testing."""
    
    def __init__(self, core_api, host="127.0.0.1", port=5555):
        """Initialize the test server."""
        self.host = host
        self.port = port
        self.core_api = core_api
        self.context = zmq.Context.instance()
        self.socket = None
        self.running = False
        self.thread = None
        self.rest_adapter = ZMQRestAPIAdapter(core_api)
        
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
def mock_core_api_service():
    """Create a mock CoreAPIService for testing."""
    mock_service = MagicMock()
    # Set up async methods using AsyncMock
    mock_service.get_system_health = AsyncMock(return_value={"status": "healthy"})
    mock_service.get_configuration = MagicMock(return_value={"burst_rate": 60})
    mock_service.update_configuration = MagicMock(return_value=True)
    mock_service.get_versions = MagicMock(return_value={"feagi": "2.0.0"})
    mock_service.get_cortical_area_types = MagicMock(return_value={"sensory": ["vision"], "motor": ["limb"]})
    mock_service.get_genome = MagicMock(return_value={"cortical_areas": {"test_area": {"name": "Test Area"}}})
    mock_service.get_cortical_areas = MagicMock(return_value=[{"id": "1", "name": "Test Area"}])
    mock_service.get_cortical_area = MagicMock(return_value={"id": "1", "name": "Test Area"})
    mock_service.genome_is_loaded = MagicMock(return_value=True)
    
    # Create a mock state manager
    mock_state_manager = MagicMock()
    mock_state_manager.is_ready = MagicMock(return_value=True)
    mock_state_manager.get_burst_engine_state = MagicMock(return_value="READY")
    mock_service.get_state_manager = MagicMock(return_value=mock_state_manager)
    
    return mock_service


@pytest.fixture
def zmq_test_server(mock_core_api_service):
    """Create and start a test ZMQ server."""
    # Use a high port to avoid conflicts
    server = TestZmqServer(mock_core_api_service, host="127.0.0.1", port=15555)
    server.start()
    
    yield server
    
    # Cleanup
    server.stop()


@pytest.fixture
def zmq_client(zmq_test_server):
    """Create a ZMQ REST client connected to the test server."""
    client = ZMQRestClient(host="127.0.0.1", port=15555, timeout=2)
    client.connect()
    
    yield client
    
    # Cleanup
    client.disconnect()


def test_get_health(zmq_client, mock_core_api_service):
    """Test getting system health status."""
    response = zmq_client.get_health()
    
    # Verify the response matches what the mock service provided
    assert response == {"status": "healthy"}
    
    # Verify the service method was called
    mock_core_api_service.get_system_health.assert_called_once()


def test_get_configuration(zmq_client, mock_core_api_service):
    """Test getting system configuration."""
    response = zmq_client.get_configuration()
    
    # Verify the response matches what the mock service provided
    assert response == {"burst_rate": 60}
    
    # Verify the service method was called
    mock_core_api_service.get_configuration.assert_called_once()


def test_update_configuration(zmq_client, mock_core_api_service):
    """Test updating system configuration."""
    response = zmq_client.update_configuration({"burst_rate": 120})
    
    # Verify the success response
    assert response == {"status": "success", "message": "Configuration updated successfully"}
    
    # Verify the service method was called with correct args
    mock_core_api_service.update_configuration.assert_called_once_with({"burst_rate": 120})


def test_get_genome_blueprint(zmq_client, mock_core_api_service):
    """Test getting genome blueprint."""
    response = zmq_client.get_genome_blueprint()
    
    # Verify the response matches what the mock service provided
    assert response == {"test_area": {"name": "Test Area"}}
    
    # Verify the service method was called
    mock_core_api_service.get_genome.assert_called_once()


def test_get_cortical_areas(zmq_client, mock_core_api_service):
    """Test getting all cortical areas."""
    response = zmq_client.get_cortical_areas()
    
    # Verify the response matches what the mock service provided
    assert response == [{"id": "1", "name": "Test Area"}]
    
    # Verify the service method was called
    mock_core_api_service.get_cortical_areas.assert_called_once()


def test_get_cortical_area(zmq_client, mock_core_api_service):
    """Test getting a specific cortical area."""
    response = zmq_client.get_cortical_area("1")
    
    # Verify the response matches what the mock service provided
    assert response == {"id": "1", "name": "Test Area"}
    
    # Verify the service method was called with correct args
    mock_core_api_service.get_cortical_area.assert_called_once_with("1")


def test_get_status(zmq_client, mock_core_api_service):
    """Test getting system status."""
    response = zmq_client.get_status()
    
    # Verify key fields in the response
    assert response["genome_availability"] is True
    assert response["brain_readiness"] is True
    assert response["burst_engine_status"] == "READY"
    assert "timestamp" in response
    
    # Verify the service methods were called
    mock_core_api_service.genome_is_loaded.assert_called()
    mock_core_api_service.get_state_manager.assert_called()


def test_error_handling(zmq_client, mock_core_api_service):
    """Test error handling in the REST API adapter."""
    # Make get_configuration raise an exception
    mock_core_api_service.get_configuration.side_effect = ValueError("Test error")
    
    # Verify the client raises an exception
    with pytest.raises(RuntimeError) as excinfo:
        zmq_client.get_configuration()
    
    # Verify the error message
    assert "Handler error" in str(excinfo.value)
    assert "Test error" in str(excinfo.value) 