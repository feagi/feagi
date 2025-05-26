"""
Tests for the ZMQ REST API adapter.

This module tests the functionality of the REST API adapter for ZMQ,
which allows REST API-style requests to be processed over ZMQ.
"""

import pytest
import json
import asyncio
from unittest.mock import MagicMock, AsyncMock, patch

from feagi.api.zmq.rest_adapter import ZMQRestAPIAdapter


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
    mock_service.get_genome = MagicMock(return_value={"cortical_areas": {}})
    mock_service.get_cortical_areas = MagicMock(return_value=[])
    mock_service.get_cortical_area = MagicMock(return_value=None)
    mock_service.genome_is_loaded = MagicMock(return_value=False)
    
    # Create a mock state manager
    mock_state_manager = MagicMock()
    mock_state_manager.is_ready = MagicMock(return_value=True)
    mock_state_manager.get_burst_engine_state = MagicMock(return_value="READY")
    mock_service.get_state_manager = MagicMock(return_value=mock_state_manager)
    
    return mock_service


@pytest.fixture
def rest_adapter(mock_core_api_service):
    """Create a REST API adapter with a mock service for testing."""
    return ZMQRestAPIAdapter(mock_core_api_service)


@pytest.mark.asyncio
async def test_parse_valid_message(rest_adapter):
    """Test parsing a valid REST API message."""
    # Create a valid message
    message = {
        "route": "/v1/system/health_check",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Parse the message
    result = rest_adapter._parse_message(message_bytes)
    
    # Verify result
    assert result is not None
    assert result["route"] == "/v1/system/health_check"
    assert result["method"] == "GET"
    assert "params" in result
    assert "query" in result
    assert "body" in result
    assert result["timestamp"] == 1621234567890


@pytest.mark.asyncio
async def test_parse_invalid_message(rest_adapter):
    """Test parsing an invalid message."""
    # Create an invalid message (not JSON)
    message_bytes = b"not json"
    
    # Parse the message
    result = rest_adapter._parse_message(message_bytes)
    
    # Verify result
    assert result is None
    
    # Create an invalid message (missing required fields)
    message = {"foo": "bar"}
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Parse the message
    result = rest_adapter._parse_message(message_bytes)
    
    # Verify result
    assert result is None


@pytest.mark.asyncio
async def test_process_health_check(rest_adapter, mock_core_api_service):
    """Test processing a health check request."""
    # Create a health check request
    message = {
        "route": "/v1/system/health_check",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response
    assert response["status"] == 200
    assert response["body"] == {"status": "healthy"}
    assert "timestamp" in response


@pytest.mark.asyncio
async def test_process_configuration(rest_adapter, mock_core_api_service):
    """Test processing a configuration request."""
    # Create a configuration request
    message = {
        "route": "/v1/system/configuration",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response
    assert response["status"] == 200
    assert response["body"] == {"burst_rate": 60}
    assert "timestamp" in response


@pytest.mark.asyncio
async def test_update_configuration(rest_adapter, mock_core_api_service):
    """Test updating configuration."""
    # Create an update configuration request
    message = {
        "route": "/v1/system/configuration",
        "method": "PUT",
        "params": {},
        "query": {},
        "body": {"burst_rate": 120},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response
    assert response["status"] == 200
    assert response["body"] == {"status": "success", "message": "Configuration updated successfully"}
    assert "timestamp" in response
    
    # Verify service method was called with correct args
    mock_core_api_service.update_configuration.assert_called_once_with({"burst_rate": 120})


@pytest.mark.asyncio
async def test_get_status(rest_adapter, mock_core_api_service):
    """Test getting system status."""
    # Create a status request
    message = {
        "route": "/v1/status",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response
    assert response["status"] == 200
    assert response["body"]["genome_availability"] is False
    assert response["body"]["brain_readiness"] is True
    assert response["body"]["burst_engine_status"] == "READY"
    assert "timestamp" in response["body"]


@pytest.mark.asyncio
async def test_invalid_route(rest_adapter):
    """Test handling an invalid route."""
    # Create a request with an invalid route
    message = {
        "route": "/v1/invalid/endpoint",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response is an error
    assert response["status"] == 404
    assert response["body"]["type"] == "error"
    assert "Endpoint not found" in response["body"]["message"]


@pytest.mark.asyncio
async def test_handler_error(rest_adapter, mock_core_api_service):
    """Test handling an error in a handler method."""
    # Make get_configuration raise an exception
    mock_core_api_service.get_configuration.side_effect = ValueError("Test error")
    
    # Create a configuration request
    message = {
        "route": "/v1/system/configuration",
        "method": "GET",
        "params": {},
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response is an error
    assert response["status"] == 500
    assert response["body"]["type"] == "error"
    assert "Handler error" in response["body"]["message"]


@pytest.mark.asyncio
async def test_route_with_parameters(rest_adapter, mock_core_api_service):
    """Test handling a route with path parameters."""
    # Set up mock to return a test cortical area
    test_area = {"id": "123", "name": "Test Area"}
    mock_core_api_service.get_cortical_area.return_value = test_area
    
    # Create a request with a path parameter
    message = {
        "route": "/v1/connectome/cortical_area/123",
        "method": "GET",
        "params": {"cortical_id": "123"},  # Path parameter
        "query": {},
        "body": {},
        "timestamp": 1621234567890
    }
    
    # Convert to bytes
    message_bytes = json.dumps(message).encode('utf-8')
    
    # Process the message
    response_bytes = await rest_adapter.process_message(message_bytes)
    
    # Parse response
    response = json.loads(response_bytes.decode('utf-8'))
    
    # Verify response
    assert response["status"] == 200
    assert response["body"] == test_area
    
    # Verify service method was called with correct args
    mock_core_api_service.get_cortical_area.assert_called_once_with("123") 