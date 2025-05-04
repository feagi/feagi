"""Tests for the Burst Engine API endpoints."""

import os
import json
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
import pytest

from feagi.api.rest.app import create_rest_app, app
from feagi.api.rest.app import get_core_api

# Create a global mock outside of fixtures so we can share it
mock_core_api_instance = MagicMock()

# Configure the mock
mock_core_api_instance.get_burst_engine_config.return_value = {
    "burst_duration": 10,
    "refractory_period": 5,
    "threshold": 0.5,
    "decay_rate": 0.1,
    "firing_threshold": 0.7,
    "membrane_potential_decay": 0.05
}
mock_core_api_instance.update_burst_engine_config.return_value = True
mock_core_api_instance.get_burst_engine_stats.return_value = {
    "average_burst_time": 8.5,
    "max_burst_time": 12.3,
    "min_burst_time": 7.1,
    "total_bursts": 1000,
    "average_active_neurons": 500,
    "memory_usage": 128.5  # MB
}

@pytest.fixture
def client():
    """Create a test client for the FastAPI app with mocked dependencies."""
    # Create a new instance of the app for testing
    test_app = create_rest_app()
    
    # Patch the dependency for testing
    test_app.dependency_overrides[get_core_api] = lambda: mock_core_api_instance
    
    # Return the test client
    return TestClient(test_app)

@pytest.fixture
def mock_core_api():
    """Return the global mock_core_api_instance."""
    # Reset mocks before each test
    mock_core_api_instance.reset_mock()
    
    # Reconfigure default responses
    mock_core_api_instance.get_burst_engine_config.return_value = {
        "burst_duration": 10,
        "refractory_period": 5,
        "threshold": 0.5,
        "decay_rate": 0.1,
        "firing_threshold": 0.7,
        "membrane_potential_decay": 0.05
    }
    mock_core_api_instance.update_burst_engine_config.return_value = True
    mock_core_api_instance.get_burst_engine_stats.return_value = {
        "average_burst_time": 8.5,
        "max_burst_time": 12.3,
        "min_burst_time": 7.1,
        "total_bursts": 1000,
        "average_active_neurons": 500,
        "memory_usage": 128.5  # MB
    }
    
    return mock_core_api_instance

def test_get_burst_engine_config(client, mock_core_api):
    """Test getting the burst engine configuration."""
    # Override the app's dependency to use our fixture
    app.dependency_overrides[get_core_api] = lambda: mock_core_api
    
    response = client.get("/api/v1/burst_engine/config")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "burst_duration" in data
    assert "refractory_period" in data
    assert "threshold" in data
    assert "decay_rate" in data
    assert "firing_threshold" in data
    assert "membrane_potential_decay" in data
    
    # Check that the values match the mock
    assert data["burst_duration"] == 10
    assert data["refractory_period"] == 5
    assert data["threshold"] == 0.5
    assert data["decay_rate"] == 0.1
    assert data["firing_threshold"] == 0.7
    assert data["membrane_potential_decay"] == 0.05

def test_update_burst_engine_config(client, mock_core_api):
    """Test updating the burst engine configuration."""
    # Override the app's dependency to use our fixture
    app.dependency_overrides[get_core_api] = lambda: mock_core_api
    
    update_data = {
        "parameters": {
            "burst_duration": 15,
            "threshold": 0.6
        }
    }
    
    response = client.put("/api/v1/burst_engine/config", json=update_data)
    assert response.status_code == 200
    data = response.json()
    
    # Check the response
    assert "message" in data
    assert "updated" in data["message"].lower()
    
    # Verify the mock was called with the correct parameters
    mock_core_api.update_burst_engine_config.assert_called_once_with(update_data["parameters"])

def test_update_burst_engine_config_failure(client, mock_core_api):
    """Test updating the burst engine configuration when it fails."""
    # Override the app's dependency to use our fixture
    app.dependency_overrides[get_core_api] = lambda: mock_core_api
    
    # Override the mock to simulate failure
    mock_core_api.update_burst_engine_config.return_value = False
    
    update_data = {
        "parameters": {
            "burst_duration": 15
        }
    }
    
    response = client.put("/api/v1/burst_engine/config", json=update_data)
    assert response.status_code == 500
    data = response.json()
    
    # Check the error response
    assert "detail" in data
    assert "failed" in data["detail"].lower()
    
    # Verify the mock was called
    mock_core_api.update_burst_engine_config.assert_called_once()

def test_get_burst_engine_stats(client, mock_core_api):
    """Test getting the burst engine statistics."""
    # Override the app's dependency to use our fixture
    app.dependency_overrides[get_core_api] = lambda: mock_core_api
    
    response = client.get("/api/v1/burst_engine/stats")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "average_burst_time" in data
    assert "max_burst_time" in data
    assert "min_burst_time" in data
    assert "total_bursts" in data
    assert "average_active_neurons" in data
    assert "memory_usage" in data
    
    # Check that the values match the mock
    assert data["average_burst_time"] == 8.5
    assert data["max_burst_time"] == 12.3
    assert data["min_burst_time"] == 7.1
    assert data["total_bursts"] == 1000
    assert data["average_active_neurons"] == 500
    assert data["memory_usage"] == 128.5 