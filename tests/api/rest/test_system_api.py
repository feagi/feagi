"""Tests for the System API endpoints."""

import os
import json
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
import pytest

from feagi.api.rest.app import create_rest_app

@pytest.fixture
def app():
    """Create a FastAPI app for testing."""
    return create_rest_app()

@pytest.fixture
def client(app):
    """Create a test client for the FastAPI app."""
    return TestClient(app)

@pytest.fixture
def mock_core_api():
    """Create a mock CoreAPIService."""
    with patch('feagi.api.gateway.APIGateway.core_api', new_callable=MagicMock) as mock:
        # Mock configuration data
        mock.get_configuration.return_value = {
            "log_level": "INFO",
            "api": {
                "host": "127.0.0.1",
                "port": 8000
            },
            "simulation": {
                "burst_duration": 10,
                "max_firing_rate": 100
            },
            "connectivity": {
                "max_synapses_per_neuron": 1000
            }
        }
        
        # Mock update configuration
        mock.update_configuration.return_value = True
        
        # Mock brain state data
        mock.get_brain_state.return_value = {
            "neurons": 10000,
            "synapses": 50000,
            "cortical_areas": 5,
            "dimensions": [100, 100, 50],
            "memory_usage": 128.5  # MB
        }
        
        yield mock

def test_get_configuration(client, mock_core_api):
    """Test getting the system configuration."""
    response = client.get("/api/v0/system/configuration/")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "configuration" in data
    config = data["configuration"]
    
    # Check that the values match the mock
    assert config["log_level"] == "INFO"
    assert config["api"]["host"] == "127.0.0.1"
    assert config["api"]["port"] == 8000
    assert config["simulation"]["burst_duration"] == 10
    assert config["simulation"]["max_firing_rate"] == 100
    assert config["connectivity"]["max_synapses_per_neuron"] == 1000

def test_update_configuration(client, mock_core_api):
    """Test updating the system configuration."""
    update_data = {
        "parameters": {
            "log_level": "DEBUG",
            "simulation": {
                "burst_duration": 5
            }
        }
    }
    
    response = client.put("/api/v0/system/configuration/", json=update_data)
    assert response.status_code == 200
    data = response.json()
    
    # Check the response
    assert "message" in data
    assert "updated" in data["message"].lower()
    
    # Verify the mock was called with the correct parameters
    mock_core_api.update_configuration.assert_called_once_with(update_data["parameters"])

def test_update_configuration_failure(client, mock_core_api):
    """Test updating the system configuration when it fails."""
    # Override the mock to simulate failure
    mock_core_api.update_configuration.return_value = False
    
    update_data = {
        "parameters": {
            "log_level": "DEBUG"
        }
    }
    
    response = client.put("/api/v0/system/configuration/", json=update_data)
    assert response.status_code == 500
    data = response.json()
    
    # Check the error response
    assert "detail" in data
    assert "failed" in data["detail"].lower()
    
    # Verify the mock was called
    mock_core_api.update_configuration.assert_called_once()

def test_get_brain_state(client, mock_core_api):
    """Test getting the brain state."""
    response = client.get("/api/v0/system/brain/")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "brain_state" in data
    brain_state = data["brain_state"]
    
    # Check that the values match the mock
    assert brain_state["neurons"] == 10000
    assert brain_state["synapses"] == 50000
    assert brain_state["cortical_areas"] == 5
    assert brain_state["dimensions"] == [100, 100, 50]
    assert brain_state["memory_usage"] == 128.5 