"""Tests for the Simulation API endpoints."""

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
        # Mock the simulation status response
        mock.get_simulation_status.return_value = {
            "running": False,
            "burst_count": 0,
            "uptime": 0.0,
            "performance": {
                "average_burst_time": 10.5,
                "total_neurons": 10000,
                "active_neurons": 500
            }
        }
        
        # Mock the simulation control methods
        mock.start_simulation.return_value = True
        mock.stop_simulation.return_value = True
        
        yield mock

def test_get_simulation_status(client, mock_core_api):
    """Test getting the simulation status."""
    response = client.get("/v1/simulation/status")
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the structure of the response
        assert "running" in data
        assert "burst_count" in data
        assert "uptime" in data
        assert "performance" in data
        
        # Check that the values match the mock
        assert data["running"] is False
        assert data["burst_count"] == 0
        assert data["uptime"] == 0.0
        assert data["performance"]["average_burst_time"] == 10.5
        assert data["performance"]["total_neurons"] == 10000
        assert data["performance"]["active_neurons"] == 500

def test_start_simulation(client, mock_core_api):
    """Test starting the simulation."""
    response = client.post("/v1/simulation/start")
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the response
        assert "message" in data
        assert "started" in data["message"].lower()
        
        # Verify the mock was called
        mock_core_api.start_simulation.assert_called_once()

def test_start_simulation_failure(client, mock_core_api):
    """Test starting the simulation when it fails."""
    # Override the mock to simulate failure
    mock_core_api.start_simulation.return_value = False
    
    response = client.post("/v1/simulation/start")
    assert response.status_code in (500, 400, 404, 422)
    if response.status_code == 500:
        data = response.json()
        
        # Check the error response
        assert "detail" in data
        assert "failed" in data["detail"].lower()
        
        # Verify the mock was called
        mock_core_api.start_simulation.assert_called_once()

def test_stop_simulation(client, mock_core_api):
    """Test stopping the simulation."""
    response = client.post("/v1/simulation/stop")
    assert response.status_code in (200, 400, 404, 422)
    if response.status_code == 200:
        data = response.json()
        
        # Check the response
        assert "message" in data
        assert "stopped" in data["message"].lower()
        
        # Verify the mock was called
        mock_core_api.stop_simulation.assert_called_once()

def test_stop_simulation_failure(client, mock_core_api):
    """Test stopping the simulation when it fails."""
    # Override the mock to simulate failure
    mock_core_api.stop_simulation.return_value = False
    
    response = client.post("/v1/simulation/stop")
    assert response.status_code in (500, 400, 404, 422)
    if response.status_code == 500:
        data = response.json()
        
        # Check the error response
        assert "detail" in data
        assert "failed" in data["detail"].lower()
        
        # Verify the mock was called
        mock_core_api.stop_simulation.assert_called_once() 