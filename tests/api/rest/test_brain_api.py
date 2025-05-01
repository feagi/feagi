"""
Tests for the FEAGI Brain REST API endpoints.

These tests verify that the REST API correctly exposes
brain functionality and handles requests properly.
"""

import pytest
import requests
import json
from unittest.mock import patch, MagicMock


@pytest.fixture
def api_base_url():
    """Return the base URL for API tests."""
    return "http://localhost:8080/api/v1"


@pytest.fixture
def mock_server():
    """Set up a mock server for API testing."""
    with patch("feagi.api.rest.server.app") as mock_app:
        # Configure mock responses
        yield mock_app


@pytest.mark.api
def test_get_brain_state(api_base_url, mock_server):
    """Test retrieving the current brain state."""
    # Create a mock response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "neurons": 1000,
        "synapses": 5000,
        "cortical_areas": 10,
        "activity_level": 0.05,
        "timestep": 42
    }
    
    # Set up the mock server to return our response
    mock_server.test_client().get.return_value = mock_response
    
    # Make the API call
    response = requests.get(f"{api_base_url}/brain/state")
    
    # Verify the response
    assert response.status_code == 200
    data = response.json()
    assert data["neurons"] == 1000
    assert data["synapses"] == 5000
    assert data["timestep"] == 42


@pytest.mark.api
def test_stimulate_neurons(api_base_url, mock_server):
    """Test stimulating neurons via the API."""
    # Create stimulation data
    stim_data = {
        "neurons": [1, 2, 3, 4, 5],
        "strength": 1.0,
        "duration": 3
    }
    
    # Create a mock response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "success": True,
        "stimulated": 5
    }
    
    # Set up the mock server
    mock_server.test_client().post.return_value = mock_response
    
    # Make the API call
    response = requests.post(
        f"{api_base_url}/brain/stimulate",
        json=stim_data
    )
    
    # Verify the response
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["stimulated"] == 5


@pytest.mark.api
def test_create_cortical_area(api_base_url, mock_server):
    """Test creating a new cortical area via the API."""
    # Create area data
    area_data = {
        "name": "Test Area",
        "type": "interconnect",
        "dimensions": [10, 10, 5],
        "position": [100, 100, 50]
    }
    
    # Create a mock response
    mock_response = MagicMock()
    mock_response.status_code = 201
    mock_response.json.return_value = {
        "id": 42,
        "name": "Test Area",
        "type": "interconnect",
        "dimensions": [10, 10, 5],
        "position": [100, 100, 50],
        "neurons": 0
    }
    
    # Set up the mock server
    mock_server.test_client().post.return_value = mock_response
    
    # Make the API call
    response = requests.post(
        f"{api_base_url}/brain/areas",
        json=area_data
    )
    
    # Verify the response
    assert response.status_code == 201
    data = response.json()
    assert data["id"] == 42
    assert data["name"] == "Test Area"
    assert data["dimensions"] == [10, 10, 5]


@pytest.mark.api
def test_get_neuron_activity(api_base_url, mock_server):
    """Test retrieving neuron activity via the API."""
    # Create a mock response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "timestep": 42,
        "active_neurons": [10, 20, 30, 40, 50],
        "activity_level": 0.05,
        "areas": {
            "1": {"active": 3, "total": 100},
            "2": {"active": 2, "total": 100}
        }
    }
    
    # Set up the mock server
    mock_server.test_client().get.return_value = mock_response
    
    # Make the API call
    response = requests.get(f"{api_base_url}/brain/activity")
    
    # Verify the response
    assert response.status_code == 200
    data = response.json()
    assert data["timestep"] == 42
    assert len(data["active_neurons"]) == 5
    assert data["areas"]["1"]["active"] == 3


@pytest.mark.api
def test_api_error_handling(api_base_url, mock_server):
    """Test API error handling."""
    # Create a mock error response
    mock_response = MagicMock()
    mock_response.status_code = 404
    mock_response.json.return_value = {
        "error": "Not Found",
        "message": "The requested resource was not found"
    }
    
    # Set up the mock server
    mock_server.test_client().get.return_value = mock_response
    
    # Make the API call to a non-existent endpoint
    response = requests.get(f"{api_base_url}/brain/nonexistent")
    
    # Verify the error response
    assert response.status_code == 404
    data = response.json()
    assert "error" in data
    assert data["error"] == "Not Found" 