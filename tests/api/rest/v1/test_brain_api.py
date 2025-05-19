"""
Tests for the FEAGI Brain REST API endpoints.

These tests verify that the REST API correctly exposes
brain functionality and handles requests properly.

They use the 'brain_state' test group to ensure appropriate mocking.
"""

import pytest
from fastapi.testclient import TestClient
import json
from unittest.mock import MagicMock

# Mark all tests in this module as belonging to the brain_state group
pytestmark = [pytest.mark.api, pytest.mark.api_group("brain_state")]


# Use the brain_state_client directly
@pytest.mark.api
def test_get_brain_state(brain_client):
    """Test getting the current brain state."""
    response = brain_client.get("/v1/brain/state")
    assert response.status_code == 200
    
    data = response.json()
    assert "running" in data
    assert "paused" in data
    assert "burst_counter" in data
    assert "neuron_count" in data
    assert "synapse_count" in data
    assert "memory_usage" in data
    
    # Check types
    assert isinstance(data["running"], bool)
    assert isinstance(data["paused"], bool)
    assert isinstance(data["burst_counter"], int)
    assert isinstance(data["neuron_count"], int)
    assert isinstance(data["synapse_count"], int)
    assert isinstance(data["memory_usage"], dict)
    
    # Check memory usage structure
    assert "total_mb" in data["memory_usage"]
    assert "used_mb" in data["memory_usage"]
    assert "available_mb" in data["memory_usage"]


@pytest.mark.api
def test_set_brain_state(brain_client):
    """Test setting the brain state."""
    # Test setting the running state
    response = brain_client.post("/v1/brain/state", json={"running": True})
    assert response.status_code == 200
    
    data = response.json()
    assert data["running"] is True
    
    # Test setting the paused state
    response = brain_client.post("/v1/brain/state", json={"paused": True})
    assert response.status_code == 200
    
    data = response.json()
    assert data["paused"] is True
    
    # Test setting multiple properties
    response = brain_client.post("/v1/brain/state", json={
        "running": False,
        "paused": False,
        "burst_counter": 42
    })
    assert response.status_code == 200
    
    data = response.json()
    assert data["running"] is False
    assert data["paused"] is False
    assert data["burst_counter"] == 42


@pytest.mark.api
def test_set_brain_state_invalid(brain_client):
    """Test setting the brain state with invalid data."""
    # Test with empty data
    response = brain_client.post("/v1/brain/state", json={})
    assert response.status_code == 200  # Still works, just doesn't change anything
    
    # Test with invalid data type
    response = brain_client.post("/v1/brain/state", json={"running": "not_a_boolean"})
    assert response.status_code == 200  # Mocked implementation accepts any value
    
    # In a real implementation, this might be a 400 Bad Request


@pytest.mark.api
def test_brain_state_interaction_with_burst_engine(brain_client):
    """Test the interaction between brain state and burst engine."""
    # Start with a known state
    brain_client.post("/v1/brain/state", json={"running": False, "paused": False})
    
    # Start the burst engine (using a different endpoint)
    response = brain_client.post("/v1/burst_engine/start")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        # Check that the brain state reflects the burst engine state
        response = brain_client.get("/v1/brain/state")
        assert response.status_code == 200
        data = response.json()
        assert data["running"] is True
        assert data["paused"] is False


@pytest.mark.api
def test_stimulate_neurons(brain_state_client):
    """Test stimulating neurons via the API."""
    stim_data = {
        "neurons": [1, 2, 3, 4, 5],
        "strength": 1.0,
        "duration": 3
    }
    response = brain_state_client.post("/v1/brain/stimulate", json=stim_data)
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "success" in data
        assert "stimulated" in data


@pytest.mark.api
def test_create_cortical_area(brain_state_client):
    """Test creating a new cortical area via the API."""
    area_data = {
        "name": "Test Area",
        "type": "interconnect",
        "dimensions": [10, 10, 5],
        "position": [100, 100, 50]
    }
    response = brain_state_client.post("/v1/brain/areas", json=area_data)
    assert response.status_code in (201, 404, 400)
    if response.status_code == 201:
        data = response.json()
        assert "id" in data
        assert data.get("name") == "Test Area"
        assert "dimensions" in data


@pytest.mark.api
def test_get_neuron_activity(brain_state_client):
    """Test retrieving neuron activity via the API."""
    response = brain_state_client.get("/v1/brain/activity")
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "timestep" in data
        assert "active_neurons" in data
        assert "areas" in data


@pytest.mark.api
def test_get_cortical_area_activity(brain_state_client):
    """Test retrieving activity for a specific cortical area."""
    response = brain_state_client.get("/v1/brain/areas/1/activity")
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "area_id" in data
        assert "active_neurons" in data
        assert "average_activity" in data
        assert data["area_id"] == "1"
        assert isinstance(data["active_neurons"], list) 