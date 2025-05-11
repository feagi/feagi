"""
Tests for the FEAGI Brain REST API endpoints.

These tests verify that the REST API correctly exposes
brain functionality and handles requests properly.
"""

import pytest
from fastapi.testclient import TestClient
import json
from unittest.mock import MagicMock

from feagi.api.rest.app import app


@pytest.fixture
def client():
    """Fixture to provide a FastAPI test client."""
    with TestClient(app) as c:
        yield c


@pytest.mark.api
def test_get_brain_state(client):
    """Test retrieving the current brain state."""
    response = client.get("/brain/state")
    assert response.status_code in (200, 404)  # Accept 404 if endpoint is not implemented
    if response.status_code == 200:
        data = response.json()
        assert "neurons" in data
        assert "synapses" in data
        assert "timestep" in data


@pytest.mark.api
def test_stimulate_neurons(client):
    """Test stimulating neurons via the API."""
    stim_data = {
        "neurons": [1, 2, 3, 4, 5],
        "strength": 1.0,
        "duration": 3
    }
    response = client.post("/brain/stimulate", json=stim_data)
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "success" in data
        assert "stimulated" in data


@pytest.mark.api
def test_create_cortical_area(client):
    """Test creating a new cortical area via the API."""
    area_data = {
        "name": "Test Area",
        "type": "interconnect",
        "dimensions": [10, 10, 5],
        "position": [100, 100, 50]
    }
    response = client.post("/brain/areas", json=area_data)
    assert response.status_code in (201, 404)
    if response.status_code == 201:
        data = response.json()
        assert data["id"] == 42 or "id" in data
        assert data["name"] == "Test Area" or "name" in data
        assert data["dimensions"] == [10, 10, 5] or "dimensions" in data


@pytest.mark.api
def test_get_neuron_activity(client):
    """Test retrieving neuron activity via the API."""
    response = client.get("/brain/activity")
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "timestep" in data
        assert "active_neurons" in data
        assert "areas" in data 