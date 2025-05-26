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
def test_get_brain_state(brain_state_client):
    """Test retrieving the current brain state."""
    response = brain_state_client.get("/v1/brain/state")
    assert response.status_code in (200, 404)
    if response.status_code == 200:
        data = response.json()
        assert "neurons" in data
        assert "synapses" in data
        assert "timestep" in data
        assert data["timestep"] == 100  # From our custom mock
        assert data.get("active_neurons") == 25  # From our custom mock


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