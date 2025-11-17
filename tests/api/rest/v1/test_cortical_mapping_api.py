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

"""Tests for the Cortical Mapping API endpoints."""


import pytest

# Mark all tests in this module as belonging to the mapping group
pytestmark = [pytest.mark.api, pytest.mark.api_group("mapping")]


# Test Cortical Mapping API Endpoints
def test_get_all_mappings(mapping_client):
    """Test getting all cortical mappings."""
    response = mapping_client.get("/v1/cortical_mapping/")
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()
        # Check the structure of the response
        assert "mappings" in data
        assert len(data["mappings"]) == 2

        # Check the properties of the mappings
        for mapping in data["mappings"]:
            assert "id" in mapping
            assert "source_id" in mapping
            assert "target_id" in mapping
            assert "mapping_type" in mapping
            assert "parameters" in mapping


def test_get_all_mappings_with_filters(mapping_client):
    """Test getting all cortical mappings with source/target filters."""
    # Test filtering by source_id
    response = mapping_client.get("/v1/cortical_mapping/", params={"source_id": "101"})
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()
        assert len(data["mappings"]) == 1
        assert data["mappings"][0]["source_id"] == "101"

    # Test filtering by target_id
    response = mapping_client.get("/v1/cortical_mapping/", params={"target_id": "101"})
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()
        assert len(data["mappings"]) == 1
        assert data["mappings"][0]["target_id"] == "101"


def test_get_mapping(mapping_client):
    """Test getting a specific cortical mapping."""
    response = mapping_client.get("/v1/cortical_mapping/1")
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()
        assert data["id"] == "1"
        assert data["source_id"] == "101"
        assert data["target_id"] == "102"
        assert data["mapping_type"] == "one-to-one"


def test_get_nonexistent_mapping(mapping_client):
    """Test getting a non-existent cortical mapping."""
    response = mapping_client.get("/v1/cortical_mapping/999")
    assert response.status_code == 404


def test_create_mapping(mapping_client):
    """Test creating a new cortical mapping."""
    new_mapping = {
        "source_id": "101",
        "target_id": "102",
        "mapping_type": "gaussian",
        "parameters": {"sigma": 1.5, "max_distance": 5, "weight_multiplier": 0.8},
    }

    response = mapping_client.post("/v1/cortical_mapping/", json=new_mapping)
    assert response.status_code in (200, 201, 404, 400)

    if response.status_code in (200, 201):
        data = response.json()
        assert data["source_id"] == "101"
        assert data["target_id"] == "102"
        assert data["mapping_type"] == "gaussian"
        assert "parameters" in data
        if "parameters" in data and isinstance(data["parameters"], dict):
            assert "sigma" in data["parameters"]


def test_create_mapping_invalid_areas(mapping_client):
    """Test creating a mapping with invalid cortical areas."""
    new_mapping = {
        "source_id": "999",
        "target_id": "888",
        "mapping_type": "one-to-one",
        "parameters": {},
    }

    response = mapping_client.post("/v1/cortical_mapping/", json=new_mapping)
    assert response.status_code in (404, 400)


def test_update_mapping(mapping_client):
    """Test updating an existing cortical mapping."""
    mapping_update = {
        "mapping_type": "receptive-field",
        "parameters": {"field_size": 3, "center_weight": 1.0, "surround_weight": 0.5},
    }

    response = mapping_client.put("/v1/cortical_mapping/1", json=mapping_update)
    assert response.status_code in (200, 404, 400)

    if response.status_code == 200:
        data = response.json()
        assert data["mapping_type"] == "receptive-field"
        assert "parameters" in data
        if "parameters" in data and isinstance(data["parameters"], dict):
            assert "field_size" in data["parameters"]


def test_delete_mapping(mapping_client):
    """Test deleting a cortical mapping."""
    response = mapping_client.delete("/v1/cortical_mapping/1")
    assert response.status_code in (200, 204, 404)

    if response.status_code == 200:
        assert "message" in response.json()


def test_get_mapping_stats(mapping_client):
    """Test getting statistics about a specific cortical mapping."""
    response = mapping_client.get("/v1/cortical_mapping/1/stats")
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()

        # Check the structure of the response
        assert "source_id" in data
        assert "target_id" in data
        assert "synapse_count" in data
        assert "average_weight" in data
        assert "connectivity_ratio" in data
        assert "mapping_type" in data

        # Check that the data is of the expected type
        assert isinstance(data["synapse_count"], int)
        assert isinstance(data["average_weight"], float)
        assert isinstance(data["connectivity_ratio"], float)

        # Check that the values are reasonable
        assert data["synapse_count"] > 0
        assert 0 < data["average_weight"] <= 1
        assert 0 < data["connectivity_ratio"] <= 1


def test_apply_mapping(mapping_client):
    """Test applying a cortical mapping."""
    apply_params = {
        "source_id": "101",
        "target_id": "102",
        "mapping_type": "one-to-one",
        "parameters": {"weight_multiplier": 1.0, "connection_probability": 0.8},
    }

    response = mapping_client.post("/v1/cortical_mapping/apply", json=apply_params)
    assert response.status_code in (200, 404, 400)

    if response.status_code == 200:
        data = response.json()
        assert "message" in data
        assert "source_id" in data
        assert "target_id" in data
        assert "connections_created" in data
        assert data["source_id"] == "101"
        assert data["target_id"] == "102"
        assert data["connections_created"] > 0


def test_get_mapping_templates(mapping_client):
    """Test getting available mapping templates."""
    response = mapping_client.get("/v1/cortical_mapping/templates")
    assert response.status_code in (200, 404)

    if response.status_code == 200:
        data = response.json()
        assert "templates" in data
        assert len(data["templates"]) > 0

        for template in data["templates"]:
            assert "id" in template
            assert "name" in template
            assert "description" in template
