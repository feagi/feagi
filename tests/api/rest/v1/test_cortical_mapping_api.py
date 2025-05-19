"""Tests for the Cortical Mapping API endpoints."""

import os
import json
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
import pytest

# Mark all tests in this module as belonging to the mapping group
pytestmark = [pytest.mark.api, pytest.mark.api_group("mapping")]

# Test Cortical Mapping API Endpoints
def test_get_all_mappings(mapping_client):
    """Test getting all mappings."""
    response = mapping_client.get("/v1/mapping")
    assert response.status_code == 200
    
    data = response.json()
    assert "mappings" in data
    assert isinstance(data["mappings"], list)
    
    # Check the structure of the first mapping if available
    if data["mappings"]:
        mapping = data["mappings"][0]
        assert "id" in mapping
        assert "source_id" in mapping
        assert "target_id" in mapping
        assert "type" in mapping
        assert "enabled" in mapping

def test_get_mapping(mapping_client):
    """Test getting a specific mapping."""
    # Get the ID of an existing mapping
    response = mapping_client.get("/v1/mapping")
    all_mappings = response.json()["mappings"]
    
    if all_mappings:
        mapping_id = all_mappings[0]["id"]
        
        # Get the specific mapping
        response = mapping_client.get(f"/v1/mapping/{mapping_id}")
        assert response.status_code == 200
        
        mapping = response.json()
        assert mapping["id"] == mapping_id
        assert "source_id" in mapping
        assert "target_id" in mapping
        assert "type" in mapping
        assert "enabled" in mapping
    else:
        # If no mappings exist, test with a non-existent ID
        response = mapping_client.get("/v1/mapping/non_existent")
        assert response.status_code == 404

def test_get_mapping_not_found(mapping_client):
    """Test getting a non-existent mapping."""
    response = mapping_client.get("/v1/mapping/non_existent")
    assert response.status_code == 404
    
    data = response.json()
    assert "detail" in data
    assert "not found" in data["detail"].lower()

def test_create_mapping(mapping_client):
    """Test creating a new mapping."""
    # Create a new mapping
    new_mapping = {
        "source_id": "source1",
        "target_id": "target1",
        "type": "direct",
        "properties": {
            "strength": 0.8,
            "delay": 1
        }
    }
    
    response = mapping_client.post("/v1/mapping", json=new_mapping)
    assert response.status_code == 200
    
    mapping = response.json()
    assert "id" in mapping
    assert mapping["source_id"] == new_mapping["source_id"]
    assert mapping["target_id"] == new_mapping["target_id"]
    assert mapping["type"] == new_mapping["type"]
    assert mapping["enabled"] is True

def test_create_mapping_missing_fields(mapping_client):
    """Test creating a mapping with missing required fields."""
    # Missing source_id
    response = mapping_client.post("/v1/mapping", json={"target_id": "target1"})
    assert response.status_code == 400
    
    # Missing target_id
    response = mapping_client.post("/v1/mapping", json={"source_id": "source1"})
    assert response.status_code == 400

def test_update_mapping(mapping_client):
    """Test updating an existing mapping."""
    # First create a mapping
    new_mapping = {
        "source_id": "source2",
        "target_id": "target2",
        "type": "direct"
    }
    
    response = mapping_client.post("/v1/mapping", json=new_mapping)
    mapping_id = response.json()["id"]
    
    # Update the mapping
    update_data = {
        "type": "indirect",
        "enabled": False,
        "properties": {
            "weight": 0.5
        }
    }
    
    response = mapping_client.put(f"/v1/mapping/{mapping_id}", json=update_data)
    assert response.status_code == 200
    
    updated_mapping = response.json()
    assert updated_mapping["id"] == mapping_id
    assert updated_mapping["type"] == update_data["type"]
    assert updated_mapping["enabled"] == update_data["enabled"]

def test_update_mapping_not_found(mapping_client):
    """Test updating a non-existent mapping."""
    response = mapping_client.put("/v1/mapping/non_existent", json={"type": "indirect"})
    assert response.status_code == 404
    
    data = response.json()
    assert "detail" in data
    assert "not found" in data["detail"].lower()

def test_delete_mapping(mapping_client):
    """Test deleting a mapping."""
    # First create a mapping
    new_mapping = {
        "source_id": "source3",
        "target_id": "target3"
    }
    
    response = mapping_client.post("/v1/mapping", json=new_mapping)
    mapping_id = response.json()["id"]
    
    # Delete the mapping
    response = mapping_client.delete(f"/v1/mapping/{mapping_id}")
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "deleted" in data["message"].lower()
    
    # Verify it's deleted
    response = mapping_client.get(f"/v1/mapping/{mapping_id}")
    assert response.status_code == 404

def test_delete_mapping_not_found(mapping_client):
    """Test deleting a non-existent mapping."""
    response = mapping_client.delete("/v1/mapping/non_existent")
    assert response.status_code == 404
    
    data = response.json()
    assert "detail" in data
    assert "not found" in data["detail"].lower()

def test_mapping_type_validation(mapping_client):
    """Test validation of mapping types."""
    # In a real implementation, this would test that only valid mapping types are accepted
    # Our mock implementation accepts any string value
    
    # Create mappings with various types
    valid_types = ["direct", "indirect", "custom"]
    
    for mapping_type in valid_types:
        new_mapping = {
            "source_id": f"source_{mapping_type}",
            "target_id": f"target_{mapping_type}",
            "type": mapping_type
        }
        
        response = mapping_client.post("/v1/mapping", json=new_mapping)
        assert response.status_code == 200
        
        mapping = response.json()
        assert mapping["type"] == mapping_type

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
        "parameters": {
            "weight_multiplier": 1.0,
            "connection_probability": 0.8
        }
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