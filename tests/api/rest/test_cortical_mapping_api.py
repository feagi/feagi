"""Tests for the Cortical Mapping API endpoints."""

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
        # Mock get_cortical_areas to return test areas
        mock.get_cortical_areas.return_value = [
            {
                "id": "101",
                "name": "Visual Cortex",
                "type": "sensory",
                "dimensions": {"width": 20, "height": 20, "depth": 5},
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "parameters": {},
                "neuron_count": 2000
            },
            {
                "id": "102",
                "name": "Motor Cortex",
                "type": "motor",
                "dimensions": {"width": 10, "height": 10, "depth": 5},
                "coordinates": {"x": 30, "y": 0, "z": 0},
                "parameters": {},
                "neuron_count": 500
            }
        ]
        
        # Mock get_genome to return a genome with connectivity mappings
        mock.get_genome.return_value = {
            "connectivity": {
                "1": {
                    "source_id": "101",
                    "target_id": "102",
                    "mapping_type": "one-to-one",
                    "weight_multiplier": 1.0,
                    "connection_probability": 0.8
                },
                "2": {
                    "source_id": "102",
                    "target_id": "101",
                    "mapping_type": "probabilistic",
                    "weight_multiplier": 0.5,
                    "connection_probability": 0.3
                }
            }
        }
        
        # Mock get_genome_filename for save operations
        mock.get_genome_filename.return_value = "test_genome.json"
        
        yield mock

# Test Cortical Mapping API Endpoints
def test_get_all_mappings(client, mock_core_api):
    """Test getting all cortical mappings."""
    response = client.get("/api/v1/cortical_mapping/")
    assert response.status_code == 200
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

def test_get_all_mappings_with_filters(client, mock_core_api):
    """Test getting all cortical mappings with source/target filters."""
    # Test filtering by source_id
    response = client.get("/api/v1/cortical_mapping/", params={"source_id": "101"})
    assert response.status_code == 200
    data = response.json()
    
    assert len(data["mappings"]) == 1
    assert data["mappings"][0]["source_id"] == "101"
    
    # Test filtering by target_id
    response = client.get("/api/v1/cortical_mapping/", params={"target_id": "101"})
    assert response.status_code == 200
    data = response.json()
    
    assert len(data["mappings"]) == 1
    assert data["mappings"][0]["target_id"] == "101"

def test_get_mapping(client, mock_core_api):
    """Test getting a specific cortical mapping."""
    response = client.get("/api/v1/cortical_mapping/1")
    assert response.status_code == 200
    data = response.json()
    
    assert data["id"] == "1"
    assert data["source_id"] == "101"
    assert data["target_id"] == "102"
    assert data["mapping_type"] == "one-to-one"

def test_get_nonexistent_mapping(client, mock_core_api):
    """Test getting a non-existent cortical mapping."""
    # Override mock to return a genome without the requested mapping
    mock_core_api.get_genome.return_value = {"connectivity": {}}
    
    response = client.get("/api/v1/cortical_mapping/999")
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()

def test_create_mapping(client, mock_core_api):
    """Test creating a new cortical mapping."""
    new_mapping = {
        "source_id": "101",
        "target_id": "102",
        "mapping_type": "gaussian",
        "parameters": {
            "sigma": 1.5,
            "max_distance": 5,
            "weight_multiplier": 0.8
        }
    }
    
    # Setup the mock to return updated genome after adding the mapping
    def mock_save_genome(genome, filename):
        # Simulate adding the mapping to the genome
        mapping_id = "3"  # New ID
        genome["connectivity"][mapping_id] = {
            "source_id": new_mapping["source_id"],
            "target_id": new_mapping["target_id"],
            "mapping_type": new_mapping["mapping_type"],
            "sigma": 1.5,
            "max_distance": 5,
            "weight_multiplier": 0.8
        }
        return True
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.api.rest.routers.v1.cortical_mapping.save_genome', side_effect=mock_save_genome):
        response = client.post("/api/v1/cortical_mapping/", json=new_mapping)
    
    assert response.status_code == 200
    data = response.json()
    assert data["source_id"] == "101"
    assert data["target_id"] == "102"
    assert data["mapping_type"] == "gaussian"
    assert "sigma" in data["parameters"]
    assert data["parameters"]["sigma"] == 1.5

def test_create_mapping_invalid_areas(client, mock_core_api):
    """Test creating a mapping with invalid cortical areas."""
    # Override mock to return an empty list of areas
    mock_core_api.get_cortical_areas.return_value = []
    
    new_mapping = {
        "source_id": "999",
        "target_id": "888",
        "mapping_type": "one-to-one",
        "parameters": {}
    }
    
    response = client.post("/api/v1/cortical_mapping/", json=new_mapping)
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()

def test_update_mapping(client, mock_core_api):
    """Test updating an existing cortical mapping."""
    mapping_update = {
        "mapping_type": "receptive-field",
        "parameters": {
            "field_size": 3,
            "center_weight": 1.0,
            "surround_weight": 0.5
        }
    }
    
    # Setup the mock to return updated genome after updating the mapping
    def mock_save_genome(genome, filename):
        # Simulate updating the mapping in the genome
        genome["connectivity"]["1"]["mapping_type"] = mapping_update["mapping_type"]
        genome["connectivity"]["1"]["field_size"] = 3
        genome["connectivity"]["1"]["center_weight"] = 1.0
        genome["connectivity"]["1"]["surround_weight"] = 0.5
        return True
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.api.rest.routers.v1.cortical_mapping.save_genome', side_effect=mock_save_genome):
        response = client.put("/api/v1/cortical_mapping/1", json=mapping_update)
    
    assert response.status_code == 200
    data = response.json()
    assert data["mapping_type"] == "receptive-field"
    assert "field_size" in data["parameters"]
    assert data["parameters"]["field_size"] == 3

def test_delete_mapping(client, mock_core_api):
    """Test deleting a cortical mapping."""
    # Setup the mock to return updated genome after deleting the mapping
    def mock_save_genome(genome, filename):
        # Simulate deleting the mapping from the genome
        if "1" in genome["connectivity"]:
            del genome["connectivity"]["1"]
        return True
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.api.rest.routers.v1.cortical_mapping.save_genome', side_effect=mock_save_genome):
        response = client.delete("/api/v1/cortical_mapping/1")
    
    assert response.status_code == 200
    assert "deleted successfully" in response.json()["message"]

def test_get_mapping_stats(client, mock_core_api):
    """Test getting statistics about a specific cortical mapping."""
    response = client.get("/api/v1/cortical_mapping/1/stats")
    assert response.status_code == 200
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

def test_apply_mapping(client, mock_core_api):
    """Test applying a cortical mapping to generate connections."""
    response = client.post("/api/v1/cortical_mapping/1/apply")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "message" in data
    assert "source_id" in data
    assert "target_id" in data
    assert "mapping_type" in data
    assert "connections_created" in data
    
    # Check that the response indicates successful application
    assert "applied successfully" in data["message"]
    assert data["connections_created"] > 0

def test_get_mapping_templates(client, mock_core_api):
    """Test getting available cortical mapping templates."""
    response = client.post("/api/v1/cortical_mapping/templates")
    assert response.status_code == 200
    data = response.json()
    
    # Check the structure of the response
    assert "templates" in data
    assert len(data["templates"]) > 0
    
    # Check that each template has the expected fields
    for template in data["templates"]:
        assert "id" in template
        assert "name" in template
        assert "description" in template
        assert "parameters" in template 