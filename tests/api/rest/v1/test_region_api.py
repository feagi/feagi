"""Tests for the Region API endpoints."""

import os
import json
import tempfile
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
from fastapi import FastAPI
import pytest
import shutil

from feagi.api.rest.app import create_rest_app

@pytest.fixture
def app():
    """Create a FastAPI app for testing."""
    return create_rest_app()

@pytest.fixture
def client(app):
    """Create a test client for the FastAPI app and load essential genome."""
    from feagi.core.state_manager import FeagiStateManager
    import os
    import json

    # Construct the absolute path to the essential genome file
    genome_path = os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..", "..", "feagi", "evo", "defaults", "genome", "essential_genome.json"
    )
    genome_path = os.path.abspath(genome_path)

    # Load essential genome
    with open(genome_path) as f:
        essential_genome = json.load(f)

    # Inject a minimal valid 'brain_regions' structure if missing
    if "brain_regions" not in essential_genome:
        essential_genome["brain_regions"] = {
            "root": {
                "title": "Root Region",
                "description": "Root region for testing",
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": [],
                "regions": [],
                "inputs": [],
                "outputs": []
            }
        }

    # Set the genome in the global state manager
    state = FeagiStateManager.instance()
    state.genome = essential_genome
    return TestClient(app)

@pytest.fixture
def mock_core_api():
    """Create a mock CoreAPIService."""
    with patch('feagi.api.gateway.APIGateway.core_api', new_callable=MagicMock) as mock:
        # Mock genome data with regions
        mock_genome = {
            "brain_regions": {
                "1": {
                    "name": "Test Region 1",
                    "description": "A test region"
                },
                "2": {
                    "name": "Test Region 2",
                    "description": "Another test region"
                }
            },
            "blueprint": {
                "101": {
                    "name": "Area 1",
                    "region": "1"
                },
                "102": {
                    "name": "Area 2",
                    "region": "1"
                },
                "201": {
                    "name": "Area 3",
                    "region": "2"
                }
            }
        }
        
        # Configure mock return values
        mock.get_genome.return_value = mock_genome
        mock.get_genome_filename.return_value = "test_genome.json"
        
        yield mock

# Test Region API Endpoints
def test_get_all_regions(client, mock_core_api):
    """Test getting all brain regions."""
    response = client.get("/v1/region/regions")
    assert response.status_code == 200
    data = response.json()
    assert "regions" in data
    assert len(data["regions"]) == 2
    assert data["regions"][0]["name"] == "Test Region 1"
    assert data["regions"][1]["name"] == "Test Region 2"
    # Verify cortical areas are included
    assert len(data["regions"][0]["cortical_areas"]) == 2
    assert len(data["regions"][1]["cortical_areas"]) == 1

def test_get_region(client, mock_core_api):
    """Test getting a specific brain region."""
    response = client.get("/v1/region/region", params={"region_id": "1"})
    assert response.status_code == 200
    data = response.json()
    assert data["id"] == "1"
    assert data["name"] == "Test Region 1"
    assert data["description"] == "A test region"
    assert len(data["cortical_areas"]) == 2
    assert "101" in data["cortical_areas"]
    assert "102" in data["cortical_areas"]

def test_get_nonexistent_region(client, mock_core_api):
    """Test getting a non-existent brain region."""
    # Override the mock to simulate no regions
    mock_core_api.get_genome.return_value = {"brain_regions": {}}
    
    response = client.get("/v1/region/region", params={"region_id": "999"})
    assert response.status_code == 404
    assert "not found" in response.json()["detail"].lower()

def test_create_region(client, mock_core_api):
    """Test creating a new brain region."""
    new_region = {
        "name": "New Test Region",
        "description": "A new test region",
        "properties": {
            "custom_prop": "value"
        }
    }
    
    # Setup the mock to return updated genome after adding the region
    def mock_save_genome(genome, filename):
        # Simulate adding the region to the genome
        region_id = "3"  # New ID
        genome["brain_regions"][region_id] = {
            "name": new_region["name"],
            "description": new_region["description"],
            "custom_prop": "value"
        }
        return True
    
    mock_core_api.get_genome.side_effect = [
        # First call: get current genome
        {"brain_regions": {"1": {"name": "Test Region 1"}, "2": {"name": "Test Region 2"}}},
        # Second call: get updated genome with new region
        {"brain_regions": {
            "1": {"name": "Test Region 1"}, 
            "2": {"name": "Test Region 2"},
            "3": {
                "name": "New Test Region",
                "description": "A new test region",
                "custom_prop": "value"
            }
        }}
    ]
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.evo.genome_editor.save_genome', side_effect=mock_save_genome):
        response = client.post("/v1/region/region", json=new_region)
    
    assert response.status_code == 200
    data = response.json()
    assert data["name"] == "New Test Region"
    assert data["description"] == "A new test region"
    assert "custom_prop" in data["properties"]
    assert data["properties"]["custom_prop"] == "value"

def test_update_region(client, mock_core_api):
    """Test updating an existing brain region."""
    region_update = {
        "name": "Updated Region Name",
        "description": "Updated description",
        "properties": {
            "custom_prop": "new value"
        }
    }
    
    # Setup the mock to return updated genome after updating the region
    def mock_save_genome(genome, filename):
        # Simulate updating the region in the genome
        genome["brain_regions"]["1"]["name"] = region_update["name"]
        genome["brain_regions"]["1"]["description"] = region_update["description"]
        genome["brain_regions"]["1"]["custom_prop"] = "new value"
        return True
    
    mock_core_api.get_genome.side_effect = [
        # First call: check if region exists
        {"brain_regions": {"1": {"name": "Test Region 1", "description": "Old description"}}},
        # Second call: get updated genome after the update
        {"brain_regions": {"1": {
            "name": "Updated Region Name",
            "description": "Updated description",
            "custom_prop": "new value"
        }}}
    ]
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.evo.genome_editor.save_genome', side_effect=mock_save_genome):
        response = client.put("/v1/region/region", json=region_update)
    
    assert response.status_code == 200
    data = response.json()
    assert data["name"] == "Updated Region Name"
    assert data["description"] == "Updated description"
    assert data["properties"]["custom_prop"] == "new value"

def test_delete_region(client, mock_core_api):
    """Test deleting a brain region."""
    # Setup the mock to return genome, then validate the region is not in use
    mock_core_api.get_genome.return_value = {
        "brain_regions": {"1": {"name": "Region to Delete"}},
        "blueprint": {}  # No cortical areas using this region
    }
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.evo.genome_editor.save_genome', return_value=True):
        response = client.request("DELETE", "/v1/region/region", json={"id": "1"})
    
    assert response.status_code == 200
    assert "deleted successfully" in response.json()["message"]

def test_delete_region_in_use(client, mock_core_api):
    """Test deleting a brain region that is still in use by cortical areas."""
    # Setup the mock to return genome with the region in use
    mock_core_api.get_genome.return_value = {
        "brain_regions": {"1": {"name": "Region in Use"}},
        "blueprint": {"101": {"name": "Area 1", "region": "1"}}  # Area is using this region
    }
    
    response = client.request("DELETE", "/v1/region/region", json={"id": "1"})
    assert response.status_code == 400
    assert "cannot delete" in response.json()["detail"].lower()

def test_add_cortical_area_to_region(client, mock_core_api):
    """Test adding a cortical area to a brain region."""
    mapping = {
        "cortical_area_id": "301"
    }
    
    # Setup the mock to return valid genome data
    mock_core_api.get_genome.return_value = {
        "brain_regions": {"1": {"name": "Test Region"}},
        "blueprint": {
            "301": {"name": "New Area"}  # Area exists but doesn't have a region assigned
        }
    }
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.evo.genome_editor.save_genome', return_value=True):
        response = client.post("/v1/region/1/cortical_areas", json=mapping)
    
    assert response.status_code == 200
    data = response.json()
    assert data["id"] == "1"
    assert "301" in data["cortical_areas"]

def test_remove_cortical_area_from_region(client, mock_core_api):
    """Test removing a cortical area from a brain region."""
    # Setup the mock to return genome with area assigned to region
    mock_core_api.get_genome.return_value = {
        "brain_regions": {"1": {"name": "Test Region"}},
        "blueprint": {
            "101": {"name": "Area 1", "region": "1"}
        }
    }
    
    # Patch the save_genome function used in the endpoint
    with patch('feagi.evo.genome_editor.save_genome', return_value=True):
        response = client.delete("/v1/region/1/cortical_areas/101")
    
    assert response.status_code == 200
    assert "removed from region" in response.json()["message"] 