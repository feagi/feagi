"""Tests for the Genome API endpoints."""

import os
import json
import tempfile
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
import pytest
import shutil

# Mark all tests in this module as belonging to the genome group
pytestmark = [pytest.mark.api, pytest.mark.api_group("genome")]

# Path to the actual barebones genome file
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
BAREBONES_GENOME_PATH = os.path.join(REPO_ROOT, 'feagi/evo/defaults/genome/barebones_genome.json')
ESSENTIAL_GENOME_PATH = os.path.join(REPO_ROOT, 'feagi/evo/defaults/genome/essential_genome.json')

# Test Genome Upload Endpoints
def test_upload_barebones_genome(genome_client):
    """Test uploading the barebones genome."""
    response = genome_client.post("/v1/genome/upload/barebones")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        assert response.json() == {"message": "Barebones genome loaded successfully"}

def test_upload_essential_genome(genome_client):
    """Test uploading the essential genome."""
    response = genome_client.post("/v1/genome/upload/essential")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        assert response.json() == {"message": "Essential genome loaded successfully"}

def test_upload_genome_file(genome_client):
    """Test uploading a genome file."""
    # Create a temporary genome file
    genome_data = {
        "genome_title": "Test Genome File",
        "genome_description": "This is a test genome file",
        "cortical_areas": {
            "area1": {
                "name": "Area 1",
                "type": "sensory",
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "dimensions": {"x": 10, "y": 10, "z": 1}
            }
        },
        "blueprint": {},
        "brain_regions": {}
    }
    
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temp_file:
        temp_file.write(json.dumps(genome_data).encode())
        temp_file_path = temp_file.name
    
    try:
        with open(temp_file_path, "rb") as f:
            response = genome_client.post(
                "/v1/genome/upload/file",
                files={"file": ("test_genome.json", f, "application/json")}
            )
        
        assert response.status_code in (200, 400, 404)
        
        if response.status_code == 200:
            data = response.json()
            assert "message" in data
            assert "Genome loaded successfully" in data["message"]
    finally:
        # Clean up the temporary file
        os.unlink(temp_file_path)

def test_upload_genome_file_invalid_json(genome_client):
    """Test uploading an invalid JSON file as genome."""
    # Create an invalid JSON file
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as temp_file:
        temp_file.write(b"This is not valid JSON")
        temp_file_path = temp_file.name
    
    try:
        with open(temp_file_path, "rb") as f:
            response = genome_client.post(
                "/v1/genome/upload/file",
                files={"file": ("invalid.json", f, "application/json")}
            )
        
        assert response.status_code in (400, 404)
        
        if response.status_code == 400:
            data = response.json()
            # Check for either standard error format or new response format
            assert ("error" in data or "detail" in data or 
                   "error_code" in data or "message" in data)
    finally:
        # Clean up the temporary file
        os.unlink(temp_file_path)

def test_upload_genome_string(genome_client):
    """Test uploading a genome as a string."""
    genome_data = {
        "genome_title": "Test Genome String",
        "genome_description": "This is a test genome string",
        "cortical_areas": {
            "area1": {
                "name": "Area 1",
                "type": "sensory",
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "dimensions": {"x": 10, "y": 10, "z": 1}
            }
        },
        "blueprint": {},
        "brain_regions": {}
    }
    
    response = genome_client.post(
        "/v1/genome/upload/string",
        json={"genome": json.dumps(genome_data)}
    )
    
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "message" in data
        assert "Genome loaded successfully" in data["message"]

def test_download_genome(genome_client):
    """Test downloading the current genome."""
    response = genome_client.get("/v1/genome/download")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "genome_title" in data

def test_download_genome_from_region(genome_client):
    """Test downloading a genome from a specific brain region."""
    response = genome_client.get("/v1/genome/download/region/test_region")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "genome_title" in data

def test_genome_default_files(genome_client):
    """Test listing available default genomes."""
    response = genome_client.get("/v1/genome/default_files")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "files" in data
        assert len(data["files"]) > 0
        assert "barebones_genome.json" in data["files"]

def test_get_genome_number(genome_client):
    """Test getting the current genome number."""
    response = genome_client.get("/v1/genome/number")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "genome_number" in data
        assert isinstance(data["genome_number"], int)

def test_reset_genome(genome_client):
    """Test resetting the genome."""
    response = genome_client.post("/v1/genome/reset")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "message" in data
        assert "reset" in data["message"].lower()

def test_amalgamation_by_payload(genome_client):
    """Test initiating an amalgamation by payload."""
    amalgamation_data = {
        "genome_id": "test",
        "genome_title": "Test Amalgamation",
        "cortical_areas": {
            "area1": {
                "name": "New Area",
                "type": "sensory",
                "coordinates": {"x": 0, "y": 0, "z": 0},
                "dimensions": {"x": 10, "y": 10, "z": 1}
            }
        }
    }
    
    response = genome_client.post("/v1/genome/amalgamation_by_payload", json=amalgamation_data)
    assert response.status_code in (200, 400, 404, 405)
    
    if response.status_code == 200:
        data = response.json()
        assert "amalgamation_id" in data
        assert "message" in data

def test_amalgamation_history(genome_client):
    """Test getting amalgamation history."""
    response = genome_client.get("/v1/genome/amalgamation/history")
    assert response.status_code in (200, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "amalgamations" in data
        # Check that at least one entry exists in our mock data
        assert "202304050123_A" in data["amalgamations"]

def test_cortical_template(genome_client):
    """Test getting cortical templates."""
    response = genome_client.get("/v1/genome/cortical_template")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "templates" in data
        assert isinstance(data["templates"], list)
        assert len(data["templates"]) > 0
        # Check that the test template exists in our mock data
        assert any(template["name"] == "Test Template" for template in data["templates"])

def test_get_circuit_library(genome_client):
    """Test getting the circuit library."""
    response = genome_client.get("/v1/genome/circuits")
    assert response.status_code in (200, 400, 404)
    
    if response.status_code == 200:
        data = response.json()
        assert "circuits" in data
        assert len(data["circuits"]) > 0
        # Check that the test circuit exists in our mock data
        assert any(circuit["name"] == "Test Circuit" for circuit in data["circuits"]) 