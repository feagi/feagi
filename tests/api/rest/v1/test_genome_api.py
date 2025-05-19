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
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "barebones genome loaded" in data["message"].lower()
    
    # Verify the genome has been reset
    response = genome_client.get("/v1/genome/download")
    reset_genome = response.json()
    assert reset_genome["genome_id"] == "reset_genome"
    assert reset_genome["genome_title"] == "Reset Genome"

def test_upload_essential_genome(genome_client):
    """Test uploading the essential genome."""
    response = genome_client.post("/v1/genome/upload/essential")
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "essential genome loaded" in data["message"].lower()
    
    # Verify the genome has been reset
    response = genome_client.get("/v1/genome/download")
    reset_genome = response.json()
    assert reset_genome["genome_id"] == "reset_genome"
    assert reset_genome["genome_title"] == "Reset Genome"

def test_upload_genome_file(genome_client):
    """Test uploading a genome file."""
    # Create a temporary genome file
    genome_data = {
        "genome_id": "test_upload_file",
        "genome_title": "Test Upload File",
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
        
        assert response.status_code == 200
        
        data = response.json()
        assert "message" in data
        assert "genome loaded successfully" in data["message"].lower()
        
        # Verify the genome has been updated
        response = genome_client.get("/v1/genome/download")
        uploaded_genome = response.json()
        assert uploaded_genome["genome_id"] == "test_upload_file"
        assert uploaded_genome["genome_title"] == "Test Upload File"
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
            assert "error" in data or "detail" in data
    finally:
        # Clean up the temporary file
        os.unlink(temp_file_path)

def test_upload_genome_string(genome_client):
    """Test uploading a genome as a string."""
    genome_data = {
        "genome_id": "test_upload_string",
        "genome_title": "Test Upload String",
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
    
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "genome loaded successfully" in data["message"].lower()
    
    # Verify the genome has been updated
    response = genome_client.get("/v1/genome/download")
    uploaded_genome = response.json()
    assert uploaded_genome["genome_id"] == "test_upload_string"
    assert uploaded_genome["genome_title"] == "Test Upload String"

def test_upload_genome_string_invalid(genome_client):
    """Test uploading an invalid genome string."""
    # Missing genome field
    response = genome_client.post("/v1/genome/upload/string", json={})
    assert response.status_code == 400
    
    data = response.json()
    assert "detail" in data
    assert "missing" in data["detail"].lower()
    
    # Invalid JSON
    response = genome_client.post(
        "/v1/genome/upload/string",
        json={"genome": "this is not valid JSON"}
    )
    assert response.status_code == 400
    
    data = response.json()
    assert "detail" in data
    assert "invalid json" in data["detail"].lower()

def test_download_genome(genome_client):
    """Test downloading the current genome."""
    response = genome_client.get("/v1/genome/download")
    assert response.status_code == 200
    
    data = response.json()
    assert "genome_id" in data
    assert "genome_title" in data
    assert "genome_description" in data
    assert "cortical_areas" in data
    
    # Check if we have at least one cortical area
    assert len(data["cortical_areas"]) > 0

def test_download_genome_from_region(genome_client):
    """Test downloading a genome from a specific brain region."""
    response = genome_client.get("/v1/genome/download/region/test_region")
    assert response.status_code == 200
    
    data = response.json()
    assert "genome_id" in data
    assert "genome_title" in data
    assert "genome_description" in data
    assert "cortical_areas" in data

def test_genome_default_files(genome_client):
    """Test listing available default genomes."""
    response = genome_client.get("/v1/genome/default_files")
    assert response.status_code == 200
    
    data = response.json()
    assert "files" in data
    assert isinstance(data["files"], list)
    assert len(data["files"]) > 0
    
    # Check for expected default genomes
    default_genomes = ["barebones_genome.json", "essential_genome.json"]
    for genome in default_genomes:
        assert genome in data["files"]

def test_get_genome_number(genome_client):
    """Test getting the current genome number."""
    response = genome_client.get("/v1/genome/number")
    assert response.status_code == 200
    
    data = response.json()
    assert "genome_number" in data
    assert isinstance(data["genome_number"], int)

def test_reset_genome(genome_client):
    """Test resetting the genome."""
    response = genome_client.post("/v1/genome/reset")
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "reset" in data["message"].lower()
    
    # Verify the genome has been reset
    response = genome_client.get("/v1/genome/download")
    reset_genome = response.json()
    assert reset_genome["genome_id"] == "reset_genome"
    assert reset_genome["genome_title"] == "Reset Genome"

def test_amalgamation_by_payload(genome_client):
    """Test initiating an amalgamation by payload."""
    amalgamation_data = {
        "genome_id": "test_amalgamation",
        "genome_title": "Test Amalgamation",
        "cortical_areas": {
            "new_area": {
                "name": "New Area",
                "type": "sensory",
                "coordinates": {"x": 30, "y": 0, "z": 0},
                "dimensions": {"x": 5, "y": 5, "z": 1}
            }
        }
    }
    
    response = genome_client.post("/v1/genome/amalgamation", json=amalgamation_data)
    assert response.status_code == 200
    
    data = response.json()
    assert "amalgamation_id" in data
    assert "message" in data
    assert "created successfully" in data["message"].lower()

def test_amalgamation_history(genome_client):
    """Test getting amalgamation history."""
    response = genome_client.get("/v1/genome/amalgamation/history")
    assert response.status_code == 200
    
    data = response.json()
    assert "amalgamations" in data
    assert isinstance(data["amalgamations"], dict)
    
    # Check if we have at least one amalgamation
    assert len(data["amalgamations"]) > 0
    
    # Check the structure of the first amalgamation
    first_id = next(iter(data["amalgamations"]))
    amalgamation = data["amalgamations"][first_id]
    assert "id" in amalgamation
    assert "timestamp" in amalgamation
    assert "changes" in amalgamation

def test_cortical_template(genome_client):
    """Test getting cortical templates."""
    response = genome_client.get("/v1/genome/cortical_template")
    assert response.status_code == 200
    
    data = response.json()
    assert "templates" in data
    assert isinstance(data["templates"], list)
    assert len(data["templates"]) > 0
    
    # Check the structure of the first template
    template = data["templates"][0]
    assert "id" in template
    assert "name" in template
    assert "description" in template
    assert "parameters" in template

def test_get_circuit_library(genome_client):
    """Test getting the circuit library."""
    response = genome_client.get("/v1/genome/circuits")
    assert response.status_code == 200
    
    data = response.json()
    assert "circuits" in data
    assert isinstance(data["circuits"], list)
    assert len(data["circuits"]) > 0
    
    # Check the structure of the first circuit
    circuit = data["circuits"][0]
    assert "id" in circuit
    assert "name" in circuit
    assert "description" in circuit
    assert "components" in circuit 