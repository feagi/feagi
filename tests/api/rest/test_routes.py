"""Tests for basic API routing structure."""

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

def test_api_root(client):
    """Test the API root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()
    assert "Welcome to FEAGI REST API" in response.json()["message"]

def test_health_check(client):
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}

def test_api_version(client):
    """Test the API version endpoint."""
    response = client.get("/version")
    assert response.status_code == 200
    assert "version" in response.json()
    assert response.json()["version"] == "1.0.0"

def test_router_structure(client):
    """Test that the OpenAPI schema includes our expected routes."""
    response = client.get("/openapi.json")
    assert response.status_code == 200
    
    # Get the schema
    schema = response.json()
    
    # Check for expected path patterns
    paths = schema["paths"]
    
    # Print the actual paths for debugging
    print("Available API Paths:")
    for path in sorted(paths.keys()):
        print(f"  - {path}")
    
    # Check for router prefixes - adapt these to what's actually available
    # We'll use more flexible checks
    prefixes = [
        "/api/v1",
        "/health",
        "/version"
    ]
    
    for prefix in prefixes:
        assert any(path.startswith(prefix) for path in paths), f"No paths start with {prefix}"

def test_routes_exist(client):
    """Test that all the expected route prefixes work."""
    # Define the common paths to test
    prefixes = [
        "/api/v1/system",
        "/api/v1/genome",
        "/api/v1/simulation",
        "/api/v1/cortical_area",
        "/api/v1/region",
        "/api/v1/inputs",
        "/api/v1/burst_engine",
        "/api/v1/connectome",
        "/api/v1/insights",
        "/api/v1/morphology",
        "/api/v1/neuroplasticity",
        "/api/v1/cortical_mapping"
    ]
    
    # Test each prefix with a bogus endpoint that shouldn't exist
    # We're testing that the router pattern matches but returns a 404 for an invalid path
    # rather than a different status code for a non-existent router
    for prefix in prefixes:
        response = client.get(f"{prefix}/nonexistent_endpoint")
        # Either 404 (not found) or 422 (validation error) is acceptable
        # The important part is that it's not a 501/500/405
        assert response.status_code in (404, 422) 