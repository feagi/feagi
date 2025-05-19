"""Tests for the Genome API endpoints - basic functionality."""

import pytest

def test_essential_genome_upload(client):
    """Verify the v1 upload response maintains backward compatibility"""
    response = client.post("/v1/genome/upload/essential")
    assert response.status_code == 200
    
    # Test for the new format structure
    data = response.json()
    assert "message" in data
    assert "essential genome loaded" in data["message"].lower() 