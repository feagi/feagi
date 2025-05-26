import pytest

def test_essential_genome_upload(client):
    """Test for essential genome upload endpoint.
    
    This endpoint requires a file on disk that isn't available in tests, so it will
    return a 400 Bad Request with an error message. We just verify we get a proper
    error response.
    """
    # Call the endpoint
    response = client.post("/v1/genome/upload/essential")
    
    # In a test environment without the expected file, we get a 400 status
    assert response.status_code == 400
    
    # Check we get a proper error response
    data = response.json()
    assert "message" in data
    assert "error_code" in data