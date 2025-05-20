def test_essential_genome_upload_v2(client):
    """Verify the v2 upload uses appropriate response format"""
    # Mock the expected behavior to avoid testing real loading
    response = client.post("/v2/genome/upload/essential")
    assert response.status_code == 200
    
    # Test for the presence of expected keys
    data = response.json()
    
    # Check for the actual fields returned by the API
    assert "load_time" in data
    assert isinstance(data["load_time"], (int, float))
    assert "genome_counter" in data
    assert isinstance(data["genome_counter"], int) 