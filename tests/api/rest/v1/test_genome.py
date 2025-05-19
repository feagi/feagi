def test_essential_genome_upload(client):
    """Verify the v1 upload response maintains backward compatibility"""
    response = client.post("/v1/genome/upload/essential")
    assert response.status_code == 200
    # Test for the legacy format structure
    assert "success" in response.json()
    assert "data" in response.json()
    assert "timestamp" in response.json() 