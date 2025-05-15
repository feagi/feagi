def test_essential_genome_upload_v2(client):
    """Verify the v2 upload uses standardized response format"""
    response = client.post("/v2/genome/upload/essential")
    assert response.status_code == 200
    # Test for the standardized format structure
    assert "success" in response.json()
    assert response.json()["success"] == True
    assert "data" in response.json()
    assert "timestamp" in response.json() 