"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

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