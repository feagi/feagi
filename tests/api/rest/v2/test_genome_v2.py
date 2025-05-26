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