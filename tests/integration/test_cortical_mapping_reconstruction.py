"""
Integration test for cortical mapping reconstruction after dimension changes.

This test verifies that when cortical area dimensions are modified, 
the intelligent routing system properly triggers a full brain rebuild
that reconstructs all cortical mappings.

Critical bug: Currently mappings are lost when dimensions change.
"""

import pytest
import requests
import time
from typing import Dict, Any, List, Optional

# Test configuration
FEAGI_API_BASE = "http://localhost:8000/v1"
TEST_TIMEOUT = 30


class TestCorticalMappingReconstruction:
    """Test cortical mapping reconstruction after structural changes."""
    
    @pytest.fixture(autouse=True)
    def setup_and_cleanup(self):
        """Setup test areas and cleanup after each test."""
        # Ensure a basic genome is loaded
        self._ensure_genome_loaded()
        
        # Use existing areas for testing mapping reconstruction
        self.test_areas = ["_death", "___pwr"]
        
        # Clean up any existing test mappings before and after test
        self._cleanup_test_mappings()
            
        yield
        
        # Clean up test mappings after test (don't delete existing areas)
        self._cleanup_test_mappings()
    
    def _api_request(self, method: str, endpoint: str, data: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
        """Make API request to FEAGI with proper error handling."""
        url = f"{FEAGI_API_BASE}{endpoint}"
        
        try:
            if method == "GET":
                response = requests.get(url, timeout=TEST_TIMEOUT)
            elif method == "POST":
                response = requests.post(url, json=data, timeout=TEST_TIMEOUT)
            elif method == "PUT":
                response = requests.put(url, json=data, timeout=TEST_TIMEOUT)
            elif method == "DELETE":
                response = requests.delete(url, timeout=TEST_TIMEOUT)
            else:
                pytest.fail(f"Unsupported HTTP method: {method}")
                
            if response.status_code in [200, 201]:
                return response.json() if response.content else {}
            elif response.status_code == 404:
                return None  # Not found is expected for some operations
            else:
                pytest.fail(f"API Error {response.status_code}: {response.text}")
                
        except requests.exceptions.RequestException as e:
            pytest.fail(f"Request failed: {e}")
            
    def _delete_area_safe(self, area_id: str) -> None:
        """Safely delete cortical area (ignore if doesn't exist)."""
        try:
            self._api_request("DELETE", f"/cortical_area/cortical_area/{area_id}")
        except:
            pass  # Ignore deletion errors
            
    def _cleanup_test_mappings(self) -> None:
        """Clean up any cortical mappings between test areas."""
        try:
            # Get all mappings
            mappings_result = self._api_request("GET", "/connectome/cortical_areas/list/mappings")
            if mappings_result and "mappings" in mappings_result:
                mappings = mappings_result["mappings"]
                
                # Get list of test area IDs to clean up
                test_area_ids = getattr(self, 'created_area_ids', [])
                
                # Delete mappings between our test areas
                for mapping in mappings:
                    src = mapping.get("src")
                    dst = mapping.get("dst")
                    
                    if (src in test_area_ids and dst in test_area_ids) or \
                       (src in test_area_ids or dst in test_area_ids):
                        try:
                            # Delete this mapping
                            delete_data = {
                                "src_cortical_area": src,
                                "dst_cortical_area": dst
                            }
                            self._api_request("DELETE", "/cortical_area/cortical_mapping", delete_data)
                        except:
                            pass  # Ignore deletion errors
        except:
            pass  # Ignore cleanup errors
            
    def _ensure_genome_loaded(self) -> None:
        """Ensure FEAGI has a genome loaded, create and load a barebones genome if needed."""
        try:
            # Check if genome is loaded by trying to get cortical areas
            result = self._api_request("GET", "/connectome/cortical_areas/list/summary")
            if result is not None and len(result) > 0:
                return  # Genome already loaded
        except:
            pass  # Genome not loaded, continue to load one
            
        print("🧬 Loading barebones genome for testing...")
        
        # Use the correct barebones genome endpoint
        try:
            upload_result = self._api_request("POST", "/genome/upload/barebones")
            if upload_result and upload_result.get("success"):
                print("✅ Barebones genome loaded successfully")
                return
            else:
                print(f"❌ Barebones genome upload failed: {upload_result}")
        except Exception as e:
            print(f"❌ Failed to upload barebones genome: {e}")
            
        # Alternative: Try the essential genome
        try:
            print("🔄 Attempting alternative: Loading essential genome...")
            upload_result = self._api_request("POST", "/genome/upload/essential")
            if upload_result and upload_result.get("success"):
                print("✅ Essential genome loaded successfully")
                return
            else:
                print(f"❌ Essential genome upload failed: {upload_result}")
        except Exception as e:
            print(f"❌ Failed to upload essential genome: {e}")
            
        # Last resort: Upload a minimal genome via string
        try:
            print("🔄 Last resort: Uploading minimal genome via string...")
            minimal_genome = {
                "genome_title": "Minimal Test Genome",
                "genome_description": "Basic genome for integration testing",
                "blueprint": {
                    "base01": {
                        "cortical_id": "base01",
                        "cortical_name": "Base Test Area",
                        "coordinates_3d": [0, 0, 0],
                        "cortical_dimensions": [2, 2, 1],
                        "cortical_type": "sensory",
                        "parameters": {
                            "firing_threshold": 5.0,
                            "refractory_period": 1
                        }
                    }
                },
                "plasticity": {},
                "morphology_rules": {
                    "default_excitatory": {
                        "type": "patterns",
                        "patterns": ["1"]
                    }
                },
                "version": "2.1"
            }
            
            upload_result = self._api_request("POST", "/genome/upload/string", minimal_genome)
            if upload_result and upload_result.get("success"):
                print("✅ Minimal genome loaded successfully via string upload")
                return
            else:
                print(f"❌ Minimal genome upload failed: {upload_result}")
        except Exception as e:
            print(f"❌ Failed to upload minimal genome: {e}")
            
        # If all methods fail, raise an error rather than skip
        pytest.fail("Unable to load any genome for testing. Please ensure FEAGI is properly initialized.")
            
    def _create_test_area(self, area_id: str, dimensions: List[int], coordinates: List[int]) -> Dict[str, Any]:
        """Create a test cortical area with specified dimensions."""
        area_data = {
            "cortical_name": f"Test_Area_{area_id.upper()}",
            "parent_region_id": "root",
            "sub_group_id": "TEST",
            "cortical_dimensions": dimensions,
            "coordinates_3d": coordinates,
            "cortical_visibility": True,
            "cortical_synaptic_attractivity": 100
        }
        
        result = self._api_request("POST", "/cortical_area/custom_cortical_area", area_data)
        assert result is not None, f"Failed to create cortical area {area_id}"
        
        # The API returns the generated cortical_id
        generated_id = result.get("cortical_id")
        if generated_id:
            print(f"Created cortical area with ID: {generated_id}")
            # Track the created ID for cleanup
            if hasattr(self, 'created_area_ids'):
                self.created_area_ids.append(generated_id)
            return {"cortical_id": generated_id}
        else:
            pytest.fail(f"No cortical_id returned when creating area {area_id}")
            
        return result
        
    def _create_cortical_mapping(self, src_area: str, dst_area: str) -> Dict[str, Any]:
        """Create cortical mapping between two areas."""
        mapping_data = {
            "src_cortical_area": src_area,
            "dst_cortical_area": dst_area,
            "mapping_string": "to_to_to_to",  # Simple full connectivity
            "morphology_id": "default_excitatory"
        }
        
        result = self._api_request("POST", "/cortical_area/cortical_mapping", mapping_data)
        assert result is not None, f"Failed to create mapping {src_area} → {dst_area}"
        return result
        
    def _get_mapping_count(self, src_area: str, dst_area: str) -> int:
        """Get count of mappings between two specific areas."""
        mappings_result = self._api_request("GET", "/connectome/cortical_areas/list/mappings")
        assert mappings_result is not None, "Failed to get cortical mappings"
        
        mappings = mappings_result.get("mappings", [])
        return len([m for m in mappings if m.get("src") == src_area and m.get("dst") == dst_area])
        
    def test_dimension_change_preserves_mappings(self):
        """
        Test that changing cortical area dimensions preserves cortical mappings.
        
        CRITICAL BUG TEST: This test currently fails because dimension changes
        do not properly trigger cortical mapping reconstruction.
        """
        # Step 1: Use existing cortical areas from the barebones genome
        # NOTE: Using existing areas due to cortical area creation API issues
        area_a_id, area_b_id = "_death", "___pwr"
        
        print(f"Using existing cortical areas for mapping test: {area_a_id} and {area_b_id}")
        
        # Verify the areas exist
        areas_list = self._api_request("GET", "/connectome/cortical_areas/list/summary")
        assert area_a_id in areas_list, f"Area {area_a_id} not found in genome"
        assert area_b_id in areas_list, f"Area {area_b_id} not found in genome"
        
        # Step 2: Create mapping between areas
        mapping = self._create_cortical_mapping(area_a_id, area_b_id)
        
        # Step 3: Verify initial mapping exists
        initial_mapping_count = self._get_mapping_count(area_a_id, area_b_id)
        assert initial_mapping_count > 0, "Initial cortical mapping not found"
        
        # Step 4: Change dimensions of area B (should trigger reconstruction)
        new_dimensions = [4, 4, 1]  # Changed from [2, 2, 1] to [4, 4, 1]
        
        update_result = self._api_request("PUT", "/cortical_area/cortical_area_properties", {
            "cortical_id": area_b_id,
            "properties": {
                "cortical_dimensions": new_dimensions
            }
        })
        
        assert update_result is not None, "Failed to update cortical area dimensions"
        
        # Step 5: Allow time for reconstruction
        time.sleep(2)
        
        # Step 6: Verify mappings still exist after dimension change
        final_mapping_count = self._get_mapping_count(area_a_id, area_b_id)
        
        # CRITICAL ASSERTION: Mappings should exist after dimension change
        assert final_mapping_count > 0, (
            f"CRITICAL BUG: Cortical mappings lost after dimension change! "
            f"Initial: {initial_mapping_count}, Final: {final_mapping_count}"
        )
        
        # Additional verification: mapping count may change due to new dimensions,
        # but mappings should definitely exist
        print(f"Mapping count: {initial_mapping_count} → {final_mapping_count}")
        
    def test_dimension_change_classification(self):
        """
        Test that dimension changes are properly classified as STRUCTURAL 
        and trigger the full rebuild path.
        """
        # This test would require access to FEAGI logs or internal state
        # For now, we verify the end result (mappings preserved)
        # TODO: Add log parsing or internal state checks
        pass
        
    def test_multiple_dimension_changes(self):
        """
        Test multiple sequential dimension changes to verify robustness.
        """
        # Create areas and mapping
        area_a_id, area_b_id = self.test_areas[0], self.test_areas[1]
        self._create_test_area(area_a_id, [2, 2, 1], [50, 0, 0])
        self._create_test_area(area_b_id, [2, 2, 1], [60, 0, 0])
        self._create_cortical_mapping(area_a_id, area_b_id)
        
        # Verify initial state
        initial_count = self._get_mapping_count(area_a_id, area_b_id)
        assert initial_count > 0
        
        # Multiple dimension changes
        dimension_sequences = [
            [3, 3, 1],
            [4, 2, 1], 
            [2, 4, 1]
        ]
        
        for i, dimensions in enumerate(dimension_sequences):
            update_result = self._api_request("PUT", "/cortical_area/cortical_area_properties", {
                "cortical_id": area_b_id,
                "properties": {"cortical_dimensions": dimensions}
            })
            
            assert update_result is not None, f"Failed dimension change {i+1}"
            time.sleep(1)  # Allow reconstruction
            
            # Verify mappings preserved after each change
            current_count = self._get_mapping_count(area_a_id, area_b_id)
            assert current_count > 0, (
                f"Mappings lost after dimension change {i+1}: {dimensions}"
            )
            
    @pytest.mark.parametrize("structural_change", [
        {"cortical_dimensions": [5, 5, 1]},
        {"coordinates_3d": [70, 0, 0]},
        {"cortical_group": "MOTOR"},
    ])
    def test_structural_changes_preserve_mappings(self, structural_change):
        """
        Test that various structural changes preserve cortical mappings.
        
        This parameterized test covers multiple types of structural changes.
        """
        # Create test setup
        area_a_id, area_b_id = self.test_areas[0], self.test_areas[1]
        self._create_test_area(area_a_id, [3, 3, 1], [50, 0, 0])
        self._create_test_area(area_b_id, [2, 2, 1], [60, 0, 0])
        self._create_cortical_mapping(area_a_id, area_b_id)
        
        # Verify initial mapping
        initial_count = self._get_mapping_count(area_a_id, area_b_id)
        assert initial_count > 0
        
        # Apply structural change
        update_result = self._api_request("PUT", "/cortical_area/cortical_area_properties", {
            "cortical_id": area_b_id,
            "properties": structural_change
        })
        
        assert update_result is not None, f"Failed to apply change: {structural_change}"
        time.sleep(2)  # Allow reconstruction
        
        # Verify mappings preserved
        final_count = self._get_mapping_count(area_a_id, area_b_id)
        assert final_count > 0, (
            f"Mappings lost after structural change: {structural_change}"
        ) 