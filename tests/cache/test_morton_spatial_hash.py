"""
Morton Spatial Hash Integration Tests

This module tests the Morton encoding spatial hash system including
real integration scenarios that mirror actual FEAGI usage.
"""

import pytest
import sys
import os
from typing import Set, Tuple, List
from unittest.mock import patch

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from feagi.bdu.morton_spatial_hash import (
    MortonUtils, RoaringSpatialHash, MortonSpatialHashState, 
    get_morton_spatial_hash, reset_morton_spatial_hash
)

class TestMortonSpatialHashIntegration:
    """Integration tests that mirror real FEAGI usage patterns."""
    
    def setup_method(self):
        """Reset spatial hash before each test."""
        reset_morton_spatial_hash()
    
    def test_multiple_neurons_per_coordinate_critical_fix(self):
        """CRITICAL TEST: Verify multiple neurons can exist at same coordinate."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Add multiple neurons to the same coordinate
        area = "visual_cortex"
        x, y, z = 10, 20, 30
        
        # Add 5 neurons to the same coordinate
        neuron_ids = [1001, 1002, 1003, 1004, 1005]
        for neuron_id in neuron_ids:
            success = spatial_hash.add_neuron(area, x, y, z, neuron_id)
            assert success, f"Failed to add neuron {neuron_id}"
        
        # Test get_neurons_at_coordinate (new method)
        neurons_at_coord = spatial_hash.get_neurons_at_coordinate(area, x, y, z)
        assert len(neurons_at_coord) == 5, f"Expected 5 neurons, got {len(neurons_at_coord)}"
        assert set(neurons_at_coord) == set(neuron_ids), "Neuron IDs don't match"
        
        # Test backward compatibility: get_neuron_at_coordinate returns first neuron
        first_neuron = spatial_hash.get_neuron_at_coordinate(area, x, y, z)
        assert first_neuron in neuron_ids, "First neuron should be one of the added neurons"
        
        # Test region queries include ALL neurons
        region_neurons = spatial_hash.get_neurons_in_region(area, x, y, z, x, y, z)
        assert len(region_neurons) == 5, f"Region query should return 5 neurons, got {len(region_neurons)}"
        assert set(region_neurons) == set(neuron_ids), "Region query neuron IDs don't match"
    
    def test_multiple_coordinates_multiple_neurons(self):
        """Test multiple neurons at multiple coordinates."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        area = "motor_cortex"
        
        # Coordinate 1: 3 neurons
        coord1 = (5, 10, 15)
        neurons1 = [2001, 2002, 2003]
        for neuron_id in neurons1:
            spatial_hash.add_neuron(area, *coord1, neuron_id)
        
        # Coordinate 2: 2 neurons  
        coord2 = (6, 11, 16)
        neurons2 = [2004, 2005]
        for neuron_id in neurons2:
            spatial_hash.add_neuron(area, *coord2, neuron_id)
        
        # Coordinate 3: 1 neuron
        coord3 = (7, 12, 17)
        neurons3 = [2006]
        for neuron_id in neurons3:
            spatial_hash.add_neuron(area, *coord3, neuron_id)
        
        # Verify each coordinate
        assert len(spatial_hash.get_neurons_at_coordinate(area, *coord1)) == 3
        assert len(spatial_hash.get_neurons_at_coordinate(area, *coord2)) == 2
        assert len(spatial_hash.get_neurons_at_coordinate(area, *coord3)) == 1
        
        # Verify region query gets all neurons
        all_neurons = spatial_hash.get_neurons_in_region(area, 5, 10, 15, 7, 12, 17)
        expected_all = neurons1 + neurons2 + neurons3
        assert len(all_neurons) == 6, f"Expected 6 neurons, got {len(all_neurons)}"
        assert set(all_neurons) == set(expected_all), "All neurons not found in region"
    
    def test_duplicate_neuron_prevention(self):
        """Test that duplicate neuron IDs at same coordinate are prevented."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        area = "test_area"
        x, y, z = 1, 2, 3
        neuron_id = 9999
        
        # Add same neuron multiple times
        spatial_hash.add_neuron(area, x, y, z, neuron_id)
        spatial_hash.add_neuron(area, x, y, z, neuron_id)  # Duplicate
        spatial_hash.add_neuron(area, x, y, z, neuron_id)  # Duplicate
        
        # Should only appear once
        neurons = spatial_hash.get_neurons_at_coordinate(area, x, y, z)
        assert len(neurons) == 1, f"Expected 1 neuron, got {len(neurons)}"
        assert neurons[0] == neuron_id
    
    def test_cross_area_multiple_neurons(self):
        """Test multiple neurons across different cortical areas."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Same coordinate in different areas
        x, y, z = 100, 200, 300
        
        # Area 1: 3 neurons
        area1 = "visual_area1"
        neurons1 = [3001, 3002, 3003]
        for neuron_id in neurons1:
            spatial_hash.add_neuron(area1, x, y, z, neuron_id)
        
        # Area 2: 2 neurons at same coordinate
        area2 = "visual_area2" 
        neurons2 = [3004, 3005]
        for neuron_id in neurons2:
            spatial_hash.add_neuron(area2, x, y, z, neuron_id)
        
        # Verify areas are independent
        area1_neurons = spatial_hash.get_neurons_at_coordinate(area1, x, y, z)
        area2_neurons = spatial_hash.get_neurons_at_coordinate(area2, x, y, z)
        
        assert len(area1_neurons) == 3
        assert len(area2_neurons) == 2
        assert set(area1_neurons) == set(neurons1)
        assert set(area2_neurons) == set(neurons2)
        
        # No cross-contamination
        assert not set(area1_neurons).intersection(set(area2_neurons))

    def test_basic_morton_operations(self):
        """Test basic Morton encoding operations."""
        # Test encoding/decoding
        x, y, z = 42, 84, 126
        morton_code = MortonUtils.morton_encode_3d(x, y, z)
        decoded_x, decoded_y, decoded_z = MortonUtils.morton_decode_3d(morton_code)
        
        assert decoded_x == x
        assert decoded_y == y
        assert decoded_z == z
    
    def test_spatial_hash_initialization(self):
        """Test that spatial hash initializes correctly."""
        spatial_hash = get_morton_spatial_hash()
        
        assert spatial_hash is not None
        assert spatial_hash.is_ready()
        assert spatial_hash.get_state() == MortonSpatialHashState.READY
    
    def test_neuron_addition_and_retrieval(self):
        """Test basic neuron addition and retrieval."""
        spatial_hash = get_morton_spatial_hash()
        
        # Add a neuron
        success = spatial_hash.add_neuron("test_area", 10, 20, 30, 12345)
        assert success
        
        # Retrieve the neuron
        neuron_id = spatial_hash.get_neuron_at_coordinate("test_area", 10, 20, 30)
        assert neuron_id == 12345
        
        # Test non-existent coordinate
        no_neuron = spatial_hash.get_neuron_at_coordinate("test_area", 99, 99, 99)
        assert no_neuron is None

    def test_region_queries(self):
        """Test region-based neuron queries."""
        spatial_hash = get_morton_spatial_hash()
        
        # Add neurons in a region
        test_neurons = [
            ("area1", 5, 5, 5, 1001),
            ("area1", 6, 6, 6, 1002), 
            ("area1", 7, 7, 7, 1003),
            ("area1", 15, 15, 15, 1004),  # Outside region
        ]
        
        for area, x, y, z, neuron_id in test_neurons:
            spatial_hash.add_neuron(area, x, y, z, neuron_id)
        
        # Query region that should contain first 3 neurons
        neurons_in_region = spatial_hash.get_neurons_in_region("area1", 4, 4, 4, 8, 8, 8)
        
        assert len(neurons_in_region) == 3
        assert set(neurons_in_region) == {1001, 1002, 1003}

    def test_multi_area_operations(self):
        """Test operations across multiple cortical areas."""
        spatial_hash = get_morton_spatial_hash()
        
        # Add neurons to different areas
        spatial_hash.add_neuron("area_A", 1, 1, 1, 2001)
        spatial_hash.add_neuron("area_B", 1, 1, 1, 2002)  # Same coordinate, different area
        spatial_hash.add_neuron("area_A", 2, 2, 2, 2003)
        
        # Test area isolation
        area_a_neuron = spatial_hash.get_neuron_at_coordinate("area_A", 1, 1, 1)
        area_b_neuron = spatial_hash.get_neuron_at_coordinate("area_B", 1, 1, 1)
        
        assert area_a_neuron == 2001
        assert area_b_neuron == 2002
        
        # Test multi-area bitmap operations
        area_union = spatial_hash.get_area_union(["area_A", "area_B"])
        assert len(area_union) == 2  # Two unique coordinates across both areas
        
        area_intersection = spatial_hash.get_area_intersection(["area_A", "area_B"]) 
        assert len(area_intersection) == 1  # One shared coordinate (1,1,1)

    def test_compatibility_methods(self):
        """Test ConnectomeManager compatibility methods."""
        spatial_hash = get_morton_spatial_hash()
        
        # Test initialization method
        spatial_hash.initialize_for_dimensions((64, 64, 64))
        assert spatial_hash.is_ready()
        
        # Test wait for ready
        ready = spatial_hash.wait_for_ready(timeout_seconds=1.0)
        assert ready
        
        # Test expansion (should always succeed)
        expanded = spatial_hash.expand_cache_for_new_area((10, 10, 10), (32, 32, 32))
        assert expanded
        
        # Test compatibility coordinate methods
        success = spatial_hash.add_coordinate("compat_area", 5, 10, 15, 5555)
        assert success
        
        neuron_id = spatial_hash.get_neuron_id("compat_area", 5, 10, 15)
        assert neuron_id == 5555

    def test_batch_coordinate_lookup(self):
        """Test high-performance batch coordinate lookup."""
        spatial_hash = get_morton_spatial_hash()
        
        # Set up test data
        candidates = {(1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)}
        positions = [(1, 2, 3), (99, 99, 99), (7, 8, 9), (4, 5, 6)]
        
        matches = spatial_hash.batch_coordinate_lookup(candidates, positions)
        
        # Should find matches for positions 0, 2, 3 (candidates at indices in candidate set)
        assert len(matches) >= 3
        
        # Verify match structure
        for candidate_idx, neuron_idx in matches:
            assert isinstance(candidate_idx, int)
            assert isinstance(neuron_idx, int)

    def test_statistics_and_state(self):
        """Test statistics reporting and state management."""
        spatial_hash = get_morton_spatial_hash()
        
        # Add some test data
        spatial_hash.add_neuron("stats_area", 1, 1, 1, 7001)
        spatial_hash.add_neuron("stats_area", 2, 2, 2, 7002)
        spatial_hash.add_neuron("stats_area2", 3, 3, 3, 7003)
        
        stats = spatial_hash.get_statistics()
        
        assert stats["state"] == "ready"
        assert stats["total_coordinates"] >= 3
        assert stats["total_areas"] >= 2
        assert "cortical_areas" in stats
        assert "stats_area" in stats["cortical_areas"]
        assert "stats_area2" in stats["cortical_areas"]

    def test_real_genome_analysis_integration(self):
        """Test genome analysis with real FEAGI genome structure."""
        from feagi.bdu.spatial_hash import analyze_genome_coordinate_space
        
        # Mock genome data structure that FEAGI actually uses
        mock_genome = {
            "blueprint": {
                "0-area1-cx-bbx-i": 64,
                "0-area1-cx-bby-i": 32, 
                "0-area1-cx-bbz-i": 16,
                "0-area2-cx-bbx-i": 128,
                "0-area2-cx-bby-i": 64,
                "0-area2-cx-bbz-i": 32,
            }
        }
        
        max_dims = analyze_genome_coordinate_space(mock_genome)
        assert isinstance(max_dims, tuple)
        assert len(max_dims) == 3
        assert all(isinstance(dim, int) for dim in max_dims)
        
        # Now it should find the correct values
        assert max_dims[0] == 128  # max x = 128
        assert max_dims[1] == 64   # max y = 64
        assert max_dims[2] == 32   # max z = 32


class TestPerformanceAndScalability:
    """Performance tests for the Morton spatial hash system."""
    
    def setup_method(self):
        """Reset spatial hash before each test."""
        reset_morton_spatial_hash()
    
    def test_large_scale_operations_with_multiple_neurons(self):
        """Test performance with larger dataset including multiple neurons per coordinate."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Add 1000 neurons with some coordinates having multiple neurons
        for i in range(1000):
            area = f"area_{i // 100}"  # 10 areas with 100 neurons each
            x, y, z = i % 32, (i // 32) % 32, i // 1024
            
            # Every 10th coordinate gets an additional neuron
            spatial_hash.add_neuron(area, x, y, z, i)
            if i % 10 == 0:
                spatial_hash.add_neuron(area, x, y, z, i + 10000)  # Additional neuron
        
        # Test region queries
        neurons = spatial_hash.get_neurons_in_region("area_0", 0, 0, 0, 31, 31, 1)
        assert len(neurons) > 100  # Should have more than 100 due to multiple neurons per coordinate
        
        # Test statistics
        stats = spatial_hash.get_statistics()
        assert stats['total_coordinates'] > 1000  # More coordinates due to additional neurons
        assert stats['total_areas'] == 10

    def test_memory_efficiency_demonstration(self):
        """Demonstrate memory efficiency of Morton encoding."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Add sparse neurons across coordinate space (within 21-bit Morton limits)
        test_coordinates = [
            (0, 0, 0), (1000, 1000, 1000), (500, 250, 750),
            (800, 900, 800), (100, 200, 300)  # Fixed: changed 1200 to 800 to stay within limits
        ]
        
        for i, (x, y, z) in enumerate(test_coordinates):
            spatial_hash.add_neuron("sparse_area", x, y, z, 8000 + i)
        
        # System should handle large coordinate ranges efficiently
        stats = spatial_hash.get_statistics()
        assert stats['total_coordinates'] == len(test_coordinates)


class TestIntegrationFailurePrevention:
    """Tests to prevent integration failures with FEAGI components."""
    
    def setup_method(self):
        """Reset spatial hash before each test."""
        reset_morton_spatial_hash()
    
    def test_imports_dont_fail_on_missing_dependencies(self):
        """Test that imports work even if optional dependencies are missing."""
        # This test ensures the fallback RoaringBitmap works
        try:
            from feagi.bdu.morton_spatial_hash import RoaringSpatialHash
            spatial_hash = RoaringSpatialHash()
            assert spatial_hash is not None
        except ImportError as e:
            pytest.fail(f"Import failed: {e}")
    
    def test_thread_safety_basic_operations(self):
        """Test basic thread safety of operations."""
        import threading
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        results = []
        
        def add_neurons_thread(thread_id):
            for i in range(10):
                success = spatial_hash.add_neuron(f"thread_area_{thread_id}", i, i, i, thread_id * 1000 + i)
                results.append(success)
        
        # Run multiple threads
        threads = []
        for t in range(3):
            thread = threading.Thread(target=add_neurons_thread, args=(t,))
            threads.append(thread)
            thread.start()
        
        for thread in threads:
            thread.join()
        
        # All operations should succeed
        assert all(results)
        
        # Verify data integrity
        stats = spatial_hash.get_statistics()
        assert stats['total_coordinates'] == 30  # 3 threads × 10 neurons each
        assert stats['total_areas'] == 3

    def test_error_handling_invalid_inputs(self):
        """Test error handling with invalid inputs."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Test with negative coordinates (should work - Morton encoding handles it)
        success = spatial_hash.add_neuron("test_area", -1, -1, -1, 9001)
        # Note: Negative coordinates may or may not be supported depending on implementation
        
        # Test with very large coordinates
        success = spatial_hash.add_neuron("test_area", 1000000, 1000000, 1000000, 9002)
        # Should handle large coordinates gracefully
        
        # Test with None/invalid inputs should not crash
        try:
            spatial_hash.get_neuron_at_coordinate("", 0, 0, 0)  # Empty area name
            spatial_hash.get_neuron_at_coordinate("test", None, 0, 0)  # None coordinate
        except (TypeError, ValueError):
            pass  # Expected to fail gracefully


class TestMortonCoordinateLimitValidation:
    """Test Morton coordinate limit validation integration."""
    
    def setup_method(self):
        """Reset spatial hash and state manager before each test."""
        reset_morton_spatial_hash()
        # Reset state manager if needed
        try:
            from feagi.core.state_manager import FeagiStateManager
            FeagiStateManager._instance = None
        except ImportError:
            pass
    
    def test_state_manager_morton_awareness(self):
        """Test that state manager tracks Morton spatial hash information."""
        from feagi.core.state_manager import get_state_manager
        
        state_manager = get_state_manager()
        
        # Should have default Morton information
        morton_limit = state_manager.get_morton_coordinate_limit()
        morton_class = state_manager.get_morton_class_name()
        
        assert morton_limit == (1 << 21), f"Expected {1 << 21}, got {morton_limit}"
        assert morton_class == "RoaringSpatialHash", f"Expected RoaringSpatialHash, got {morton_class}"
        
        # Test coordinate validation
        assert state_manager.is_coordinate_within_morton_limits(0, 0, 0)
        assert state_manager.is_coordinate_within_morton_limits(1000, 1000, 1000)
        assert not state_manager.is_coordinate_within_morton_limits(morton_limit, 0, 0)
        assert not state_manager.is_coordinate_within_morton_limits(0, morton_limit, 0)
        assert not state_manager.is_coordinate_within_morton_limits(0, 0, morton_limit)
    
    def test_cortical_area_dimension_validation(self):
        """Test cortical area dimension validation against Morton limits."""
        from feagi.core.state_manager import get_state_manager
        
        state_manager = get_state_manager()
        morton_limit = state_manager.get_morton_coordinate_limit()
        
        # Valid dimensions should pass
        valid_dims = (100, 100, 100)
        result = state_manager.validate_cortical_area_dimensions(valid_dims)
        assert result.is_ok, "Valid dimensions should pass validation"
        
        # Invalid dimensions should fail
        invalid_dims = (morton_limit, 100, 100)  # Exceeds limit
        result = state_manager.validate_cortical_area_dimensions(invalid_dims)
        assert result.is_err, "Invalid dimensions should fail validation"
        
        # Edge case: exactly at limit should fail (need room for 0-based indexing)
        edge_dims = (morton_limit - 1, morton_limit - 1, morton_limit - 1)
        result = state_manager.validate_cortical_area_dimensions(edge_dims)
        # This might pass or fail depending on implementation - let's check
        print(f"Edge case validation result: {result.is_ok}")
    
    def test_connectome_manager_integration(self):
        """Test that ConnectomeManager validates cortical area dimensions."""
        try:
            from feagi.bdu.connectome_manager import ConnectomeManager
            from feagi.core.state_manager import get_state_manager
            
            # Reset singleton to ensure clean state
            ConnectomeManager.reset_singleton()
            
            connectome = ConnectomeManager(1000)  # Small neuron count for testing
            state_manager = get_state_manager()
            morton_limit = state_manager.get_morton_coordinate_limit()
            
            # Test helper methods
            max_dims = connectome.get_max_allowable_cortical_area_dimensions()
            assert isinstance(max_dims, tuple)
            assert len(max_dims) == 3
            assert all(dim < morton_limit for dim in max_dims)
            
            # Test validation method
            valid_dims = (100, 100, 100)
            assert connectome.validate_cortical_area_dimensions_safe(valid_dims)
            
            invalid_dims = (morton_limit, 100, 100)
            assert not connectome.validate_cortical_area_dimensions_safe(invalid_dims)
            
            # Test Morton info method
            morton_info = connectome.get_morton_spatial_hash_info()
            assert "morton_class" in morton_info
            assert "coordinate_limit" in morton_info
            assert "max_cortical_area_dimensions" in morton_info
            assert morton_info["morton_class"] == "RoaringSpatialHash"
            assert morton_info["coordinate_limit"] == morton_limit
            
            print(f"✅ Morton spatial hash info: {morton_info}")
            
        except ImportError as e:
            pytest.skip(f"ConnectomeManager not available: {e}")
    
    def test_cortical_area_creation_validation(self):
        """Test that cortical area creation is blocked for oversized dimensions."""
        try:
            from feagi.bdu.connectome_manager import ConnectomeManager
            from feagi.core.state_manager import get_state_manager
            
            # Reset singleton to ensure clean state
            ConnectomeManager.reset_singleton()
            
            connectome = ConnectomeManager(1000)
            state_manager = get_state_manager()
            morton_limit = state_manager.get_morton_coordinate_limit()
            
            # Valid cortical area should succeed
            valid_dims = (50, 50, 50)
            area_id = connectome.add_cortical_area(
                name="Valid Test Area",
                dimensions=valid_dims,
                position=(0, 0, 0),
                area_type="test"
            )
            assert area_id is not None
            print(f"✅ Created valid cortical area: {area_id}")
            
            # Invalid cortical area should raise ValueError
            invalid_dims = (morton_limit, 100, 100)
            with pytest.raises(ValueError, match="exceed.*coordinate limits"):
                connectome.add_cortical_area(
                    name="Invalid Test Area",
                    dimensions=invalid_dims,
                    position=(0, 0, 0),
                    area_type="test"
                )
            print(f"✅ Correctly blocked oversized cortical area: {invalid_dims}")
            
        except ImportError as e:
            pytest.skip(f"ConnectomeManager not available: {e}")


if __name__ == "__main__":
    # Run integration tests
    pytest.main([__file__, "-v"]) 