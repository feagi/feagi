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
    
    def test_full_feagi_import_chain(self):
        """Test that all imports work through the full FEAGI chain."""
        # This should work without any import errors
        from feagi.bdu.spatial_hash import (
            initialize_spatial_hash, 
            SpatialHashConfig,
            get_spatial_hash,
            GlobalSpatialHash,
            analyze_genome_coordinate_space
        )
        
        # Test SpatialHashConfig with all legacy parameters
        config = SpatialHashConfig(
            max_dimension=256,
            genome_based_sizing=True,
            enable_simd=True,      # This parameter was missing!
            hash_prime=73856093,   # This parameter was missing!
            cache_size=1000000,    # This parameter was missing!
            enable_caching=True
        )
        
        # Initialize should work
        spatial_hash = initialize_spatial_hash(config)
        assert spatial_hash is not None
        
        # Global access should work
        global_hash = get_spatial_hash()
        assert global_hash is not None
    
    def test_connectome_manager_integration(self):
        """Test integration patterns used by ConnectomeManager."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Test that batch_coordinate_lookup exists and works
        assert hasattr(spatial_hash, 'batch_coordinate_lookup')
        
        # Test the exact usage pattern from ConnectomeManager
        candidate_positions = {(1, 2, 3), (4, 5, 6), (7, 8, 9)}
        neuron_positions = [(1, 2, 3), (10, 11, 12), (4, 5, 6)]
        
        matches = spatial_hash.batch_coordinate_lookup(candidate_positions, neuron_positions)
        
        # Should return list of (candidate_idx, neuron_idx) tuples
        assert isinstance(matches, list)
        assert len(matches) == 2  # Two matches: (1,2,3) and (4,5,6)
        
        # Verify match structure
        for match in matches:
            assert isinstance(match, tuple)
            assert len(match) == 2
            assert isinstance(match[0], int)  # candidate_idx
            assert isinstance(match[1], int)  # neuron_idx
    
    def test_spatial_hash_adapter_all_methods(self):
        """Test that all expected adapter methods exist and work."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # Methods that ConnectomeManager expects
        required_methods = [
            'add_coordinate',
            'get_neuron_id', 
            'expand_cache_for_new_area',
            'get_statistics',
            'clear',
            'save_cache',
            'load_cache',
            'batch_coordinate_lookup',  # This was missing!
            'initialize_for_dimensions',
            'get_state',
            'is_ready',
            'wait_for_ready'
        ]
        
        for method_name in required_methods:
            assert hasattr(spatial_hash, method_name), f"Missing method: {method_name}"
            method = getattr(spatial_hash, method_name)
            assert callable(method), f"Method {method_name} is not callable"
    
    def test_type_annotations_are_valid(self):
        """Test that all type annotations can be resolved."""
        # This would catch the missing 'Set' import
        from feagi.bdu.spatial_hash_adapter import GlobalSpatialHashAdapter
        
        # Create instance to ensure class definition is valid
        adapter = GlobalSpatialHashAdapter()
        
        # Test batch_coordinate_lookup with proper types
        candidate_positions: Set[Tuple[int, int, int]] = {(1, 2, 3), (4, 5, 6)}
        neuron_positions: List[Tuple[int, int, int]] = [(1, 2, 3), (7, 8, 9)]
        
        result = adapter.batch_coordinate_lookup(candidate_positions, neuron_positions)
        assert isinstance(result, list)
    
    def test_connectome_manager_initialization_pattern(self):
        """Test the exact initialization pattern used by ConnectomeManager."""
        # This mirrors the exact code in ConnectomeManager.__init__
        from feagi.bdu.spatial_hash import initialize_spatial_hash, SpatialHashConfig
        
        spatial_config = SpatialHashConfig(
            max_dimension=256,
            enable_simd=True,      # ConnectomeManager passes this
            genome_based_sizing=True
        )
        
        spatial_hash = initialize_spatial_hash(spatial_config)
        assert spatial_hash is not None
        # Initialize dimensions to make it ready
        spatial_hash.initialize_for_dimensions((256, 256, 256))
        assert spatial_hash.is_ready()
    
    def test_real_genome_analysis_integration(self):
        """Test genome analysis with real FEAGI genome structure."""
        from feagi.bdu.spatial_hash import analyze_genome_coordinate_space
        
        # Mock genome data structure that FEAGI actually uses
        mock_genome = {
            "blueprint": {
                "0-area1-cx-___bbx-i": 64,
                "0-area1-cx-___bby-i": 32, 
                "0-area1-cx-___bbz-i": 16,
                "0-area2-cx-___bbx-i": 128,
                "0-area2-cx-___bby-i": 64,
                "0-area2-cx-___bbz-i": 32,
            }
        }
        
        max_dims = analyze_genome_coordinate_space(mock_genome)
        assert isinstance(max_dims, tuple)
        assert len(max_dims) == 3
        assert all(isinstance(dim, int) for dim in max_dims)
    
    def test_thread_safety_with_concurrent_access(self):
        """Test thread safety patterns used in FEAGI."""
        import threading
        import time
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        errors = []
        
        def worker(worker_id):
            try:
                for i in range(10):
                    # Add coordinates
                    spatial_hash.add_coordinate(f"area_{worker_id}", i, i, i, worker_id * 100 + i)
                    
                    # Lookup coordinates
                    result = spatial_hash.get_neuron_id(f"area_{worker_id}", i, i, i)
                    assert result == worker_id * 100 + i
                    
                    time.sleep(0.001)  # Small delay
            except Exception as e:
                errors.append(f"Worker {worker_id}: {e}")
        
        # Run multiple threads concurrently
        threads = []
        for i in range(5):
            thread = threading.Thread(target=worker, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads
        for thread in threads:
            thread.join()
        
        # Check for errors
        assert not errors, f"Thread safety errors: {errors}"


class TestIntegrationFailurePrevention:
    """Tests specifically designed to catch the issues we missed."""
    
    def test_imports_dont_fail_on_missing_dependencies(self):
        """Ensure imports work in real FEAGI environment."""
        try:
            # This exact import pattern that failed
            from feagi.bdu.spatial_hash_adapter import (
                GlobalSpatialHashAdapter,
                SpatialHashConfig, 
                SpatialHashState,
                GlobalSpatialHash,
                analyze_genome_coordinate_space
            )
            
            # Test class instantiation works
            adapter = GlobalSpatialHashAdapter()
            config = SpatialHashConfig()
            global_hash = GlobalSpatialHash()
            
        except Exception as e:
            pytest.fail(f"Import failed: {e}")
    
    def test_all_expected_parameters_accepted(self):
        """Test that all legacy parameters are accepted."""
        from feagi.bdu.spatial_hash import SpatialHashConfig
        
        # This exact parameter set that caused the crash
        try:
            config = SpatialHashConfig(
                max_dimension=256,
                enable_simd=True,
                genome_based_sizing=True
            )
            assert config.enable_simd == True
            assert config.genome_based_sizing == True
            assert config.max_dimension == 256
        except TypeError as e:
            pytest.fail(f"SpatialHashConfig rejected valid parameters: {e}")
    
    def test_batch_coordinate_lookup_method_exists(self):
        """Specifically test for the method that was missing."""
        from feagi.bdu.spatial_hash import get_spatial_hash
        
        spatial_hash = get_spatial_hash()
        
        # This exact method call that failed
        assert hasattr(spatial_hash, 'batch_coordinate_lookup')
        
        # Test it actually works
        candidates = {(1, 2, 3)}
        neurons = [(1, 2, 3), (4, 5, 6)]
        
        try:
            result = spatial_hash.batch_coordinate_lookup(candidates, neurons)
            assert isinstance(result, list)
        except Exception as e:
            pytest.fail(f"batch_coordinate_lookup failed: {e}")
    
    def test_type_hints_dont_cause_runtime_errors(self):
        """Test that type hints are properly imported."""
        # This would catch the missing Set import
        try:
            from feagi.bdu.spatial_hash_adapter import GlobalSpatialHashAdapter
            
            adapter = GlobalSpatialHashAdapter()
            
            # Use the method with Set type hint
            candidates = {(1, 2, 3)}
            neurons = [(1, 2, 3)]
            
            result = adapter.batch_coordinate_lookup(candidates, neurons)
            assert isinstance(result, list)
            
        except NameError as e:
            if "Set" in str(e):
                pytest.fail(f"Set type not properly imported: {e}")
            raise


if __name__ == "__main__":
    # Run integration tests
    pytest.main([__file__, "-v"]) 