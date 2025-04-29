"""
Tests for the Hierarchical Fire Candidate List (FCL) Manager

This module contains tests for the HierarchicalFCL class from feagi.npu.fcl_manager.
"""

import pytest
from typing import Dict, List, Optional, Set
from collections import defaultdict
try:
    import pyroaring
    PYROARING_AVAILABLE = True
except ImportError:
    PYROARING_AVAILABLE = False

# Import the code to test, handling potential import errors
try:
    from feagi.npu.fcl_manager import HierarchicalFCL, BitMap
except ImportError:
    pytest.skip("feagi.npu.fcl_manager not found", allow_module_level=True)


@pytest.fixture
def fcl_manager():
    """Create a HierarchicalFCL instance for testing."""
    return HierarchicalFCL(window_size=5)


@pytest.fixture
def sample_firing_data():
    """Create sample firing data for two timesteps."""
    # First timestep: area_id -> neurons firing
    firing_t1 = {
        100: BitMap([1001, 1002, 1005, 1008]),
        200: BitMap([2001, 2010, 2015]),
        300: BitMap([3004, 3007])
    }
    
    # Second timestep
    firing_t2 = {
        100: BitMap([1002, 1003, 1009]),
        200: BitMap([2001, 2005]),
        400: BitMap([4001, 4002])
    }
    
    return firing_t1, firing_t2


class TestHierarchicalFCL:
    """Test cases for the HierarchicalFCL class."""
    
    def test_initialization(self, fcl_manager):
        """Test that FCL manager initializes correctly."""
        assert fcl_manager.window_size == 5
        assert len(fcl_manager.global_fcl_history) == 5
        assert fcl_manager.current_window_index == 0
        assert fcl_manager.total_neurons_fired == 0
        
    def test_update_fcl(self, fcl_manager, sample_firing_data):
        """Test updating the FCL with firing neurons."""
        firing_t1, _ = sample_firing_data
        
        # Update FCL with first timestep data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Check global FCL
        global_fcl = fcl_manager.get_global_fcl()
        assert len(global_fcl) == 9  # Total neurons across all areas
        
        # Check area-specific FCLs
        area_100_fcl = fcl_manager.get_area_fcl(100)
        assert len(area_100_fcl) == 4
        assert 1001 in area_100_fcl
        assert 1002 in area_100_fcl
        
        # Check active areas
        active_areas = fcl_manager.get_active_areas()
        assert len(active_areas) == 3
        assert 100 in active_areas
        assert 200 in active_areas
        assert 300 in active_areas
        
        # Check statistics
        stats = fcl_manager.get_firing_statistics()
        assert stats["total_neurons_fired"] == 9
        assert stats["active_areas_count"] == 3
        
    def test_fcl_by_area(self, fcl_manager, sample_firing_data):
        """Test retrieving FCL data grouped by cortical area."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Get FCL by area
        fcl_by_area = fcl_manager.get_fcl_by_area()
        
        # Check structure
        assert len(fcl_by_area) == 3
        assert 100 in fcl_by_area
        assert 200 in fcl_by_area
        assert 300 in fcl_by_area
        
        # Check content
        assert len(fcl_by_area[100]) == 4
        assert len(fcl_by_area[200]) == 3
        assert len(fcl_by_area[300]) == 2
        
    def test_temporal_fcl_tracking(self, fcl_manager, sample_firing_data):
        """Test tracking neuron activations across timesteps."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Update FCL with two timesteps of data
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)
        
        # Test neurons fired in last 2 steps (all areas)
        recent_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2)
        assert len(recent_neurons) == 13  # Total unique neurons across both timesteps
        
        # Test neurons fired in last 2 steps (area 100 only)
        area_100_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2, [100])
        assert len(area_100_neurons) == 6
        assert 1001 in area_100_neurons
        assert 1002 in area_100_neurons
        assert 1003 in area_100_neurons
        assert 1005 in area_100_neurons
        assert 1008 in area_100_neurons
        assert 1009 in area_100_neurons
        
    def test_consistent_activations(self, fcl_manager, sample_firing_data):
        """Test identifying neurons that consistently fire across timesteps."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Create modified data where some neurons fire consistently
        consistent_firing = {
            100: BitMap([1002, 1005]),  # 1002 consistent with t2, 1005 not in t2
            200: BitMap([2001]),        # 2001 consistent with t2
            300: BitMap([3007])         # Not in t2
        }
        
        # Update FCL with consistent data then t2
        fcl_manager.update_fcl(1, consistent_firing)
        fcl_manager.update_fcl(2, firing_t2)
        
        # Test consistently active neurons (all areas)
        consistent_neurons = fcl_manager.get_consistently_active_neurons(2)
        assert len(consistent_neurons) == 2
        assert 1002 in consistent_neurons
        assert 2001 in consistent_neurons
        
        # Test consistently active neurons (area 100 only)
        area_100_consistent = fcl_manager.get_consistently_active_neurons(2, [100])
        assert len(area_100_consistent) == 1
        assert 1002 in area_100_consistent
        
    def test_fcl_delta(self, fcl_manager, sample_firing_data):
        """Test identifying neurons that became active between timesteps."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Update FCL with both timesteps
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)
        
        # Test delta between timesteps (all areas)
        delta_neurons = fcl_manager.get_fcl_delta(1, 2)
        assert len(delta_neurons) == 5  # New neurons in t2 not in t1
        assert 1003 in delta_neurons
        assert 1009 in delta_neurons
        assert 2005 in delta_neurons
        assert 4001 in delta_neurons
        assert 4002 in delta_neurons
        
        # Test delta for specific area
        area_100_delta = fcl_manager.get_fcl_delta(1, 2, [100])
        assert len(area_100_delta) == 2
        assert 1003 in area_100_delta
        assert 1009 in area_100_delta
        
    def test_count_firing_neurons(self, fcl_manager, sample_firing_data):
        """Test counting firing neurons globally and by area."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Test global count
        assert fcl_manager.count_firing_neurons() == 9
        
        # Test area-specific counts
        assert fcl_manager.count_firing_neurons(area_id=100) == 4
        assert fcl_manager.count_firing_neurons(area_id=200) == 3
        assert fcl_manager.count_firing_neurons(area_id=300) == 2
        assert fcl_manager.count_firing_neurons(area_id=400) == 0  # Non-existent area
        
    @pytest.mark.parametrize("invalid_area", [500, 600, 700])
    def test_nonexistent_area(self, fcl_manager, sample_firing_data, invalid_area):
        """Test querying for areas that don't exist returns empty results."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Area-specific queries for non-existent area
        assert len(fcl_manager.get_area_fcl(invalid_area)) == 0
        assert fcl_manager.count_firing_neurons(area_id=invalid_area) == 0
        
    def test_empty_fcl(self, fcl_manager):
        """Test behavior with empty FCLs."""
        # Update with empty data
        fcl_manager.update_fcl(1, {})
        
        # Verify results
        assert fcl_manager.count_firing_neurons() == 0
        assert len(fcl_manager.get_active_areas()) == 0
        assert len(fcl_manager.get_global_fcl()) == 0
        
    def test_window_size_boundary(self, fcl_manager, sample_firing_data):
        """Test behavior at window size boundaries."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Fill the entire window
        for t in range(5):
            fcl_manager.update_fcl(t, firing_t1 if t % 2 == 0 else firing_t2)
            
        # Add one more update which should overwrite the first entry
        fcl_manager.update_fcl(5, {100: BitMap([9999])})
        
        # Timestep 0 data should be gone, replaced by timestep 5
        with pytest.raises(ValueError):
            # This should fail as the time difference exceeds window size
            fcl_manager.get_fcl_delta(0, 5)
            
        # Check we can still access the most recent data
        assert 9999 in fcl_manager.get_area_fcl(100) 