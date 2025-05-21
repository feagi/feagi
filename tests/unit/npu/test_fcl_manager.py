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
    from feagi.npu.fcl_manager import FCLManager, BitMap, TimestepOutOfRangeError
except ImportError:
    pytest.skip("feagi.npu.fcl_manager not found", allow_module_level=True)


@pytest.fixture
def fcl_manager():
    """Create a FCLManager instance for testing."""
    return FCLManager(window_size=5)


@pytest.fixture
def sample_firing_data():
    """Create sample firing data for two timesteps."""
    # First timestep: cortical_idx -> neurons firing
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
    """Test cases for the FCLManager class."""
    
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
        assert len(global_fcl) == 9  # Total neurons across all corticals
        
        # Check cortical-specific FCLs
        cortical_100_fcl = fcl_manager.get_cortical_fcl(100)
        assert len(cortical_100_fcl) == 4
        assert 1001 in cortical_100_fcl
        assert 1002 in cortical_100_fcl
        
        # Check active corticals
        active_corticals = fcl_manager.get_active_corticals()
        assert len(active_corticals) == 3
        assert 100 in active_corticals
        assert 200 in active_corticals
        assert 300 in active_corticals
        
        # Check statistics
        stats = fcl_manager.get_firing_statistics()
        assert stats["total_neurons_fired"] == 9
        assert stats["active_corticals_count"] == 3
        
    def test_fcl_by_area(self, fcl_manager, sample_firing_data):
        """Test retrieving FCL data grouped by cortical area."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Get FCL by cortical
        fcl_by_cortical = fcl_manager.get_fcl_by_cortical()
        
        # Check structure
        assert len(fcl_by_cortical) == 3
        assert 100 in fcl_by_cortical
        assert 200 in fcl_by_cortical
        assert 300 in fcl_by_cortical
        
        # Check content
        assert len(fcl_by_cortical[100]) == 4
        assert len(fcl_by_cortical[200]) == 3
        assert len(fcl_by_cortical[300]) == 2
        
    def test_temporal_fcl_tracking(self, fcl_manager, sample_firing_data):
        """Test tracking neuron activations across timesteps."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Update FCL with two timesteps of data
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)
        
        # Test neurons fired in last 2 steps (all corticals)
        recent_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2)
        assert len(recent_neurons) == 14  # Total unique neurons across both timesteps
        
        # Test neurons fired in last 2 steps (cortical 100 only)
        cortical_100_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2, [100])
        assert len(cortical_100_neurons) == 6  # Neurons from cortical 100 in both timesteps
        assert 1002 in cortical_100_neurons  # This neuron appears in both timesteps
        
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
        
        # Test consistently active neurons (all corticals)
        consistent_neurons = fcl_manager.get_consistently_active_neurons(2)
        assert len(consistent_neurons) == 2
        assert 1002 in consistent_neurons
        assert 2001 in consistent_neurons
        
        # Test consistently active neurons (cortical 100 only)
        cortical_100_consistent = fcl_manager.get_consistently_active_neurons(2, [100])
        assert len(cortical_100_consistent) == 1
        assert 1002 in cortical_100_consistent
        
    def test_fcl_delta(self, fcl_manager, sample_firing_data):
        """Test identifying neurons that became active between timesteps."""
        firing_t1, firing_t2 = sample_firing_data
        
        # Update FCL with both timesteps
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)
        
        # Test delta between timesteps (all corticals)
        delta_neurons = fcl_manager.get_fcl_delta(1, 2)
        assert len(delta_neurons) == 5  # New neurons in t2 not in t1
        assert 1003 in delta_neurons
        assert 1009 in delta_neurons
        assert 2005 in delta_neurons
        assert 4001 in delta_neurons
        assert 4002 in delta_neurons
        
        # Test delta for specific cortical
        cortical_100_delta = fcl_manager.get_fcl_delta(1, 2, [100])
        assert len(cortical_100_delta) == 2
        assert 1003 in cortical_100_delta
        assert 1009 in cortical_100_delta
        
    def test_count_firing_neurons(self, fcl_manager, sample_firing_data):
        """Test counting firing neurons globally and by cortical."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Test global count
        assert fcl_manager.count_firing_neurons() == 9
        
        # Test cortical-specific counts
        assert fcl_manager.count_firing_neurons(cortical_idx=100) == 4
        assert fcl_manager.count_firing_neurons(cortical_idx=200) == 3
        assert fcl_manager.count_firing_neurons(cortical_idx=300) == 2
        assert fcl_manager.count_firing_neurons(cortical_idx=400) == 0  # Non-existent cortical
        
    @pytest.mark.parametrize("invalid_cortical", [500, 600, 700])
    def test_nonexistent_area(self, fcl_manager, sample_firing_data, invalid_cortical):
        """Test querying for corticals that don't exist returns empty results."""
        firing_t1, _ = sample_firing_data
        fcl_manager.update_fcl(1, firing_t1)
        
        # Cortical-specific queries for non-existent cortical
        assert len(fcl_manager.get_cortical_fcl(invalid_cortical)) == 0
        assert fcl_manager.count_firing_neurons(cortical_idx=invalid_cortical) == 0
        
    def test_empty_fcl(self, fcl_manager):
        """Test behavior with empty FCLs."""
        # Update with empty data
        fcl_manager.update_fcl(1, {})
        
        # Verify results
        assert fcl_manager.count_firing_neurons() == 0
        assert len(fcl_manager.get_active_corticals()) == 0
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
        with pytest.raises(TimestepOutOfRangeError):
            # This should fail as the time difference exceeds window size
            fcl_manager.get_fcl_delta(0, 5)
        
        # But we should still be able to access timestep 5
        assert 9999 in fcl_manager.get_cortical_fcl(100) 