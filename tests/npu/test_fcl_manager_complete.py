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

"""
Comprehensive tests for the FCL Manager.

This module provides complete test coverage for both FCLManager and EnhancedFCLManager
classes in the feagi.npu.fcl_manager module.
"""

import time
from queue import Queue
from unittest.mock import MagicMock, Mock, patch

import pytest

from feagi.npu.fcl_manager import BitMap, CorticalIdx, EnhancedFCLManager, FCLManager


@pytest.fixture
def fcl_manager():
    """Create a basic FCL manager for testing."""
    return FCLManager(window_size=5)


@pytest.fixture
def enhanced_fcl_manager():
    """Create an enhanced FCL manager for testing."""
    return EnhancedFCLManager(default_window_size=5)


@pytest.fixture
def sample_firing_data():
    """Create sample firing neuron data for testing."""
    # First timestep
    firing_t1 = {
        100: {1001, 1002, 1005, 1008},  # Cortical area 100
        200: {2001, 2010, 2015},  # Cortical area 200
        300: {3001, 3002, 3003, 3004, 3005},  # Cortical area 300
    }

    # Second timestep
    firing_t2 = {
        100: {1002, 1003, 1009},  # Cortical area 100
        200: {2001, 2005},  # Cortical area 200
        300: [3002, 3005, 3010],  # Cortical area 300 (as list)
    }

    return firing_t1, firing_t2


class TestFCLManager:
    """Tests for the FCLManager class."""

    def test_initialization(self, fcl_manager):
        """Test that FCL manager initializes correctly."""
        assert fcl_manager.window_size == 5
        assert len(fcl_manager.global_fcl_history) == 5
        assert fcl_manager.current_window_index == 0
        assert fcl_manager.total_neurons_fired == 0

    def test_update_fcl(self, fcl_manager, sample_firing_data):
        """Test updating the FCL with firing neurons."""
        firing_t1, _ = sample_firing_data

        # Update FCL with timestep 1 data
        fcl_manager.update_fcl(1, firing_t1)

        # Check current state
        assert fcl_manager.current_timestep == 1
        assert fcl_manager.current_window_index == 1

        # Check global FCL
        global_fcl = fcl_manager.get_global_fcl()
        assert len(global_fcl) == 12  # Total across all areas

        # Check cortical FCLs
        assert len(fcl_manager.get_cortical_fcl(100)) == 4
        assert len(fcl_manager.get_cortical_fcl(200)) == 3
        assert len(fcl_manager.get_cortical_fcl(300)) == 5

        # Check total neurons fired
        assert fcl_manager.total_neurons_fired == 12

    def test_update_fcl_multiple_timesteps(self, fcl_manager, sample_firing_data):
        """Test updating FCL across multiple timesteps."""
        firing_t1, firing_t2 = sample_firing_data

        # Update with first timestep
        fcl_manager.update_fcl(1, firing_t1)

        # Update with second timestep
        fcl_manager.update_fcl(2, firing_t2)

        # Check second timestep FCL
        global_fcl_t2 = fcl_manager.get_global_fcl()
        assert len(global_fcl_t2) == 8  # Total in second timestep

        # Check first timestep FCL
        global_fcl_t1 = fcl_manager.get_global_fcl(timestep=1)
        assert len(global_fcl_t1) == 12  # Total in first timestep

    def test_window_wrapping(self, fcl_manager, sample_firing_data):
        """Test that FCL history wraps around when window is full."""
        firing_t1, _ = sample_firing_data

        # Fill the window and wrap around
        for i in range(1, 8):  # Exceeds window size of 5
            fcl_manager.update_fcl(i, firing_t1)

        # Check that current window index wrapped around
        assert fcl_manager.current_window_index < 5
        assert fcl_manager.current_timestep == 7

        # Check we can access both oldest and newest data
        global_fcl_newest = fcl_manager.get_global_fcl()
        global_fcl_oldest_valid = fcl_manager.get_global_fcl(
            timestep=3
        )  # Should be valid

        # The newest FCL should have firing neurons
        assert len(global_fcl_newest) > 0
        # The oldest valid FCL should have firing neurons
        assert len(global_fcl_oldest_valid) > 0

        # Check that accessing timestep outside window raises error
        with pytest.raises(Exception):
            fcl_manager.get_global_fcl(timestep=1)  # Too old, overwritten

    def test_ensure_cortical_initialized(self, fcl_manager):
        """Test that cortical areas are automatically initialized."""
        # Verify cortical area 999 not in history yet
        assert 999 not in fcl_manager.cortical_fcl_history

        # Call the method to ensure it's initialized
        fcl_manager._ensure_cortical_initialized(999)

        # Now it should exist
        assert 999 in fcl_manager.cortical_fcl_history
        assert len(fcl_manager.cortical_fcl_history[999]) == 5  # Window size

    def test_get_index_for_timestep(self, fcl_manager):
        """Test calculating the index for a given timestep."""
        # Set current timestep
        fcl_manager.current_timestep = 10
        fcl_manager.current_window_index = 0

        # Test current timestep
        assert fcl_manager._get_index_for_timestep(None) == 0  # Default is current
        assert fcl_manager._get_index_for_timestep(10) == 0

        # Test previous timesteps within window
        assert fcl_manager._get_index_for_timestep(9) == 4  # Wraps to end of window
        assert fcl_manager._get_index_for_timestep(6) == 1

        # Test timestep outside window
        with pytest.raises(Exception):
            fcl_manager._get_index_for_timestep(4)  # Too old

    def test_get_neurons_by_corticals(self, fcl_manager, sample_firing_data):
        """Test getting neurons from multiple cortical areas."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        fcl_manager.update_fcl(1, firing_t1)

        # Get neurons from areas 100 and 200
        combined_fcl = fcl_manager.get_neurons_by_corticals([100, 200])

        # Check combined FCL
        assert len(combined_fcl) == 7  # 4 from area 100 + 3 from area 200
        assert 1001 in combined_fcl  # From area 100
        assert 2001 in combined_fcl  # From area 200

    def test_get_firing_statistics(self, fcl_manager, sample_firing_data):
        """Test getting firing statistics."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        fcl_manager.update_fcl(1, firing_t1)

        # Get statistics
        stats = fcl_manager.get_firing_statistics()

        # Check statistics
        assert stats["total_neurons_fired"] == 12
        assert stats["active_corticals_count"] == 3
        assert "neurons_per_cortical" in stats
        assert len(stats["active_corticals"]) == 3

    def test_firing_rate_tracking(self, fcl_manager, sample_firing_data):
        """Test tracking of neuron firing rates."""
        firing_t1, firing_t2 = sample_firing_data

        # Update multiple timesteps to generate firing history
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)

        # Get neurons fired in last 2 steps
        consistently_firing = fcl_manager.get_neurons_fired_in_last_n_steps(2)

        # Neuron 1002 fires in both timesteps and should be included
        assert 1002 in consistently_firing
        # Total count should be the number of all unique neurons that fired in the last 2 steps
        assert len(consistently_firing) > 0

    def test_add_to_current_fcl(self, fcl_manager):
        """Test adding neurons to the current FCL."""
        # Initialize FCL with timestep 1
        fcl_manager.update_fcl(1, {100: [1001, 1002]})

        # Add more neurons to current FCL
        fcl_manager.add_to_current_fcl([1003, 1004])

        # Check current FCL
        global_fcl = fcl_manager.get_global_fcl()
        assert len(global_fcl) == 4
        assert 1001 in global_fcl
        assert 1003 in global_fcl

    def test_get_fcl_with_offset(self, fcl_manager, sample_firing_data):
        """Test getting FCL with offset."""
        firing_t1, firing_t2 = sample_firing_data

        # Update FCL with two timesteps
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)

        # Get FCL with offset -1 (previous timestep)
        prev_fcl = fcl_manager.get_fcl(offset=-1)

        # Should match timestep 1
        assert len(prev_fcl) == 12
        assert 1001 in prev_fcl  # Present in timestep 1

        # Current FCL (offset 0)
        current_fcl = fcl_manager.get_fcl()
        assert len(current_fcl) == 8
        assert 1003 in current_fcl  # Present in timestep 2

    def test_get_firing_neurons(self, fcl_manager, sample_firing_data):
        """Test getting list of firing neurons."""
        firing_t1, firing_t2 = sample_firing_data

        # Update FCL with two timesteps
        fcl_manager.update_fcl(1, firing_t1)
        fcl_manager.update_fcl(2, firing_t2)

        # Get firing neurons from previous timestep
        firing_list = fcl_manager.get_firing_neurons(offset=-1)

        # Check list
        assert isinstance(firing_list, list)
        assert len(firing_list) == 12
        assert 1001 in firing_list

    def test_get_nonexistent_cortical(self, fcl_manager):
        """Test getting FCL for nonexistent cortical area."""
        # Initialize with a cortical area
        fcl_manager.update_fcl(1, {100: [1001, 1002]})

        # Try to get nonexistent area
        nonexistent_fcl = fcl_manager.get_cortical_fcl(999)

        # Should return empty bitmap
        assert len(nonexistent_fcl) == 0


class TestEnhancedFCLManager:
    """Tests for the EnhancedFCLManager class."""

    def test_initialization(self, enhanced_fcl_manager):
        """Test that enhanced FCL manager initializes correctly."""
        assert enhanced_fcl_manager.default_window_size == 5
        assert len(enhanced_fcl_manager.global_fcl_history) == 5
        assert enhanced_fcl_manager.current_window_index == 0
        assert enhanced_fcl_manager.total_neurons_fired == 0

    def test_update_fcl(self, enhanced_fcl_manager, sample_firing_data):
        """Test updating the FCL with firing neurons."""
        firing_t1, _ = sample_firing_data

        # Update FCL with first timestep data
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Check global FCL
        global_fcl = enhanced_fcl_manager.get_global_fcl()
        assert len(global_fcl) == 12  # Total neurons across all corticals

    def test_temporal_history(self, enhanced_fcl_manager, sample_firing_data):
        """Test temporal history tracking."""
        firing_t1, firing_t2 = sample_firing_data

        # Update multiple timesteps
        enhanced_fcl_manager.update_fcl(1, firing_t1)
        enhanced_fcl_manager.update_fcl(2, firing_t2)

        # Get neurons fired in last 2 steps
        temporal_fcl = enhanced_fcl_manager.get_neurons_fired_in_last_n_steps(2, [100])

        # Check result
        assert len(temporal_fcl) > 0
        assert 1002 in temporal_fcl  # Neuron active in both timesteps

    def test_area_fcl_history(self, enhanced_fcl_manager, sample_firing_data):
        """Test retrieving area-specific FCL history."""
        firing_t1, firing_t2 = sample_firing_data

        # Register area 100 as memory cortical
        enhanced_fcl_manager.register_memory_cortical(100, window_size=10)

        # Update multiple timesteps
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # For memory corticals, they're stored in custom_cortical_history, not cortical_fcl_history
        window_size, history_array, start_timestep = (
            enhanced_fcl_manager.custom_cortical_history[100]
        )

        # Custom index for the current timestep
        custom_index = enhanced_fcl_manager._get_custom_cortical_index(100, 1)

        # Check the bitmap at this index directly
        area_fcl = history_array[custom_index]
        assert len(area_fcl) == 4  # 4 neurons in area 100 at timestep 1

        # Update with second timestep
        enhanced_fcl_manager.update_fcl(2, firing_t2)

        # Get the custom index for timestep 2
        custom_index = enhanced_fcl_manager._get_custom_cortical_index(100, 2)

        # Check the bitmap at this index directly
        area_fcl = history_array[custom_index]
        assert len(area_fcl) == 3  # 3 neurons in area 100 at timestep 2

        # Verify we can also access it using the get_cortical_temporal_pattern method
        temporal_pattern = enhanced_fcl_manager.get_cortical_temporal_pattern(100, 2)
        assert len(temporal_pattern) > 0  # Should contain neurons from both timesteps

    def test_register_memory_cortical(self, enhanced_fcl_manager):
        """Test registering a memory cortical area."""
        # Register memory cortical
        enhanced_fcl_manager.register_memory_cortical(500, window_size=10)

        # Check registration
        assert enhanced_fcl_manager.is_memory_cortical(500)
        assert enhanced_fcl_manager.get_cortical_window_size(500) == 10

        # Try registering with invalid window size
        with pytest.raises(ValueError):
            enhanced_fcl_manager.register_memory_cortical(
                600, window_size=3
            )  # Too small

    def test_get_global_count(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting global neuron count."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get global count
        global_count = enhanced_fcl_manager.count_firing_neurons()
        assert global_count == 12

    def test_get_cortical_count(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting cortical-specific neuron count."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get cortical count
        cortical_count = enhanced_fcl_manager.count_firing_neurons(cortical_idx=100)
        assert cortical_count == 4

    def test_add_timestamp(self, enhanced_fcl_manager, sample_firing_data):
        """Test adding a new timestamp with firing neurons."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Check timestamp
        assert enhanced_fcl_manager.current_timestep == 1

    def test_get_all_active_corticals(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting all active cortical areas."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get active corticals
        active_corticals = enhanced_fcl_manager.get_active_corticals()

        # Check result
        assert len(active_corticals) == 3
        assert 100 in active_corticals
        assert 200 in active_corticals
        assert 300 in active_corticals

    def test_get_cortical_fcl_lists(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting lists of firing neurons by cortical area."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get firing neurons for cortical 100
        fcl_list = list(enhanced_fcl_manager.get_cortical_fcl(100))

        # Check lists
        assert len(fcl_list) == 4
        assert 1001 in fcl_list

    def test_get_global_fcl_list(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting global list of firing neurons."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get global list
        global_list = list(enhanced_fcl_manager.get_global_fcl())

        # Check list
        assert isinstance(global_list, list)
        assert len(global_list) == 12
        assert 1001 in global_list
        assert 2001 in global_list

    def test_get_activity_metrics(self, enhanced_fcl_manager, sample_firing_data):
        """Test getting activity metrics."""
        firing_t1, _ = sample_firing_data

        # Update FCL
        enhanced_fcl_manager.update_fcl(1, firing_t1)

        # Get activity metrics
        metrics = enhanced_fcl_manager.get_firing_statistics()

        # Check metrics
        assert "total_neurons_fired" in metrics
        assert metrics["total_neurons_fired"] == 12
        assert "active_corticals" in metrics
        assert metrics["active_corticals"] == 3
        assert "neurons_per_cortical" in metrics
        assert len(metrics["neurons_per_cortical"]) == 3


if __name__ == "__main__":
    pytest.main(["-v", __file__])
