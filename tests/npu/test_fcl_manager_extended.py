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
Extended tests for the FCL Manager.

This module provides additional test coverage for the FCLManager and EnhancedFCLManager
classes in the feagi.npu.fcl_manager module, focusing on advanced features and error handling.
"""


import pytest

from feagi.npu.fcl_manager import (
    BitMap,
    EnhancedFCLManager,
    FCLManager,
    TimestepOutOfRangeError,
)


class TestAdvancedFCLManagerFeatures:
    """Test advanced features of the FCLManager class."""

    @pytest.fixture
    def fcl_manager(self):
        """Create a basic FCL manager for testing."""
        return FCLManager(window_size=5)

    @pytest.fixture
    def setup_timesteps(self, fcl_manager):
        """Set up multiple timesteps with specific firing patterns."""
        # Timestep 1: Areas 100, 200, 300
        fcl_manager.update_fcl(
            1,
            {100: {1001, 1002, 1003, 1004}, 200: {2001, 2002}, 300: {3001, 3002, 3003}},
        )

        # Timestep 2: Areas 100, 200, 300
        fcl_manager.update_fcl(
            2, {100: {1002, 1003, 1005}, 200: {2001, 2003}, 300: {3002, 3004}}
        )

        # Timestep 3: Areas 100, 200, 300
        fcl_manager.update_fcl(
            3, {100: {1001, 1003, 1006}, 200: {2001, 2004}, 300: {3001, 3002}}
        )

        # Timestep 4: Areas 100, 200, 300
        fcl_manager.update_fcl(4, {100: {1002, 1003}, 200: {2001}, 300: {3002}})

        # Timestep 5: Areas 100, 200, 300
        fcl_manager.update_fcl(
            5, {100: {1003, 1007}, 200: {2001, 2005}, 300: {3002, 3005}}
        )

        return fcl_manager

    def test_get_fcl_delta(self, setup_timesteps):
        """Test getting FCL delta between timesteps."""
        fcl_manager = setup_timesteps

        # Get delta between timestep 1 and 3
        delta = fcl_manager.get_fcl_delta(1, 3)

        # Delta should contain neurons in t3 but not in t1
        assert 1006 in delta  # In t3, not in t1
        assert 2004 in delta  # In t3, not in t1
        assert 1001 not in delta  # In both t1 and t3
        assert 3003 not in delta  # In t1, not in t3

        # Test with specific cortical areas
        delta_area_100 = fcl_manager.get_fcl_delta(1, 3, cortical_indices=[100])
        assert 1006 in delta_area_100
        assert 2004 not in delta_area_100  # Not in area 100

    def test_get_fcl_xor(self, setup_timesteps):
        """Test getting XOR of FCLs between timesteps."""
        fcl_manager = setup_timesteps

        # Get XOR between timestep 1 and 3
        xor = fcl_manager.get_fcl_xor(1, 3)

        # Should contain neurons in t1 but not t3, and in t3 but not t1
        assert 1004 in xor  # In t1, not in t3
        assert 1006 in xor  # In t3, not in t1
        assert 3003 in xor  # In t1, not in t3
        assert 1003 not in xor  # In both t1 and t3
        assert 3002 not in xor  # In both t1 and t3

        # Test with specific cortical areas
        xor_area_100 = fcl_manager.get_fcl_xor(1, 3, cortical_indices=[100])
        assert 1004 in xor_area_100
        assert 1006 in xor_area_100
        assert 3003 not in xor_area_100  # Not in area 100

    def test_get_neurons_fired_in_last_n_steps(self, setup_timesteps):
        """Test getting neurons that fired in the last N steps."""
        fcl_manager = setup_timesteps

        # Get neurons fired in last 3 steps (timesteps 3, 4, 5)
        fired = fcl_manager.get_neurons_fired_in_last_n_steps(3)

        # Should contain all neurons that fired in any of those timesteps
        assert 1001 in fired  # Fired in t3
        assert 1002 in fired  # Fired in t4
        assert 1003 in fired  # Fired in t3, t4, t5
        assert 1006 in fired  # Fired in t3
        assert 1007 in fired  # Fired in t5
        assert 2001 in fired  # Fired in t3, t4, t5
        assert 3002 in fired  # Fired in t3, t4, t5

        # Test with specific cortical areas
        fired_area_100 = fcl_manager.get_neurons_fired_in_last_n_steps(
            3, cortical_indices=[100]
        )
        assert 1001 in fired_area_100
        assert 2001 not in fired_area_100  # Not in area 100

    def test_get_consistently_active_neurons(self, setup_timesteps):
        """Test getting neurons that were consistently active across timesteps."""
        fcl_manager = setup_timesteps

        # Get neurons that fired in all of the last 3 steps
        consistent = fcl_manager.get_consistently_active_neurons(3)

        # Only neurons that fired in all 3 of the last timesteps
        assert 1003 in consistent  # Fired in t3, t4, t5
        assert 2001 in consistent  # Fired in t3, t4, t5
        assert 3002 in consistent  # Fired in t3, t4, t5
        assert 1001 not in consistent  # Only fired in t3
        assert 1007 not in consistent  # Only fired in t5

        # Test with specific cortical areas
        consistent_area_100 = fcl_manager.get_consistently_active_neurons(
            3, cortical_indices=[100]
        )
        assert 1003 in consistent_area_100
        assert 2001 not in consistent_area_100  # Not in area 100

    def test_membrane_update_queue(self, fcl_manager):
        """Test queuing and processing membrane potential updates."""
        # Queue several updates to different neurons
        fcl_manager.queue_membrane_update(1, 0.1)
        fcl_manager.queue_membrane_update(2, 0.2)
        fcl_manager.queue_membrane_update(1, 0.3)  # Another update for neuron 1

        # Process the queue
        updates = fcl_manager.process_update_queue()

        # Check the updates - should be aggregated by neuron
        assert len(updates) == 2  # 2 neurons updated

        # Convert to dict for easier testing
        update_dict = dict(updates)
        assert update_dict[1] == 0.4  # 0.1 + 0.3
        assert update_dict[2] == 0.2

        # Queue should be empty after processing
        assert fcl_manager.process_update_queue() == []

    def test_error_handling(self, fcl_manager):
        """Test error handling for invalid timestep access."""
        fcl_manager.update_fcl(1, {100: {1, 2, 3}})
        fcl_manager.update_fcl(2, {100: {2, 3, 4}})

        # Trying to access a timestep outside the window should raise an error
        with pytest.raises(TimestepOutOfRangeError):
            fcl_manager.get_global_fcl(timestep=-10)

        with pytest.raises(TimestepOutOfRangeError):
            fcl_manager.get_cortical_fcl(100, timestep=10)


class TestEnhancedFCLManagerAdvancedFeatures:
    """Test advanced features of the EnhancedFCLManager class."""

    @pytest.fixture
    def enhanced_fcl_manager(self):
        """Create enhanced FCL manager for testing."""
        return EnhancedFCLManager(window_size=5)

    @pytest.fixture
    def setup_memory_cortical(self, enhanced_fcl_manager):
        """Set up memory corticals with data."""
        # Register two memory corticals
        enhanced_fcl_manager.register_memory_cortical(100, window_size=10)
        enhanced_fcl_manager.register_memory_cortical(200, window_size=8)

        # Update with multiple timesteps
        for t in range(1, 8):
            enhanced_fcl_manager.update_fcl(
                t,
                {
                    # Area 100: neuron 1003 always fires, others vary
                    100: {1001, 1002, 1003} if t % 2 == 1 else {1003, 1004, 1005},
                    # Area 200: neuron 2001 always fires
                    200: {2001, 2002, 2003} if t % 3 == 0 else {2001, 2004, 2005},
                },
            )

        return enhanced_fcl_manager

    def test_get_cortical_temporal_pattern(self, setup_memory_cortical):
        """Test getting temporal pattern for a memory cortical area."""
        enhanced_fcl = setup_memory_cortical

        # Get patterns for the last 5 timesteps in area 100
        pattern = enhanced_fcl.get_cortical_temporal_pattern(100, 5)

        # Should contain all neurons that fired in the last 5 timesteps
        assert 1001 in pattern
        assert 1002 in pattern
        assert 1003 in pattern  # Fired in all timesteps
        assert 1004 in pattern
        assert 1005 in pattern

        # Test with a different area
        pattern_200 = enhanced_fcl.get_cortical_temporal_pattern(200, 3)
        assert 2001 in pattern_200  # Fired in all timesteps

        # Test with non-memory cortical should raise error
        with pytest.raises(ValueError):
            enhanced_fcl.get_cortical_temporal_pattern(300, 3)

    def test_get_memory_cortical_consistency(self, setup_memory_cortical):
        """Test calculating consistency of patterns in memory corticals."""
        enhanced_fcl = setup_memory_cortical

        # Reset the manager to have a clean state for testing patterns
        # Create a simple pattern in the memory cortical area
        pattern = BitMap([1, 2, 3])
        enhanced_fcl.update_fcl(enhanced_fcl.current_timestep, {100: pattern})

        # Exercise the memory_cortical_consistency method with various parameters
        # Just checking that it runs without error
        consistency1 = enhanced_fcl.get_memory_cortical_consistency(
            cortical_idx=100, pattern_duration=1, window_duration=2
        )
        assert isinstance(consistency1, float)
        assert 0.0 <= consistency1 <= 1.0  # Consistency should be in valid range

        # Try with different parameters
        consistency2 = enhanced_fcl.get_memory_cortical_consistency(
            cortical_idx=100, pattern_duration=2, window_duration=4
        )
        assert isinstance(consistency2, float)
        assert 0.0 <= consistency2 <= 1.0

    def test_get_consistent_neurons_in_memory_cortical(self, setup_memory_cortical):
        """Test getting consistently active neurons in a memory cortical."""
        enhanced_fcl = setup_memory_cortical

        # Get neurons that were active in all of the last 3 timesteps
        consistent = enhanced_fcl.get_consistent_neurons_in_memory_cortical(100, 3)

        # Neuron 1003 fired in all timesteps
        assert 1003 in consistent
        assert 1001 not in consistent  # Only fired in odd timesteps
        assert 1004 not in consistent  # Only fired in even timesteps

        # Test with another area
        consistent_200 = enhanced_fcl.get_consistent_neurons_in_memory_cortical(200, 5)
        assert 2001 in consistent_200  # Fired in all timesteps

    def test_advanced_error_handling(self, enhanced_fcl_manager):
        """Test advanced error handling for memory corticals."""
        # Register a memory cortical
        enhanced_fcl_manager.register_memory_cortical(100, window_size=10)

        # Test with invalid parameters
        with pytest.raises(ValueError):
            # Pattern duration can't exceed window duration
            enhanced_fcl_manager.get_memory_cortical_consistency(
                100, pattern_duration=5, window_duration=3
            )

        with pytest.raises(ValueError):
            # Non-memory cortical should raise an error
            enhanced_fcl_manager.get_memory_cortical_consistency(
                300, pattern_duration=2, window_duration=4
            )

        with pytest.raises(ValueError):
            # n_steps must be positive
            enhanced_fcl_manager.get_cortical_temporal_pattern(100, n_steps=0)

        # Test registering memory cortical with invalid window size
        with pytest.raises(ValueError):
            enhanced_fcl_manager.register_memory_cortical(
                300, window_size=3
            )  # Smaller than default
