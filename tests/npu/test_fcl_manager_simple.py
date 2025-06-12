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
Simple tests for FCL Manager to improve test coverage.

These tests focus on basic functionality and API correctness to ensure
the FCL manager works as expected.
"""

from unittest.mock import Mock, patch

import pytest

from feagi.npu.fcl_manager import EnhancedFCLManager, FCLManager


def test_fcl_manager_basic_initialization():
    """Test FCL manager initialization with default settings."""
    fcl_manager = FCLManager()

    assert fcl_manager.window_size == 20
    assert fcl_manager.current_timestep == 0
    assert isinstance(fcl_manager.cortical_fcl_history, dict)


def test_fcl_manager_custom_window_size():
    """Test FCL manager initialization with custom window size."""
    fcl_manager = FCLManager(window_size=10)

    assert fcl_manager.window_size == 10
    assert fcl_manager.current_timestep == 0


def test_fcl_manager_advance_timestep():
    """Test timestep advancement."""
    fcl_manager = FCLManager()

    initial_timestep = fcl_manager.current_timestep
    fcl_manager.advance_timestep()

    assert fcl_manager.current_timestep == initial_timestep + 1


def test_fcl_manager_update_fcl():
    """Test updating FCL with neuron data."""
    fcl_manager = FCLManager()

    # Update with neurons for cortical area 1
    neurons_by_cortical = {1: [10, 20, 30]}
    fcl_manager.update_fcl(0, neurons_by_cortical)

    # Check that neurons were added
    cortical_fcl = fcl_manager.get_cortical_fcl(1, 0)
    assert 10 in cortical_fcl
    assert 20 in cortical_fcl
    assert 30 in cortical_fcl


def test_fcl_manager_get_global_fcl():
    """Test getting global FCL."""
    fcl_manager = FCLManager()

    # Add some neurons
    neurons_by_cortical = {1: [10, 20], 2: [30, 40]}
    fcl_manager.update_fcl(0, neurons_by_cortical)

    # Get global FCL
    global_fcl = fcl_manager.get_global_fcl(0)

    # Should contain all neurons
    assert 10 in global_fcl
    assert 20 in global_fcl
    assert 30 in global_fcl
    assert 40 in global_fcl


def test_fcl_manager_get_active_corticals():
    """Test getting active cortical areas."""
    fcl_manager = FCLManager()

    # Add neurons to cortical areas 1 and 3
    neurons_by_cortical = {1: [10, 20], 3: [30, 40]}
    fcl_manager.update_fcl(0, neurons_by_cortical)

    # Get active corticals
    active_corticals = fcl_manager.get_active_corticals(0)

    assert 1 in active_corticals
    assert 3 in active_corticals
    assert 2 not in active_corticals  # Not active


def test_fcl_manager_count_firing_neurons():
    """Test counting firing neurons."""
    fcl_manager = FCLManager()

    # Add neurons
    neurons_by_cortical = {1: [10, 20, 30]}
    fcl_manager.update_fcl(0, neurons_by_cortical)

    # Count global neurons
    global_count = fcl_manager.count_firing_neurons(0)
    assert global_count == 3

    # Count cortical-specific neurons
    cortical_count = fcl_manager.count_firing_neurons(0, cortical_idx=1)
    assert cortical_count == 3


def test_fcl_manager_membrane_updates():
    """Test membrane potential update queue."""
    fcl_manager = FCLManager()

    # Queue some updates
    fcl_manager.queue_membrane_update(10, 0.5)
    fcl_manager.queue_membrane_update(20, -0.3, source_neuron_idx=5)

    # Process the queue
    updates = fcl_manager.process_update_queue()
    assert len(updates) == 2
    assert updates[0] == (10, 0.5)
    assert updates[1] == (20, -0.3)

    # Queue should be empty after processing
    assert len(fcl_manager.membrane_update_queue) == 0


def test_enhanced_fcl_manager_initialization():
    """Test Enhanced FCL manager initialization."""
    enhanced_fcl = EnhancedFCLManager()

    assert enhanced_fcl.default_window_size == 20
    assert isinstance(enhanced_fcl.custom_cortical_history, dict)


def test_enhanced_fcl_manager_memory_cortical():
    """Test memory cortical registration."""
    enhanced_fcl = EnhancedFCLManager()

    # Register a memory cortical area (must use window size >= default)
    enhanced_fcl.register_memory_cortical(100, window_size=50)

    # Check registration
    assert enhanced_fcl.is_memory_cortical(100)
    assert enhanced_fcl.get_cortical_window_size(100) == 50

    # Check non-memory cortical
    assert not enhanced_fcl.is_memory_cortical(200)
    assert enhanced_fcl.get_cortical_window_size(200) == 20  # default


def test_fcl_manager_add_to_current_fcl():
    """Test adding neurons to current FCL."""
    fcl_manager = FCLManager()

    # Add neurons directly to current FCL
    fcl_manager.add_to_current_fcl([100, 101, 102])

    # Check they were added
    current_fcl = fcl_manager.get_fcl(0)
    assert 100 in current_fcl
    assert 101 in current_fcl
    assert 102 in current_fcl


def test_fcl_manager_get_firing_neurons():
    """Test getting firing neurons as list."""
    fcl_manager = FCLManager()

    # Add neurons
    neurons_by_cortical = {1: [10, 20, 30]}
    fcl_manager.update_fcl(0, neurons_by_cortical)

    # Get as list - use current timestep (0 offset)
    firing_neurons = fcl_manager.get_firing_neurons(offset=0)

    # Should contain the neurons (order may vary)
    assert set(firing_neurons) == {10, 20, 30}


def test_fcl_manager_get_firing_statistics():
    """Test getting firing statistics."""
    fcl_manager = FCLManager()

    # Add neurons to multiple timesteps
    fcl_manager.update_fcl(0, {1: [10, 20]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(1, {1: [30], 2: [40, 50]})

    # Get statistics
    stats = fcl_manager.get_firing_statistics()

    assert isinstance(stats, dict)
    assert "total_neurons_fired" in stats
    assert "active_corticals" in stats


def test_fcl_manager_ensure_cortical_initialized():
    """Test cortical area initialization."""
    fcl_manager = FCLManager()

    # Initialize cortical area manually
    fcl_manager._ensure_cortical_initialized(5)

    # Should be in cortical_fcl_history
    assert 5 in fcl_manager.cortical_fcl_history
    assert isinstance(fcl_manager.cortical_fcl_history[5], list)


def test_fcl_manager_get_neurons_by_corticals():
    """Test getting neurons from multiple cortical areas."""
    fcl_manager = FCLManager()

    # Add neurons to different areas
    fcl_manager.update_fcl(0, {1: [10, 20], 2: [30, 40], 3: [50]})

    # Get neurons from areas 1 and 3
    neurons = fcl_manager.get_neurons_by_corticals([1, 3], 0)

    # Should contain neurons from areas 1 and 3, but not 2
    assert 10 in neurons
    assert 20 in neurons
    assert 50 in neurons
    assert 30 not in neurons
    assert 40 not in neurons


def test_fcl_manager_get_fcl_by_cortical():
    """Test getting FCL organized by cortical area."""
    fcl_manager = FCLManager()

    # Add neurons
    fcl_manager.update_fcl(0, {1: [10, 20], 2: [30, 40]})

    # Get FCL by cortical
    fcl_by_cortical = fcl_manager.get_fcl_by_cortical(0)

    assert isinstance(fcl_by_cortical, dict)
    assert 1 in fcl_by_cortical
    assert 2 in fcl_by_cortical

    # Check contents
    assert 10 in fcl_by_cortical[1]
    assert 20 in fcl_by_cortical[1]
    assert 30 in fcl_by_cortical[2]
    assert 40 in fcl_by_cortical[2]


def test_enhanced_fcl_manager_get_cortical_temporal_pattern():
    """Test getting temporal patterns for enhanced FCL manager."""
    enhanced_fcl = EnhancedFCLManager()

    # Register as memory cortical (must use >= default window size)
    enhanced_fcl.register_memory_cortical(10, window_size=30)

    # Add neurons across multiple timesteps
    for timestep in range(3):
        enhanced_fcl.update_fcl(timestep, {10: [100 + timestep]})
        enhanced_fcl.advance_timestep()

    # Get temporal pattern
    pattern = enhanced_fcl.get_cortical_temporal_pattern(10, n_steps=3)

    # Should return a BitMap-like object
    assert hasattr(pattern, "__contains__")  # Should be iterable/membership testable


def test_fcl_manager_neurons_fired_in_last_n_steps():
    """Test getting neurons that fired in the last N steps."""
    fcl_manager = FCLManager()

    # Add neurons across multiple timesteps
    fcl_manager.update_fcl(0, {1: [10, 20]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(1, {1: [20, 30]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(2, {1: [30, 40]})

    # Get neurons from last 2 steps
    recent_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2)

    # Should contain neurons from timesteps 1 and 2
    assert 20 in recent_neurons
    assert 30 in recent_neurons
    assert 40 in recent_neurons


def test_fcl_manager_get_fcl_delta():
    """Test getting FCL delta between timesteps."""
    fcl_manager = FCLManager()

    # Add different neurons at different timesteps
    fcl_manager.update_fcl(0, {1: [10, 20]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(1, {1: [20, 30]})

    # Get delta (what changed)
    delta = fcl_manager.get_fcl_delta(0, 1)

    # Should contain neurons that were different between the timesteps
    assert isinstance(delta, type(fcl_manager.get_global_fcl(0)))


if __name__ == "__main__":
    pytest.main(["-v", __file__])
