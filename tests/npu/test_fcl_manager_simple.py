"""
Simple test coverage for fcl_manager.py to boost coverage.

This module tests the FCL (Firing Candidate List) manager basic functionality
using the correct API.
"""

import pytest
from feagi.npu.fcl_manager import FCLManager, EnhancedFCLManager, BitMap, NeuronCollection


def test_fcl_manager_basic_initialization():
    """Test FCL manager initialization with default settings."""
    fcl_manager = FCLManager()
    
    assert fcl_manager.window_size == 20
    assert fcl_manager.current_timestep == 0
    assert isinstance(fcl_manager._cortical_fcls, dict)
    assert isinstance(fcl_manager._membrane_update_queue, list)


def test_fcl_manager_custom_window_size():
    """Test FCL manager initialization with custom window size."""
    fcl_manager = FCLManager(window_size=10)
    
    assert fcl_manager.window_size == 10
    assert fcl_manager.current_timestep == 0


def test_fcl_manager_basic_operations():
    """Test basic FCL operations."""
    fcl_manager = FCLManager(window_size=5)
    
    # Test update_fcl with neurons
    neurons_by_cortical = {
        1: [10, 20, 30],
        2: [40, 50],
    }
    
    fcl_manager.update_fcl(0, neurons_by_cortical)
    
    # Test get_global_fcl
    global_fcl = fcl_manager.get_global_fcl()
    assert isinstance(global_fcl, BitMap)
    assert len(global_fcl) == 5  # Should have 5 neurons total
    
    # Test get_cortical_fcl
    cortical_fcl_1 = fcl_manager.get_cortical_fcl(1)
    assert isinstance(cortical_fcl_1, BitMap)
    assert len(cortical_fcl_1) == 3  # Should have 3 neurons from cortical 1


def test_fcl_manager_advance_timestep():
    """Test advancing timestep."""
    fcl_manager = FCLManager()
    
    initial_timestep = fcl_manager.current_timestep
    fcl_manager.advance_timestep()
    
    assert fcl_manager.current_timestep == initial_timestep + 1


def test_fcl_manager_count_firing_neurons():
    """Test counting firing neurons."""
    fcl_manager = FCLManager()
    
    # Add some neurons
    neurons_by_cortical = {1: [10, 20, 30]}
    fcl_manager.update_fcl(0, neurons_by_cortical)
    
    # Count global firing neurons
    count = fcl_manager.count_firing_neurons()
    assert count == 3
    
    # Count for specific cortical
    count_cortical = fcl_manager.count_firing_neurons(cortical_idx=1)
    assert count_cortical == 3


def test_fcl_manager_get_firing_statistics():
    """Test getting firing statistics."""
    fcl_manager = FCLManager()
    
    # Add some neurons
    neurons_by_cortical = {
        1: [10, 20],
        2: [30, 40, 50],
    }
    fcl_manager.update_fcl(0, neurons_by_cortical)
    
    stats = fcl_manager.get_firing_statistics()
    assert isinstance(stats, dict)
    assert 'total_neurons_fired' in stats
    assert stats['total_neurons_fired'] == 5


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
    assert len(fcl_manager._membrane_update_queue) == 0


def test_enhanced_fcl_manager_initialization():
    """Test Enhanced FCL manager initialization."""
    enhanced_fcl = EnhancedFCLManager()
    
    assert enhanced_fcl.default_window_size == 20
    assert isinstance(enhanced_fcl._memory_corticals, dict)
    assert isinstance(enhanced_fcl._custom_cortical_fcls, dict)


def test_enhanced_fcl_manager_memory_cortical():
    """Test enhanced FCL manager memory cortical functionality."""
    enhanced_fcl = EnhancedFCLManager()
    
    # Register a memory cortical
    enhanced_fcl.register_memory_cortical(100, window_size=50)
    
    # Check if it's registered
    assert enhanced_fcl.is_memory_cortical(100)
    assert enhanced_fcl.get_cortical_window_size(100) == 50
    
    # Check non-memory cortical
    assert not enhanced_fcl.is_memory_cortical(200)
    assert enhanced_fcl.get_cortical_window_size(200) == 20  # Default


def test_bitmap_operations():
    """Test bitmap operations."""
    # Test with list
    bitmap1 = BitMap([1, 2, 3])
    assert len(bitmap1) == 3
    assert 1 in bitmap1
    assert 4 not in bitmap1
    
    # Test adding elements
    bitmap1.add(4)
    assert 4 in bitmap1
    assert len(bitmap1) == 4
    
    # Test copy
    bitmap2 = bitmap1.copy()
    assert len(bitmap2) == 4
    
    # Test clear
    bitmap1.clear()
    assert len(bitmap1) == 0
    assert bitmap1.is_empty()


def test_neuron_collection():
    """Test NeuronCollection utility class."""
    # Test from list
    collection_list = NeuronCollection.from_any([1, 2, 3])
    bitmap_from_list = collection_list.to_bitmap()
    assert len(bitmap_from_list) == 3
    
    # Test from set
    collection_set = NeuronCollection.from_any({4, 5, 6})
    bitmap_from_set = collection_set.to_bitmap()
    assert len(bitmap_from_set) == 3
    
    # Test from bitmap
    original_bitmap = BitMap([7, 8, 9])
    collection_bitmap = NeuronCollection.from_any(original_bitmap)
    bitmap_from_bitmap = collection_bitmap.to_bitmap()
    assert len(bitmap_from_bitmap) == 3


def test_fcl_manager_temporal_operations():
    """Test temporal operations on FCL."""
    fcl_manager = FCLManager(window_size=5)
    
    # Add neurons at different timesteps
    fcl_manager.update_fcl(0, {1: [10, 20]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(1, {1: [30, 40]})
    fcl_manager.advance_timestep()
    fcl_manager.update_fcl(2, {1: [50, 60]})
    
    # Test getting neurons from previous steps
    recent_neurons = fcl_manager.get_neurons_fired_in_last_n_steps(2)
    assert len(recent_neurons) >= 4  # Should have neurons from last 2 steps
    
    # Test consistent neurons (this tests a more complex operation)
    consistent_neurons = fcl_manager.get_consistently_active_neurons(2)
    assert isinstance(consistent_neurons, BitMap)


def test_enhanced_fcl_temporal_patterns():
    """Test enhanced FCL temporal pattern operations."""
    enhanced_fcl = EnhancedFCLManager()
    
    # Register a memory cortical with valid window size
    enhanced_fcl.register_memory_cortical(1, window_size=30)  # Must be >= 20
    
    # Add some data
    enhanced_fcl.update_fcl(0, {1: [10, 20, 30]})
    enhanced_fcl.advance_timestep()
    enhanced_fcl.update_fcl(1, {1: [20, 30, 40]})
    enhanced_fcl.advance_timestep()
    
    # Test cortical temporal pattern
    pattern = enhanced_fcl.get_cortical_temporal_pattern(1, n_steps=2)
    assert isinstance(pattern, BitMap)
    
    # Test consistent neurons in memory cortical
    consistent = enhanced_fcl.get_consistent_neurons_in_memory_cortical(1, n_steps=2)
    assert isinstance(consistent, BitMap)


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 