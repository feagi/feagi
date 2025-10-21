"""
Tests for Rust Morton spatial hash implementation.

Phase 3B: Morton encoding + Roaring bitmaps for spatial indexing.
"""

import pytest
import time


def test_morton_encode_decode():
    """Test Morton encode/decode roundtrip."""
    from feagi_bdu import py_morton_encode_3d, py_morton_decode_3d
    
    coords = [(0, 0, 0), (10, 20, 30), (100, 200, 300)]
    
    for x, y, z in coords:
        code = py_morton_encode_3d(x, y, z)
        dx, dy, dz = py_morton_decode_3d(code)
        assert (x, y, z) == (dx, dy, dz), f"Round-trip failed for {x}, {y}, {z}"


def test_morton_spatial_hash_basic():
    """Test basic spatial hash operations."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    # Add neurons
    assert hash.add_neuron("v1", 10, 20, 30, 1001)
    assert hash.add_neuron("v1", 10, 20, 30, 1002)  # Same position
    assert hash.add_neuron("v1", 15, 25, 35, 2001)
    assert hash.add_neuron("v2", 5, 5, 5, 3001)
    
    # Get single neuron (returns first)
    neuron = hash.get_neuron_at_coordinate("v1", 10, 20, 30)
    assert neuron == 1001
    
    # Get all neurons at position
    neurons = hash.get_neurons_at_coordinate("v1", 10, 20, 30)
    assert len(neurons) == 2
    assert 1001 in neurons
    assert 1002 in neurons
    
    # Get from different area
    neuron = hash.get_neuron_at_coordinate("v2", 5, 5, 5)
    assert neuron == 3001
    
    # Get non-existent
    assert hash.get_neuron_at_coordinate("v1", 99, 99, 99) is None


def test_morton_region_query():
    """Test region queries (critical for projections)."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    # Add neurons in 10x10x10 grid
    for x in range(10):
        for y in range(10):
            for z in range(10):
                neuron_id = x * 100 + y * 10 + z
                hash.add_neuron("v1", x, y, z, neuron_id)
    
    # Query 2x2x2 subregion (8 neurons)
    neurons = hash.get_neurons_in_region("v1", 0, 0, 0, 1, 1, 1)
    assert len(neurons) == 8, f"Expected 8 neurons, got {len(neurons)}"
    
    # Verify specific neurons
    assert 0 in neurons  # (0,0,0)
    assert 1 in neurons  # (0,0,1)
    assert 101 in neurons  # (1,0,1)
    assert 111 in neurons  # (1,1,1)


def test_morton_performance():
    """Test that Rust spatial hash is FAST."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    # Add 10,000 neurons (128x128x1 area approximately)
    start = time.time()
    for i in range(10000):
        x = i % 128
        y = (i // 128) % 128
        z = 0
        hash.add_neuron("v1", x, y, z, i)
    add_time = (time.time() - start) * 1000
    
    assert add_time < 100, f"Adding 10K neurons took {add_time}ms (should be < 100ms)"
    
    # Lookup performance (should be microseconds)
    start = time.time()
    for _ in range(1000):
        neurons = hash.get_neurons_at_coordinate("v1", 64, 64, 0)
    lookup_time = (time.time() - start) * 1000
    
    assert lookup_time < 10, f"1000 lookups took {lookup_time}ms (should be < 10ms)"
    
    # Region query performance
    start = time.time()
    neurons = hash.get_neurons_in_region("v1", 0, 0, 0, 127, 127, 0)
    region_time = (time.time() - start) * 1000
    
    assert region_time < 50, f"Region query took {region_time}ms (should be < 50ms)"
    assert len(neurons) == 10000


def test_get_neuron_position():
    """Test reverse lookup: neuron_id -> position."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    hash.add_neuron("v1", 42, 84, 126, 999)
    
    pos = hash.get_neuron_position(999)
    assert pos is not None
    area, x, y, z = pos
    assert area == "v1"
    assert (x, y, z) == (42, 84, 126)


def test_remove_neuron():
    """Test neuron removal."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    hash.add_neuron("v1", 10, 20, 30, 1001)
    assert hash.get_neuron_at_coordinate("v1", 10, 20, 30) == 1001
    
    assert hash.remove_neuron(1001)
    assert hash.get_neuron_at_coordinate("v1", 10, 20, 30) is None


def test_statistics():
    """Test spatial hash statistics."""
    from feagi_bdu import PyMortonSpatialHash
    
    hash = PyMortonSpatialHash()
    
    # Add neurons to 2 areas
    for i in range(100):
        hash.add_neuron("v1", i, 0, 0, i)
    for i in range(50):
        hash.add_neuron("v2", i, 0, 0, i + 1000)
    
    stats = hash.get_stats()
    assert stats["total_areas"] == 2
    assert stats["total_neurons"] == 150
    assert stats["total_occupied_positions"] == 150  # Each position has 1 neuron


def test_morton_spatial_locality():
    """Verify Morton codes preserve spatial locality."""
    from feagi_bdu import py_morton_encode_3d
    
    # Nearby points should have nearby Morton codes
    c1 = py_morton_encode_3d(10, 10, 10)
    c2 = py_morton_encode_3d(11, 10, 10)  # Neighbor
    c3 = py_morton_encode_3d(100, 100, 100)  # Far away
    
    diff_near = abs(c1 - c2)
    diff_far = abs(c1 - c3)
    
    assert diff_near < diff_far, "Spatial locality not preserved!"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

