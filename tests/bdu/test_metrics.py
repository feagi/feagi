"""
Test module for metrics utility functions.
"""

import pytest
import numpy as np
from feagi.bdu.utils.metrics import calculate_neuron_density

def test_calculate_neuron_density_uniform():
    """Test calculating neuron density for a uniform distribution."""
    # Create a synthetic neuron position dataset
    dimensions = (10, 10, 10)
    neuron_count = 1000
    
    # Create positions - one neuron per voxel
    positions = []
    for x in range(10):
        for y in range(10):
            for z in range(10):
                positions.append((x, y, z))
    
    # Calculate density
    density_map = calculate_neuron_density(positions, dimensions, bin_size=1)
    
    # Every voxel should have exactly 1 neuron
    assert density_map.shape == dimensions
    assert np.all(density_map == 1)

def test_calculate_neuron_density_nonuniform():
    """Test calculating neuron density for a non-uniform distribution."""
    # Create a synthetic neuron position dataset
    dimensions = (10, 10, 10)
    
    # Create positions with higher density in one region
    positions = []
    
    # Add 10 neurons to position (1, 1, 1)
    positions.extend([(1, 1, 1)] * 10)
    
    # Add 5 neurons to position (5, 5, 5)
    positions.extend([(5, 5, 5)] * 5)
    
    # Add 1 neuron to various other positions
    for x in range(10):
        positions.append((x, 0, 0))
    
    # Calculate density
    density_map = calculate_neuron_density(positions, dimensions, bin_size=1)
    
    # Check specific positions
    assert density_map[1, 1, 1] == 10  # 10 neurons here
    assert density_map[5, 5, 5] == 5   # 5 neurons here
    assert density_map[0, 0, 0] == 1   # 1 neuron here
    assert density_map[9, 0, 0] == 1   # 1 neuron here
    assert density_map[2, 2, 2] == 0   # No neurons here

def test_calculate_neuron_density_with_binning():
    """Test neuron density calculation with different bin sizes."""
    # Create a synthetic neuron position dataset
    dimensions = (10, 10, 10)
    
    # Create positions
    positions = []
    # Add neurons to the first octant (0-4, 0-4, 0-4)
    for x in range(5):
        for y in range(5):
            for z in range(5):
                positions.append((x, y, z))
    
    # Calculate density with bin_size=1
    density_map_1 = calculate_neuron_density(positions, dimensions, bin_size=1)
    
    # With bin_size=1, each voxel in the first octant should have exactly 1 neuron
    assert density_map_1.shape == dimensions
    for x in range(5):
        for y in range(5):
            for z in range(5):
                assert density_map_1[x, y, z] == 1
    
    # All voxels outside the first octant should have 0 neurons
    for x in range(5, 10):
        for y in range(10):
            for z in range(10):
                assert density_map_1[x, y, z] == 0
    
    # Calculate density with bin_size=2
    density_map_2 = calculate_neuron_density(positions, dimensions, bin_size=2)
    
    # With bin_size=2, the shape should be halved
    assert density_map_2.shape == (5, 5, 5)
    
    # Each bin in the first 3x3x3 bins should have 8 neurons (2x2x2)
    for x in range(2):
        for y in range(2):
            for z in range(2):
                assert density_map_2[x, y, z] == 8
    
    # The last bins might have partial coverage

def test_calculate_neuron_density_empty():
    """Test neuron density calculation with no neurons."""
    dimensions = (10, 10, 10)
    positions = []
    
    density_map = calculate_neuron_density(positions, dimensions, bin_size=1)
    
    assert density_map.shape == dimensions
    assert np.all(density_map == 0)

def test_calculate_neuron_density_out_of_bounds():
    """Test neuron density calculation with positions outside the volume."""
    dimensions = (10, 10, 10)
    
    # Create positions, some of which are out of bounds
    positions = [
        (5, 5, 5),      # Valid
        (0, 0, 0),      # Valid
        (9, 9, 9),      # Valid
        (10, 5, 5),     # Invalid X
        (5, 10, 5),     # Invalid Y
        (5, 5, 10),     # Invalid Z
        (-1, 5, 5),     # Invalid X
        (5, -1, 5),     # Invalid Y
        (5, 5, -1),     # Invalid Z
        (20, 20, 20)    # All invalid
    ]
    
    # Out of bounds positions should be ignored
    density_map = calculate_neuron_density(positions, dimensions, bin_size=1)
    
    assert density_map.shape == dimensions
    assert density_map[5, 5, 5] == 1
    assert density_map[0, 0, 0] == 1
    assert density_map[9, 9, 9] == 1
    
    # Check total count (should only include the valid positions)
    assert np.sum(density_map) == 3 