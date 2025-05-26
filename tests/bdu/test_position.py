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
Test module for position utility functions.
"""

import pytest
from feagi.bdu.utils.position import (
    linearize_position,
    delinearize_position,
    validate_position
)

def test_linearize_position():
    """Test linearizing 3D positions to 1D indices."""
    # Test with dimensions [10, 10, 10]
    dimensions = (10, 10, 10)
    
    # Test positions across the space
    assert linearize_position((0, 0, 0), dimensions) == 0
    assert linearize_position((1, 0, 0), dimensions) == 1
    assert linearize_position((0, 1, 0), dimensions) == 10
    assert linearize_position((0, 0, 1), dimensions) == 100
    assert linearize_position((5, 5, 5), dimensions) == 555
    assert linearize_position((9, 9, 9), dimensions) == 999
    
    # Test with different dimensions
    dimensions = (5, 7, 9)
    assert linearize_position((0, 0, 0), dimensions) == 0
    assert linearize_position((1, 0, 0), dimensions) == 1
    assert linearize_position((0, 1, 0), dimensions) == 5
    assert linearize_position((0, 0, 1), dimensions) == 35
    assert linearize_position((4, 6, 8), dimensions) == 314  # Last position

def test_delinearize_position():
    """Test converting 1D indices back to 3D positions."""
    # Test with dimensions [10, 10, 10]
    dimensions = (10, 10, 10)
    
    # Test positions across the space
    assert delinearize_position(0, dimensions) == (0, 0, 0)
    assert delinearize_position(1, dimensions) == (1, 0, 0)
    assert delinearize_position(10, dimensions) == (0, 1, 0)
    assert delinearize_position(100, dimensions) == (0, 0, 1)
    assert delinearize_position(555, dimensions) == (5, 5, 5)
    assert delinearize_position(999, dimensions) == (9, 9, 9)
    
    # Test with different dimensions
    dimensions = (5, 7, 9)
    assert delinearize_position(0, dimensions) == (0, 0, 0)
    assert delinearize_position(1, dimensions) == (1, 0, 0)
    assert delinearize_position(5, dimensions) == (0, 1, 0)
    assert delinearize_position(35, dimensions) == (0, 0, 1)
    assert delinearize_position(314, dimensions) == (4, 6, 8)  # Last position

def test_roundtrip_conversion():
    """Test that linearize and delinearize are inverses of each other."""
    dimensions = (10, 10, 10)
    
    # Test various positions
    for x in range(10):
        for y in range(10):
            for z in range(10):
                position = (x, y, z)
                linear_index = linearize_position(position, dimensions)
                roundtrip_position = delinearize_position(linear_index, dimensions)
                assert position == roundtrip_position
    
    # Test with different dimensions
    dimensions = (5, 7, 9)
    for x in range(5):
        for y in range(7):
            for z in range(9):
                position = (x, y, z)
                linear_index = linearize_position(position, dimensions)
                roundtrip_position = delinearize_position(linear_index, dimensions)
                assert position == roundtrip_position

def test_validate_position():
    """Test validation of positions within dimensions."""
    dimensions = (10, 10, 10)
    
    # Valid positions
    assert validate_position((0, 0, 0), dimensions) == True
    assert validate_position((5, 5, 5), dimensions) == True
    assert validate_position((9, 9, 9), dimensions) == True
    
    # Invalid positions - negative coordinates
    assert validate_position((-1, 0, 0), dimensions) == False
    assert validate_position((0, -1, 0), dimensions) == False
    assert validate_position((0, 0, -1), dimensions) == False
    
    # Invalid positions - outside bounds
    assert validate_position((10, 0, 0), dimensions) == False
    assert validate_position((0, 10, 0), dimensions) == False
    assert validate_position((0, 0, 10), dimensions) == False
    
    # Invalid positions - completely outside
    assert validate_position((20, 20, 20), dimensions) == False
    
    # Invalid input format
    with pytest.raises(ValueError):
        validate_position((0, 0), dimensions)  # Missing z coordinate
    
    with pytest.raises(TypeError):
        validate_position(0, dimensions)  # Not a tuple

def test_unusual_positions():
    """Test edge cases and unusual inputs."""
    # Test with flat dimensions (2D-like)
    dimensions = (10, 10, 1)
    assert linearize_position((5, 5, 0), dimensions) == 55
    assert delinearize_position(55, dimensions) == (5, 5, 0)
    
    # Test with single-cell dimensions
    dimensions = (1, 1, 1)
    assert linearize_position((0, 0, 0), dimensions) == 0
    assert delinearize_position(0, dimensions) == (0, 0, 0)
    
    # Test with very large dimensions
    dimensions = (100, 100, 100)
    large_position = (99, 99, 99)
    linear_index = linearize_position(large_position, dimensions)
    assert linear_index == 999999
    assert delinearize_position(linear_index, dimensions) == large_position 