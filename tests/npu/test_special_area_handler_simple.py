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
Simplified test coverage for special_area_handler.py optimized for 100kHz performance.

This module tests the Special Area Handler simplified functionality
for direct core power area access (cortical_idx=1).
"""

import pytest
from unittest.mock import Mock, MagicMock
from feagi.npu.special_area_handler import SpecialAreaHandler


class MockConnectomeManager:
    def __init__(self):
        # Mock cortical areas with neurons
        self.mock_neurons = {
            0: [100, 101, 102],  # _death neurons  
            1: [200, 201, 202, 203, 204],  # ___pwr neurons (cortical_idx=1)
            2: [300, 301, 302]   # Other area neurons
        }
    
    def get_neurons_by_cortical_idx(self, cortical_idx):
        """Mock method to return neurons for a given cortical_idx."""
        if cortical_idx == 1:  # Core power area (___pwr)
            return self.mock_neurons.get(cortical_idx, [])
        return self.mock_neurons.get(cortical_idx, [])


@pytest.fixture
def mock_connectome_manager():
    return MockConnectomeManager()


@pytest.fixture
def special_handler(mock_connectome_manager):
    return SpecialAreaHandler(connectome_manager=mock_connectome_manager)


def test_special_area_handler_initialization(mock_connectome_manager):
    """Test basic special area handler initialization."""
    handler = SpecialAreaHandler(connectome_manager=mock_connectome_manager)
    
    assert handler.connectome_manager is mock_connectome_manager
    assert handler.injection_count == 0


def test_special_area_handler_with_config(mock_connectome_manager):
    """Test special area handler initialization with config (config ignored but accepted)."""
    config = {'some_setting': 50}
    handler = SpecialAreaHandler(connectome_manager=mock_connectome_manager, config=config)
    
    # Config is accepted but not used in simplified version
    assert handler.injection_count == 0


def test_get_power_area_neurons_success(special_handler):
    """Test getting power area neurons directly from cortical_idx=1."""
    neurons = special_handler.get_power_area_neurons()
    assert neurons == [200, 201, 202, 203, 204]


def test_get_power_area_neurons_empty(mock_connectome_manager):
    """Test getting power area neurons when none exist."""
    # Mock empty power area
    mock_connectome_manager.mock_neurons[1] = []
    special_handler = SpecialAreaHandler(connectome_manager=mock_connectome_manager)
    
    neurons = special_handler.get_power_area_neurons()
    assert neurons == []


def test_get_power_area_neurons_error_handling(special_handler):
    """Test error handling when accessing power area neurons."""
    # Mock an error in the connectome manager
    special_handler.connectome_manager.get_neurons_by_cortical_idx = Mock(side_effect=Exception("Mock error"))
    
    # Should return empty list on error
    neurons = special_handler.get_power_area_neurons()
    assert neurons == []


def test_get_statistics(special_handler):
    """Test getting statistics."""
    special_handler.record_injection()  # Record at least one injection
    
    stats = special_handler.get_statistics()
    assert isinstance(stats, dict)
    assert 'injection_count' in stats
    assert 'last_injection_time' in stats
    assert 'core_power_area' in stats
    assert stats['core_power_area'] == "cortical_idx=1 (___pwr)"
    assert stats['injection_count'] == 1


def test_record_injection(special_handler):
    """Test recording injections."""
    initial_count = special_handler.injection_count
    initial_time = special_handler.last_injection_time
    
    special_handler.record_injection()
    
    assert special_handler.injection_count == initial_count + 1
    assert special_handler.last_injection_time > initial_time


def test_multiple_injections(special_handler):
    """Test multiple injection recordings."""
    # Record multiple injections
    for i in range(5):
        special_handler.record_injection()
    
    assert special_handler.injection_count == 5
    
    # Get final statistics
    stats = special_handler.get_statistics()
    assert stats['injection_count'] == 5


def test_connectome_manager_integration(special_handler):
    """Test integration with connectome manager."""
    # Verify direct access to cortical_idx=1
    neurons = special_handler.get_power_area_neurons()
    
    # Should call get_neurons_by_cortical_idx with cortical_idx=1
    assert neurons == [200, 201, 202, 203, 204]


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 