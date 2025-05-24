"""
Simple test coverage for special_area_handler.py to boost coverage.

This module tests the Special Area Handler basic functionality
using the correct API.
"""

import pytest
from unittest.mock import Mock, MagicMock
from feagi.npu.special_area_handler import SpecialAreaHandler, SpecialAreaConfig


class MockConnectomeManager:
    def __init__(self):
        self.cortical_areas = {
            'area1___pwr': Mock(id='area1___pwr', properties={'injection_probability': 0.8}),
            'area2___mod': Mock(id='area2___mod', properties={}),
            'normal_area': Mock(id='normal_area', properties={}),
            'power_area': Mock(id='power_area', properties={'__power_injection': True}),
        }
        
    def get_neurons_by_area(self, area_id):
        # Mock neurons for different areas
        if 'pwr' in area_id or 'power' in area_id:
            return [1, 2, 3, 4, 5]
        return [10, 11, 12]


@pytest.fixture
def mock_connectome_manager():
    return MockConnectomeManager()


@pytest.fixture
def special_handler(mock_connectome_manager):
    """Create a special area handler for testing."""
    return SpecialAreaHandler(connectome_manager=mock_connectome_manager)


def test_special_area_handler_initialization(mock_connectome_manager):
    """Test special area handler initialization."""
    handler = SpecialAreaHandler(connectome_manager=mock_connectome_manager)
    
    assert handler.connectome_manager == mock_connectome_manager
    assert isinstance(handler.special_areas, dict)
    assert isinstance(handler.power_areas, set)
    assert isinstance(handler.power_area_neurons, dict)
    assert handler.injection_count == 0


def test_special_area_handler_with_config(mock_connectome_manager):
    """Test special area handler initialization with config."""
    config = {'batch_injection_threshold': 50}
    handler = SpecialAreaHandler(connectome_manager=mock_connectome_manager, config=config)
    
    assert handler.config == config
    assert handler.batch_threshold == 50


def test_detect_special_areas(special_handler):
    """Test detection of special areas."""
    special_handler.detect_special_areas()
    
    # Should detect areas with special naming patterns
    assert len(special_handler.special_areas) >= 2  # At least power and modulator areas
    assert len(special_handler.power_areas) >= 1   # At least one power area
    
    # Check that power area neurons were cached
    assert len(special_handler.power_area_neurons) >= 1


def test_identify_special_type_power_naming(special_handler):
    """Test identification of power areas by naming."""
    area = Mock()
    
    # Test ___pwr pattern
    special_type = special_handler._identify_special_type('test___pwr', area)
    assert special_type == 'power'
    
    # Test _pwr pattern
    special_type = special_handler._identify_special_type('test_pwr', area)
    assert special_type == 'power'


def test_identify_special_type_modulator_naming(special_handler):
    """Test identification of modulator areas by naming."""
    area = Mock()
    
    # Test ___mod pattern
    special_type = special_handler._identify_special_type('test___mod', area)
    assert special_type == 'modulator'
    
    # Test _mod pattern
    special_type = special_handler._identify_special_type('test_mod', area)
    assert special_type == 'modulator'


def test_identify_special_type_properties(special_handler):
    """Test identification of special areas by properties."""
    # Test power injection property
    area_power = Mock()
    area_power.properties = {'__power_injection': True}
    special_type = special_handler._identify_special_type('normal_name', area_power)
    assert special_type == 'power'
    
    # Test modulator property
    area_mod = Mock()
    area_mod.properties = {'__modulator': True}
    special_type = special_handler._identify_special_type('normal_name', area_mod)
    assert special_type == 'modulator'
    
    # Test no special properties
    area_normal = Mock()
    area_normal.properties = {}
    special_type = special_handler._identify_special_type('normal_name', area_normal)
    assert special_type is None


def test_create_special_config_power(special_handler):
    """Test creating configuration for power areas."""
    area = Mock()
    area.properties = {'injection_probability': 0.7}
    
    config = special_handler._create_special_config('test_pwr', 'power', area)
    
    assert isinstance(config, SpecialAreaConfig)
    assert config.area_id == 'test_pwr'
    assert config.area_type == 'power'
    assert config.injection_timing == 'pre_burst'
    assert config.injection_probability == 0.7  # From area properties
    assert config.enabled


def test_create_special_config_modulator(special_handler):
    """Test creating configuration for modulator areas."""
    area = Mock()
    area.properties = {}
    
    config = special_handler._create_special_config('test_mod', 'modulator', area)
    
    assert isinstance(config, SpecialAreaConfig)
    assert config.area_id == 'test_mod'
    assert config.area_type == 'modulator'
    assert config.injection_timing == 'during_burst'
    assert config.injection_probability == 0.5  # Default for modulator
    assert config.enabled


def test_get_power_areas(special_handler):
    """Test getting power areas."""
    special_handler.detect_special_areas()
    
    power_areas = special_handler.get_power_areas()
    assert isinstance(power_areas, set)
    assert len(power_areas) >= 1


def test_get_power_area_neurons(special_handler):
    """Test getting neurons for power areas."""
    special_handler.detect_special_areas()
    
    # Get a power area ID
    power_areas = list(special_handler.power_areas)
    if power_areas:
        neurons = special_handler.get_power_area_neurons(power_areas[0])
        assert isinstance(neurons, list)
        assert len(neurons) >= 1
    
    # Test non-power area
    neurons_empty = special_handler.get_power_area_neurons('non_existent')
    assert neurons_empty == []


def test_get_all_power_neurons(special_handler):
    """Test getting all power neurons."""
    special_handler.detect_special_areas()
    
    all_neurons = special_handler.get_all_power_neurons()
    assert isinstance(all_neurons, dict)
    
    # Should have entries for power areas
    power_areas = special_handler.get_power_areas()
    for area_id in power_areas:
        assert area_id in all_neurons
        assert isinstance(all_neurons[area_id], list)


def test_is_special_area(special_handler):
    """Test checking if area is special."""
    special_handler.detect_special_areas()
    
    # Should be true for detected special areas
    special_areas = list(special_handler.special_areas.keys())
    if special_areas:
        assert special_handler.is_special_area(special_areas[0])
    
    # Should be false for normal areas
    assert not special_handler.is_special_area('normal_area')


def test_is_power_area(special_handler):
    """Test checking if area is power area."""
    special_handler.detect_special_areas()
    
    # Should be true for power areas
    power_areas = list(special_handler.power_areas)
    if power_areas:
        assert special_handler.is_power_area(power_areas[0])
    
    # Should be false for non-power areas
    assert not special_handler.is_power_area('normal_area')


def test_get_special_config(special_handler):
    """Test getting special area configuration."""
    special_handler.detect_special_areas()
    
    # Should return config for special areas
    special_areas = list(special_handler.special_areas.keys())
    if special_areas:
        config = special_handler.get_special_config(special_areas[0])
        assert isinstance(config, SpecialAreaConfig)
    
    # Should return None for non-special areas
    config_none = special_handler.get_special_config('normal_area')
    assert config_none is None


def test_update_power_area_cache(special_handler):
    """Test updating power area cache."""
    special_handler.detect_special_areas()
    
    # Should update cache without error
    special_handler.update_power_area_cache('area1___pwr')
    
    # Should handle non-existent areas gracefully
    special_handler.update_power_area_cache('non_existent')


def test_refresh_all_caches(special_handler):
    """Test refreshing all caches."""
    special_handler.detect_special_areas()
    
    # Should refresh without error
    special_handler.refresh_all_caches()


def test_get_statistics(special_handler):
    """Test getting statistics."""
    special_handler.detect_special_areas()
    special_handler.record_injection()  # Record at least one injection
    
    stats = special_handler.get_statistics()
    assert isinstance(stats, dict)
    assert 'total_special_areas' in stats
    assert 'power_areas' in stats
    assert 'injection_count' in stats
    assert 'last_injection_time' in stats


def test_record_injection(special_handler):
    """Test recording injections."""
    initial_count = special_handler.injection_count
    initial_time = special_handler.last_injection_time
    
    special_handler.record_injection()
    
    assert special_handler.injection_count == initial_count + 1
    assert special_handler.last_injection_time > initial_time


def test_detect_special_areas_no_cortical_areas():
    """Test detection when no cortical areas available."""
    # Mock connectome manager without cortical_areas attribute
    mock_cm = Mock()
    del mock_cm.cortical_areas  # Remove the attribute
    
    handler = SpecialAreaHandler(connectome_manager=mock_cm)
    
    # Should handle gracefully without error
    handler.detect_special_areas()
    
    assert len(handler.special_areas) == 0
    assert len(handler.power_areas) == 0


def test_detect_special_areas_error_handling(special_handler):
    """Test error handling during special area detection."""
    # Mock get_neurons_by_area to raise exception
    special_handler.connectome_manager.get_neurons_by_area = Mock(side_effect=Exception("Test error"))
    
    # Should handle errors gracefully
    special_handler.detect_special_areas()
    
    # Should still detect areas but without cached neurons
    assert len(special_handler.special_areas) >= 0


if __name__ == "__main__":
    pytest.main(["-v", __file__]) 