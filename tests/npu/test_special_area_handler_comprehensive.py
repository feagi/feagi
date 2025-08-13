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
Comprehensive test coverage for Special Area Handler.

This module tests all functionality in special_area_handler.py to achieve
high code coverage, including special area detection, configuration, caching,
and statistics.
"""

from typing import Any, Dict, List
from unittest.mock import Mock, patch

import pytest

from feagi.npu.special_area_handler import SpecialAreaHandler


class MockCorticalArea:
    """Mock cortical area for testing."""

    def __init__(self, cortical_id: str, properties: Dict[str, Any] = None):
        self.id = cortical_id
        self.properties = properties or {}


class MockConnectomeManager:
    """Mock connectome manager for testing."""

    def __init__(self):
        self.cortical_areas = {
            "regular_area": MockCorticalArea("regular_area"),
            "power_area_pwr": MockCorticalArea("power_area_pwr"),
            "_power": MockCorticalArea("_power"),
            "modulator_mod": MockCorticalArea("modulator_mod"),
            "___mod": MockCorticalArea("___mod"),
            "memory_mem": MockCorticalArea("memory_mem"),
            "___mem": MockCorticalArea("___mem"),
            "property_power": MockCorticalArea(
                "property_power", {"__power_injection": True}
            ),
            "property_modulator": MockCorticalArea(
                "property_modulator", {"__modulator": True}
            ),
            "complex_power": MockCorticalArea(
                "complex_power",
                {
                    "__power_injection": True,
                    "injection_timing": "during_burst",
                    "injection_probability": 0.8,
                },
            ),
        }

        self.neuron_lists = {
            "power_area_pwr": [1, 2, 3, 4, 5],
            "_power": [10, 11, 12],
            "property_power": [20, 21, 22, 23],
            "complex_power": [30, 31, 32],
        }

    def get_neurons_by_area(self, cortical_id: str) -> List[int]:
        return self.neuron_lists.get(cortical_id, [])


@pytest.fixture
def mock_connectome_manager():
    return MockConnectomeManager()


@pytest.fixture
def handler(mock_connectome_manager):
    """Create special area handler with default configuration."""
    return SpecialAreaHandler(mock_connectome_manager)


# Test basic initialization
def test_special_area_handler_initialization(mock_connectome_manager):
    """Test special area handler initialization."""
    handler = SpecialAreaHandler(mock_connectome_manager)

    assert handler.connectome_manager == mock_connectome_manager
    assert handler.config == {}
    assert isinstance(handler.special_areas, dict)
    assert isinstance(handler.power_areas, set)
    assert isinstance(handler.power_area_neurons, dict)
    assert handler.injection_count == 0
    assert handler.last_injection_time == 0.0
    assert handler.batch_threshold == 100  # default


def test_special_area_handler_custom_config(mock_connectome_manager):
    """Test initialization with custom configuration."""
    config = {"batch_injection_threshold": 50}
    handler = SpecialAreaHandler(mock_connectome_manager, config)

    assert handler.config == config
    assert handler.batch_threshold == 50


# Test special area detection
def test_detect_special_areas(handler):
    """Test detection of special areas."""
    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.detect_special_areas()

        # Should have detected multiple special areas
        assert len(handler.special_areas) > 0
        assert len(handler.power_areas) > 0

        # Check specific power areas
        expected_power_areas = {
            "power_area_pwr",
            "_power",
            "property_power",
            "complex_power",
        }
        assert expected_power_areas.issubset(handler.power_areas)

        # Should have cached neurons for power areas
        assert "power_area_pwr" in handler.power_area_neurons
        assert handler.power_area_neurons["power_area_pwr"] == [1, 2, 3, 4, 5]

        # Should have logged detection
        mock_logger.info.assert_called()


def test_detect_special_areas_no_cortical_areas(mock_connectome_manager):
    """Test detection when connectome manager has no cortical_areas attribute."""
    # Remove cortical_areas attribute
    del mock_connectome_manager.cortical_areas

    handler = SpecialAreaHandler(mock_connectome_manager)

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.detect_special_areas()

        # Should warn and continue gracefully
        mock_logger.warning.assert_called_with(
            "No cortical areas available for special area detection"
        )
        assert len(handler.special_areas) == 0


def test_detect_special_areas_neuron_error(handler):
    """Test detection when getting neurons fails for some areas."""

    # Mock get_neurons_by_area to raise exception for one area
    def side_effect(cortical_id):
        if cortical_id == "power_area_pwr":
            raise Exception("Neuron error")
        return handler.connectome_manager.neuron_lists.get(cortical_id, [])

    handler.connectome_manager.get_neurons_by_area = Mock(side_effect=side_effect)

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.detect_special_areas()

        # Should handle error gracefully and log it
        mock_logger.error.assert_called()

        # Should still detect other power areas
        assert len(handler.power_areas) > 0


# Test area type identification
def test_identify_special_type_naming_patterns(handler):
    """Test identification of special types by naming patterns."""
    mock_area = Mock()
    mock_area.properties = {}  # Ensure no conflicting properties

    # Test power area patterns
    assert handler._identify_special_type("area_pwr", mock_area) == "power"
    assert handler._identify_special_type("_power", mock_area) == "power"

    # Test modulator patterns
    assert handler._identify_special_type("area_mod", mock_area) == "modulator"
    assert handler._identify_special_type("___mod", mock_area) == "modulator"

    # Test memory patterns
    assert handler._identify_special_type("area_mem", mock_area) == "enhanced_memory"
    assert handler._identify_special_type("___mem", mock_area) == "enhanced_memory"

    # Test regular area
    assert handler._identify_special_type("regular_area", mock_area) is None


def test_identify_special_type_property_based(handler):
    """Test identification based on area properties."""
    # Test power injection property
    power_area = Mock()
    power_area.properties = {"__power_injection": True}
    assert handler._identify_special_type("any_name", power_area) == "power"

    # Test modulator property
    mod_area = Mock()
    mod_area.properties = {"__modulator": True}
    assert handler._identify_special_type("any_name", mod_area) == "modulator"

    # Test area without properties
    no_props_area = Mock()
    del no_props_area.properties
    assert handler._identify_special_type("any_name", no_props_area) is None


# Test configuration creation
def test_create_special_config_power(handler):
    """Test creation of power area configuration."""
    mock_area = Mock()
    mock_area.properties = {}

    config = handler._create_special_config("test_power", "power", mock_area)

    assert config.area_id == "test_power"
    assert config.area_type == "power"
    assert config.injection_timing == "pre_burst"
    assert config.injection_probability == 1.0
    assert config.enabled == True


def test_create_special_config_modulator(handler):
    """Test creation of modulator area configuration."""
    mock_area = Mock()
    mock_area.properties = {}

    config = handler._create_special_config("test_mod", "modulator", mock_area)

    assert config.area_id == "test_mod"
    assert config.area_type == "modulator"
    assert config.injection_timing == "during_burst"
    assert config.injection_probability == 0.5
    assert config.enabled == True


def test_create_special_config_memory(handler):
    """Test creation of memory area configuration."""
    mock_area = Mock()
    mock_area.properties = {}

    config = handler._create_special_config("test_mem", "enhanced_memory", mock_area)

    assert config.area_id == "test_mem"
    assert config.area_type == "enhanced_memory"
    assert config.injection_timing == "post_burst"
    assert config.injection_probability == 1.0
    assert config.enabled == True


def test_create_special_config_with_overrides(handler):
    """Test configuration creation with property overrides."""
    mock_area = Mock()
    mock_area.properties = {
        "injection_timing": "custom_timing",
        "injection_probability": 0.75,
    }

    config = handler._create_special_config("test_area", "power", mock_area)

    assert config.injection_timing == "custom_timing"
    assert config.injection_probability == 0.75


def test_create_special_config_no_properties(handler):
    """Test configuration creation with area that has no properties."""
    mock_area = Mock()
    del mock_area.properties

    config = handler._create_special_config("test_area", "power", mock_area)

    # Should use defaults
    assert config.injection_timing == "pre_burst"
    assert config.injection_probability == 1.0


# Test getter methods
def test_get_power_areas(handler):
    """Test getting power areas."""
    handler.detect_special_areas()

    power_areas = handler.get_power_areas()

    assert isinstance(power_areas, set)
    assert "power_area_pwr" in power_areas
    assert "_power" in power_areas

    # Should return a copy (modification doesn't affect original)
    power_areas.add("fake_area")
    assert "fake_area" not in handler.power_areas


def test_get_power_area_neurons_cached(handler):
    """Test getting neurons for a power area (cached)."""
    handler.detect_special_areas()

    neurons = handler.get_power_area_neurons("power_area_pwr")

    assert neurons == [1, 2, 3, 4, 5]

    # Should return a copy
    neurons.append(999)
    assert 999 not in handler.power_area_neurons["power_area_pwr"]


def test_get_power_area_neurons_not_power_area(handler):
    """Test getting neurons for a non-power area."""
    handler.detect_special_areas()

    neurons = handler.get_power_area_neurons("regular_area")

    assert neurons == []


def test_get_power_area_neurons_fallback(handler):
    """Test getting neurons with fallback to connectome manager."""
    handler.detect_special_areas()

    # Clear cache to force fallback
    del handler.power_area_neurons["power_area_pwr"]

    neurons = handler.get_power_area_neurons("power_area_pwr")

    # Should get from connectome manager and update cache
    assert neurons == [1, 2, 3, 4, 5]
    assert handler.power_area_neurons["power_area_pwr"] == [1, 2, 3, 4, 5]


def test_get_power_area_neurons_error(handler):
    """Test error handling when getting neurons fails."""
    handler.detect_special_areas()

    # Clear cache and mock error
    del handler.power_area_neurons["power_area_pwr"]
    handler.connectome_manager.get_neurons_by_area = Mock(
        side_effect=Exception("Error")
    )

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        neurons = handler.get_power_area_neurons("power_area_pwr")

        assert neurons == []
        mock_logger.error.assert_called()


def test_get_all_power_neurons(handler):
    """Test getting all power neurons."""
    handler.detect_special_areas()

    all_neurons = handler.get_all_power_neurons()

    assert isinstance(all_neurons, dict)
    assert "power_area_pwr" in all_neurons
    assert all_neurons["power_area_pwr"] == [1, 2, 3, 4, 5]
    assert "_power" in all_neurons
    assert all_neurons["_power"] == [10, 11, 12]


def test_get_all_power_neurons_empty_area(handler):
    """Test getting all power neurons when some areas have no neurons."""
    handler.detect_special_areas()

    # Mock one area to return empty
    handler.power_area_neurons["_power"] = []

    all_neurons = handler.get_all_power_neurons()

    # Should not include empty areas
    assert "_power" not in all_neurons
    assert "power_area_pwr" in all_neurons


# Test utility methods
def test_is_special_area(handler):
    """Test checking if an area is special."""
    handler.detect_special_areas()

    assert handler.is_special_area("power_area_pwr")
    assert handler.is_special_area("modulator_mod")
    assert not handler.is_special_area("regular_area")
    assert not handler.is_special_area("nonexistent_area")


def test_is_power_area(handler):
    """Test checking if an area is a power area."""
    handler.detect_special_areas()

    assert handler.is_power_area("power_area_pwr")
    assert handler.is_power_area("_power")
    assert not handler.is_power_area("modulator_mod")
    assert not handler.is_power_area("regular_area")


def test_get_special_config(handler):
    """Test getting configuration for special areas."""
    handler.detect_special_areas()

    config = handler.get_special_config("power_area_pwr")
    assert config is not None
    assert config.area_type == "power"

    config = handler.get_special_config("regular_area")
    assert config is None


def test_update_power_area_cache(handler):
    """Test updating power area cache."""
    handler.detect_special_areas()

    # Change the neuron list in connectome manager
    handler.connectome_manager.neuron_lists["power_area_pwr"] = [100, 101, 102]

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.update_power_area_cache("power_area_pwr")

        # Should update cache
        assert handler.power_area_neurons["power_area_pwr"] == [100, 101, 102]
        mock_logger.debug.assert_called()


def test_update_power_area_cache_non_power_area(handler):
    """Test updating cache for non-power area."""
    handler.detect_special_areas()

    # Should do nothing for non-power area
    handler.update_power_area_cache("regular_area")

    # No change or error


def test_update_power_area_cache_error(handler):
    """Test error handling when updating cache."""
    handler.detect_special_areas()

    handler.connectome_manager.get_neurons_by_area = Mock(
        side_effect=Exception("Error")
    )

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.update_power_area_cache("power_area_pwr")

        mock_logger.error.assert_called()


def test_refresh_all_caches(handler):
    """Test refreshing all caches."""
    # Initial detection
    handler.detect_special_areas()
    initial_count = len(handler.special_areas)

    # Add new special area to connectome
    handler.connectome_manager.cortical_areas["new_power_pwr"] = MockCorticalArea(
        "new_power_pwr"
    )
    handler.connectome_manager.neuron_lists["new_power_pwr"] = [200, 201]

    with patch("feagi.npu.special_area_handler.logger") as mock_logger:
        handler.refresh_all_caches()

        # Should detect new area
        assert len(handler.special_areas) > initial_count
        assert "new_power_pwr" in handler.power_areas

        mock_logger.info.assert_called()


def test_get_statistics(handler):
    """Test getting statistics."""
    handler.detect_special_areas()

    # Record some injections
    handler.record_injection()
    handler.record_injection()

    stats = handler.get_statistics()

    assert isinstance(stats, dict)
    assert "total_special_areas" in stats
    assert "power_areas_count" in stats
    assert "total_power_neurons" in stats
    assert "injection_count" in stats
    assert "last_injection_time" in stats
    assert "power_areas" in stats
    assert "special_area_types" in stats

    assert stats["injection_count"] == 2
    assert stats["power_areas_count"] > 0
    assert isinstance(stats["power_areas"], list)
    assert isinstance(stats["special_area_types"], list)


def test_record_injection(handler):
    """Test recording injection for statistics."""
    initial_count = handler.injection_count
    initial_time = handler.last_injection_time

    with patch("time.perf_counter", return_value=123.456):
        handler.record_injection()

    assert handler.injection_count == initial_count + 1
    assert handler.last_injection_time == 123.456


# Test integration scenarios
def test_full_workflow(handler):
    """Test full workflow from detection to injection."""
    # 1. Detect special areas
    handler.detect_special_areas()

    # 2. Verify detection
    assert len(handler.power_areas) > 0
    assert "power_area_pwr" in handler.power_areas

    # 3. Get power neurons for injection
    all_neurons = handler.get_all_power_neurons()
    assert len(all_neurons) > 0

    # 4. Get specific area config
    config = handler.get_special_config("power_area_pwr")
    assert config.area_type == "power"
    assert config.injection_timing == "pre_burst"

    # 5. Record injection
    handler.record_injection()

    # 6. Check statistics
    stats = handler.get_statistics()
    assert stats["injection_count"] == 1


def test_complex_area_properties(handler):
    """Test handling of complex area with custom properties."""
    handler.detect_special_areas()

    # Should detect complex_power area
    assert "complex_power" in handler.power_areas

    # Should use custom properties
    config = handler.get_special_config("complex_power")
    assert config.injection_timing == "during_burst"
    assert config.injection_probability == 0.8


def test_multiple_detection_calls(handler):
    """Test that multiple detection calls work correctly."""
    # First detection
    handler.detect_special_areas()
    first_count = len(handler.special_areas)

    # Second detection (should clear and re-detect)
    handler.detect_special_areas()
    second_count = len(handler.special_areas)

    # Should have same count (cleared and re-detected)
    assert first_count == second_count


if __name__ == "__main__":
    pytest.main(["-v", __file__])
