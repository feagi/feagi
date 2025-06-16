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
Unit tests for Special Area Handler.

Tests the detection and management of special cortical areas,
particularly power areas with automatic FCL injection.
"""

from typing import Any, Dict, List

import pytest

from feagi.npu.special_area_handler import SpecialAreaConfig, SpecialAreaHandler


class MockCorticalArea:
    """Mock cortical area for testing."""

    def __init__(self, cortical_id: str, name: str, properties: Dict[str, Any] = None):
        self.cortical_id = cortical_id
        self.id = cortical_id  # For backward compatibility
        self.name = name
        self.properties = properties or {}


class MockConnectomeManager:
    """Mock connectome manager for testing."""

    def __init__(self):
        self.cortical_areas = {}
        self.neuron_mappings = {}

    def add_area(self, cortical_id: str, name: str, properties: Dict[str, Any] = None):
        """Add a cortical area for testing."""
        area = MockCorticalArea(cortical_id, name, properties)
        self.cortical_areas[cortical_id] = area
        return area

    def get_neurons_by_area(self, cortical_id: str) -> List[int]:
        """Return mock neurons for an area."""
        return self.neuron_mappings.get(cortical_id, [])

    def set_area_neurons(self, cortical_id: str, neuron_ids: List[int]):
        """Set neurons for an area (for testing)."""
        self.neuron_mappings[cortical_id] = neuron_ids


@pytest.fixture
def mock_connectome_manager():
    """Create a mock connectome manager."""
    return MockConnectomeManager()


@pytest.fixture
def special_area_handler(mock_connectome_manager):
    """Create a special area handler with mock connectome."""
    return SpecialAreaHandler(mock_connectome_manager)


class TestSpecialAreaHandler:
    """Test cases for SpecialAreaHandler."""

    def test_init(self, special_area_handler):
        """Test initialization of special area handler."""
        assert special_area_handler.connectome_manager is not None
        assert special_area_handler.special_areas == {}
        assert special_area_handler.power_areas == set()
        assert special_area_handler.power_area_neurons == {}
        assert special_area_handler.injection_count == 0

    def test_detect_power_area_by_suffix(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test detection of power areas by suffix pattern."""
        # Add power areas with different suffix patterns
        mock_connectome_manager.add_area("motor_pwr", "Motor Power", {})
        mock_connectome_manager.add_area("visual_pwr", "Visual Power", {})
        mock_connectome_manager.add_area("regular_area", "Regular Area", {})

        # Set up neurons for power areas
        mock_connectome_manager.set_area_neurons("motor_pwr", [1001, 1002, 1003])
        mock_connectome_manager.set_area_neurons("visual_pwr", [2001, 2002])
        mock_connectome_manager.set_area_neurons("regular_area", [3001, 3002])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify power areas detected
        assert "motor_pwr" in special_area_handler.power_areas
        assert "visual_pwr" in special_area_handler.power_areas
        assert "regular_area" not in special_area_handler.power_areas

        # Verify neurons cached
        assert special_area_handler.power_area_neurons["motor_pwr"] == [
            1001,
            1002,
            1003,
        ]
        assert special_area_handler.power_area_neurons["visual_pwr"] == [2001, 2002]

    def test_detect_power_area_by_exact_name(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test detection of power area by exact name match."""
        # Add exact match power area
        mock_connectome_manager.add_area("___pwr", "Global Power", {})
        mock_connectome_manager.add_area("other_area", "Other Area", {})

        # Set up neurons
        mock_connectome_manager.set_area_neurons("___pwr", [5001, 5002, 5003, 5004])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify detection
        assert "___pwr" in special_area_handler.power_areas
        assert "other_area" not in special_area_handler.power_areas
        assert special_area_handler.power_area_neurons["___pwr"] == [
            5001,
            5002,
            5003,
            5004,
        ]

    def test_detect_power_area_by_property(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test detection of power area by property flag."""
        # Add area with power injection property
        mock_connectome_manager.add_area(
            "attention",
            "Attention Area",
            {"__power_injection": True, "injection_timing": "pre_burst"},
        )
        mock_connectome_manager.add_area(
            "memory", "Memory Area", {"some_other_property": True}
        )

        # Set up neurons
        mock_connectome_manager.set_area_neurons("attention", [4001, 4002])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify detection
        assert "attention" in special_area_handler.power_areas
        assert "memory" not in special_area_handler.power_areas
        assert special_area_handler.power_area_neurons["attention"] == [4001, 4002]

    def test_detect_modulator_areas(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test detection of modulator areas."""
        # Add modulator areas
        mock_connectome_manager.add_area("dopamine_mod", "Dopamine Modulator", {})
        mock_connectome_manager.add_area(
            "serotonin", "Serotonin Area", {"__modulator": True}
        )

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify modulator detection (not power areas)
        assert "dopamine_mod" in special_area_handler.special_areas
        assert "serotonin" in special_area_handler.special_areas
        assert "dopamine_mod" not in special_area_handler.power_areas
        assert "serotonin" not in special_area_handler.power_areas

        # Verify configurations
        config1 = special_area_handler.get_special_config("dopamine_mod")
        config2 = special_area_handler.get_special_config("serotonin")
        assert config1.area_type == "modulator"
        assert config2.area_type == "modulator"

    def test_special_area_configuration(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test special area configuration creation."""
        # Add power area with custom properties
        mock_connectome_manager.add_area(
            "custom_pwr",
            "Custom Power",
            {"injection_timing": "during_burst", "injection_probability": 0.7},
        )

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify configuration
        config = special_area_handler.get_special_config("custom_pwr")
        assert config is not None
        assert config.area_id == "custom_pwr"
        assert config.area_type == "power"
        assert config.injection_timing == "during_burst"
        assert config.injection_probability == 0.7
        assert config.enabled is True

    def test_get_power_area_neurons(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test retrieving neurons for power areas."""
        # Add power area
        mock_connectome_manager.add_area("test_pwr", "Test Power", {})
        mock_connectome_manager.set_area_neurons("test_pwr", [7001, 7002, 7003])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Test neuron retrieval
        neurons = special_area_handler.get_power_area_neurons("test_pwr")
        assert neurons == [7001, 7002, 7003]

        # Test non-existent area
        empty_neurons = special_area_handler.get_power_area_neurons("non_existent")
        assert empty_neurons == []

        # Test non-power area
        mock_connectome_manager.add_area("regular", "Regular Area", {})
        regular_neurons = special_area_handler.get_power_area_neurons("regular")
        assert regular_neurons == []

    def test_get_all_power_neurons(self, special_area_handler, mock_connectome_manager):
        """Test getting all power area neurons at once."""
        # Add multiple power areas
        mock_connectome_manager.add_area("power1_pwr", "Power 1", {})
        mock_connectome_manager.add_area("power2_pwr", "Power 2", {})
        mock_connectome_manager.add_area("regular", "Regular", {})

        # Set up neurons
        mock_connectome_manager.set_area_neurons("power1_pwr", [1001, 1002])
        mock_connectome_manager.set_area_neurons("power2_pwr", [2001, 2002, 2003])
        mock_connectome_manager.set_area_neurons("regular", [3001])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Get all power neurons
        all_power_neurons = special_area_handler.get_all_power_neurons()

        # Verify results
        assert "power1_pwr" in all_power_neurons
        assert "power2_pwr" in all_power_neurons
        assert "regular" not in all_power_neurons
        assert all_power_neurons["power1_pwr"] == [1001, 1002]
        assert all_power_neurons["power2_pwr"] == [2001, 2002, 2003]

    def test_is_special_area(self, special_area_handler, mock_connectome_manager):
        """Test checking if an area is special."""
        # Add areas
        mock_connectome_manager.add_area("special_pwr", "Special Power", {})
        mock_connectome_manager.add_area("regular", "Regular Area", {})

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Test checks
        assert special_area_handler.is_special_area("special_pwr") is True
        assert special_area_handler.is_special_area("regular") is False
        assert special_area_handler.is_special_area("non_existent") is False

    def test_is_power_area(self, special_area_handler, mock_connectome_manager):
        """Test checking if an area is a power area."""
        # Add areas
        mock_connectome_manager.add_area("power_pwr", "Power Area", {})
        mock_connectome_manager.add_area("mod_mod", "Modulator Area", {})
        mock_connectome_manager.add_area("regular", "Regular Area", {})

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Test checks
        assert special_area_handler.is_power_area("power_pwr") is True
        assert special_area_handler.is_power_area("mod_mod") is False
        assert special_area_handler.is_power_area("regular") is False
        assert special_area_handler.is_power_area("non_existent") is False

    def test_update_power_area_cache(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test updating power area neuron cache."""
        # Add power area
        mock_connectome_manager.add_area("dynamic_pwr", "Dynamic Power", {})
        mock_connectome_manager.set_area_neurons("dynamic_pwr", [1001, 1002])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Verify initial cache
        assert special_area_handler.power_area_neurons["dynamic_pwr"] == [1001, 1002]

        # Change neurons in connectome
        mock_connectome_manager.set_area_neurons(
            "dynamic_pwr", [1001, 1002, 1003, 1004]
        )

        # Update cache
        special_area_handler.update_power_area_cache("dynamic_pwr")

        # Verify cache updated
        assert special_area_handler.power_area_neurons["dynamic_pwr"] == [
            1001,
            1002,
            1003,
            1004,
        ]

    def test_refresh_all_caches(self, special_area_handler, mock_connectome_manager):
        """Test refreshing all caches."""
        # Add initial areas
        mock_connectome_manager.add_area("old_pwr", "Old Power", {})
        mock_connectome_manager.set_area_neurons("old_pwr", [1001])

        # Detect special areas
        special_area_handler.detect_special_areas()
        assert len(special_area_handler.power_areas) == 1

        # Add new power area
        mock_connectome_manager.add_area("new_pwr", "New Power", {})
        mock_connectome_manager.set_area_neurons("new_pwr", [2001, 2002])

        # Refresh caches
        special_area_handler.refresh_all_caches()

        # Verify new area detected
        assert len(special_area_handler.power_areas) == 2
        assert "new_pwr" in special_area_handler.power_areas
        assert special_area_handler.power_area_neurons["new_pwr"] == [2001, 2002]

    def test_get_statistics(self, special_area_handler, mock_connectome_manager):
        """Test getting handler statistics."""
        # Add various special areas
        mock_connectome_manager.add_area("power1_pwr", "Power 1", {})
        mock_connectome_manager.add_area("power2_pwr", "Power 2", {})
        mock_connectome_manager.add_area("mod1_mod", "Modulator 1", {})

        # Set up neurons
        mock_connectome_manager.set_area_neurons("power1_pwr", [1001, 1002])
        mock_connectome_manager.set_area_neurons("power2_pwr", [2001, 2002, 2003])

        # Detect special areas
        special_area_handler.detect_special_areas()

        # Record some injections
        special_area_handler.record_injection()
        special_area_handler.record_injection()

        # Get statistics
        stats = special_area_handler.get_statistics()

        # Verify statistics
        assert stats["total_special_areas"] == 3
        assert stats["power_areas_count"] == 2
        assert stats["total_power_neurons"] == 5  # 2 + 3 neurons
        assert stats["injection_count"] == 2
        assert "power1_pwr" in stats["power_areas"]
        assert "power2_pwr" in stats["power_areas"]
        assert "power" in stats["special_area_types"]
        assert "modulator" in stats["special_area_types"]

    def test_record_injection(self, special_area_handler):
        """Test recording injection statistics."""
        initial_count = special_area_handler.injection_count
        initial_time = special_area_handler.last_injection_time

        # Record injection
        special_area_handler.record_injection()

        # Verify updates
        assert special_area_handler.injection_count == initial_count + 1
        assert special_area_handler.last_injection_time > initial_time

    def test_no_cortical_areas(self, mock_connectome_manager):
        """Test handler with no cortical areas."""
        # Remove cortical_areas attribute
        delattr(mock_connectome_manager, "cortical_areas")

        handler = SpecialAreaHandler(mock_connectome_manager)
        handler.detect_special_areas()

        # Should handle gracefully
        assert len(handler.special_areas) == 0
        assert len(handler.power_areas) == 0

    def test_error_handling_in_neuron_retrieval(
        self, special_area_handler, mock_connectome_manager
    ):
        """Test error handling when neuron retrieval fails."""
        # Add power area
        mock_connectome_manager.add_area("error_pwr", "Error Power", {})

        # Make get_neurons_by_area raise an exception
        def raise_error(cortical_id):
            raise Exception("Neuron retrieval failed")

        mock_connectome_manager.get_neurons_by_area = raise_error

        # Detect special areas (should handle error gracefully)
        special_area_handler.detect_special_areas()

        # Area should be detected but neurons should be empty
        assert "error_pwr" in special_area_handler.power_areas
        assert special_area_handler.power_area_neurons.get("error_pwr", []) == []


class TestSpecialAreaConfig:
    """Test cases for SpecialAreaConfig dataclass."""

    def test_config_creation(self):
        """Test creating a special area configuration."""
        config = SpecialAreaConfig(
            area_id="test_pwr",
            area_type="power",
            injection_timing="pre_burst",
            injection_probability=0.8,
            target_neurons={1001, 1002, 1003},
        )

        assert config.area_id == "test_pwr"
        assert config.area_type == "power"
        assert config.injection_timing == "pre_burst"
        assert config.injection_probability == 0.8
        assert config.target_neurons == {1001, 1002, 1003}
        assert config.enabled is True

    def test_config_defaults(self):
        """Test default values in configuration."""
        config = SpecialAreaConfig(
            area_id="test",
            area_type="power",
            injection_timing="pre_burst",
            injection_probability=1.0,
        )

        assert config.target_neurons is None
        assert config.enabled is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
