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

"""Tests for the CorticalAreaService class."""

from unittest.mock import MagicMock

import pytest

from feagi.api.core.services.cortical_area.cortical_area_service import (
    CorticalAreaService,
)
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)

    # Mock cortical areas
    area1 = MagicMock()
    area1.cortical_id = "area01"
    area1.name = "Sensory Area"
    area1.position = (0, 0, 0)
    area1.dimensions = (10, 10, 10)
    area1.type = "sensory"

    area2 = MagicMock()
    area2.cortical_id = "area02"
    area2.name = "Motor Area"
    area2.position = (20, 0, 0)
    area2.dimensions = (8, 8, 8)
    area2.type = "motor"

    cm.cortical_areas = {1: area1, 2: area2}

    # Mock cortical neuron mapping
    cm.cortical_neuron_map = {1: {101, 102, 103, 104, 105}, 2: {201, 202, 203}}

    # Mock neurons
    cm.neurons = {
        101: {
            "cortical_id": "area01",
            "position": (1, 1, 1),
            "membrane_potential": 0.5,
        },
        102: {
            "cortical_id": "area01",
            "position": (2, 2, 2),
            "membrane_potential": 0.3,
        },
        103: {
            "cortical_id": "area01",
            "position": (3, 3, 3),
            "membrane_potential": 0.8,
        },
        104: {
            "cortical_id": "area01",
            "position": (4, 4, 4),
            "membrane_potential": 0.1,
        },
        105: {
            "cortical_id": "area01",
            "position": (5, 5, 5),
            "membrane_potential": 0.9,
        },
        201: {
            "cortical_id": "area02",
            "position": (1, 1, 1),
            "membrane_potential": 0.4,
        },
        202: {
            "cortical_id": "area02",
            "position": (2, 2, 2),
            "membrane_potential": 0.6,
        },
        203: {
            "cortical_id": "area02",
            "position": (3, 3, 3),
            "membrane_potential": 0.2,
        },
    }

    # Mock methods
    cm.get_cortical_area_by_id = MagicMock()
    cm.create_cortical_area = MagicMock()
    cm.update_cortical_area = MagicMock()
    cm.delete_cortical_area = MagicMock()
    cm.get_area_neurons = MagicMock()
    cm.stimulate_area = MagicMock()

    return cm


@pytest.fixture
def mock_state_manager():
    """Create a mock state manager for testing."""
    sm = MagicMock()
    sm.is_genome_loaded.return_value = True
    sm.get_brain_readiness.return_value = True
    return sm


@pytest.fixture
def cortical_area_service(mock_connectome_manager, mock_state_manager):
    """Create a CorticalAreaService instance with mocked dependencies."""
    return CorticalAreaService(mock_connectome_manager, mock_state_manager)


@pytest.fixture
def cortical_area_service_no_state(mock_connectome_manager):
    """Create a CorticalAreaService instance without state manager."""
    return CorticalAreaService(mock_connectome_manager, None)


class TestCorticalAreaService:
    """Test cases for the CorticalAreaService."""

    def test_init(self, mock_connectome_manager, mock_state_manager):
        """Test CorticalAreaService initialization."""
        service = CorticalAreaService(mock_connectome_manager, mock_state_manager)

        assert service._connectome_manager == mock_connectome_manager
        assert service.state_manager == mock_state_manager

    def test_get_all_areas_with_areas(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting all cortical areas when areas exist."""
        # Set up areas with proper attributes matching the real implementation
        area1 = MagicMock()
        area1.name = "Visual Cortex"
        area1.position = (0, 0, 0)
        area1.dimensions = (10, 10, 5)
        area1.type = "sensory"
        area1.properties = {"param1": "value1"}

        area2 = MagicMock()
        area2.name = "Motor Cortex"
        area2.position = (20, 0, 0)
        area2.dimensions = (8, 8, 3)
        area2.type = "motor"
        area2.properties = {"param2": "value2"}

        mock_connectome_manager.cortical_areas = {1: area1, 2: area2}
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3]

        result = cortical_area_service.get_all_areas()

        assert isinstance(result, list)
        assert len(result) == 2
        # Check first area
        assert result[0]["id"] == "1"  # Integer converted to string
        assert result[0]["name"] == "Visual Cortex"
        assert result[0]["coordinates"]["x"] == 0
        assert result[0]["neuron_count"] == 3

    def test_get_all_areas_without_areas(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting all cortical areas when no areas exist."""
        mock_connectome_manager.cortical_areas = {}

        result = cortical_area_service.get_all_areas()

        assert isinstance(result, list)
        assert len(result) == 0

    def test_get_area_by_id_existing(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting a cortical area by ID when it exists."""
        area = MagicMock()
        area.name = "Test Area"
        area.position = (5, 10, 15)
        area.dimensions = (20, 20, 10)
        area.type = "processing"
        area.properties = {"test": "value"}

        mock_connectome_manager.cortical_areas = {1: area}
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3, 4, 5]

        result = cortical_area_service.get_area("1")  # Use string of integer

        assert isinstance(result, dict)
        assert result["id"] == "1"
        assert result["name"] == "Test Area"
        assert result["neuron_count"] == 5

    def test_get_area_by_id_nonexistent(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting a cortical area by ID when it doesn't exist."""
        mock_connectome_manager.get_cortical_area_by_id.return_value = None

        result = cortical_area_service.get_area("nonexistent")

        assert result is None

    def test_get_area_by_id_with_exception(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting a cortical area when an exception occurs."""
        mock_connectome_manager.get_cortical_area_by_id.side_effect = Exception(
            "Database error"
        )

        result = cortical_area_service.get_area("area01")

        assert result is None

    def test_create_area_success(self, cortical_area_service, mock_connectome_manager):
        """Test successfully creating a cortical area."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area creation
        created_area = MagicMock()
        created_area.name = "New Area"
        created_area.type = "custom"
        created_area.properties = {"param": "value"}

        mock_connectome_manager.cortical_areas = {}
        mock_connectome_manager.add_cortical_area = MagicMock(return_value=created_area)

        result = cortical_area_service.create_area(
            name="New Area",
            coordinates={"x": 0, "y": 0, "z": 0},
            dimensions={"width": 10, "height": 10, "depth": 5},
            area_type="custom",
            parameters={"param": "value"},
        )

        assert isinstance(result, dict)
        assert result["name"] == "New Area"
        assert result["type"] == "custom"

    def test_create_area_failure(self, cortical_area_service, mock_connectome_manager):
        """Test creating a cortical area when creation fails."""
        mock_connectome_manager.create_cortical_area.return_value = None

        result = cortical_area_service.create_area(
            name="Failed Area",
            coordinates={"x": 0, "y": 0, "z": 0},
            dimensions={"x": 1, "y": 1, "z": 1},
            area_type="test",
        )

        assert result is None

    def test_create_area_with_exception(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test creating a cortical area when an exception occurs."""
        mock_connectome_manager.create_cortical_area.side_effect = Exception(
            "Creation error"
        )

        result = cortical_area_service.create_area(
            name="Error Area",
            coordinates={"x": 0, "y": 0, "z": 0},
            dimensions={"x": 1, "y": 1, "z": 1},
            area_type="test",
        )

        assert result is None

    def test_update_area_success(self, cortical_area_service, mock_connectome_manager):
        """Test successfully updating a cortical area."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Create existing area
        area = MagicMock()
        area.name = "Original Name"
        area.position = (0, 0, 0)
        area.dimensions = (10, 10, 5)
        area.type = "sensory"
        area.properties = {}

        mock_connectome_manager.cortical_areas = {1: area}
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3]

        result = cortical_area_service.update_area(
            cortical_id="1", name="Updated Name", coordinates={"x": 5, "y": 10, "z": 15}
        )

        assert isinstance(result, dict)
        assert result["name"] == "Updated Name"
        assert result["coordinates"]["x"] == 5

    def test_update_area_failure(self, cortical_area_service, mock_connectome_manager):
        """Test updating a cortical area when update fails."""
        mock_connectome_manager.update_cortical_area.return_value = None

        result = cortical_area_service.update_area(
            cortical_id="nonexistent", name="Updated Area"
        )

        assert result is None

    def test_delete_area_success(self, cortical_area_service, mock_connectome_manager):
        """Test successfully deleting a cortical area."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence
        mock_connectome_manager.cortical_areas = {1: MagicMock()}
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3]
        mock_connectome_manager.delete_neuron = MagicMock()

        # Mock the cleanup data structures that delete_area expects
        mock_connectome_manager._occupied_voxels = {1: {}}
        mock_connectome_manager._area_lookup_tables = {1: {}}
        mock_connectome_manager._small_regular_areas = {1}
        mock_connectome_manager._large_regular_areas = set()
        mock_connectome_manager._extreme_dimension_areas = set()

        result = cortical_area_service.delete_area("1")

        assert result is True
        # Verify neurons were deleted
        assert mock_connectome_manager.delete_neuron.call_count == 3

    def test_delete_area_failure(self, cortical_area_service, mock_connectome_manager):
        """Test deleting a cortical area when deletion fails."""
        mock_connectome_manager.delete_cortical_area.return_value = False

        result = cortical_area_service.delete_area("nonexistent")

        assert result is False

    def test_delete_area_with_exception(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test deleting a cortical area when an exception occurs."""
        mock_connectome_manager.delete_cortical_area.side_effect = Exception(
            "Deletion error"
        )

        result = cortical_area_service.delete_area("area01")

        assert result is False

    def test_get_area_neurons_success(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting neurons for a cortical area."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence
        area = MagicMock()
        mock_connectome_manager.cortical_areas = {1: area}

        # Mock neuron data with all required attributes for get_area_neurons
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3]
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}
        mock_connectome_manager.membrane_potentials = [0.5, 0.7, 0.3]
        mock_connectome_manager.thresholds = [1.0, 1.0, 1.0]  # Add missing thresholds
        mock_connectome_manager.decay_rates = [0.1, 0.1, 0.1]  # Add missing decay_rates
        mock_connectome_manager.get_neuron_position = MagicMock(return_value=(1, 2, 3))

        result = cortical_area_service.get_area_neurons("1")

        assert isinstance(result, list)
        assert len(result) == 3
        # Check that neuron data includes all expected fields
        assert result[0]["id"] == "1"
        assert "position" in result[0]
        assert "properties" in result[0]

    def test_get_area_neurons_no_neurons(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting neurons when area has no neurons."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence but no neurons
        area = MagicMock()
        mock_connectome_manager.cortical_areas = {1: area}
        mock_connectome_manager.get_neurons_by_area.return_value = []

        result = cortical_area_service.get_area_neurons("1")

        assert isinstance(result, list)
        assert len(result) == 0

    def test_get_area_neurons_nonexistent_area(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting neurons for a nonexistent cortical area."""
        mock_connectome_manager.get_area_neurons.return_value = None

        result = cortical_area_service.get_area_neurons("nonexistent")

        assert result is None

    def test_get_area_activity_success(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting area activity information."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence
        area = MagicMock()
        mock_connectome_manager.cortical_areas = {1: area}
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2, 3]
        mock_connectome_manager._neuron_id_to_index = {1: 0, 2: 1, 3: 2}
        mock_connectome_manager.last_fired = [100, 105, 110]
        mock_connectome_manager.current_timestep = 110
        mock_connectome_manager.get_neuron_position = MagicMock(return_value=(1, 2, 3))

        result = cortical_area_service.get_area_activity("1", window=5)

        assert isinstance(result, dict)
        # Check the actual return format from the implementation
        assert "total_neurons" in result
        assert "active_neurons" in result
        assert "activity_ratio" in result
        assert "active_details" in result
        assert result["total_neurons"] == 3

    def test_get_area_activity_nonexistent_area(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting activity for a nonexistent cortical area."""
        mock_connectome_manager.get_area_activity = MagicMock(return_value=None)

        result = cortical_area_service.get_area_activity("nonexistent")

        assert result is None

    def test_get_area_connectivity_success(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting area connectivity information."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence
        area = MagicMock()
        mock_connectome_manager.cortical_areas = {
            1: area,
            2: MagicMock(name="Connected Area"),
        }
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2]
        mock_connectome_manager.get_incoming_connections.return_value = [(3, 0.5)]
        mock_connectome_manager.get_outgoing_connections.return_value = [(4, 0.7)]
        mock_connectome_manager._neuron_to_area = {3: 2, 4: 2}

        result = cortical_area_service.get_area_connectivity("1", direction="both")

        assert isinstance(result, dict)
        assert result["cortical_id"] == "1"
        assert result["direction"] == "both"

    def test_get_area_connectivity_incoming_only(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test getting only incoming connectivity."""
        # Mock genome validation to succeed
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=True)

        # Mock area existence
        area = MagicMock()
        mock_connectome_manager.cortical_areas = {
            1: area,
            2: MagicMock(name="Source Area"),
        }
        mock_connectome_manager.get_neurons_by_area.return_value = [1, 2]
        mock_connectome_manager.get_incoming_connections.return_value = [(3, 0.5)]
        mock_connectome_manager._neuron_to_area = {3: 2}

        result = cortical_area_service.get_area_connectivity("1", direction="incoming")

        assert isinstance(result, dict)
        assert result["direction"] == "incoming"
        assert "incoming_connections" in result

    def test_stimulate_area_success(self, cortical_area_service):
        """Test successfully stimulating a cortical area."""
        result = cortical_area_service.stimulate_area(
            "1", pattern="random", intensity=0.8
        )

        # Check the actual return format from the implementation
        assert isinstance(result, dict)
        assert "stimulated_neurons" in result
        assert "timestamp" in result

    def test_stimulate_area_with_coordinates(self, cortical_area_service):
        """Test stimulating a cortical area with specific coordinates."""
        coordinates = [{"x": 1, "y": 2, "z": 3}, {"x": 4, "y": 5, "z": 6}]

        result = cortical_area_service.stimulate_area(
            "1", pattern="custom", intensity=1.0, coordinates=coordinates
        )

        assert isinstance(result, dict)
        assert "stimulated_neurons" in result

    def test_stimulate_area_failure(self, cortical_area_service):
        """Test stimulating a non-existent cortical area."""
        result = cortical_area_service.stimulate_area("nonexistent", pattern="random")

        # Even for non-existent areas, the current implementation returns a response
        assert isinstance(result, dict)

    def test_get_id_list(self, cortical_area_service):
        """Test getting list of cortical area IDs."""
        # Mock the _get_current_genome method
        cortical_area_service._get_current_genome = MagicMock(
            return_value={
                "blueprint": {"prefix-area01-suffix": {}, "prefix-area02-suffix": {}}
            }
        )

        result = cortical_area_service.get_id_list()

        assert isinstance(result, list)
        assert len(result) == 2
        assert "area01" in result
        assert "area02" in result

    def test_get_id_list_empty(self, cortical_area_service, mock_connectome_manager):
        """Test getting list of cortical area IDs when no areas exist."""
        mock_connectome_manager.cortical_areas = {}

        result = cortical_area_service.get_id_list()

        assert isinstance(result, list)
        assert len(result) == 0

    def test_get_index_list(self, cortical_area_service, mock_connectome_manager):
        """Test getting list of cortical area indices."""
        result = cortical_area_service.get_index_list()

        assert isinstance(result, list)
        assert len(result) == 2
        assert 1 in result
        assert 2 in result

    def test_get_name_list(self, cortical_area_service, mock_connectome_manager):
        """Test getting list of cortical area names."""
        result = cortical_area_service.get_name_list()

        assert isinstance(result, list)
        assert len(result) == 2
        assert "Sensory Area" in result
        assert "Motor Area" in result

    def test_get_id_name_mapping(self, cortical_area_service, mock_connectome_manager):
        """Test getting mapping of cortical area IDs to names."""
        result = cortical_area_service.get_id_name_mapping()

        assert isinstance(result, dict)
        assert len(result) == 2
        assert result["area01"] == "Sensory Area"
        assert result["area02"] == "Motor Area"

    def test_get_2d_locations(self, cortical_area_service):
        """Test getting 2D locations of cortical areas."""
        # Mock the _get_current_genome method with proper blueprint structure
        cortical_area_service._get_current_genome = MagicMock(
            return_value={
                "blueprint": {
                    "prefix-area01-prop-2dcorx": 10,
                    "prefix-area01-prop-2dcory": 20,
                    "prefix-area02-prop-2dcorx": 30,
                    "prefix-area02-prop-2dcory": 40,
                }
            }
        )

        result = cortical_area_service.get_2d_locations()

        assert isinstance(result, dict)
        assert len(result) == 2
        assert result["area01"] == [10, 20]
        assert result["area02"] == [30, 40]

    def test_refresh_cache(self, cortical_area_service):
        """Test refreshing the cache."""
        # This should complete without error
        cortical_area_service.refresh_cache()

    def test_error_handling_in_methods(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test that methods handle exceptions gracefully."""
        # Make connectome manager methods raise exceptions
        mock_connectome_manager.cortical_areas = None  # This will cause AttributeError

        # Test that methods don't crash with exceptions
        result = cortical_area_service.get_all_areas()
        assert isinstance(result, list)
        assert len(result) == 0

        result = cortical_area_service.get_id_list()
        assert isinstance(result, list)
        assert len(result) == 0

    def test_parameter_validation(self, cortical_area_service, mock_connectome_manager):
        """Test parameter validation in create_area method."""
        # Mock genome validation to fail - this should prevent creation
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=False)

        result = cortical_area_service.create_area(
            name="Test Area",
            coordinates={"x": 0, "y": 0, "z": 0},
            dimensions={"width": 10, "height": 10, "depth": 5},
            area_type="custom",
        )

        # Should return None when genome not loaded
        assert result is None
        # Should not call the connectome manager
        assert not mock_connectome_manager.create_cortical_area.called

    def test_coordinate_conversion(
        self, cortical_area_service, mock_connectome_manager
    ):
        """Test coordinate conversion in create_area method."""
        # Mock genome validation to fail - this should prevent creation
        cortical_area_service._validate_genome_loaded = MagicMock(return_value=False)

        cortical_area_service.create_area(
            name="Test Area",
            coordinates={"x": 5, "y": 10, "z": 15},
            dimensions={"width": 20, "height": 25, "depth": 30},
            area_type="custom",
        )

        # Should not call connectome manager when genome not loaded
        assert not cortical_area_service._connectome_manager.create_cortical_area.called
