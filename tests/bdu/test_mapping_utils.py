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
Test module for mapping utility functions.
"""

from unittest.mock import MagicMock, patch

import pytest

from feagi.bdu.connectivity.mapping_utils import (
    build_power_connections,
    get_detailed_cortical_map,
)


class MockState:
    """Mock state manager for testing."""

    def __init__(self, blueprint_data):
        self.genome = {"blueprint": blueprint_data}


def test_get_detailed_cortical_map():
    """Test building a detailed cortical map."""
    # Create a mock blueprint structure
    blueprint = {
        "area1": {
            "cortical_mapping_dst": {
                "area2": [{"mapping_id": "mapping1"}],
                "area3": [{"mapping_id": "mapping2"}, {"mapping_id": "mapping3"}],
            }
        },
        "area2": {"cortical_mapping_dst": {"area3": [{"mapping_id": "mapping4"}]}},
        "area3": {},  # No outgoing mappings
    }

    # Create a mock state
    state = MockState(blueprint)

    # Call the function
    cortical_map = get_detailed_cortical_map(state)

    # Check the results
    assert "area1" in cortical_map
    assert "area2" in cortical_map
    assert "area3" in cortical_map

    # Check mappings from area1
    assert "area2" in cortical_map["area1"]
    assert "area3" in cortical_map["area1"]
    assert len(cortical_map["area1"]["area2"]) == 1
    assert cortical_map["area1"]["area2"][0]["mapping_id"] == "mapping1"
    assert len(cortical_map["area1"]["area3"]) == 2
    assert cortical_map["area1"]["area3"][0]["mapping_id"] == "mapping2"
    assert cortical_map["area1"]["area3"][1]["mapping_id"] == "mapping3"

    # Check mappings from area2
    assert "area3" in cortical_map["area2"]
    assert len(cortical_map["area2"]["area3"]) == 1
    assert cortical_map["area2"]["area3"][0]["mapping_id"] == "mapping4"

    # Check area3 (no outgoing mappings)
    assert cortical_map["area3"] == {}


def test_get_detailed_cortical_map_empty():
    """Test with an empty blueprint."""
    state = MockState({})
    cortical_map = get_detailed_cortical_map(state)
    assert cortical_map == {}


def test_get_detailed_cortical_map_no_mappings():
    """Test with no cortical mappings."""
    blueprint = {"area1": {}, "area2": {}}
    state = MockState(blueprint)
    cortical_map = get_detailed_cortical_map(state)
    assert cortical_map == {"area1": {}, "area2": {}}


@patch("feagi.bdu.connectivity.mapping_utils.FeagiStateManager")
def test_build_power_connections(mock_state_manager):
    """Test building power connections."""
    # Mock connectome manager
    connectome = MagicMock()
    connectome.add_core_cortical_area = MagicMock()
    connectome.update_cortical_mappings = MagicMock()

    # Set up mock state manager
    mock_instance = MagicMock()
    mock_state_manager.get_instance.return_value = mock_instance

    # Set up genome mock
    mock_instance.genome = {
        "blueprint": {"___pwr": {}, "target_area": {"block_boundaries": [10, 10, 5]}},
        "neuron_morphologies": {},
    }

    # Set up cortical types mock
    with patch(
        "feagi.bdu.connectivity.mapping_utils.cortical_types",
        {
            "custom": {
                "supported_devices": {
                    "target_area": {
                        "cortical_name": "Target Area",
                        "coordinate_3d": [0, 0, 0],
                    }
                }
            }
        },
    ):
        # Call the function with a target area that already exists
        mapping_dict = {
            "0": 0.0,  # Map to depth 0 (z=0)
            "5": 0.5,  # Map to middle depth (z=2)
            "9": 1.0,  # Map to maximum depth (z=4)
        }

        build_power_connections(connectome, "target_area", "custom", mapping_dict)

        # Check that add_core_cortical_area was not called (area already exists)
        connectome.add_core_cortical_area.assert_not_called()

        # Check that update_cortical_mappings was called with correct parameters
        connectome.update_cortical_mappings.assert_called_once()
        call_args = connectome.update_cortical_mappings.call_args[0][0]
        assert call_args["src_cortical_area"] == "___pwr"
        assert call_args["dst_cortical_area"] == "target_area"

        # Check that the morphology was created in the genome
        morphology_name = f"system-___pwr-target_area"
        assert morphology_name in mock_instance.genome["neuron_morphologies"]

        # Check patterns in the morphology
        patterns = mock_instance.genome["neuron_morphologies"][morphology_name][
            "parameters"
        ]["patterns"]
        assert len(patterns) == 3

        # Check specific pattern values
        assert [0, 0, 0] in [pattern[0] for pattern in patterns]

        # Check for target positions
        target_positions = [pattern[1] for pattern in patterns]
        assert [0, 0, 0] in target_positions  # Mapping for "0": 0.0
        assert [5, 0, 2] in target_positions  # Mapping for "5": 0.5
        assert [9, 0, 4] in target_positions  # Mapping for "9": 1.0


@patch("feagi.bdu.connectivity.mapping_utils.FeagiStateManager")
def test_build_power_connections_new_area(mock_state_manager):
    """Test building power connections for a new area."""
    # Mock connectome manager
    connectome = MagicMock()
    connectome.add_core_cortical_area = MagicMock()
    connectome.update_cortical_mappings = MagicMock()

    # Set up mock state manager
    mock_instance = MagicMock()
    mock_state_manager.get_instance.return_value = mock_instance

    # Set up genome mock without the target area
    mock_instance.genome = {"blueprint": {"___pwr": {}}, "neuron_morphologies": {}}

    # Set up cortical types mock
    with patch(
        "feagi.bdu.connectivity.mapping_utils.cortical_types",
        {
            "custom": {
                "supported_devices": {
                    "new_area": {
                        "cortical_name": "New Area",
                        "coordinate_3d": [10, 10, 0],
                    }
                }
            }
        },
    ):
        # Add a new area through power connections
        mapping_dict = {"0": 0.5}

        # Need to mock the blueprint data that would be updated by add_core_cortical_area
        def mock_add_core_cortical_area(area_data):
            mock_instance.genome["blueprint"]["new_area"] = {
                "block_boundaries": [10, 10, 5]
            }

        connectome.add_core_cortical_area.side_effect = mock_add_core_cortical_area

        build_power_connections(connectome, "new_area", "custom", mapping_dict)

        # Check that add_core_cortical_area was called
        connectome.add_core_cortical_area.assert_called_once()

        # Check the parameters
        call_args = connectome.add_core_cortical_area.call_args[0][0]
        assert call_args["cortical_id"] == "new_area"
        assert call_args["cortical_type"] == "custom"

        # Check that update_cortical_mappings was called
        connectome.update_cortical_mappings.assert_called_once()

        # Check that a pattern was created
        morphology_name = f"system-___pwr-new_area"
        assert morphology_name in mock_instance.genome["neuron_morphologies"]
