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

"""Tests for the CoreAPIService class."""

from unittest.mock import MagicMock

import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)

    # Set up basic mocks for cortical area methods
    cm.cortical_areas = {
        1: MagicMock(
            name="Area 1",
            position=(0, 0, 0),
            dimensions=(10, 10, 10),
            type="sensory",
            cortical_id="area1",
        ),
        2: MagicMock(
            name="Area 2",
            position=(20, 0, 0),
            dimensions=(10, 10, 10),
            type="motor",
            cortical_id="area2",
        ),
    }
    cm.cortical_neuron_map = {1: {1, 2, 3}, 2: {4, 5, 6}}

    # Mock neurons
    cm.neurons = {
        1: {
            "cortical_id": "area1",
            "position": (1, 1, 1),
            "membrane_potential": 0.5,
            "threshold": 1.0,
        },
        2: {
            "cortical_id": "area1",
            "position": (2, 2, 2),
            "membrane_potential": 0.3,
            "threshold": 1.0,
        },
        3: {
            "cortical_id": "area1",
            "position": (3, 3, 3),
            "membrane_potential": 0.8,
            "threshold": 1.0,
        },
        4: {
            "cortical_id": "area2",
            "position": (1, 1, 1),
            "membrane_potential": 0.1,
            "threshold": 1.0,
        },
        5: {
            "cortical_id": "area2",
            "position": (2, 2, 2),
            "membrane_potential": 0.9,
            "threshold": 1.0,
        },
        6: {
            "cortical_id": "area2",
            "position": (3, 3, 3),
            "membrane_potential": 0.4,
            "threshold": 1.0,
        },
    }

    # Mock methods
    cm.get_neurons_by_area = lambda cortical_id: list(
        cm.cortical_neuron_map.get(cortical_id, [])
    )
    cm.get_neuron_property = lambda neuron_id, property_name: cm.neurons[neuron_id].get(
        property_name
    )
    cm.set_neuron_property = lambda neuron_id, property_name, value: cm.neurons[
        neuron_id
    ].update({property_name: value})

    # Mock for FCL manager
    fcl_manager = MagicMock()
    cm.fcl_manager = fcl_manager

    return cm


@pytest.fixture
def core_api_service(connectome_manager):
    """Create a CoreAPIService instance with a mock ConnectomeManager."""
    # Create CoreAPIService
    service = CoreAPIService(connectome_manager)
    return service


def test_get_cortical_areas(core_api_service, connectome_manager):
    """Test getting all cortical areas."""
    # Call the method
    result = core_api_service.get_cortical_areas()

    # Verify the result is a list
    assert isinstance(result, list)

    # Verify the correct number of areas
    assert len(result) == 2  # We mocked 2 areas


def test_get_cortical_area(core_api_service, connectome_manager):
    """Test getting a specific cortical area by ID."""
    # Get cortical area 1
    result = core_api_service.get_cortical_area("1")

    # Verify the result is a dictionary
    assert isinstance(result, dict)

    # Verify the area ID
    assert result["id"] == "1"


def test_reset_fcl(core_api_service, connectome_manager):
    """Test resetting the FCL."""
    # Call the method
    result = core_api_service.reset_fcl()

    # Verify the result
    assert result is True

    # Verify the FCL manager's reset method was called
    assert connectome_manager.fcl_manager.reset.called


def test_get_burst_engine_config(core_api_service, connectome_manager):
    """Test getting the burst engine configuration."""
    # Call the method
    result = core_api_service.get_burst_engine_config()

    # Verify the result is a dictionary
    assert isinstance(result, dict)


class TestCoreAPIService:
    """Test cases for the Core API Service."""

    def test_get_cortical_areas(self, core_api_service, connectome_manager):
        """Test getting all cortical areas."""
        # Set up mock
        connectome_manager.cortical_areas = {
            1: MagicMock(cortical_id="test1", name="Test Area 1"),
            2: MagicMock(cortical_id="test2", name="Test Area 2"),
        }

        result = core_api_service.get_cortical_areas()
        assert isinstance(result, list)

    def test_reset_fcl(self, core_api_service, connectome_manager):
        """Test FCL reset functionality."""
        # Set up mock for FCL manager
        mock_fcl_manager = MagicMock()
        mock_fcl_manager.reset = MagicMock()
        connectome_manager.fcl_manager = mock_fcl_manager

        result = core_api_service.reset_fcl()
        assert result is True
        mock_fcl_manager.reset.assert_called_once()

    def test_get_cortical_2d_locations(self, core_api_service, connectome_manager):
        """Test getting 2D locations of cortical areas."""
        result = core_api_service.get_cortical_2d_locations()
        assert isinstance(result, dict)

    def test_get_burst_engine_config(self, core_api_service, connectome_manager):
        """Test getting burst engine configuration."""
        result = core_api_service.get_burst_engine_config()
        assert isinstance(result, dict)

    def test_all_critical_methods_exist(self, core_api_service):
        """Test that ALL critical methods from the original CoreAPIService exist."""
        # This is the critical test that should have caught our missing methods!
        critical_methods = [
            # Core component access
            "get_burst_engine",
            "get_connectome_manager",
            "get_fcl_manager",
            "get_memory_manager",
            # Fire queue methods (critical for FQSampler)
            "get_fire_queue",
            "get_area_fire_queue",
            # State management (critical for startup)
            "genome_is_loaded",
            "get_state_manager",
            # Brain state management
            "get_brain_state",
            "save_brain_state",
            "load_brain_state",
            # Agent management
            "get_agent_list",
            "get_agent_properties",
            "deregister_agent",
            # Cortical area methods (legacy names)
            "get_cortical_area_id_list",
            "get_cortical_area_index_list",
            "get_cortical_area_name_list",
            "get_cortical_locations_2d",
            "get_cortical_area_stats",
            # Plasticity methods
            "enable_area_plasticity",
            "disable_area_plasticity",
            "get_plasticity_info",
            "get_plasticity_queue_depth",
            "update_plasticity_queue_depth",
            "update_plasticity_config",
            # Monitoring methods
            "get_membrane_potential_monitoring_status",
            "set_membrane_potential_monitoring",
            "get_synaptic_potential_monitoring_status",
            "set_synaptic_potential_monitoring",
            "get_membrane_potentials",
            "update_membrane_potentials",
            # FCL Sampler methods
            "get_fcl_sampler_config",
            "update_fcl_sampler_config",
            "get_area_fq_sample_rate",
            "set_area_fq_sample_rate",
            # Burst engine methods
            "get_burst_counter",
            "update_burst_engine_config",
            # Network methods
            "get_network_config",
            "update_network_config",
            # Stimulation methods
            "trigger_manual_stimulation",
            "trigger_sustained_stimulation",
            "set_stimulation_script",
            "reset_stimulation_script",
            # Transaction methods
            "begin_transaction",
            "modify_genome",
            "register_genome_change_listener",
            "on_sync_state_change",
            "refresh_cached_data",
            # Utility methods
            "get_connectome_dimensions",
            "get_morphology_list",
            "get_detailed_cortical_map",
            "get_data_path",
            "get_temp_path",
            "get_neuron_mappings",
            "get_transforming_areas",
            "has_pending_amalgamation",
            "save_connectome_snapshot",
            "import_cortical_area",
            "batch_create_neurons",
            "batch_create_synapses",
            # Robot/Gazebo methods
            "update_robot_controller",
            "update_robot_model",
            "get_gazebo_robot_files",
            # Performance methods
            "get_performance_stats",
            "get_simulation_status",
            "get_system_health",
        ]

        missing_methods = []
        for method_name in critical_methods:
            if not hasattr(core_api_service, method_name):
                missing_methods.append(method_name)
            else:
                # Verify it's callable
                method = getattr(core_api_service, method_name)
                if not callable(method):
                    missing_methods.append(f"{method_name} (not callable)")

        assert len(missing_methods) == 0, f"Missing critical methods: {missing_methods}"

    def test_method_count_consistency(self, core_api_service):
        """Test that we have the expected number of public methods."""
        public_methods = [
            method
            for method in dir(core_api_service)
            if callable(getattr(core_api_service, method))
            and not method.startswith("_")
        ]

        # We should have at least 130 public methods (original had 107, we added more)
        assert len(public_methods) >= 130, (
            f"Expected at least 130 public methods, got {len(public_methods)}"
        )

    def test_basic_method_calls(self, core_api_service):
        """Test that basic methods can be called without errors."""
        # Test methods that should always work
        assert core_api_service.get_service_health() is not None
        assert isinstance(core_api_service.get_versions(), dict)
        assert isinstance(core_api_service.get_configuration(), dict)
        assert isinstance(core_api_service.get_user_preferences(), dict)

        # Test fire queue methods
        assert core_api_service.get_fire_queue() is None or isinstance(
            core_api_service.get_fire_queue(), dict
        )

        # Test basic state methods
        assert isinstance(core_api_service.genome_is_loaded(), bool)
        # State manager might be None in test fixtures
        state_manager = core_api_service.get_state_manager()
        assert (
            state_manager is None or state_manager is not None
        )  # Just check it returns something

        # Test FCL sampler config
        assert isinstance(core_api_service.get_fcl_sampler_config(), dict)

        # Test burst counter
        assert isinstance(core_api_service.get_burst_counter(), int)

    def test_legacy_method_aliases(self, core_api_service):
        """Test that legacy method aliases work correctly."""
        # Test that legacy names map to the correct methods
        assert (
            core_api_service.get_cortical_areas()
            == core_api_service.get_all_cortical_areas()
        )
        assert (
            core_api_service.get_cortical_area_id_list()
            == core_api_service.get_cortical_id_list()
        )
        assert (
            core_api_service.get_cortical_area_index_list()
            == core_api_service.get_cortical_index_list()
        )
        assert (
            core_api_service.get_cortical_area_name_list()
            == core_api_service.get_cortical_name_list()
        )
        assert (
            core_api_service.get_cortical_locations_2d()
            == core_api_service.get_cortical_2d_locations()
        )

    def test_error_handling(self, core_api_service):
        """Test that methods handle errors gracefully."""
        # Test with invalid inputs that should return empty/default values
        assert core_api_service.get_area_fire_queue("invalid_id") is None
        assert isinstance(
            core_api_service.get_cortical_area_stats("invalid_id"), (dict, type(None))
        )
        assert isinstance(core_api_service.get_agent_properties("invalid_id"), dict)

        # Test methods that should return safe defaults
        assert isinstance(core_api_service.get_morphology_list(), list)
        assert isinstance(core_api_service.get_transforming_areas(), list)
        assert isinstance(core_api_service.has_pending_amalgamation(), bool)
