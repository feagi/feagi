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

"""Tests for the BrainService class."""

import pytest
from unittest.mock import MagicMock, patch, PropertyMock

from feagi.api.core.services.brain.brain_service import BrainService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    cm = MagicMock(spec=ConnectomeManager)
    cm.neurons = {
        1: {"membrane_potential": 0.5, "threshold": 1.0},
        2: {"membrane_potential": 0.8, "threshold": 1.0},
        3: {"membrane_potential": 0.3, "threshold": 1.0}
    }
    cm.synapses = {
        (1, 2): {"weight": 0.7},
        (2, 3): {"weight": 0.9}
    }
    cm.cortical_areas = {
        1: MagicMock(cortical_id="area1", name="Area 1"),
        2: MagicMock(cortical_id="area2", name="Area 2")
    }
    return cm


@pytest.fixture
def mock_state_manager():
    """Create a mock state manager for testing."""
    sm = MagicMock()
    sm.is_genome_loaded.return_value = True
    sm.get_brain_readiness.return_value = True
    sm.exit_condition = False
    sm.current_burst_id = 100
    sm.performance_metrics = {
        "burst_duration": 0.05,
        "neurons_fired": 25,
        "synapses_activated": 45
    }
    return sm


@pytest.fixture
def mock_burst_engine():
    """Create a mock burst engine for testing."""
    be = MagicMock()
    be.is_running.return_value = True
    be.start = MagicMock(return_value=True)
    be.stop = MagicMock(return_value=True)
    be.get_status.return_value = {
        "running": True,
        "burst_count": 100,
        "last_burst_duration": 0.05
    }
    be.get_config.return_value = {
        "burst_frequency": 20.0,
        "max_burst_duration": 1.0
    }
    return be


@pytest.fixture
def brain_service(mock_connectome_manager, mock_state_manager):
    """Create a BrainService instance with mocked dependencies."""
    return BrainService(mock_connectome_manager, mock_state_manager)


@pytest.fixture
def brain_service_no_state(mock_connectome_manager):
    """Create a BrainService instance without state manager."""
    return BrainService(mock_connectome_manager, None)


class TestBrainService:
    """Test cases for the BrainService."""

    def test_init(self, mock_connectome_manager, mock_state_manager):
        """Test BrainService initialization."""
        service = BrainService(mock_connectome_manager, mock_state_manager)
        
        assert service._connectome_manager == mock_connectome_manager
        assert service.state_manager == mock_state_manager

    def test_get_burst_engine_status_with_engine(self, brain_service, mock_state_manager):
        """Test getting burst engine status when engine exists."""
        mock_state_manager.exit_condition = False  # Running
        mock_state_manager.current_burst_id = 150
        mock_state_manager.get_brain_readiness.return_value = True
        mock_state_manager.is_genome_loaded.return_value = True
        
        result = brain_service.get_burst_engine_status()
        
        assert isinstance(result, dict)
        assert result["status"] == "running"
        assert result["current_burst"] == 150
        assert result["brain_ready"] is True
        assert result["genome_loaded"] is True

    def test_get_burst_engine_status_without_engine(self, brain_service, mock_state_manager):
        """Test getting burst engine status when engine doesn't exist."""
        mock_state_manager.exit_condition = True  # Stopped
        mock_state_manager.current_burst_id = 0
        mock_state_manager.get_brain_readiness.return_value = False
        mock_state_manager.is_genome_loaded.return_value = False
        
        result = brain_service.get_burst_engine_status()
        
        assert isinstance(result, dict)
        assert result["status"] == "stopped"

    def test_start_burst_engine_success(self, brain_service, mock_state_manager):
        """Test successfully starting the burst engine."""
        result = brain_service.start_burst_engine()
        
        assert result is True
        # Should set exit_condition to False to start
        assert mock_state_manager.exit_condition is False

    def test_start_burst_engine_without_engine(self, brain_service_no_state):
        """Test starting burst engine when engine doesn't exist."""
        result = brain_service_no_state.start_burst_engine()
        
        assert result is False

    def test_start_burst_engine_failure(self, brain_service, mock_state_manager):
        """Test burst engine start failure."""
        # Mock an exception during start
        type(mock_state_manager).exit_condition = PropertyMock(side_effect=Exception("Start error"))
        
        result = brain_service.start_burst_engine()
        
        assert result is False

    def test_stop_burst_engine_success(self, brain_service, mock_state_manager):
        """Test successfully stopping the burst engine."""
        result = brain_service.stop_burst_engine()
        
        assert result is True
        # Should set exit_condition to True to stop
        assert mock_state_manager.exit_condition is True

    def test_stop_burst_engine_without_engine(self, brain_service_no_state):
        """Test stopping burst engine when engine doesn't exist."""
        result = brain_service_no_state.stop_burst_engine()
        
        assert result is False

    def test_get_brain_statistics_with_state_manager(self, brain_service, mock_state_manager, mock_connectome_manager):
        """Test getting brain statistics with state manager."""
        # Mock brain_stats to return proper values
        mock_state_manager.brain_stats = {
            "neuron_count": 100,
            "synapse_count": 200
        }
        mock_state_manager.cortical_list = ["area1", "area2", "area3"]
        mock_state_manager.is_genome_loaded.return_value = True
        mock_state_manager.get_brain_readiness.return_value = True
        
        result = brain_service.get_brain_statistics()
        
        assert isinstance(result, dict)
        assert result["neuron_count"] == 100
        assert result["synapse_count"] == 200
        assert result["cortical_area_count"] == 3
        assert result["brain_ready"] is True
        assert result["genome_loaded"] is True

    def test_get_brain_statistics_without_state_manager(self, brain_service_no_state, mock_connectome_manager):
        """Test getting brain statistics without state manager."""
        result = brain_service_no_state.get_brain_statistics()
        
        assert isinstance(result, dict)
        assert len(result) == 0  # Should return empty dict

    def test_get_activity_summary_with_state_manager(self, brain_service, mock_state_manager, mock_connectome_manager):
        """Test getting activity summary with state manager."""
        # Mock _validate_genome_loaded to return True
        brain_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock connectome manager with minimal data to avoid division by zero
        mock_connectome_manager.cortical_areas = {}
        mock_connectome_manager.current_timestep = 100
        
        result = brain_service.get_activity_summary(5)
        
        assert isinstance(result, dict)
        assert "time_window" in result
        assert result["time_window"] == 5

    def test_get_activity_summary_without_state_manager(self, brain_service_no_state):
        """Test getting activity summary without state manager."""
        # Mock _validate_genome_loaded to return False
        brain_service_no_state._validate_genome_loaded = MagicMock(return_value=False)
        
        result = brain_service_no_state.get_activity_summary(10)
        
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_reset_brain_state_with_state_manager(self, brain_service, mock_state_manager, mock_connectome_manager):
        """Test resetting brain state with state manager."""
        # Mock connectome manager methods
        mock_connectome_manager.reset_neural_states = MagicMock()
        mock_connectome_manager.current_timestep = 100
        mock_connectome_manager.reset_fcl = MagicMock()
        
        result = brain_service.reset_brain_state()
        
        assert result is True

    def test_reset_brain_state_without_state_manager(self, brain_service_no_state):
        """Test resetting brain state without state manager."""
        # Should still work with connectome manager
        result = brain_service_no_state.reset_brain_state()
        
        assert result is True

    def test_get_performance_metrics_with_state_manager(self, brain_service, mock_state_manager):
        """Test getting performance metrics with state manager."""
        mock_state_manager.last_burst_duration = 50
        mock_state_manager.bursts_per_second = 20
        mock_state_manager.neurons_processed_last_burst = 100
        mock_state_manager.synapses_processed_last_burst = 300
        
        result = brain_service.get_performance_metrics()
        
        assert isinstance(result, dict)
        assert result["burst_duration_ms"] == 50
        assert result["bursts_per_second"] == 20
        assert result["neurons_processed"] == 100
        assert result["synapses_processed"] == 300

    def test_get_performance_metrics_without_state_manager(self, brain_service_no_state):
        """Test getting performance metrics without state manager."""
        result = brain_service_no_state.get_performance_metrics()
        
        assert isinstance(result, dict)
        assert len(result) == 0

    def test_stimulate_neurons_success(self, brain_service, mock_connectome_manager):
        """Test successfully stimulating neurons."""
        # Mock validation
        brain_service._validate_genome_loaded = MagicMock(return_value=True)
        
        # Mock connectome manager attributes
        mock_connectome_manager._neuron_id_to_index = {"1": 0, "2": 1, "3": 2}
        mock_connectome_manager.membrane_potentials = [0.0, 0.0, 0.0]
        
        result = brain_service.stimulate_neurons(["1", "2", "3"], 0.8)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["stimulated_count"] == 3
        assert result["failed_count"] == 0
        assert result["intensity"] == 0.8

    def test_stimulate_neurons_failure(self, brain_service, mock_connectome_manager):
        """Test neuron stimulation failure."""
        # Mock validation to fail
        brain_service._validate_genome_loaded = MagicMock(return_value=False)
        
        result = brain_service.stimulate_neurons([1, 2, 3], 0.8)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_stimulate_neurons_without_connectome(self, brain_service):
        """Test neuron stimulation without connectome method."""
        # Mock validation to fail due to no genome
        brain_service._validate_genome_loaded = MagicMock(return_value=False)
        
        result = brain_service.stimulate_neurons([1, 2, 3], 0.8)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_get_burst_engine_config_with_engine(self, brain_service, mock_state_manager):
        """Test getting burst engine configuration when engine exists."""
        mock_state_manager.burst_frequency = 25.0
        mock_state_manager.max_neurons_per_burst = 1500
        mock_state_manager.burst_timeout = 2000
        mock_state_manager.auto_restart = True
        mock_state_manager.performance_mode = "high"
        
        result = brain_service.get_burst_engine_config()
        
        assert isinstance(result, dict)
        assert result["burst_frequency_hz"] == 25.0
        assert result["max_neurons_per_burst"] == 1500
        assert result["burst_timeout_ms"] == 2000
        assert result["auto_restart"] is True
        assert result["performance_mode"] == "high"

    def test_get_burst_engine_config_without_engine(self, brain_service_no_state):
        """Test getting burst engine configuration when engine doesn't exist."""
        result = brain_service_no_state.get_burst_engine_config()
        
        assert isinstance(result, dict)
        assert "error" in result

    def test_error_handling_in_methods(self, brain_service, mock_state_manager, mock_connectome_manager):
        """Test that methods handle exceptions gracefully."""
        # Make state manager methods raise exceptions
        mock_state_manager.brain_stats = None
        
        # Test that methods don't crash with exceptions
        stats = brain_service.get_brain_statistics()
        assert isinstance(stats, dict)
        
        metrics = brain_service.get_performance_metrics()
        assert isinstance(metrics, dict)

    def test_brain_statistics_calculation(self, brain_service, mock_state_manager, mock_connectome_manager):
        """Test that brain statistics are calculated correctly."""
        # Mock brain_stats properly
        mock_state_manager.brain_stats = {
            "neuron_count": 100,
            "synapse_count": 99
        }
        mock_state_manager.cortical_list = [f"area{i}" for i in range(1, 6)]  # 5 areas
        mock_state_manager.is_genome_loaded.return_value = True
        mock_state_manager.get_brain_readiness.return_value = True
        
        stats = brain_service.get_brain_statistics()
        
        assert stats["neuron_count"] == 100
        assert stats["synapse_count"] == 99
        assert stats["cortical_area_count"] == 5

    def test_activity_summary_calculation(self, brain_service, mock_connectome_manager):
        """Test activity summary calculation logic."""
        # Mock validation to succeed
        brain_service._validate_genome_loaded = MagicMock(return_value=True)
        mock_connectome_manager.cortical_areas = {}
        mock_connectome_manager.current_timestep = 100
        
        # Test with default window
        summary = brain_service.get_activity_summary()
        assert "time_window" in summary
        assert summary["time_window"] == 10  # Default window
        
        # Test with custom window
        summary = brain_service.get_activity_summary(25)
        assert summary["time_window"] == 25

    def test_stimulate_neurons_input_validation(self, brain_service, mock_connectome_manager):
        """Test neuron stimulation with various input types."""
        brain_service._validate_genome_loaded = MagicMock(return_value=True)
        mock_connectome_manager._neuron_id_to_index = {"1": 0, "2": 1, "3": 2}
        mock_connectome_manager.membrane_potentials = [0.0, 0.0, 0.0]
        
        # Test with string neuron IDs
        result = brain_service.stimulate_neurons(["1", "2", "3"], 0.5)
        assert result["stimulated_count"] == 3
        assert result["intensity"] == 0.5
        
        # Test with empty list
        result = brain_service.stimulate_neurons([], 1.0)
        assert result["stimulated_count"] == 0
        assert result["total_requested"] == 0
        
        # Test with different intensity values
        result = brain_service.stimulate_neurons(["1"], 0.0)
        assert result["intensity"] == 0.0
        
        result = brain_service.stimulate_neurons(["1"], 2.0)
        assert result["intensity"] == 2.0 