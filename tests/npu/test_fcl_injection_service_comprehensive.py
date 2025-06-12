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
Comprehensive test coverage for FCL Injection Service.

This module tests all functionality in fcl_injection_service.py to achieve
high code coverage, including injection timing, batch processing, statistics,
and error handling.
"""

import time
from typing import Any, Dict, List
from unittest.mock import MagicMock, Mock, patch

import pytest

from feagi.npu.fcl_injection_service import (
    FCLInjectionService,
    InjectionBatch,
    InjectionTiming,
)


class MockSpecialAreaHandler:
    """Mock special area handler for testing."""

    def __init__(self):
        self.power_neurons = {
            "power_area_1": [1, 2, 3, 4, 5],
            "power_area_2": [10, 11, 12],
            "modulator_area": [20, 21],
        }
        self.special_configs = {
            "power_area_1": Mock(
                enabled=True, injection_timing="pre_burst", injection_probability=1.0
            ),
            "power_area_2": Mock(
                enabled=True, injection_timing="during_burst", injection_probability=0.8
            ),
            "modulator_area": Mock(
                enabled=False, injection_timing="post_burst", injection_probability=0.5
            ),
        }
        self.injection_count = 0

    def get_all_power_neurons(self):
        return self.power_neurons

    def get_special_config(self, cortical_id):
        return self.special_configs.get(cortical_id)

    def record_injection(self):
        self.injection_count += 1


class MockFCLManager:
    """Mock FCL manager for testing."""

    def __init__(self):
        self.fcl_updates = []
        self.direct_injections = []
        self.has_update_fcl = True
        self.has_add_to_current_fcl = True

    def update_fcl(self, timestep, neurons_by_cortical):
        self.fcl_updates.append((timestep, neurons_by_cortical))

    def add_to_current_fcl(self, neuron_ids):
        self.direct_injections.append(neuron_ids)


@pytest.fixture
def mock_fcl_manager():
    return MockFCLManager()


@pytest.fixture
def mock_special_area_handler():
    return MockSpecialAreaHandler()


@pytest.fixture
def injection_service(mock_fcl_manager, mock_special_area_handler):
    """Create injection service with default configuration."""
    return FCLInjectionService(
        fcl_manager=mock_fcl_manager,
        special_area_handler=mock_special_area_handler,
        config={"batch_injection_size": 10},
    )


# Test basic initialization
def test_fcl_injection_service_initialization(
    mock_fcl_manager, mock_special_area_handler
):
    """Test FCL injection service initialization."""
    service = FCLInjectionService(
        fcl_manager=mock_fcl_manager, special_area_handler=mock_special_area_handler
    )

    assert service.fcl_manager == mock_fcl_manager
    assert service.special_area_handler == mock_special_area_handler
    assert service.config == {}
    assert service.batch_size == 1000  # default
    assert service.enable_probabilistic == True
    assert service.enable_timing_optimization == True
    assert service.total_injections == 0
    assert service.total_neurons_injected == 0


def test_fcl_injection_service_custom_config(
    mock_fcl_manager, mock_special_area_handler
):
    """Test initialization with custom configuration."""
    config = {
        "batch_injection_size": 500,
        "enable_probabilistic_injection": False,
        "enable_timing_optimization": False,
    }

    service = FCLInjectionService(
        fcl_manager=mock_fcl_manager,
        special_area_handler=mock_special_area_handler,
        config=config,
    )

    assert service.batch_size == 500
    assert service.enable_probabilistic == False
    assert service.enable_timing_optimization == False


# Test injection batch preparation
def test_prepare_injection_batches(injection_service):
    """Test preparation of injection batches."""
    # Should have prepared batches for enabled areas only
    pre_burst_batches = injection_service._injection_batches[InjectionTiming.PRE_BURST]
    during_burst_batches = injection_service._injection_batches[
        InjectionTiming.DURING_BURST
    ]
    post_burst_batches = injection_service._injection_batches[
        InjectionTiming.POST_BURST
    ]

    # power_area_1 (enabled, pre_burst)
    assert len(pre_burst_batches) == 1
    assert pre_burst_batches[0].cortical_id == "power_area_1"
    assert pre_burst_batches[0].neuron_ids == [1, 2, 3, 4, 5]
    assert pre_burst_batches[0].timing == InjectionTiming.PRE_BURST

    # power_area_2 (enabled, during_burst)
    assert len(during_burst_batches) == 1
    assert during_burst_batches[0].cortical_id == "power_area_2"
    assert during_burst_batches[0].neuron_ids == [10, 11, 12]

    # modulator_area (disabled) - should not have batches
    assert len(post_burst_batches) == 0


def test_prepare_large_batches(mock_fcl_manager, mock_special_area_handler):
    """Test preparation when neuron list exceeds batch size."""
    # Create large neuron list
    mock_special_area_handler.power_neurons["large_area"] = list(range(25))
    mock_special_area_handler.special_configs["large_area"] = Mock(
        enabled=True, injection_timing="pre_burst", injection_probability=1.0
    )

    service = FCLInjectionService(
        fcl_manager=mock_fcl_manager,
        special_area_handler=mock_special_area_handler,
        config={"batch_injection_size": 10},
    )

    # Should create 3 batches (25 neurons / 10 per batch = 3 batches)
    pre_burst_batches = service._injection_batches[InjectionTiming.PRE_BURST]
    large_area_batches = [b for b in pre_burst_batches if "large_area" in b.cortical_id]

    assert len(large_area_batches) == 3
    assert large_area_batches[0].cortical_id == "large_area_batch_0"
    assert large_area_batches[1].cortical_id == "large_area_batch_1"
    assert large_area_batches[2].cortical_id == "large_area_batch_2"

    # Check neuron distribution
    total_neurons = sum(len(batch.neuron_ids) for batch in large_area_batches)
    assert total_neurons == 25


def test_prepare_batches_invalid_timing(mock_fcl_manager, mock_special_area_handler):
    """Test preparation with invalid timing configuration."""
    mock_special_area_handler.special_configs["power_area_1"].injection_timing = (
        "invalid_timing"
    )

    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        service = FCLInjectionService(
            fcl_manager=mock_fcl_manager, special_area_handler=mock_special_area_handler
        )

        # Should default to PRE_BURST and log warning
        mock_logger.warning.assert_called()
        pre_burst_batches = service._injection_batches[InjectionTiming.PRE_BURST]
        # Should have power_area_1 (defaulted to pre_burst) but not power_area_2 (still during_burst)
        assert len(pre_burst_batches) == 1


# Test injection phases
def test_inject_pre_burst(injection_service):
    """Test pre-burst injection."""
    result = injection_service.inject_pre_burst(current_timestep=100)

    # Should inject power_area_1 neurons (5 neurons)
    assert result == 5
    assert injection_service.total_injections == 1
    assert injection_service.total_neurons_injected == 5
    assert injection_service.injection_timing_stats[InjectionTiming.PRE_BURST] == 1

    # Check FCL manager was called
    assert len(injection_service.fcl_manager.fcl_updates) == 1
    timestep, neurons_by_cortical = injection_service.fcl_manager.fcl_updates[0]
    assert timestep == 100
    assert "power_area_1" in neurons_by_cortical
    assert neurons_by_cortical["power_area_1"] == [1, 2, 3, 4, 5]


def test_inject_during_burst(injection_service):
    """Test during-burst injection."""
    # Use a fixed seed for deterministic test
    with patch("random.random", return_value=0.5):  # Below 0.8 threshold
        result = injection_service.inject_during_burst(current_timestep=101)

        # Should inject power_area_2 neurons (3 neurons)
        assert result == 3
        assert injection_service.total_injections == 1
        assert injection_service.total_neurons_injected == 3
        assert (
            injection_service.injection_timing_stats[InjectionTiming.DURING_BURST] == 1
        )


def test_inject_post_burst(injection_service):
    """Test post-burst injection."""
    # No enabled post-burst areas in mock
    result = injection_service.inject_post_burst(current_timestep=102)

    assert result == 0
    assert injection_service.total_injections == 0
    assert injection_service.total_neurons_injected == 0


def test_inject_with_no_batches(mock_fcl_manager, mock_special_area_handler):
    """Test injection when no batches are available."""
    # Disable all areas
    for config in mock_special_area_handler.special_configs.values():
        config.enabled = False

    service = FCLInjectionService(mock_fcl_manager, mock_special_area_handler)

    result = service.inject_pre_burst(100)
    assert result == 0


# Test probabilistic injection
def test_probabilistic_injection_success(injection_service):
    """Test probabilistic injection when probability check passes."""
    with patch("random.random", return_value=0.5):  # Below 0.8 threshold
        result = injection_service.inject_during_burst(100)
        assert result == 3  # Should inject power_area_2


def test_probabilistic_injection_failure(injection_service):
    """Test probabilistic injection when probability check fails."""
    with patch("random.random", return_value=0.9):  # Above 0.8 threshold
        result = injection_service.inject_during_burst(100)
        assert result == 0  # Should not inject power_area_2


def test_probabilistic_injection_disabled(mock_fcl_manager, mock_special_area_handler):
    """Test injection with probabilistic injection disabled."""
    service = FCLInjectionService(
        fcl_manager=mock_fcl_manager,
        special_area_handler=mock_special_area_handler,
        config={"enable_probabilistic_injection": False},
    )

    with patch("random.random", return_value=0.9):  # Would normally fail
        result = service.inject_during_burst(100)
        assert result == 3  # Should still inject because probabilistic is disabled


# Test batch injection with different FCL manager capabilities
def test_inject_batch_with_update_fcl(injection_service):
    """Test batch injection using update_fcl method."""
    batch = InjectionBatch(
        cortical_id="test_area",
        neuron_ids=[100, 101, 102],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    result = injection_service._inject_batch(batch, 200)

    assert result == 3
    assert len(injection_service.fcl_manager.fcl_updates) == 1
    timestep, neurons_by_cortical = injection_service.fcl_manager.fcl_updates[0]
    assert timestep == 200
    assert neurons_by_cortical["test_area"] == [100, 101, 102]


def test_inject_batch_with_add_to_current_fcl(mock_special_area_handler):
    """Test batch injection using fallback add_to_current_fcl method."""
    # Create FCL manager without update_fcl
    fcl_manager = Mock()
    del fcl_manager.update_fcl  # Remove update_fcl method
    fcl_manager.add_to_current_fcl = Mock()

    service = FCLInjectionService(fcl_manager, mock_special_area_handler)

    batch = InjectionBatch(
        cortical_id="test_area",
        neuron_ids=[100, 101, 102],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    result = service._inject_batch(batch, 200)

    assert result == 3
    fcl_manager.add_to_current_fcl.assert_called_once_with([100, 101, 102])


def test_inject_batch_no_fcl_methods(mock_special_area_handler):
    """Test batch injection when FCL manager has no injection methods."""
    # Create FCL manager without injection methods
    fcl_manager = Mock()
    del fcl_manager.update_fcl
    del fcl_manager.add_to_current_fcl

    service = FCLInjectionService(fcl_manager, mock_special_area_handler)

    batch = InjectionBatch(
        cortical_id="test_area",
        neuron_ids=[100, 101, 102],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        result = service._inject_batch(batch, 200)

        assert result == 0
        mock_logger.warning.assert_called_with(
            "FCL manager does not support neuron injection"
        )


def test_inject_batch_empty_neurons(injection_service):
    """Test batch injection with empty neuron list."""
    batch = InjectionBatch(
        cortical_id="test_area",
        neuron_ids=[],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    result = injection_service._inject_batch(batch, 200)
    assert result == 0


def test_inject_batch_with_batch_suffix(injection_service):
    """Test batch injection with batch suffix in cortical_id."""
    batch = InjectionBatch(
        cortical_id="large_area_batch_2",
        neuron_ids=[100, 101, 102],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    result = injection_service._inject_batch(batch, 200)

    # Should extract base cortical_id
    assert result == 3
    timestep, neurons_by_cortical = injection_service.fcl_manager.fcl_updates[0]
    assert "large_area" in neurons_by_cortical  # Not 'large_area_batch_2'


def test_inject_batch_error_handling(injection_service):
    """Test error handling during batch injection."""
    # Replace the update_fcl method with a mock that raises an exception
    injection_service.fcl_manager.update_fcl = Mock(side_effect=Exception("FCL error"))

    batch = InjectionBatch(
        cortical_id="test_area",
        neuron_ids=[100, 101, 102],
        timing=InjectionTiming.PRE_BURST,
        probability=1.0,
    )

    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        result = injection_service._inject_batch(batch, 200)

        assert result == 0
        mock_logger.error.assert_called()


# Test utility methods
def test_refresh_injection_batches(injection_service):
    """Test refreshing injection batches."""
    # Modify power neurons
    injection_service.special_area_handler.power_neurons["new_area"] = [50, 51, 52]
    injection_service.special_area_handler.special_configs["new_area"] = Mock(
        enabled=True, injection_timing="pre_burst", injection_probability=1.0
    )

    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        injection_service.refresh_injection_batches()

        # Check for both log messages
        assert mock_logger.info.call_count >= 1
        # Should have refreshing message somewhere in the calls
        call_args_list = [str(call) for call in mock_logger.info.call_args_list]
        refresh_called = any("Refreshing" in call_str for call_str in call_args_list)
        assert refresh_called

    # Should now have new_area in pre_burst batches
    pre_burst_batches = injection_service._injection_batches[InjectionTiming.PRE_BURST]
    area_ids = [batch.cortical_id for batch in pre_burst_batches]
    assert "new_area" in area_ids


def test_get_statistics(injection_service):
    """Test getting injection statistics."""
    # Perform some injections
    injection_service.inject_pre_burst(100)
    with patch(
        "random.random", return_value=0.5
    ):  # Ensure during_burst injection succeeds
        injection_service.inject_during_burst(101)

    stats = injection_service.get_statistics()

    expected_stats = {
        "total_injections": 2,
        "total_neurons_injected": 8,  # 5 + 3
        "injection_timing_stats": {
            InjectionTiming.PRE_BURST: 1,
            InjectionTiming.DURING_BURST: 1,
            InjectionTiming.POST_BURST: 0,
        },
        "last_injection_duration": stats["last_injection_duration"],  # Variable
        "prepared_batches": {"pre_burst": 1, "during_burst": 1, "post_burst": 0},
        "batch_size": 10,
        "enable_probabilistic": True,
    }

    assert stats == expected_stats
    assert isinstance(stats["last_injection_duration"], float)
    assert stats["last_injection_duration"] >= 0


def test_set_injection_enabled_success(injection_service):
    """Test enabling/disabling injection for a cortical area."""
    # Disable power_area_1
    result = injection_service.set_injection_enabled("power_area_1", False)

    assert result == True
    assert (
        injection_service.special_area_handler.special_configs["power_area_1"].enabled
        == False
    )

    # Should have refreshed batches (no pre_burst batches now)
    pre_burst_batches = injection_service._injection_batches[InjectionTiming.PRE_BURST]
    assert len(pre_burst_batches) == 0


def test_set_injection_enabled_area_not_found(injection_service):
    """Test setting injection enabled for non-existent area."""
    result = injection_service.set_injection_enabled("nonexistent_area", True)
    assert result == False


def test_get_power_injection_preview(injection_service):
    """Test getting power injection preview."""
    preview = injection_service.get_power_injection_preview()

    expected_preview = {
        "pre_burst_neurons": 5,  # power_area_1
        "during_burst_neurons": 3,  # power_area_2
        "post_burst_neurons": 0,  # no enabled post_burst areas
        "total_batches": 2,
        "areas_involved": ["power_area_1", "power_area_2"],
    }

    # Check all fields except areas_involved
    assert preview["pre_burst_neurons"] == expected_preview["pre_burst_neurons"]
    assert preview["during_burst_neurons"] == expected_preview["during_burst_neurons"]
    assert preview["post_burst_neurons"] == expected_preview["post_burst_neurons"]
    assert preview["total_batches"] == expected_preview["total_batches"]

    # Check areas_involved as sets since order may vary
    assert set(preview["areas_involved"]) == set(expected_preview["areas_involved"])


def test_get_power_injection_preview_with_batch_suffix(
    mock_fcl_manager, mock_special_area_handler
):
    """Test preview with batched areas."""
    # Add large area that will be batched
    mock_special_area_handler.power_neurons["large_area"] = list(range(25))
    mock_special_area_handler.special_configs["large_area"] = Mock(
        enabled=True, injection_timing="pre_burst", injection_probability=1.0
    )

    service = FCLInjectionService(
        mock_fcl_manager, mock_special_area_handler, config={"batch_injection_size": 10}
    )

    preview = service.get_power_injection_preview()

    # Should include large_area in areas_involved (without batch suffix)
    assert "large_area" in preview["areas_involved"]
    assert preview["total_batches"] > 2  # More than original 2 due to batching


# Test timing measurement
def test_injection_timing_measurement(injection_service):
    """Test that injection timing is properly measured."""
    with patch("time.perf_counter", side_effect=[1.0, 1.005]):  # 5ms duration
        injection_service.inject_pre_burst(100)

        # Allow for floating point precision differences
        assert abs(injection_service.last_injection_duration - 0.005) < 0.001


def test_debug_logging_during_injection(injection_service):
    """Test debug logging when neurons are injected."""
    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        injection_service.inject_pre_burst(100)

        # Should log debug message for successful injection
        mock_logger.debug.assert_called()
        debug_call = mock_logger.debug.call_args[0][0]
        assert "Injected 5 neurons in pre_burst phase" in debug_call


def test_no_debug_logging_for_zero_injection(injection_service):
    """Test no debug logging when no neurons are injected."""
    with patch("feagi.npu.fcl_injection_service.logger") as mock_logger:
        injection_service.inject_post_burst(100)  # No post_burst batches

        # Should not log debug message for zero injection
        mock_logger.debug.assert_not_called()


if __name__ == "__main__":
    pytest.main(["-v", __file__])
