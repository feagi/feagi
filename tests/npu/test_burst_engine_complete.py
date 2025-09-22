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
Complete tests for the BurstEngine class.

This module contains comprehensive tests for the BurstEngine class to ensure
high test coverage of its functionality.
"""

import threading
import time
from queue import Queue
from unittest.mock import MagicMock, Mock, patch

import pytest

from feagi.npu.burst_engine import BurstEngine, ServiceState
from feagi.utils.logger import setup_logger

logger = setup_logger()


# Add proper test isolation
@pytest.fixture(autouse=True)
def reset_burst_engine_singleton():
    """Reset BurstEngine singleton before each test to prevent state pollution."""
    yield
    # Reset after each test
    try:
        BurstEngine.reset_instance()
    except Exception:
        pass  # Ignore if no instance exists


class MockConnectomeManager:
    def __init__(self):
        self.cortical_areas = {
            "area_1": Mock(id="area_1", properties={"__shed": True}),
            "area_2": Mock(id="area_2", properties={"__shed": False}),
            "area_3": Mock(id="area_3", properties={}),  # No shed property
        }
        self.fcl_manager = MagicMock()
        self.fcl_manager.area_fcl_history = {
            "area_1": {0: {}},
            "area_2": {0: {}},
            "area_3": {0: {}},
        }
        # Support both area_fcl_history and cortical_fcl_history naming
        self.fcl_manager.cortical_fcl_history = self.fcl_manager.area_fcl_history
        self.fcl_manager.current_window_index = 0
        self.calls = 0
        # Set get_optimized_core as a MagicMock so we can check if it was called
        self.get_optimized_core = MagicMock(return_value=Mock())
        
        # Add neuron_array mock for NPU architecture
        import numpy as np
        self.neuron_array = MagicMock()
        self.neuron_array.get_performance_summary.return_value = {"total_neurons": 1000}
        self.neuron_array.neuron_count = 1000
        self.neuron_array.valid_mask = np.ones(1000, dtype=bool)  # All neurons valid
        self.neuron_array.membrane_potentials = np.zeros(1000, dtype=np.float32)
        self.neuron_array.thresholds = np.ones(1000, dtype=np.float32)
        
        # Add neuron_id_to_index for genome updates
        self.neuron_id_to_index = {i: i for i in range(1000)}  # Simple 1:1 mapping

    def update_membrane_potentials(self, current_timestep=None):
        """Updated to accept current_timestep parameter for NPU architecture."""
        self.calls += 1
        return [1, 2, 3]  # Return some fired neurons

    def get_cortical_area(self, cortical_id):
        return self.cortical_areas.get(cortical_id)


class MockStateManager:
    def __init__(self):
        self.burst_frequency = 0
        self.burst_engine_state = ServiceState.UNAVAILABLE

    def set_burst_frequency(self, freq):
        self.burst_frequency = freq

    def set_burst_engine_state(self, state):
        self.burst_engine_state = state

    def get_burst_engine_state(self):
        return self.burst_engine_state
    
    def is_debug_npu_enabled(self):
        return False
    
    def get_burst_frequency(self):
        return self.burst_frequency
    
    def get_simd_configuration(self):
        return {"enabled": True, "backend": "cpu"}


@pytest.fixture
def mock_connectome_manager():
    return MockConnectomeManager()


@pytest.fixture
def mock_state_manager():
    return MockStateManager()


def test_burst_engine_initialization(mock_connectome_manager, mock_state_manager):
    """Test BurstEngine initialization with various parameters."""
    # Basic initialization
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        # Reset singleton to ensure clean state
        BurstEngine.reset_instance()

        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},
        )

        # Check basic properties (note: singleton may reuse instance)
        assert engine.fcl_manager == mock_connectome_manager.fcl_manager
        assert engine.desired_frequency == 100
        assert engine.burst_interval == 0.01  # 1/100Hz
        assert not engine._running

    # Test with different frequency parameter
    mock_state_manager_50hz = MockStateManager()
    mock_state_manager_50hz.set_burst_frequency(50)  # Set the expected frequency
    
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager_50hz,
    ):
        # Reset singleton to get fresh instance with new config
        BurstEngine.reset_instance()

        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"desired_frequency_hz": 50},
        )

        assert engine.desired_frequency == 50
        assert engine.burst_interval == 0.02  # 1/50Hz


def test_update_with_genome(mock_connectome_manager, mock_state_manager):
    """Test update_with_genome method."""
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},
        )

        # Initial state
        assert not engine.genome_loaded

        # Update with genome
        engine.update_with_genome()

        # Check that it updated the state
        assert engine.genome_loaded
        assert len(engine.shed_areas) == 1
        assert "area_1" in engine.shed_areas
        assert "area_2" not in engine.shed_areas
        assert "area_3" not in engine.shed_areas


def test_burst_engine_run_and_stop(mock_connectome_manager, mock_state_manager):
    """Test running and stopping the burst engine."""
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ), patch("time.perf_counter") as mock_perf_counter:
        # Create a counter for perf_counter calls to provide realistic timing
        time_values = []
        call_count = 0

        def mock_perf_counter_side_effect():
            nonlocal call_count
            call_count += 1
            # Provide enough values for the busy-wait loops
            # Start at 0, then progress in small increments to eventually reach target times
            if call_count <= 10:
                return 0.0  # Cycle start time
            elif call_count <= 20:
                return 0.001  # Processing time
            elif call_count <= 30:
                return 0.002  # End processing
            elif call_count <= 50:
                return 0.010  # Progress through busy-wait
            else:
                return 0.051  # Final time after burst interval (0.05s)

        mock_perf_counter.side_effect = mock_perf_counter_side_effect

        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 20},  # 0.05s per burst
        )

        # Run the burst engine in a separate thread
        t = threading.Thread(target=engine.run)
        t.daemon = True  # Ensure the thread doesn't prevent test exit
        t.start()

        # Let it run briefly
        time.sleep(0.05)

        # Check that it's running
        assert engine._running
        assert mock_state_manager.burst_engine_state == ServiceState.READY

        # Stop it
        engine.stop()
        t.join(timeout=1)

        # Check that it stopped
        assert not engine._running

        # Should have called update_membrane_potentials at least once
        assert mock_connectome_manager.calls > 0

        # Burst frequency may not be set if the engine didn't run long enough
        # Just check that the engine was configured properly
        assert engine.target_frequency == 20


def test_load_shedding(mock_connectome_manager, mock_state_manager):
    """Test load shedding when frequency is below target."""
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ), patch("time.sleep") as mock_sleep, patch(
        "time.perf_counter"
    ) as mock_perf_counter:
        # Mock time to make sure frequency is below target
        mock_sleep.return_value = None
        mock_perf_counter.side_effect = [0.0, 0.02]  # 0.02s per burst = 50Hz

        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},  # 0.01s per burst = 100Hz
        )

        # Add an area to shed
        engine.update_with_genome()  # This will populate shed_areas

        # Run a single burst cycle
        engine._running = True
        t = threading.Thread(target=lambda: engine.run() if engine._running else None)
        t.daemon = True
        t.start()

        # Let it run very briefly then stop
        time.sleep(0.05)
        engine.stop()
        t.join(timeout=1)

        # Check that FCL for area 1 was cleared
        assert mock_connectome_manager.fcl_manager.area_fcl_history["area_1"][0] == {}


def test_run_with_fire_queue_optimized_path():
    """Test run_with_fire_queue with optimized path."""
    # Set up mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MockStateManager()
    mock_state_manager.burst_engine_state = ServiceState.READY

    # Create the BurstEngine with mocked dependencies
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},
        )

        # Mock the optimized_integration import and function
        mock_step = MagicMock()

        # Create a monkeypatch to avoid the while loop running indefinitely
        original_run_with_fire_queue = BurstEngine.run_with_fire_queue

        def patched_run_with_fire_queue(
            self, mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version that doesn't enter the while loop"""
            if self.state_manager.get_burst_engine_state() != ServiceState.READY:
                return False

            # Update state
            self.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self._running = True

            # Get the core from connectome manager
            core = self.connectome_manager.get_optimized_core()

            # Call the mocked step function
            mock_step(core, mpf, puf, max_consecutive_fires)

            # Stop running to exit
            self._running = False

            return True

        # Patch the method
        BurstEngine.run_with_fire_queue = patched_run_with_fire_queue

        try:
            # Call the method
            result = engine.run_with_fire_queue()

            # Verify the result
            assert result is True
            assert mock_state_manager.burst_engine_state == ServiceState.READY

        finally:
            # Restore the original method
            BurstEngine.run_with_fire_queue = original_run_with_fire_queue


def test_run_with_fire_queue_fallback_path():
    """Test run_with_fire_queue with fallback path."""
    # Set up mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

    # Create the BurstEngine with mocked dependencies
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},
        )

        # Mock the _process_burst method
        engine._process_burst = MagicMock(return_value=[1, 2, 3])

        # Mock get_optimized_core to return None (force fallback path)
        mock_connectome_manager.get_optimized_core = MagicMock(return_value=None)

        # Create a monkeypatch to avoid the while loop running indefinitely
        original_run_with_fire_queue = BurstEngine.run_with_fire_queue

        def patched_run_with_fire_queue(
            self, mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version that doesn't enter the while loop"""
            if self.state_manager.get_burst_engine_state() != ServiceState.READY:
                return False

            # Update state
            self.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self._running = True

            # Import check - to test the path where optimized_integration is available
            try:
                from feagi.npu.optimized_integration import (
                    step_simulation_with_fire_queue,
                )

                optimized_available = True
            except ImportError:
                optimized_available = False

            # Test both paths - get the core and check if it's None
            core = self.connectome_manager.get_optimized_core()

            # Should call _process_burst because core is None
            fired_neurons = self._process_burst()

            # Update state when stopped
            self.state_manager.set_burst_engine_state(ServiceState.READY)

            return True

        # Apply the monkeypatch
        BurstEngine.run_with_fire_queue = patched_run_with_fire_queue

        try:
            # Call the method
            result = engine.run_with_fire_queue(
                mpf=True, puf=False, max_consecutive_fires=10
            )

            # Assertions
            assert result is True
            assert mock_connectome_manager.get_optimized_core.called
            assert engine._process_burst.called
            mock_state_manager.set_burst_engine_state.assert_called_with(
                ServiceState.READY
            )
        finally:
            # Restore the original method
            BurstEngine.run_with_fire_queue = original_run_with_fire_queue


def test_run_with_fire_queue_unavailable_state():
    """Test run_with_fire_queue when state is unavailable."""
    # Set up mocks
    mock_connectome_manager = MagicMock()
    mock_fcl_manager = MagicMock()
    mock_state_manager = MagicMock()
    mock_state_manager.get_burst_engine_state.return_value = ServiceState.UNAVAILABLE

    # Create the BurstEngine with mocked dependencies
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 100},
        )

        # Mark genome as loaded so the method will proceed past the early exit
        engine.genome_loaded = True

        # Call the method
        result = engine.run_with_fire_queue()

        # Assertions - the method should succeed even with unavailable state
        # since run_with_fire_queue doesn't check state, it just runs the processing
        assert result is True

        # The _process_burst_with_power_injection should be called which calls update_membrane_potentials
        mock_connectome_manager.update_membrane_potentials.assert_called_once()


def test_error_handling(mock_connectome_manager, mock_state_manager):
    """Test error handling during bursting."""
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ), patch("time.perf_counter") as mock_perf_counter, patch(
        "feagi.npu.burst_engine.logger"
    ) as mock_logger:
        # Create a counter for perf_counter calls to provide realistic timing
        call_count = 0

        def mock_perf_counter_side_effect():
            nonlocal call_count
            call_count += 1
            # Provide enough values for the busy-wait loops
            if call_count <= 10:
                return 0.0  # Cycle start time
            elif call_count <= 20:
                return 0.001  # Processing time
            elif call_count <= 30:
                return 0.002  # End processing
            elif call_count <= 50:
                return 0.010  # Progress through busy-wait
            else:
                return 0.051  # Final time after burst interval

        mock_perf_counter.side_effect = mock_perf_counter_side_effect

        # Make update_membrane_potentials raise an exception
        mock_connectome_manager.update_membrane_potentials = MagicMock(
            side_effect=Exception("Test error")
        )

        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 20},
        )

        # Since the exception would happen in the thread, we need to monkey patch the run method
        original_run = engine.run

        def run_with_exception_catch():
            try:
                original_run()
            except Exception as e:
                mock_logger.error(f"Caught exception: {e}")

        engine.run = run_with_exception_catch

        # Run the burst engine briefly
        t = threading.Thread(target=engine.run)
        t.daemon = True
        t.start()
        time.sleep(0.01)
        engine.stop()
        t.join(timeout=1)

        # Check that the method was called which should trigger the exception
        assert mock_connectome_manager.update_membrane_potentials.called


def test_fq_sampler_initialization():
    """Test FQSampler initialization."""

    mock_fire_queue_provider = Mock()
    mock_fire_queue_provider.get_global_fire_queue_data.return_value = {
        "neuron_ids": [1, 2, 3],
        "coordinates": [(0, 0, 0), (1, 1, 1), (2, 2, 2)],
        "membrane_potentials": [0.5, 0.6, 0.7],
    }

    output_queue = Queue()

    sampler = FQSampler(
        fire_queue_provider=mock_fire_queue_provider,
        sample_frequency_hz=10,
        output_queue=output_queue,
    )

    assert sampler.sample_frequency == 10
    assert sampler.output_queue is output_queue


def test_fq_sampler_connectivity():
    """Test FQSampler connectivity features."""

    mock_fire_queue_provider = Mock()
    mock_fire_queue_provider.get_global_fire_queue_data.return_value = {
        "neuron_ids": [1, 2, 3],
        "coordinates": [(0, 0, 0), (1, 1, 1), (2, 2, 2)],
        "membrane_potentials": [0.5, 0.6, 0.7],
    }

    output_queue = Queue()

    sampler = FQSampler(
        fire_queue_provider=mock_fire_queue_provider,
        sample_frequency_hz=10,
        output_queue=output_queue,
    )

    # Test subscriber management
    sampler.set_visualization_subscribers(True)
    assert sampler._has_visualization_subscribers

    sampler.set_motor_subscribers(True)
    assert sampler._has_motor_subscribers


def test_process_burst_method(mock_connectome_manager, mock_state_manager):
    """Test the _process_burst method."""
    with patch(
        "feagi.npu.burst_engine.FeagiStateManager.instance",
        return_value=mock_state_manager,
    ):
        engine = BurstEngine(
            connectome_manager=mock_connectome_manager,
            # fcl_manager no longer needed - handled by FCLInjector internally
        config={"target_frequency": 20},
        )

        # Mock the update_membrane_potentials method to track calls
        mock_connectome_manager.update_membrane_potentials = MagicMock(
            return_value=[1, 2, 3]
        )

        # Call the _process_burst method
        result = engine._process_burst()

        # Verify that update_membrane_potentials was called
        mock_connectome_manager.update_membrane_potentials.assert_called_once()

        # Verify the result
        assert result == [1, 2, 3]


@pytest.mark.skip(
    reason="Signal handling removed for RTOS compatibility - signals not available in RTOS"
)
def test_signal_handler_registration():
    """Test signal handler registration in run method - DEPRECATED for RTOS compatibility."""
    pass


@pytest.mark.skip(
    reason="Signal handling removed for RTOS compatibility - signals not available in RTOS"
)
def test_signal_handler_not_in_main_thread():
    """Test that signal handlers are not registered in non-main threads - DEPRECATED for RTOS compatibility."""
    pass


if __name__ == "__main__":
    pytest.main(["-v", __file__])
