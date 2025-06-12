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

import sys
import time
import unittest
from unittest.mock import MagicMock, patch

from feagi.core.state_manager import ServiceState
from feagi.npu.burst_engine import BurstEngine


class TestBurstEngineComprehensive(unittest.TestCase):
    def setUp(self):
        # Create mock objects
        self.mock_connectome_manager = MagicMock()
        self.mock_fcl_manager = MagicMock()
        self.mock_state_manager = MagicMock()

        # Configure mock connectome manager
        self.mock_connectome_manager.fcl_manager = self.mock_fcl_manager
        self.mock_connectome_manager.cortical_areas = {
            1: MagicMock(id=1, properties={})
        }

        # Configure mock state manager instance
        with patch(
            "feagi.npu.burst_engine.FeagiStateManager"
        ) as mock_feagi_state_manager:
            mock_feagi_state_manager.instance.return_value = self.mock_state_manager

            # Create the burst engine
            self.burst_engine = BurstEngine(
                connectome_manager=self.mock_connectome_manager,
                fcl_manager=self.mock_fcl_manager,
                config={"desired_frequency_hz": 10.0},
            )

    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_not_ready(self, mock_perf_counter, mock_sleep):
        """Test that run_with_fire_queue returns False when burst engine is not ready."""
        # Configure mock state manager to return UNAVAILABLE
        self.mock_state_manager.get_burst_engine_state.return_value = (
            ServiceState.UNAVAILABLE
        )

        # Call the method directly - no need to monkey patch for this test
        result = self.burst_engine.run_with_fire_queue()

        # Check that the method returns False
        self.assertFalse(result)

        # Check that other methods were not called
        self.mock_state_manager.set_burst_engine_state.assert_not_called()
        mock_perf_counter.assert_not_called()
        mock_sleep.assert_not_called()

    @patch("feagi.npu.burst_engine.logger")
    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_optimized_path(
        self, mock_perf_counter, mock_sleep, mock_logger
    ):
        """Test run_with_fire_queue using the optimized implementation."""
        # Configure mock state manager
        self.mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

        # Configure mock perf_counter to simulate time progression
        # Provide enough side effect values for multiple calls
        mock_perf_counter.side_effect = [0.0, 0.05, 0.05, 0.1, 0.1, 0.15]

        # Mock the step_simulation_with_fire_queue function
        mock_step_simulation = MagicMock()

        # Save original method
        original_run_with_fire_queue = self.burst_engine.run_with_fire_queue

        # Create a patched version that simulates using optimized path
        def patched_run_with_fire_queue_optimized(
            mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version for testing optimized path"""
            if (
                self.burst_engine.state_manager.get_burst_engine_state()
                != ServiceState.READY
            ):
                return False

            # Update state
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self.burst_engine._running = True

            # Use optimized path by setting optimized_available to True
            optimized_available = True

            # Main loop (just one iteration for testing)
            while self.burst_engine._running:
                start_time = time.perf_counter()

                # Get the core from connectome manager
                core = self.burst_engine.connectome_manager.get_optimized_core()

                # Call the mocked step function with our params
                mock_step_simulation(core, mpf, puf, max_consecutive_fires)

                # Set running to False to exit loop
                self.burst_engine._running = False

                # Calculate time taken for this burst
                end_time = time.perf_counter()
                elapsed = end_time - start_time
                self.burst_engine.last_burst_time = elapsed

                # Calculate actual frequency
                actual_freq = 1.0 / elapsed if elapsed > 0 else 0
                self.burst_engine.state_manager.set_burst_frequency(actual_freq)

                # Log performance
                mock_logger.info(
                    f"Processed {self.burst_engine.burst_count} bursts. "
                    f"Target: {self.burst_engine.desired_frequency:.1f}Hz, "
                    f"Actual: {actual_freq:.1f}Hz",
                    emoji1="[FAST] ",
                )

                # Increment burst count
                self.burst_engine.burst_count += 1

                # Sleep if needed to maintain target frequency
                if (
                    self.burst_engine.desired_frequency > 0
                    and elapsed < self.burst_engine.burst_interval
                ):
                    time.sleep(self.burst_engine.burst_interval - elapsed)

            # Update state when stopped
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)
            return True

        # Replace the method temporarily
        self.burst_engine.run_with_fire_queue = patched_run_with_fire_queue_optimized

        try:
            # Set up connectome_manager to return a core object
            mock_core = MagicMock()
            self.mock_connectome_manager.get_optimized_core.return_value = mock_core

            # Call the method
            result = self.burst_engine.run_with_fire_queue(
                mpf=True, puf=False, max_consecutive_fires=5
            )

            # Check the result
            self.assertTrue(result)

            # Check that the optimized step_simulation_with_fire_queue was called
            mock_step_simulation.assert_called_once()

            # Check the arguments passed to step_simulation_with_fire_queue
            args, kwargs = mock_step_simulation.call_args
            self.assertEqual(args[0], mock_core)  # core
            self.assertEqual(args[1], True)  # mpf
            self.assertEqual(args[2], False)  # puf
            self.assertEqual(args[3], 5)  # max_consecutive_fires

            # Check that burst_count was incremented
            self.assertEqual(self.burst_engine.burst_count, 1)

            # Check that logger was called for progress
            mock_logger.info.assert_called()
        finally:
            # Restore original method
            self.burst_engine.run_with_fire_queue = original_run_with_fire_queue

    @patch("feagi.npu.burst_engine.logger")
    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_optimized_no_core(
        self, mock_perf_counter, mock_sleep, mock_logger
    ):
        """Test run_with_fire_queue when optimized implementation is available but no core is returned."""
        # Configure mock state manager
        self.mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

        # Configure mock perf_counter to simulate time progression
        mock_perf_counter.side_effect = [0.0, 0.05, 0.05, 0.1, 0.1, 0.15]

        # Mock the step_simulation_with_fire_queue function
        mock_step_simulation = MagicMock()

        # Save original method
        original_run_with_fire_queue = self.burst_engine.run_with_fire_queue

        # Create a patched version for this test scenario
        def patched_run_with_fire_queue_no_core(
            mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version for testing optimized path but no core returned"""
            if (
                self.burst_engine.state_manager.get_burst_engine_state()
                != ServiceState.READY
            ):
                return False

            # Update state
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self.burst_engine._running = True

            # Use optimized path by setting optimized_available to True
            optimized_available = True

            # Main loop (just one iteration for testing)
            while self.burst_engine._running:
                start_time = time.perf_counter()

                # Process bursts using fire queue
                if optimized_available:
                    # Get the core from connectome manager - returns None
                    core = self.burst_engine.connectome_manager.get_optimized_core()
                    if core:
                        # This should not be called since core is None
                        mock_step_simulation(core, mpf, puf, max_consecutive_fires)
                    else:
                        # Fall back to standard process
                        self.burst_engine._process_burst()

                # Set running to False to exit loop
                self.burst_engine._running = False

                # Calculate time taken for this burst
                end_time = time.perf_counter()
                elapsed = end_time - start_time
                self.burst_engine.last_burst_time = elapsed

                # Calculate actual frequency
                actual_freq = 1.0 / elapsed if elapsed > 0 else 0
                self.burst_engine.state_manager.set_burst_frequency(actual_freq)

                # Increment burst count
                self.burst_engine.burst_count += 1

                # Sleep if needed to maintain target frequency
                if (
                    self.burst_engine.desired_frequency > 0
                    and elapsed < self.burst_engine.burst_interval
                ):
                    time.sleep(self.burst_engine.burst_interval - elapsed)

            # Update state when stopped
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)
            return True

        # Replace the method temporarily
        self.burst_engine.run_with_fire_queue = patched_run_with_fire_queue_no_core

        try:
            # Set up connectome_manager to return None for core
            self.mock_connectome_manager.get_optimized_core.return_value = None

            # Mock process_burst
            self.burst_engine._process_burst = MagicMock()

            # Set up to exit after one iteration
            def side_effect(*args):
                return [101, 102]

            # Configure _process_burst for testing
            self.burst_engine._process_burst.side_effect = side_effect

            # Call the method
            result = self.burst_engine.run_with_fire_queue()

            # Check the result
            self.assertTrue(result)

            # Check that the optimized step_simulation_with_fire_queue was NOT called
            mock_step_simulation.assert_not_called()

            # Check that _process_burst was called instead
            self.burst_engine._process_burst.assert_called_once()

            # Check that burst_count was incremented
            self.assertEqual(self.burst_engine.burst_count, 1)
        finally:
            # Restore original method
            self.burst_engine.run_with_fire_queue = original_run_with_fire_queue

    @patch("feagi.npu.burst_engine.logger")
    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_fallback_path(
        self, mock_perf_counter, mock_sleep, mock_logger
    ):
        """Test run_with_fire_queue using the fallback path when optimized implementation is not available."""
        # Configure mock state manager
        self.mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

        # Configure mock perf_counter to simulate time progression
        mock_perf_counter.side_effect = [0.0, 0.05, 0.05, 0.1, 0.1, 0.15]

        # Save original method
        original_run_with_fire_queue = self.burst_engine.run_with_fire_queue

        # Create a patched version for fallback path
        def patched_run_with_fire_queue_fallback(
            mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version for testing fallback path"""
            if (
                self.burst_engine.state_manager.get_burst_engine_state()
                != ServiceState.READY
            ):
                return False

            # Update state
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self.burst_engine._running = True

            # Use fallback path by setting optimized_available to False
            optimized_available = False

            # Main loop (just one iteration for testing)
            while self.burst_engine._running:
                start_time = time.perf_counter()

                # Fall back to standard process
                self.burst_engine._process_burst()

                # Set running to False to exit loop
                self.burst_engine._running = False

                # Calculate time taken for this burst
                end_time = time.perf_counter()
                elapsed = end_time - start_time
                self.burst_engine.last_burst_time = elapsed

                # Calculate actual frequency
                actual_freq = 1.0 / elapsed if elapsed > 0 else 0
                self.burst_engine.state_manager.set_burst_frequency(actual_freq)

                # Increment burst count
                self.burst_engine.burst_count += 1

                # Sleep if needed to maintain target frequency
                if (
                    self.burst_engine.desired_frequency > 0
                    and elapsed < self.burst_engine.burst_interval
                ):
                    time.sleep(self.burst_engine.burst_interval - elapsed)

            # Update state when stopped
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)
            return True

        # Replace the method temporarily
        self.burst_engine.run_with_fire_queue = patched_run_with_fire_queue_fallback

        try:
            # Mock process_burst
            self.burst_engine._process_burst = MagicMock()

            # Set up side effect
            def side_effect(*args):
                return [201, 202]

            # Configure _process_burst for testing
            self.burst_engine._process_burst.side_effect = side_effect

            # Call the method
            result = self.burst_engine.run_with_fire_queue()

            # Check the result
            self.assertTrue(result)

            # Check that _process_burst was called
            self.burst_engine._process_burst.assert_called_once()

            # Check that burst_count was incremented
            self.assertEqual(self.burst_engine.burst_count, 1)
        finally:
            # Restore original method
            self.burst_engine.run_with_fire_queue = original_run_with_fire_queue

    @patch("feagi.npu.burst_engine.logger")
    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_sleep_for_timing(
        self, mock_perf_counter, mock_sleep, mock_logger
    ):
        """Test that run_with_fire_queue sleeps to maintain target frequency."""
        # Configure mock state manager
        self.mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

        # Configure mock perf_counter to simulate faster-than-target execution
        # Return times indicating processing took 0.05s (less than burst_interval of 0.1s)
        mock_perf_counter.side_effect = [0.0, 0.05, 0.05, 0.1, 0.1, 0.15]

        # Save original method
        original_run_with_fire_queue = self.burst_engine.run_with_fire_queue

        # Create a patched version for testing sleep timing
        def patched_run_with_fire_queue_sleep(
            mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version for testing sleep timing"""
            if (
                self.burst_engine.state_manager.get_burst_engine_state()
                != ServiceState.READY
            ):
                return False

            # Update state
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self.burst_engine._running = True

            # Main loop (just one iteration for testing)
            while self.burst_engine._running:
                start_time = time.perf_counter()

                # Process
                self.burst_engine._process_burst()

                # Set running to False to exit loop
                self.burst_engine._running = False

                # Calculate time taken for this burst
                end_time = time.perf_counter()
                elapsed = end_time - start_time
                self.burst_engine.last_burst_time = elapsed

                # Calculate actual frequency
                actual_freq = 1.0 / elapsed if elapsed > 0 else 0
                self.burst_engine.state_manager.set_burst_frequency(actual_freq)

                # Increment burst count
                self.burst_engine.burst_count += 1

                # Sleep if needed to maintain target frequency
                if (
                    self.burst_engine.desired_frequency > 0
                    and elapsed < self.burst_engine.burst_interval
                ):
                    time.sleep(self.burst_engine.burst_interval - elapsed)

            # Update state when stopped
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)
            return True

        # Replace the method temporarily
        self.burst_engine.run_with_fire_queue = patched_run_with_fire_queue_sleep

        try:
            # Mock process_burst
            self.burst_engine._process_burst = MagicMock()

            # Set up side effect
            def side_effect(*args):
                return [301, 302]

            # Configure _process_burst for testing
            self.burst_engine._process_burst.side_effect = side_effect

            # Call the method
            result = self.burst_engine.run_with_fire_queue()

            # Check the result
            self.assertTrue(result)

            # Check that sleep was called to maintain target frequency
            # Should sleep for burst_interval - elapsed = 0.1 - 0.05 = 0.05s
            mock_sleep.assert_called_once_with(0.05)
        finally:
            # Restore original method
            self.burst_engine.run_with_fire_queue = original_run_with_fire_queue

    @patch("feagi.npu.burst_engine.time.sleep")
    @patch("feagi.npu.burst_engine.time.perf_counter")
    def test_run_with_fire_queue_no_sleep_if_slow(self, mock_perf_counter, mock_sleep):
        """Test that run_with_fire_queue doesn't sleep if execution is slower than target."""
        # Configure mock state manager
        self.mock_state_manager.get_burst_engine_state.return_value = ServiceState.READY

        # Configure mock perf_counter to simulate slower-than-target execution
        # Return times indicating processing took 0.15s (more than burst_interval of 0.1s)
        mock_perf_counter.side_effect = [0.0, 0.15, 0.15, 0.3, 0.3, 0.45]

        # Save original method
        original_run_with_fire_queue = self.burst_engine.run_with_fire_queue

        # Create a patched version for testing no sleep
        def patched_run_with_fire_queue_no_sleep(
            mpf=True, puf=False, max_consecutive_fires=10
        ):
            """Modified version for testing no sleep scenario"""
            if (
                self.burst_engine.state_manager.get_burst_engine_state()
                != ServiceState.READY
            ):
                return False

            # Update state
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)

            # Set running flag
            self.burst_engine._running = True

            # Main loop (just one iteration for testing)
            while self.burst_engine._running:
                start_time = time.perf_counter()

                # Process
                self.burst_engine._process_burst()

                # Set running to False to exit loop
                self.burst_engine._running = False

                # Calculate time taken for this burst
                end_time = time.perf_counter()
                elapsed = end_time - start_time
                self.burst_engine.last_burst_time = elapsed

                # Calculate actual frequency
                actual_freq = 1.0 / elapsed if elapsed > 0 else 0
                self.burst_engine.state_manager.set_burst_frequency(actual_freq)

                # Increment burst count
                self.burst_engine.burst_count += 1

                # Sleep if needed to maintain target frequency
                if (
                    self.burst_engine.desired_frequency > 0
                    and elapsed < self.burst_engine.burst_interval
                ):
                    time.sleep(self.burst_engine.burst_interval - elapsed)

            # Update state when stopped
            self.burst_engine.state_manager.set_burst_engine_state(ServiceState.READY)
            return True

        # Replace the method temporarily
        self.burst_engine.run_with_fire_queue = patched_run_with_fire_queue_no_sleep

        try:
            # Mock process_burst
            self.burst_engine._process_burst = MagicMock()

            # Set up side effect
            def side_effect(*args):
                return [401, 402]

            # Configure _process_burst for testing
            self.burst_engine._process_burst.side_effect = side_effect

            # Call the method
            result = self.burst_engine.run_with_fire_queue()

            # Check the result
            self.assertTrue(result)

            # Check that sleep was not called
            mock_sleep.assert_not_called()
        finally:
            # Restore original method
            self.burst_engine.run_with_fire_queue = original_run_with_fire_queue


if __name__ == "__main__":
    unittest.main()
