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

import signal
import unittest
from unittest.mock import MagicMock, patch

from feagi.npu.burst_engine import BurstEngine


class TestBurstEngineSignalHandling(unittest.TestCase):
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

    @patch("feagi.npu.burst_engine.signal.signal")
    @patch("feagi.npu.burst_engine.threading.current_thread")
    @patch("feagi.npu.burst_engine.threading.main_thread")
    @patch("feagi.npu.burst_engine.logger")
    def test_signal_handlers_registered_in_main_thread(
        self, mock_logger, mock_main_thread, mock_current_thread, mock_signal
    ):
        """Test that signal handlers are registered when running in the main thread."""
        # Create mock thread objects
        mock_thread = MagicMock()
        mock_thread.name = "MainThread"

        # Configure mocks for threading to return the same thread object
        mock_current_thread.return_value = mock_thread
        mock_main_thread.return_value = mock_thread

        # Set up run to exit immediately
        def side_effect(*args):
            self.burst_engine._running = False
            return [101, 102, 103]

        self.mock_connectome_manager.update_membrane_potentials.side_effect = (
            side_effect
        )

        # Call the method
        self.burst_engine.run()

        # Check if signal registration was called twice (for SIGINT and SIGTERM)
        self.assertEqual(mock_signal.call_count, 2)

        # Check the signals that were registered
        signal_calls = [call[0][0] for call in mock_signal.call_args_list]
        self.assertIn(signal.SIGINT, signal_calls)
        self.assertIn(signal.SIGTERM, signal_calls)

    @patch("feagi.npu.burst_engine.signal.signal")
    @patch("feagi.npu.burst_engine.threading.current_thread")
    @patch("feagi.npu.burst_engine.threading.main_thread")
    @patch("feagi.npu.burst_engine.logger")
    def test_signal_handlers_not_registered_in_non_main_thread(
        self, mock_logger, mock_main_thread, mock_current_thread, mock_signal
    ):
        """Test that signal handlers are not registered when running in a non-main thread."""
        # Create different mock thread objects
        mock_main_thread_obj = MagicMock()
        mock_main_thread_obj.name = "MainThread"

        mock_worker_thread = MagicMock()
        mock_worker_thread.name = "WorkerThread"

        # Configure mocks for threading to return different thread objects
        mock_current_thread.return_value = mock_worker_thread
        mock_main_thread.return_value = mock_main_thread_obj

        # Set up run to exit immediately
        def side_effect(*args):
            self.burst_engine._running = False
            return [101, 102, 103]

        self.mock_connectome_manager.update_membrane_potentials.side_effect = (
            side_effect
        )

        # Call the method
        self.burst_engine.run()

        # Check that signal registration was not called
        mock_signal.assert_not_called()

    @patch("feagi.npu.burst_engine.signal.signal")
    @patch("feagi.npu.burst_engine.threading.current_thread")
    @patch("feagi.npu.burst_engine.threading.main_thread")
    @patch("feagi.npu.burst_engine.logger")
    def test_handle_signal_stops_engine(
        self, mock_logger, mock_main_thread, mock_current_thread, mock_signal
    ):
        """Test that the signal handler stops the burst engine."""
        # Create mock thread objects
        mock_thread = MagicMock()
        mock_thread.name = "MainThread"

        # Configure mocks for threading to return the same thread object
        mock_current_thread.return_value = mock_thread
        mock_main_thread.return_value = mock_thread

        # Set up the burst engine
        self.burst_engine._running = True

        # Set up run to exit after registering handlers
        def side_effect(*args):
            self.burst_engine._running = False
            return [101, 102, 103]

        self.mock_connectome_manager.update_membrane_potentials.side_effect = (
            side_effect
        )

        # Call run() to register handlers
        self.burst_engine.run()

        # Extract the signal handler function
        handler_func = mock_signal.call_args_list[0][0][1]

        # Reset _running to True
        self.burst_engine._running = True

        # Call the handler
        handler_func(signal.SIGINT, None)

        # Check if _running was set to False
        self.assertFalse(self.burst_engine._running)


if __name__ == "__main__":
    unittest.main()
