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
        self.mock_connectome_manager.cortical_areas = {1: MagicMock(id=1, properties={})}

        # Configure mock state manager instance
        with patch("feagi.npu.burst_engine.FeagiStateManager") as mock_feagi_state_manager:
            mock_feagi_state_manager.instance.return_value = self.mock_state_manager

            # Create the burst engine
            self.burst_engine = BurstEngine(
                connectome_manager=self.mock_connectome_manager,
                fcl_manager=self.mock_fcl_manager,
                config={"desired_frequency_hz": 10.0},
            )

    def test_run_stops_cleanly(self):
        """Validate that run() starts and can stop cleanly without OS signal/threading dependencies."""
        # Ensure injection is disabled to avoid side-effects during test
        self.burst_engine.injection_service = None
        if hasattr(self.burst_engine, "enable_injection"):
            self.burst_engine.enable_injection = False
        # Simulate one iteration then stop
        def side_effect(*args, **kwargs):
            self.burst_engine._running = False
            return [101, 102, 103]

        self.mock_connectome_manager.update_membrane_potentials.side_effect = side_effect

        # Mark genome loaded
        self.burst_engine.genome_loaded = True

        # Execute
        self.burst_engine.run()

        # Assert the loop stopped and update was called
        self.assertFalse(self.burst_engine._running)
        self.mock_connectome_manager.update_membrane_potentials.assert_called()


if __name__ == "__main__":
    unittest.main()
