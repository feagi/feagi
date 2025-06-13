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

import os
import tempfile
from unittest.mock import patch

import pytest

from feagi.api.core.services.core_api_service import CoreAPIService
from feagi.bdu.connectome_manager import ConnectomeManager
from feagi.core.state_manager import FeagiStateManager


@pytest.fixture
def setup_environment():
    """Set up test environment with state manager and core API service"""
    temp_file = tempfile.NamedTemporaryFile(delete=False)
    temp_file.close()

    try:
        state_manager = FeagiStateManager(temp_file.name)
        connectome = ConnectomeManager()
        connectome.initialize_arrays()
        core_api = CoreAPIService(connectome, state_manager)
        yield state_manager, connectome, core_api
    finally:
        # Clean up
        if os.path.exists(temp_file.name):
            os.unlink(temp_file.name)


def test_end_to_end_transaction_flow(setup_environment):
    """Test the full transaction flow from API to connectome"""
    state_manager, connectome, core_api = setup_environment

    # Load a minimal test genome
    test_genome = {"genome_id": "test", "blueprint": {"cortical_areas": {}}}

    with patch.object(
        CoreAPIService, "load_genome", return_value={"success": True, "duration": 0.1}
    ):
        core_api._current_genome = test_genome  # Directly set for testing

        # Create a transaction through the API
        transaction = core_api.begin_transaction()

        # Add a cortical area
        transaction.record_change(
            "add_cortical_area",
            None,
            None,
            {
                "name": "Test Area",
                "type": "sensory",
                "dimensions": [10, 10, 5],
                "position": [0, 0, 0],
            },
        )

        # Mock the commit process
        with patch.object(transaction, "commit", return_value=True):
            # Test that the API properly handles the transaction
            result = core_api.modify_genome(transaction)
            assert result is True
