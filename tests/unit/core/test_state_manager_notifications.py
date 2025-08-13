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

from unittest.mock import Mock

from feagi.core.state_manager import FeagiStateManager, GenomeState
from feagi.core.state_storage import MemoryStorage


def test_register_notification_callback():
    """Test registering notification callbacks"""
    storage = MemoryStorage()
    state_manager = FeagiStateManager(storage)
    callback = Mock()

    result = state_manager.register_notification_callback("genome", callback)
    assert result is True
    assert callback in state_manager._notification_callbacks["genome"]

    # Test invalid category
    result = state_manager.register_notification_callback("invalid", callback)
    assert result is False


def test_notification_callback_execution():
    """Test that callbacks are executed when state changes"""
    storage = MemoryStorage()
    state_manager = FeagiStateManager(storage)
    callback = Mock()

    state_manager.register_notification_callback("genome", callback)

    # Change state to trigger notification
    old_state = state_manager.get_genome_state()
    new_state = GenomeState.LOADED
    state_manager.set_genome_state(new_state)

    callback.assert_called_once_with(old_state, new_state)


def test_notification_error_handling():
    """Test that errors in callbacks don't crash the state manager"""
    storage = MemoryStorage()
    state_manager = FeagiStateManager(storage)

    def failing_callback(old, new):
        raise RuntimeError("Test error")

    state_manager.register_notification_callback("genome", failing_callback)

    # Should not raise exception
    state_manager.set_genome_state(GenomeState.LOADED)
