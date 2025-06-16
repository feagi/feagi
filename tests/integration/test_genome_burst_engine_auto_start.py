"""
Test for automatic burst engine start after genome load.

This test verifies the signal flow:
1. Genome service loads genome
2. Genome service emits GENOME_LOADED event
3. Process manager receives event
4. Process manager starts burst engine
"""

import time
from unittest.mock import Mock, patch

import pytest

from feagi.api.core.services.genome.genome_service import GenomeService
from feagi.api.shared_memory.events import (
    EventNotificationSystem,
    EventPriority,
    EventType,
)
from feagi.core.state_manager import ServiceState
from feagi.process_manager import ProcessManager


@pytest.fixture
def mock_connectome_manager():
    """Mock connectome manager."""
    mock = Mock()
    mock.cortical_areas = {}
    mock.neuron_count = 0
    mock.synapse_count = 0
    return mock


@pytest.fixture
def mock_state_manager():
    """Mock state manager."""
    mock = Mock()
    mock.get_burst_engine_state.return_value = ServiceState.UNINITIALIZED
    mock.set_genome_state = Mock()
    mock.set_brain_readiness = Mock()
    mock.get_genome_counter.return_value = 1
    mock.increment_genome_counter = Mock()
    mock.set_genome_timestamp = Mock()
    mock.brain_stats = {}
    mock.cortical_list = []
    mock.genome_validity = True
    mock.changes_saved_externally = False
    mock.exit_condition = False
    return mock


@pytest.fixture
def event_system():
    """Create a real event system for testing."""
    system = EventNotificationSystem("test_genome_events")
    yield system
    system.stop()


@pytest.fixture
def genome_service(mock_connectome_manager, mock_state_manager):
    """Create genome service with mocked dependencies."""
    return GenomeService(mock_connectome_manager, mock_state_manager)


class TestGenomeBurstEngineAutoStart:
    """Test automatic burst engine start after genome load."""

    def test_genome_service_emits_genome_loaded_event(
        self, genome_service, event_system
    ):
        """Test that genome service emits GENOME_LOADED event after successful load."""
        # Set up event handler to capture the event
        received_events = []

        def event_handler(event):
            received_events.append(event)

        event_system.register_handler(EventType.GENOME_LOADED, event_handler)
        event_system.start()

        # Mock the neuroembryogenesis process
        with patch(
            "feagi.bdu.embryogenesis.neuroembryogenesis.NeuroEmbryogenesis"
        ) as mock_neuro:
            mock_instance = Mock()
            mock_instance.develop_brain_from_genome.return_value = (
                {"success": True, "cortical_area_count": 24},
                {"valid": True, "errors": []},
                {"recovery_performed": False},
            )
            mock_neuro.return_value = mock_instance

            # Mock the brain service imports
            with patch(
                "feagi.api.core.services.brain.brain_service.BrainService"
            ) as mock_brain_service:
                mock_brain_service.return_value.start_burst_engine.return_value = True

                # Mock the burst engine update
                with patch(
                    "feagi.npu.burst_engine.BurstEngine.get_instance"
                ) as mock_get_instance:
                    mock_burst_engine = Mock()
                    mock_get_instance.return_value = mock_burst_engine

                    # Create test genome data
                    test_genome = {
                        "blueprint": {
                            "test_area": {
                                "cortical_id": "test01",
                                "cortical_name": "Test Area",
                            }
                        }
                    }

                    # Inject event system into genome service
                    genome_service._event_system = event_system

                    # Load genome
                    result = genome_service.load_genome(test_genome, "test_genome.json")

                    # Wait for event to be processed
                    time.sleep(0.1)

                    # Verify event was emitted
                    assert len(received_events) == 1
                    event = received_events[0]
                    assert event.event_type == EventType.GENOME_LOADED
                    assert event.data["filename"] == "test_genome.json"
                    assert event.data["cortical_areas"] == 24
                    assert event.priority == EventPriority.HIGH

    def test_process_manager_handles_genome_loaded_event(
        self, mock_connectome_manager, event_system
    ):
        """Test that process manager handles GENOME_LOADED event and starts burst engine."""
        # Track whether the event handler was called
        handler_called = []

        # Create a wrapper for the event handler to track calls
        def track_event_handler(event):
            handler_called.append(event)
            # Call a mock brain service instead of the real one
            mock_brain_service = Mock()
            mock_brain_service.start_burst_engine.return_value = True

        # Create process manager with mocked dependencies
        with patch(
            "feagi.process_manager.FeagiStateManager"
        ) as mock_state_manager_class:
            mock_state_manager = Mock()
            mock_state_manager.get_burst_engine_state.return_value = (
                ServiceState.UNINITIALIZED
            )
            mock_state_manager_class.instance.return_value = mock_state_manager

            with patch(
                "feagi.api.core.services.brain.brain_service.BrainService"
            ) as mock_brain_service_class:
                mock_brain_service = Mock()
                mock_brain_service.start_burst_engine.return_value = True
                mock_brain_service_class.return_value = mock_brain_service

                # Mock the ConnectomeManager.instance() call in ProcessManager
                with patch(
                    "feagi.bdu.connectome_manager.ConnectomeManager.instance"
                ) as mock_cm_instance:
                    mock_cm_instance.return_value = mock_connectome_manager

                    # Create process manager (it will create its own connectome manager)
                    process_manager = ProcessManager()

                    # Inject event system
                    process_manager._event_system = event_system

                    # Register the handler manually (since we're bypassing the normal init)
                    event_system.register_handler(
                        EventType.GENOME_LOADED,
                        process_manager._handle_genome_loaded_event,
                    )
                    event_system.start()

                    # Send GENOME_LOADED event
                    event_system.send_event(
                        EventType.GENOME_LOADED,
                        data={"filename": "test_genome.json", "cortical_areas": 24},
                        priority=EventPriority.HIGH,
                    )

                    # Wait for event to be processed
                    time.sleep(0.1)

                    # Verify brain service was called to start burst engine
                    # The exact mock verification is less important than verifying the flow works
                    mock_brain_service_class.assert_called_once()
                    mock_brain_service.start_burst_engine.assert_called_once()

    def test_process_manager_skips_start_if_burst_engine_already_running(
        self, mock_connectome_manager, event_system
    ):
        """Test that process manager doesn't start burst engine if already running."""
        with patch(
            "feagi.process_manager.FeagiStateManager"
        ) as mock_state_manager_class:
            mock_state_manager = Mock()
            mock_state_manager.get_burst_engine_state.return_value = (
                ServiceState.READY
            )  # Already running
            mock_state_manager_class.instance.return_value = mock_state_manager

            with patch(
                "feagi.api.core.services.brain.brain_service.BrainService"
            ) as mock_brain_service_class:
                mock_brain_service = Mock()
                mock_brain_service_class.return_value = mock_brain_service

                # Mock the ConnectomeManager.instance() call in ProcessManager
                with patch(
                    "feagi.bdu.connectome_manager.ConnectomeManager.instance"
                ) as mock_cm_instance:
                    mock_cm_instance.return_value = mock_connectome_manager

                    # Create process manager
                    process_manager = ProcessManager()

                    # Inject event system
                    process_manager._event_system = event_system

                    # Register the handler manually
                    event_system.register_handler(
                        EventType.GENOME_LOADED,
                        process_manager._handle_genome_loaded_event,
                    )
                    event_system.start()

                    # Send GENOME_LOADED event
                    event_system.send_event(
                        EventType.GENOME_LOADED,
                        data={"filename": "test_genome.json", "cortical_areas": 24},
                        priority=EventPriority.HIGH,
                    )

                    # Wait for event to be processed
                    time.sleep(0.1)

                    # Verify brain service was NOT called since burst engine already running
                    mock_brain_service.start_burst_engine.assert_not_called()

    def test_event_system_error_handling(self, genome_service, event_system):
        """Test that genome loading doesn't fail if event system has issues."""
        # Mock neuroembryogenesis
        with patch(
            "feagi.bdu.embryogenesis.neuroembryogenesis.NeuroEmbryogenesis"
        ) as mock_neuro:
            mock_instance = Mock()
            mock_instance.develop_brain_from_genome.return_value = (
                {"success": True, "cortical_area_count": 24},
                {"valid": True, "errors": []},
                {"recovery_performed": False},
            )
            mock_neuro.return_value = mock_instance

            with patch("feagi.api.core.services.brain.brain_service.BrainService"):
                with patch(
                    "feagi.npu.burst_engine.BurstEngine.get_instance"
                ) as mock_get_instance:
                    mock_burst_engine = Mock()
                    mock_get_instance.return_value = mock_burst_engine

                    # Create a broken event system
                    broken_event_system = Mock()
                    broken_event_system.send_event.side_effect = Exception(
                        "Event system broken"
                    )

                    # Inject broken event system
                    genome_service._event_system = broken_event_system

                    # Create test genome data
                    test_genome = {
                        "blueprint": {
                            "test_area": {
                                "cortical_id": "test01",
                                "cortical_name": "Test Area",
                            }
                        }
                    }

                    # Load genome should still succeed despite event system failure
                    result = genome_service.load_genome(test_genome, "test_genome.json")

                    # Verify genome load succeeded despite event system failure
                    assert result["success"] == True
                    assert "cortical_area_count" in result

    def test_simple_event_emission_check(self, genome_service):
        """Simple test to verify event emission mechanism works."""
        # Create a mock event system
        mock_event_system = Mock()
        genome_service._event_system = mock_event_system

        # Mock all the dependencies
        with patch(
            "feagi.bdu.embryogenesis.neuroembryogenesis.NeuroEmbryogenesis"
        ) as mock_neuro:
            mock_instance = Mock()
            mock_instance.develop_brain_from_genome.return_value = (
                {"success": True, "cortical_area_count": 5},
                {"valid": True, "errors": []},
                {"recovery_performed": False},
            )
            mock_neuro.return_value = mock_instance

            with patch("feagi.api.core.services.brain.brain_service.BrainService"):
                with patch(
                    "feagi.npu.burst_engine.BurstEngine.get_instance"
                ) as mock_get_instance:
                    mock_burst_engine = Mock()
                    mock_get_instance.return_value = mock_burst_engine

                    # Create simple test genome
                    test_genome = {"blueprint": {"test": {"cortical_id": "test01"}}}

                    # Load genome
                    result = genome_service.load_genome(test_genome, "simple_test.json")

                    # Verify genome load succeeded
                    assert result["success"] == True

                    # Verify event was sent
                    mock_event_system.send_event.assert_called_once()
                    call_args = mock_event_system.send_event.call_args

                    # Check the event type and data
                    assert call_args[0][0] == EventType.GENOME_LOADED
                    assert call_args[1]["data"]["filename"] == "simple_test.json"
                    assert call_args[1]["priority"] == EventPriority.HIGH
