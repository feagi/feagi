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

"""Tests for the AgentsService class."""

from unittest.mock import MagicMock

import pytest

from feagi.api.core.services.agents.agents_service import AgentsService
from feagi.bdu.connectome_manager import ConnectomeManager


@pytest.fixture
def mock_connectome_manager():
    """Create a mock ConnectomeManager for testing."""
    return MagicMock(spec=ConnectomeManager)


@pytest.fixture
def mock_state_manager():
    """Create a mock state manager for testing."""
    sm = MagicMock()
    sm.connected_agents = {
        "agent1": {
            "id": "agent1",
            "type": "sensor",
            "status": "active",
            "last_seen": "2023-01-01T10:00:00",
        },
        "agent2": {
            "id": "agent2",
            "type": "motor",
            "status": "connected",
            "last_seen": "2023-01-01T10:05:00",
        },
    }
    return sm


@pytest.fixture
def agents_service(mock_connectome_manager, mock_state_manager):
    """Create an AgentsService instance with mocked dependencies."""
    return AgentsService(mock_connectome_manager, mock_state_manager)


@pytest.fixture
def agents_service_no_state(mock_connectome_manager):
    """Create an AgentsService instance without state manager."""
    return AgentsService(mock_connectome_manager, None)


class TestAgentsService:
    """Test cases for the AgentsService."""

    def test_init(self, mock_connectome_manager, mock_state_manager):
        """Test AgentsService initialization."""
        service = AgentsService(mock_connectome_manager, mock_state_manager)

        assert service._connectome_manager == mock_connectome_manager
        assert service.state_manager == mock_state_manager

    def test_get_connected_agents_with_agents(self, agents_service, mock_state_manager):
        """Test getting connected agents when agents exist."""
        # Mock connected agents in the expected format
        mock_state_manager.connected_agents = {
            "agent1": {
                "type": "sensor",
                "status": "connected",
                "last_seen": "2023-01-01T12:00:00Z",
                "capabilities": ["vision", "movement"],
                "address": "192.168.1.100",
                "metadata": {},
            },
            "agent2": {
                "type": "actuator",
                "status": "disconnected",
                "last_seen": "2023-01-01T11:00:00Z",
                "capabilities": ["movement"],
                "address": "192.168.1.101",
                "metadata": {},
            },
        }

        result = agents_service.get_connected_agents()

        assert isinstance(result, list)
        assert len(result) == 2
        # Check first agent
        assert result[0]["agent_id"] == "agent1"
        assert result[0]["agent_type"] == "sensor"
        assert result[0]["status"] == "connected"
        assert result[0]["capabilities"] == ["vision", "movement"]

    def test_get_connected_agents_empty(self, agents_service, mock_state_manager):
        """Test getting connected agents when none exist."""
        mock_state_manager.connected_agents = {}

        result = agents_service.get_connected_agents()

        assert isinstance(result, list)
        assert len(result) == 0

    def test_get_connected_agents_no_state_manager(self, agents_service_no_state):
        """Test getting connected agents when no state manager exists."""
        result = agents_service_no_state.get_connected_agents()

        assert isinstance(result, list)
        assert len(result) == 0

    def test_register_agent_success(self, agents_service, mock_state_manager):
        """Test successfully registering a new agent."""
        agent_data = {
            "agent_id": "new_agent",
            "type": "sensor",
            "capabilities": ["vision"],
            "address": "192.168.1.200",
            "timestamp": "2023-01-01T12:00:00Z",
            "metadata": {"version": "1.0"},
        }

        # Initialize connected_agents as empty
        mock_state_manager.connected_agents = {}

        result = agents_service.register_agent(agent_data)

        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agent_id"] == "new_agent"
        assert result["message"] == "Agent registered successfully"
        # Check that agent was added to connected_agents
        assert "new_agent" in mock_state_manager.connected_agents
        assert mock_state_manager.connected_agents["new_agent"]["status"] == "connected"

    def test_register_agent_missing_id(self, agents_service, mock_state_manager):
        """Test registering agent without required agent_id."""
        agent_data = {"type": "sensor", "capabilities": ["vision"]}

        mock_state_manager.connected_agents = {}

        result = agents_service.register_agent(agent_data)

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "Agent ID required"

    def test_register_agent_no_state_manager(self, agents_service_no_state):
        """Test registering agent when no state manager exists."""
        agent_data = {"agent_id": "test_agent", "type": "sensor"}

        result = agents_service_no_state.register_agent(agent_data)

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "State manager not available"

    def test_unregister_agent_success(self, agents_service, mock_state_manager):
        """Test successfully unregistering an agent."""
        # Setup existing agent
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"}
        }

        result = agents_service.unregister_agent("agent1")

        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["message"] == "Agent unregistered successfully"
        # Check that agent was removed
        assert "agent1" not in mock_state_manager.connected_agents

    def test_unregister_agent_not_found(self, agents_service, mock_state_manager):
        """Test unregistering a non-existent agent."""
        mock_state_manager.connected_agents = {}

        result = agents_service.unregister_agent("nonexistent")

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "Agent not found"

    def test_unregister_agent_no_state_manager(self, agents_service_no_state):
        """Test unregistering agent when no state manager exists."""
        result = agents_service_no_state.unregister_agent("test_agent")

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "State manager not available"

    def test_update_agent_status_success(self, agents_service, mock_state_manager):
        """Test successfully updating agent status."""
        # Setup existing agent
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected", "metadata": {}}
        }

        metadata = {"timestamp": "2023-01-01T13:00:00Z", "extra": "data"}
        result = agents_service.update_agent_status("agent1", "active", metadata)

        assert result is True
        # Check that status was updated
        assert mock_state_manager.connected_agents["agent1"]["status"] == "active"
        assert (
            mock_state_manager.connected_agents["agent1"]["last_seen"]
            == "2023-01-01T13:00:00Z"
        )

    def test_update_agent_status_not_found(self, agents_service, mock_state_manager):
        """Test updating status of non-existent agent."""
        mock_state_manager.connected_agents = {}

        result = agents_service.update_agent_status("nonexistent", "active")

        assert result is False

    def test_update_agent_status_no_state_manager(self, agents_service_no_state):
        """Test updating agent status when no state manager exists."""
        result = agents_service_no_state.update_agent_status("test_agent", "active")

        assert result is False

    def test_get_agent_details_success(self, agents_service, mock_state_manager):
        """Test getting details for an existing agent."""
        # Setup existing agent
        mock_state_manager.connected_agents = {
            "agent1": {
                "type": "sensor",
                "status": "connected",
                "last_seen": "2023-01-01T12:00:00Z",
                "capabilities": ["vision"],
                "address": "192.168.1.100",
                "metadata": {"version": "1.0"},
            }
        }

        result = agents_service.get_agent_details("agent1")

        assert isinstance(result, dict)
        assert result["agent_id"] == "agent1"
        assert result["type"] == "sensor"
        assert result["status"] == "connected"
        assert result["capabilities"] == ["vision"]
        assert result["metadata"] == {"version": "1.0"}

    def test_get_agent_details_not_found(self, agents_service, mock_state_manager):
        """Test getting details for non-existent agent."""
        mock_state_manager.connected_agents = {}

        result = agents_service.get_agent_details("nonexistent")

        assert result is None

    def test_get_agent_details_no_state_manager(self, agents_service_no_state):
        """Test getting agent details when no state manager exists."""
        result = agents_service_no_state.get_agent_details("test_agent")

        assert result is None

    def test_send_message_to_agent_success(self, agents_service, mock_state_manager):
        """Test successfully sending message to an agent."""
        # Setup existing agent
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"}
        }

        message = {"command": "start_recording", "duration": 60}
        result = agents_service.send_message_to_agent("agent1", message)

        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agent_id"] == "agent1"
        assert "message_id" in result
        assert result["status"] == "sent"

    def test_send_message_to_agent_not_found(self, agents_service, mock_state_manager):
        """Test sending message to non-existent agent."""
        mock_state_manager.connected_agents = {}

        message = {"command": "test"}
        result = agents_service.send_message_to_agent("nonexistent", message)

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "Agent not found"

    def test_send_message_to_agent_no_state_manager(self, agents_service_no_state):
        """Test sending message when no state manager exists."""
        message = {"command": "test"}
        result = agents_service_no_state.send_message_to_agent("test_agent", message)

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "State manager not available"

    def test_broadcast_message_success(self, agents_service, mock_state_manager):
        """Test successfully broadcasting a message."""
        # Setup multiple agents
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"},
            "agent2": {"type": "actuator", "status": "connected"},
            "agent3": {"type": "sensor", "status": "disconnected"},
        }

        message = {"command": "system_status", "urgent": True}
        result = agents_service.broadcast_message(message)

        assert isinstance(result, dict)
        assert result["success"] is True
        assert "message_id" in result
        assert result["status"] == "sent"
        assert len(result["target_agents"]) == 3  # All agents

    def test_broadcast_message_with_filter(self, agents_service, mock_state_manager):
        """Test broadcasting with agent type filter."""
        # Setup multiple agents
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"},
            "agent2": {"type": "actuator", "status": "connected"},
            "agent3": {"type": "sensor", "status": "disconnected"},
        }

        message = {"command": "calibrate"}
        agent_filter = {"type": "sensor"}
        result = agents_service.broadcast_message(message, agent_filter)

        assert isinstance(result, dict)
        assert result["success"] is True
        assert len(result["target_agents"]) == 2  # Only sensor agents

    def test_broadcast_message_with_status_filter(
        self, agents_service, mock_state_manager
    ):
        """Test broadcasting with status filter."""
        # Setup multiple agents
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"},
            "agent2": {"type": "actuator", "status": "connected"},
            "agent3": {"type": "sensor", "status": "disconnected"},
        }

        message = {"command": "urgent_stop"}
        agent_filter = {"status": "connected"}
        result = agents_service.broadcast_message(message, agent_filter)

        assert isinstance(result, dict)
        assert result["success"] is True
        assert len(result["target_agents"]) == 2  # Only connected agents

    def test_broadcast_message_no_state_manager(self, agents_service_no_state):
        """Test broadcasting when no state manager exists."""
        message = {"command": "test"}
        result = agents_service_no_state.broadcast_message(message)

        assert isinstance(result, dict)
        assert result["success"] is False
        assert result["error"] == "State manager not available"

    def test_get_agent_statistics_success(self, agents_service, mock_state_manager):
        """Test getting agent statistics."""
        # Setup multiple agents
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"},
            "agent2": {"type": "actuator", "status": "connected"},
            "agent3": {"type": "sensor", "status": "disconnected"},
            "agent4": {"type": "sensor", "status": "connected"},
        }

        result = agents_service.get_agent_statistics()

        assert isinstance(result, dict)
        assert result["total_agents"] == 4
        assert result["agents_by_type"]["sensor"] == 3
        assert result["agents_by_type"]["actuator"] == 1
        assert result["agents_by_status"]["connected"] == 3
        assert result["agents_by_status"]["disconnected"] == 1

    def test_get_agent_statistics_empty(self, agents_service, mock_state_manager):
        """Test getting statistics when no agents exist."""
        mock_state_manager.connected_agents = {}

        result = agents_service.get_agent_statistics()

        assert isinstance(result, dict)
        assert result["total_agents"] == 0
        assert result["agents_by_type"] == {}
        assert result["agents_by_status"] == {}

    def test_get_agent_statistics_no_state_manager(self, agents_service_no_state):
        """Test getting statistics when no state manager exists."""
        result = agents_service_no_state.get_agent_statistics()

        assert isinstance(result, dict)
        assert len(result) == 0  # Empty dict

    def test_agent_filter_matching(self, agents_service):
        """Test agent filter functionality within broadcast_message."""
        # Test filtering functionality through the actual broadcast_message method
        # since _matches_filter is internal implementation detail

        # Setup agents with different types and statuses
        agents_service.state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"},
            "agent2": {"type": "actuator", "status": "disconnected"},
        }

        # Test type filter
        result = agents_service.broadcast_message(
            {"command": "test"}, {"type": "sensor"}
        )
        assert result["success"] is True
        assert len(result["target_agents"]) == 1
        assert "agent1" in result["target_agents"]

    def test_agent_filter_with_missing_attributes(self, agents_service):
        """Test filtering when agents have missing attributes."""
        # Setup agent with missing status attribute
        agents_service.state_manager.connected_agents = {
            "agent1": {"type": "sensor"}  # Missing status
        }

        # Filter should handle missing attributes gracefully
        result = agents_service.broadcast_message(
            {"command": "test"}, {"status": "connected"}
        )
        assert result["success"] is True
        # Agent without status won't match the filter
        assert len(result["target_agents"]) == 0

    def test_error_handling_in_methods(self, agents_service, mock_state_manager):
        """Test that methods handle exceptions gracefully."""
        # Set connected_agents to None to cause AttributeError
        mock_state_manager.connected_agents = None

        # Test that methods handle errors gracefully
        agents = agents_service.get_connected_agents()
        assert isinstance(agents, list)
        assert len(agents) == 0

        stats = agents_service.get_agent_statistics()
        assert isinstance(stats, dict)
        # When there's an error, it returns empty dict
        assert len(stats) == 0

    def test_message_validation(self, agents_service, mock_state_manager):
        """Test message handling with various message types."""
        # Setup an agent
        mock_state_manager.connected_agents = {
            "agent1": {"type": "sensor", "status": "connected"}
        }

        # Test with empty message - current implementation accepts any message
        result = agents_service.send_message_to_agent("agent1", {})
        assert (
            result["success"] is True
        )  # Implementation doesn't validate message content

        # Test with None message - implementation handles this
        result = agents_service.send_message_to_agent("agent1", None)
        assert (
            result["success"] is True
        )  # Implementation converts to string for logging

    def test_agent_lifecycle_workflow(self, agents_service, mock_state_manager):
        """Test complete agent lifecycle workflow."""
        # Initialize empty connected agents
        mock_state_manager.connected_agents = {}

        # Register agent with required agent_id
        agent_data = {
            "agent_id": "workflow_agent",  # Required field
            "type": "sensor",
            "capabilities": ["vision"],
        }

        register_result = agents_service.register_agent(agent_data)
        assert register_result["success"] is True

        # Update status
        status_result = agents_service.update_agent_status("workflow_agent", "active")
        assert status_result is True

        # Get details
        details = agents_service.get_agent_details("workflow_agent")
        assert details is not None
        assert details["agent_id"] == "workflow_agent"

        # Send message
        message_result = agents_service.send_message_to_agent(
            "workflow_agent", {"test": "message"}
        )
        assert message_result["success"] is True

        # Unregister
        unregister_result = agents_service.unregister_agent("workflow_agent")
        assert unregister_result["success"] is True

    def test_concurrent_agent_operations(self, agents_service, mock_state_manager):
        """Test concurrent agent operations."""
        # Initialize empty connected agents
        mock_state_manager.connected_agents = {}

        # Register multiple agents with required agent_id
        agent1_data = {"agent_id": "concurrent1", "type": "sensor"}
        agent2_data = {"agent_id": "concurrent2", "type": "actuator"}

        result1 = agents_service.register_agent(agent1_data)
        result2 = agents_service.register_agent(agent2_data)

        assert result1["success"] is True
        assert result2["success"] is True

        # Get statistics to verify both are registered
        stats = agents_service.get_agent_statistics()
        assert stats["total_agents"] == 2

        # Broadcast to all
        broadcast_result = agents_service.broadcast_message({"command": "ping"})
        assert broadcast_result["success"] is True
        assert len(broadcast_result["target_agents"]) == 2
