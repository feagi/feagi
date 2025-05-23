"""Tests for the AgentsService class."""

import pytest
from unittest.mock import MagicMock

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
            "last_seen": "2023-01-01T10:00:00"
        },
        "agent2": {
            "id": "agent2", 
            "type": "motor",
            "status": "connected",
            "last_seen": "2023-01-01T10:05:00"
        }
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

    def test_get_connected_agents_with_state_manager(self, agents_service, mock_state_manager):
        """Test getting connected agents with state manager."""
        result = agents_service.get_connected_agents()
        
        assert isinstance(result, list)
        assert len(result) == 2
        
        # Check agent data
        agent_ids = [agent["id"] for agent in result]
        assert "agent1" in agent_ids
        assert "agent2" in agent_ids

    def test_get_connected_agents_without_state_manager(self, agents_service_no_state):
        """Test getting connected agents without state manager."""
        result = agents_service_no_state.get_connected_agents()
        
        assert isinstance(result, list)
        assert len(result) == 0

    def test_get_connected_agents_empty(self, agents_service, mock_state_manager):
        """Test getting connected agents when none are connected."""
        mock_state_manager.connected_agents = {}
        
        result = agents_service.get_connected_agents()
        
        assert isinstance(result, list)
        assert len(result) == 0

    def test_register_agent_success(self, agents_service, mock_state_manager):
        """Test successfully registering a new agent."""
        agent_data = {
            "id": "agent3",
            "type": "vision",
            "capabilities": ["image_processing"],
            "metadata": {"version": "1.0"}
        }
        
        result = agents_service.register_agent(agent_data)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agent_id"] == "agent3"

    def test_register_agent_duplicate_id(self, agents_service, mock_state_manager):
        """Test registering an agent with duplicate ID."""
        agent_data = {
            "id": "agent1",  # Already exists
            "type": "duplicate"
        }
        
        result = agents_service.register_agent(agent_data)
        
        assert isinstance(result, dict)
        assert result["success"] is False
        assert "already registered" in result["error"]

    def test_register_agent_invalid_data(self, agents_service, mock_state_manager):
        """Test registering an agent with invalid data."""
        agent_data = {
            # Missing required 'id' field
            "type": "invalid"
        }
        
        result = agents_service.register_agent(agent_data)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_register_agent_without_state_manager(self, agents_service_no_state):
        """Test registering an agent without state manager."""
        agent_data = {"id": "test_agent", "type": "test"}
        
        result = agents_service_no_state.register_agent(agent_data)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_unregister_agent_success(self, agents_service, mock_state_manager):
        """Test successfully unregistering an agent."""
        result = agents_service.unregister_agent("agent1")
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agent_id"] == "agent1"

    def test_unregister_agent_nonexistent(self, agents_service, mock_state_manager):
        """Test unregistering a nonexistent agent."""
        result = agents_service.unregister_agent("nonexistent")
        
        assert isinstance(result, dict)
        assert result["success"] is False
        assert "not found" in result["error"]

    def test_unregister_agent_without_state_manager(self, agents_service_no_state):
        """Test unregistering an agent without state manager."""
        result = agents_service_no_state.unregister_agent("agent1")
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_update_agent_status_success(self, agents_service, mock_state_manager):
        """Test successfully updating agent status."""
        metadata = {"last_activity": "processing"}
        
        result = agents_service.update_agent_status("agent1", "busy", metadata)
        
        assert result is True

    def test_update_agent_status_nonexistent_agent(self, agents_service, mock_state_manager):
        """Test updating status of nonexistent agent."""
        result = agents_service.update_agent_status("nonexistent", "active")
        
        assert result is False

    def test_update_agent_status_without_state_manager(self, agents_service_no_state):
        """Test updating agent status without state manager."""
        result = agents_service_no_state.update_agent_status("agent1", "active")
        
        assert result is False

    def test_get_agent_details_success(self, agents_service, mock_state_manager):
        """Test successfully getting agent details."""
        result = agents_service.get_agent_details("agent1")
        
        assert isinstance(result, dict)
        assert result["id"] == "agent1"
        assert result["type"] == "sensor"
        assert result["status"] == "active"

    def test_get_agent_details_nonexistent(self, agents_service, mock_state_manager):
        """Test getting details of nonexistent agent."""
        result = agents_service.get_agent_details("nonexistent")
        
        assert result is None

    def test_get_agent_details_without_state_manager(self, agents_service_no_state):
        """Test getting agent details without state manager."""
        result = agents_service_no_state.get_agent_details("agent1")
        
        assert result is None

    def test_send_message_to_agent_success(self, agents_service, mock_state_manager):
        """Test successfully sending message to agent."""
        message = {"command": "start_recording", "duration": 30}
        
        result = agents_service.send_message_to_agent("agent1", message)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agent_id"] == "agent1"

    def test_send_message_to_agent_nonexistent(self, agents_service, mock_state_manager):
        """Test sending message to nonexistent agent."""
        message = {"command": "test"}
        
        result = agents_service.send_message_to_agent("nonexistent", message)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_send_message_to_agent_without_state_manager(self, agents_service_no_state):
        """Test sending message to agent without state manager."""
        message = {"command": "test"}
        
        result = agents_service_no_state.send_message_to_agent("agent1", message)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_broadcast_message_success(self, agents_service, mock_state_manager):
        """Test successfully broadcasting message to all agents."""
        message = {"command": "shutdown", "reason": "maintenance"}
        
        result = agents_service.broadcast_message(message)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agents_notified"] == 2

    def test_broadcast_message_with_filter(self, agents_service, mock_state_manager):
        """Test broadcasting message with agent filter."""
        message = {"command": "calibrate"}
        agent_filter = {"type": "sensor"}
        
        result = agents_service.broadcast_message(message, agent_filter)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        # Should filter to only sensor agents
        assert result["agents_notified"] == 1

    def test_broadcast_message_empty_agents(self, agents_service, mock_state_manager):
        """Test broadcasting message when no agents are connected."""
        mock_state_manager.connected_agents = {}
        message = {"command": "test"}
        
        result = agents_service.broadcast_message(message)
        
        assert isinstance(result, dict)
        assert result["success"] is True
        assert result["agents_notified"] == 0

    def test_broadcast_message_without_state_manager(self, agents_service_no_state):
        """Test broadcasting message without state manager."""
        message = {"command": "test"}
        
        result = agents_service_no_state.broadcast_message(message)
        
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_get_agent_statistics_success(self, agents_service, mock_state_manager):
        """Test successfully getting agent statistics."""
        result = agents_service.get_agent_statistics()
        
        assert isinstance(result, dict)
        assert result["total_agents"] == 2
        assert result["active_agents"] == 1
        assert result["connected_agents"] == 1
        assert "agents_by_type" in result
        assert result["agents_by_type"]["sensor"] == 1
        assert result["agents_by_type"]["motor"] == 1

    def test_get_agent_statistics_empty(self, agents_service, mock_state_manager):
        """Test getting agent statistics when no agents are connected."""
        mock_state_manager.connected_agents = {}
        
        result = agents_service.get_agent_statistics()
        
        assert isinstance(result, dict)
        assert result["total_agents"] == 0
        assert result["active_agents"] == 0
        assert result["connected_agents"] == 0

    def test_get_agent_statistics_without_state_manager(self, agents_service_no_state):
        """Test getting agent statistics without state manager."""
        result = agents_service_no_state.get_agent_statistics()
        
        assert isinstance(result, dict)
        assert result["total_agents"] == 0

    def test_agent_filter_matching(self, agents_service):
        """Test agent filter matching logic."""
        agent = {"id": "test", "type": "sensor", "status": "active"}
        
        # Test exact match
        assert agents_service._matches_filter(agent, {"type": "sensor"}) is True
        assert agents_service._matches_filter(agent, {"type": "motor"}) is False
        
        # Test multiple criteria
        assert agents_service._matches_filter(agent, {"type": "sensor", "status": "active"}) is True
        assert agents_service._matches_filter(agent, {"type": "sensor", "status": "inactive"}) is False
        
        # Test empty filter (should match all)
        assert agents_service._matches_filter(agent, {}) is True

    def test_agent_filter_with_missing_attributes(self, agents_service):
        """Test agent filter with missing agent attributes."""
        agent = {"id": "test", "type": "sensor"}  # Missing status
        
        # Should not crash when filtering on missing attributes
        result = agents_service._matches_filter(agent, {"status": "active"})
        assert result is False

    def test_error_handling_in_methods(self, agents_service, mock_state_manager):
        """Test that methods handle exceptions gracefully."""
        # Make state manager raise exceptions
        mock_state_manager.connected_agents = None  # This will cause errors
        
        # Test that methods don't crash with exceptions
        result = agents_service.get_connected_agents()
        assert isinstance(result, list)
        assert len(result) == 0
        
        stats = agents_service.get_agent_statistics()
        assert isinstance(stats, dict)
        assert stats["total_agents"] == 0

    def test_message_validation(self, agents_service, mock_state_manager):
        """Test message validation in send methods."""
        # Test with empty message
        result = agents_service.send_message_to_agent("agent1", {})
        assert isinstance(result, dict)
        assert result["success"] is True  # Empty messages should be allowed
        
        # Test with None message
        result = agents_service.send_message_to_agent("agent1", None)
        assert isinstance(result, dict)
        assert result["success"] is False

    def test_agent_lifecycle_workflow(self, agents_service, mock_state_manager):
        """Test complete agent lifecycle workflow."""
        # Register new agent
        agent_data = {"id": "workflow_agent", "type": "test"}
        register_result = agents_service.register_agent(agent_data)
        assert register_result["success"] is True
        
        # Update status
        update_result = agents_service.update_agent_status("workflow_agent", "processing")
        assert update_result is True
        
        # Send message
        message_result = agents_service.send_message_to_agent("workflow_agent", {"cmd": "test"})
        assert message_result["success"] is True
        
        # Get details
        details = agents_service.get_agent_details("workflow_agent")
        assert details is not None
        assert details["id"] == "workflow_agent"
        
        # Unregister
        unregister_result = agents_service.unregister_agent("workflow_agent")
        assert unregister_result["success"] is True

    def test_concurrent_agent_operations(self, agents_service, mock_state_manager):
        """Test handling of concurrent agent operations."""
        # Simulate concurrent registration of same agent
        agent_data = {"id": "concurrent_agent", "type": "test"}
        
        result1 = agents_service.register_agent(agent_data)
        result2 = agents_service.register_agent(agent_data)  # Duplicate
        
        assert result1["success"] is True
        assert result2["success"] is False  # Should reject duplicate 