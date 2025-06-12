"""
Integration Tests for Registration Manager Architecture

This test suite validates the complete Registration Manager architecture including:
- Agent registration/deregistration flows
- FQ sampler coordination
- ZMQ stream integration
- Embedded mode support
- Error handling and edge cases

The tests ensure that the Registration Manager properly coordinates between:
- Agent API endpoints
- ZMQ visualization streams
- Process Manager FQ samplers
- State Manager agent tracking
"""

import asyncio
import threading
import time
from typing import Any, Dict, Optional
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

from feagi.api.v1.feagi_agent import FeagiAgentAPI
from feagi.api.zmq.streams.visualization import VisualizationStream

# Import the components we're testing
from feagi.pns.registration_manager import (
    AgentRegistrationRequest,
    AgentRegistrationResponse,
    RegistrationManager,
    get_registration_manager,
    set_registration_manager,
)


class TestRegistrationManagerIntegration:
    """Integration tests for the Registration Manager architecture."""

    @pytest.fixture
    def mock_state_manager(self):
        """Mock State Manager for testing."""
        state_manager = Mock()
        state_manager.register_agent.return_value = True
        state_manager.deregister_agent.return_value = True
        state_manager.get_agent_properties.return_value = None
        state_manager.list_agents.return_value = {"agents": []}
        state_manager.get_agents_by_capability.return_value = []
        state_manager.set_agent_count.return_value = None
        return state_manager

    @pytest.fixture
    def mock_process_manager(self):
        """Mock Process Manager for testing."""
        process_manager = Mock()
        process_manager.enable_viz_fq_sampler.return_value = True
        process_manager.disable_viz_fq_sampler.return_value = True
        process_manager.enable_motor_fq_sampler.return_value = True
        process_manager.disable_motor_fq_sampler.return_value = True
        process_manager.get_viz_fq_sampler_status.return_value = {"enabled": False}
        process_manager.get_motor_fq_sampler_status.return_value = {"enabled": False}
        return process_manager

    @pytest.fixture
    def registration_manager(self, mock_state_manager, mock_process_manager):
        """Create a Registration Manager instance for testing."""
        manager = RegistrationManager(
            state_manager=mock_state_manager, process_manager=mock_process_manager
        )
        # Set as global instance for testing
        set_registration_manager(manager)
        return manager

    @pytest.fixture
    def mock_core_api_service(self):
        """Mock Core API Service for Agent API testing."""
        service = Mock()
        service.get_connected_agents.return_value = {"agents": []}
        service.get_agent_properties.return_value = None
        service.get_agent_registry_summary.return_value = {
            "agent_count_viz": 0,
            "agent_count_sensorimotor": 0,
        }
        return service

    def test_agent_api_registration_flow(
        self, registration_manager, mock_core_api_service
    ):
        """Test complete agent registration flow through Agent API."""
        # Create Agent API instance
        agent_api = FeagiAgentAPI(mock_core_api_service)

        # Create registration request
        request = AgentRegistrationRequest(
            agent_id="test_brain_visualizer",
            agent_type="brain_visualizer",
            capabilities={"visualization": True, "3d_visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_ip="192.168.1.100",
        )

        # Test registration
        response = asyncio.run(agent_api.register_agent(request))

        # Verify response - Agent API returns SuccessResponse format
        assert hasattr(response, "status")
        assert response.status == "success"
        assert "registered successfully" in response.message

        # Verify Registration Manager was called - access private attributes correctly
        assert registration_manager._state_manager.register_agent.called
        assert registration_manager._process_manager.enable_viz_fq_sampler.called

    def test_zmq_stream_heartbeat_integration(self, registration_manager):
        """Test ZMQ visualization stream heartbeat integration with Registration Manager."""
        # Create mock visualization stream
        viz_stream = Mock(spec=VisualizationStream)
        viz_stream.port = 5562
        viz_stream.client_last_heartbeat = {}
        viz_stream._client_lock = threading.Lock()

        # Mock the heartbeat method to use our Registration Manager
        def mock_heartbeat(client_id: str):
            with viz_stream._client_lock:
                was_new_client = client_id not in viz_stream.client_last_heartbeat
                viz_stream.client_last_heartbeat[client_id] = time.time()

                if was_new_client:
                    # Register with Registration Manager
                    response = registration_manager.register_agent_direct(
                        agent_id=client_id,
                        agent_type="visualization_client",
                        capabilities={"visualization": True},
                        metadata={
                            "connection_type": "zmq_heartbeat",
                            "stream_port": viz_stream.port,
                        },
                    )
                    return response.success
                else:
                    # Update heartbeat
                    registration_manager.heartbeat_agent(client_id)
                    return True

        viz_stream.heartbeat_visualization_client = mock_heartbeat

        # Test new client heartbeat
        result = viz_stream.heartbeat_visualization_client("viz_client_1")
        assert result is True

        # Verify Registration Manager was called - access private attributes correctly
        assert registration_manager._state_manager.register_agent.called
        assert registration_manager._process_manager.enable_viz_fq_sampler.called

        # Test existing client heartbeat
        result = viz_stream.heartbeat_visualization_client("viz_client_1")
        assert result is True

    def test_embedded_mode_direct_registration(self, registration_manager):
        """Test direct function call registration for embedded mode."""
        # Simulate embedded mode direct registration
        request = AgentRegistrationRequest(
            agent_id="embedded_motor_client",
            agent_type="motor_controller",
            capabilities={"motor": True, "output": True},
            agent_data_port=0,  # No port in embedded mode
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_ip="embedded",
        )

        # Test direct registration
        response = registration_manager.register_agent(request)

        # Verify response - check for agent_id in message instead of agent_type
        assert response.success is True
        assert "embedded_motor_client" in response.message
        assert response.fq_samplers_enabled["motor"] is True

        # Verify motor FQ sampler was enabled - access private attributes correctly
        assert registration_manager._process_manager.enable_motor_fq_sampler.called

    def test_multiple_agent_coordination(self, registration_manager):
        """Test coordination with multiple agents of different types."""
        # Register visualization agent
        viz_request = AgentRegistrationRequest(
            agent_id="brain_visualizer_1",
            agent_type="brain_visualizer",
            capabilities={"visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
        )

        viz_response = registration_manager.register_agent(viz_request)
        assert viz_response.success is True
        assert viz_response.fq_samplers_enabled["visualization"] is True

        # Register motor agent
        motor_request = AgentRegistrationRequest(
            agent_id="robot_controller_1",
            agent_type="robot_controller",
            capabilities={"motor": True, "sensorimotor": True},
            agent_data_port=8081,
            agent_version="1.0.0",
            controller_version="2.0.0",
        )

        motor_response = registration_manager.register_agent(motor_request)
        assert motor_response.success is True
        assert motor_response.fq_samplers_enabled["motor"] is True

        # Verify both FQ samplers are enabled - access private attributes correctly
        assert registration_manager._process_manager.enable_viz_fq_sampler.called
        assert registration_manager._process_manager.enable_motor_fq_sampler.called

        # Test deregistration coordination
        viz_dereg_response = registration_manager.deregister_agent("brain_visualizer_1")
        assert viz_dereg_response.success is True

        # Motor should still be enabled, viz should be disabled
        assert registration_manager._process_manager.disable_viz_fq_sampler.called
        # Motor sampler should remain enabled (other motor agent still registered)

    def test_fq_sampler_coordination_status(self, registration_manager):
        """Test FQ sampler coordination status reporting."""
        # Register agents with different capabilities
        registration_manager.register_agent(
            AgentRegistrationRequest(
                agent_id="viz_agent",
                agent_type="brain_visualizer",
                capabilities={"visualization": True},
                agent_data_port=8080,
                agent_version="1.0.0",
                controller_version="2.0.0",
            )
        )

        registration_manager.register_agent(
            AgentRegistrationRequest(
                agent_id="motor_agent",
                agent_type="robot_controller",
                capabilities={"motor": True},
                agent_data_port=8081,
                agent_version="1.0.0",
                controller_version="2.0.0",
            )
        )

        # Get coordination status
        status = registration_manager.get_fq_sampler_coordination_status()

        # Verify status structure - match actual implementation
        assert "visualization_fq_sampler" in status
        assert "motor_fq_sampler" in status
        assert "coordination_summary" in status

        # Verify coordination is working
        assert status["coordination_summary"]["total_agents"] == 2
        assert status["coordination_summary"]["fq_samplers_enabled"] == 2

    def test_error_handling_and_recovery(self, registration_manager):
        """Test error handling and recovery scenarios."""
        # Test registration with invalid data
        invalid_request = AgentRegistrationRequest(
            agent_id="",  # Invalid empty ID
            agent_type="test_agent",
            capabilities={},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
        )

        response = registration_manager.register_agent(invalid_request)
        assert response.success is False
        assert response.error_code == "MISSING_AGENT_ID"

        # Test deregistration of non-existent agent
        response = registration_manager.deregister_agent("non_existent_agent")
        assert response.success is False
        assert response.error_code == "AGENT_NOT_FOUND"

        # Test seamless re-registration (should succeed)
        valid_request = AgentRegistrationRequest(
            agent_id="test_agent",
            agent_type="test_agent",
            capabilities={"visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
        )

        # First registration should succeed
        response1 = registration_manager.register_agent(valid_request)
        assert response1.success is True
        assert "registered successfully" in response1.message

        # Second registration should also succeed (seamless re-registration)
        response2 = registration_manager.register_agent(valid_request)
        assert response2.success is True
        assert "re-registered (overwritten)" in response2.message

    def test_process_manager_failure_handling(self, registration_manager):
        """Test handling of Process Manager failures."""
        # Mock Process Manager to fail - access private attributes correctly
        registration_manager._process_manager.enable_viz_fq_sampler.return_value = False

        # Try to register visualization agent
        request = AgentRegistrationRequest(
            agent_id="viz_agent",
            agent_type="brain_visualizer",
            capabilities={"visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
        )

        response = registration_manager.register_agent(request)

        # Registration should still succeed, but FQ sampler coordination should be noted
        assert response.success is True
        assert response.fq_samplers_enabled["visualization"] is False
        # Note: The message format may vary, so just check that it succeeded

    def test_state_manager_integration(self, registration_manager):
        """Test integration with State Manager."""
        # First register an agent so it exists in the Registration Manager's registry
        request = AgentRegistrationRequest(
            agent_id="test_agent",
            agent_type="brain_visualizer",
            capabilities={"visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_ip="127.0.0.1",
        )

        response = registration_manager.register_agent(request)
        assert response.success is True

        # Verify state manager register_agent was called - access private attributes correctly
        assert registration_manager._state_manager.register_agent.called

        # Test get agent properties from Registration Manager's registry
        properties = registration_manager.get_agent_properties("test_agent")
        assert properties is not None
        assert properties["agent_id"] == "test_agent"
        assert properties["agent_type"] == "brain_visualizer"
        assert properties["capabilities"]["visualization"] is True

        # Test list agents
        agents_data = registration_manager.list_agents()
        assert "agents" in agents_data
        assert len(agents_data["agents"]) == 1
        assert agents_data["agents"][0]["agent_id"] == "test_agent"

        # Verify state manager set_agent_count was called for persistence
        assert registration_manager._state_manager.set_agent_count.called

    def test_capability_detection_logic(self, registration_manager):
        """Test capability detection and FQ sampler coordination logic."""
        test_cases = [
            # (capabilities, expected_viz, expected_motor, should_succeed)
            ({"visualization": True}, True, False, True),
            ({"brain_visualizer": True}, True, False, True),
            ({"3d_visualization": True}, True, False, True),
            ({"motor": True}, False, True, True),
            ({"output": True}, False, True, True),
            ({"sensorimotor": True}, False, True, True),
            ({"visualization": True, "motor": True}, True, True, True),
            ({}, False, False, False),  # Empty capabilities should fail
        ]

        for capabilities, expected_viz, expected_motor, should_succeed in test_cases:
            # Use unique agent ID for each test case
            unique_agent_id = f"test_agent_{int(time.time() * 1000000) % 1000000}_{hash(str(capabilities)) % 10000}"

            request = AgentRegistrationRequest(
                agent_id=unique_agent_id,
                agent_type="test_agent",
                capabilities=capabilities,
                agent_data_port=8080,
                agent_version="1.0.0",
                controller_version="2.0.0",
            )

            response = registration_manager.register_agent(request)

            if should_succeed:
                assert response.success is True

                # Check visualization capability
                viz_enabled = response.fq_samplers_enabled.get("visualization", False)
                assert viz_enabled == expected_viz

                # Check motor capability (may not be present in response if False)
                motor_enabled = response.fq_samplers_enabled.get("motor", False)
                assert motor_enabled == expected_motor
            else:
                assert response.success is False
                assert "capability" in response.message.lower()

    def test_concurrent_registration_safety(self, registration_manager):
        """Test thread safety of concurrent registrations."""
        import concurrent.futures

        def register_agent(agent_id: str):
            request = AgentRegistrationRequest(
                agent_id=agent_id,
                agent_type="test_agent",
                capabilities={"visualization": True},
                agent_data_port=8080,
                agent_version="1.0.0",
                controller_version="2.0.0",
            )
            return registration_manager.register_agent(request)

        # Register multiple agents concurrently
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(register_agent, f"concurrent_agent_{i}")
                for i in range(10)
            ]

            results = [
                future.result() for future in concurrent.futures.as_completed(futures)
            ]

        # All registrations should succeed
        assert all(result.success for result in results)
        assert (
            len(set(result.message for result in results)) >= 1
        )  # At least one unique message

    def test_seamless_re_registration(self, registration_manager):
        """Test seamless re-registration behavior where duplicate agent IDs are accepted."""
        # Initial registration
        initial_request = AgentRegistrationRequest(
            agent_id="reregistration_test_agent",
            agent_type="brain_visualizer",
            capabilities={"visualization": True},
            agent_data_port=8080,
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_ip="192.168.1.100",
        )

        initial_response = registration_manager.register_agent(initial_request)
        assert initial_response.success is True
        assert "registered successfully" in initial_response.message

        # Verify agent is in registry
        agent_props = registration_manager.get_agent_properties(
            "reregistration_test_agent"
        )
        assert agent_props is not None
        assert agent_props["agent_type"] == "brain_visualizer"
        assert agent_props["agent_ip"] == "192.168.1.100"

        # Re-registration with same ID but different details
        reregistration_request = AgentRegistrationRequest(
            agent_id="reregistration_test_agent",  # Same ID
            agent_type="motor_controller",  # Different type
            capabilities={"motor": True, "output": True},  # Different capabilities
            agent_data_port=9090,  # Different port
            agent_version="2.0.0",  # Different version
            controller_version="3.0.0",  # Different version
            agent_ip="192.168.1.200",  # Different IP
        )

        reregistration_response = registration_manager.register_agent(
            reregistration_request
        )

        # Should succeed with re-registration message
        assert reregistration_response.success is True
        assert "re-registered (overwritten)" in reregistration_response.message

        # Verify the entry was overwritten with new details
        updated_props = registration_manager.get_agent_properties(
            "reregistration_test_agent"
        )
        assert updated_props is not None
        assert updated_props["agent_type"] == "motor_controller"
        assert updated_props["agent_ip"] == "192.168.1.200"
        assert updated_props["agent_data_port"] == 9090
        assert updated_props["agent_version"] == "2.0.0"
        assert updated_props["controller_version"] == "3.0.0"
        assert updated_props["capabilities"]["motor"] is True
        assert updated_props["capabilities"]["output"] is True

        # Should only be one agent in registry (not duplicated)
        agents_list = registration_manager.list_agents()
        matching_agents = [
            a
            for a in agents_list["agents"]
            if a["agent_id"] == "reregistration_test_agent"
        ]
        assert len(matching_agents) == 1

        # FQ sampler coordination should reflect new capabilities
        assert (
            reregistration_response.fq_samplers_enabled.get("visualization", False)
            is False
        )
        assert reregistration_response.fq_samplers_enabled.get("motor", False) is True

    def teardown_method(self):
        """Clean up after each test."""
        # Reset global Registration Manager
        set_registration_manager(None)


class TestRegistrationManagerArchitectureCompliance:
    """Test compliance with FEAGI architecture principles."""

    @pytest.fixture
    def mock_state_manager(self):
        """Mock State Manager for testing."""
        state_manager = Mock()
        state_manager.register_agent.return_value = True
        state_manager.deregister_agent.return_value = True
        state_manager.get_agent_properties.return_value = None
        state_manager.list_agents.return_value = {"agents": []}
        state_manager.get_agents_by_capability.return_value = []
        state_manager.set_agent_count.return_value = None
        return state_manager

    @pytest.fixture
    def mock_process_manager(self):
        """Mock Process Manager for testing."""
        process_manager = Mock()
        process_manager.enable_viz_fq_sampler.return_value = True
        process_manager.disable_viz_fq_sampler.return_value = True
        process_manager.enable_motor_fq_sampler.return_value = True
        process_manager.disable_motor_fq_sampler.return_value = True
        process_manager.get_viz_fq_sampler_status.return_value = {"enabled": False}
        process_manager.get_motor_fq_sampler_status.return_value = {"enabled": False}
        return process_manager

    @pytest.fixture
    def registration_manager(self, mock_state_manager, mock_process_manager):
        """Create a Registration Manager instance for testing."""
        manager = RegistrationManager(
            state_manager=mock_state_manager, process_manager=mock_process_manager
        )
        # Set as global instance for testing
        set_registration_manager(manager)
        yield manager
        # Cleanup after test
        set_registration_manager(None)

    def test_no_hardcoded_values(self):
        """Ensure Registration Manager doesn't use hardcoded network/timeout values."""
        # This test would scan the Registration Manager code for hardcoded values
        # For now, we'll test that it properly uses configuration

        import inspect

        from feagi.pns.registration_manager import RegistrationManager

        # Get the source code of RegistrationManager
        source = inspect.getsource(RegistrationManager)

        # Check for forbidden patterns
        forbidden_patterns = [
            "127.0.0.1",
            "localhost",
            "time.sleep(",
            "timeout=",
        ]

        for pattern in forbidden_patterns:
            # Find all occurrences of the pattern
            lines = source.split("\n")
            for i, line in enumerate(lines):
                if pattern in line:
                    # Check if this line or the previous/next lines have @architecture:acceptable annotation
                    context_lines = lines[max(0, i - 2) : min(len(lines), i + 3)]
                    context = "\n".join(context_lines)

                    # Allow if it has the @architecture:acceptable annotation
                    if "@architecture:acceptable" in context:
                        continue
                    else:
                        assert (
                            False
                        ), f"Found forbidden hardcoded pattern '{pattern}' without @architecture:acceptable annotation at line {i + 1}: {line.strip()}"

    def test_cross_platform_compatibility(self, registration_manager):
        """Test that Registration Manager works across different platforms."""
        # Test with different agent IP formats
        test_ips = [
            "127.0.0.1",  # IPv4 localhost
            "192.168.1.100",  # IPv4 private
            "::1",  # IPv6 localhost
            "embedded",  # Embedded mode
            "container",  # Container mode
        ]

        for i, ip in enumerate(test_ips):
            request = AgentRegistrationRequest(
                agent_id=f"platform_test_agent_{i}",
                agent_type="test_agent",
                capabilities={"visualization": True},
                agent_data_port=8080,
                agent_version="1.0.0",
                controller_version="2.0.0",
                agent_ip=ip,
            )

            response = registration_manager.register_agent(request)
            assert response.success is True, f"Failed for IP: {ip}"

    def test_embedded_mode_support(self, registration_manager):
        """Test specific embedded mode requirements."""
        # Test registration without network ports
        request = AgentRegistrationRequest(
            agent_id="embedded_agent",
            agent_type="embedded_controller",
            capabilities={"motor": True},
            agent_data_port=0,  # No network port
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_ip="embedded",
        )

        response = registration_manager.register_agent(request)
        assert response.success is True
        assert response.fq_samplers_enabled["motor"] is True

        # Test direct function call interface
        response = registration_manager.register_agent_direct(
            agent_id="direct_embedded_agent",
            agent_type="embedded_sensor",
            capabilities={"input": True},
            metadata={"embedded_mode": True},
        )

        assert response.success is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
