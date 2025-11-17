"""
Test suite to diagnose why sensory polling fails to find registered agents.

This test systematically validates:
1. Agent registration stores capability rates correctly
2. Capability rate manager can retrieve registered agents
3. Sensory polling loop can find agents with sensory capabilities
4. End-to-end flow from registration to polling
"""
import pytest
import time
from unittest.mock import Mock, MagicMock, patch

from feagi.core.capability_rate_manager import CapabilityRateManager, get_capability_rate_manager
from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec
from feagi.pns.registration_manager import RegistrationManager
from feagi.api.v1.schemas import AgentRegistrationRequest


class TestSensoryRegistrationFlow:
    """Test the complete flow from agent registration to sensory polling."""
    
    def test_capability_rate_manager_stores_agent(self):
        """Test that capability rate manager stores agent registration correctly."""
        manager = CapabilityRateManager(feagi_global_rate_hz=30.0)
        
        specs = [
            CapabilityRateSpec(
                capability_type=CapabilityType.SENSORY,
                requested_rate_hz=29.97,
                required=True
            )
        ]
        
        approved, rejected = manager.register_agent_capabilities("test-agent", specs)
        
        assert len(approved) == 1, "Should approve 1 capability"
        assert len(rejected) == 0, "Should not reject any capabilities"
        assert approved[0].capability_type == CapabilityType.SENSORY
        assert approved[0].approved_rate_hz == 29.97
        
        # Verify agent is in registry
        assert "test-agent" in manager._agent_registries
        print("✅ Capability rate manager stores agent correctly")
    
    def test_capability_rate_manager_returns_agents_for_polling(self):
        """Test that capability rate manager returns agents for polling."""
        manager = CapabilityRateManager(feagi_global_rate_hz=30.0)
        
        specs = [
            CapabilityRateSpec(
                capability_type=CapabilityType.SENSORY,
                requested_rate_hz=29.97,
                required=True
            )
        ]
        
        manager.register_agent_capabilities("test-agent", specs)
        
        # Get agents for polling
        current_time_ns = time.time_ns()
        agents = manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns
        )
        
        assert len(agents) > 0, "Should return at least 1 agent for polling"
        assert agents[0].approved_rate_hz == 29.97
        print(f"✅ Capability rate manager returns {len(agents)} agents for polling")
    
    def test_registration_manager_stores_in_capability_manager(self):
        """Test that registration manager stores capabilities in capability rate manager."""
        # Mock dependencies
        mock_state_manager = Mock()
        mock_process_manager = Mock()
        mock_process_manager.create_fq_sampler.return_value = True
        
        reg_manager = RegistrationManager(
            state_manager=mock_state_manager,
            process_manager=mock_process_manager
        )
        
        # Create registration request
        request = AgentRegistrationRequest(
            agent_id="video-agent-1",
            agent_type="external",
            capabilities={
                "video": True,
                "sensory": {"rate_hz": 29.97},
                "motor": {"enabled": True}
            },
            agent_version="1.0.0",
            controller_version="2.0.0",
            agent_data_port=0,
            agent_ip="127.0.0.1"
        )
        
        # Register agent
        response = reg_manager.register_agent(request)
        
        assert response.success, f"Registration should succeed: {response.message}"
        
        # Check if capability rate manager has the agent
        cap_manager = get_capability_rate_manager()
        assert "video-agent-1" in cap_manager._agent_registries, \
            "Agent should be in capability rate manager registry"
        
        registry = cap_manager._agent_registries["video-agent-1"]
        sensory_config = registry.get_capability_rate(CapabilityType.SENSORY)
        
        assert sensory_config is not None, "Agent should have sensory capability"
        assert sensory_config.approved_rate_hz == 29.97, \
            f"Expected 29.97 Hz, got {sensory_config.approved_rate_hz}"
        
        print(f"✅ Registration manager stores agent in capability manager at {sensory_config.approved_rate_hz} Hz")
    
    def test_sensory_polling_finds_registered_agent(self):
        """Test that sensory polling can find a registered agent."""
        cap_manager = get_capability_rate_manager()
        
        # Register an agent
        specs = [
            CapabilityRateSpec(
                capability_type=CapabilityType.SENSORY,
                requested_rate_hz=29.97,
                required=True
            )
        ]
        cap_manager.register_agent_capabilities("test-polling-agent", specs)
        
        # Simulate sensory polling loop
        current_time_ns = time.time_ns()
        agents_to_poll = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns
        )
        
        assert len(agents_to_poll) > 0, "Sensory polling should find registered agent"
        
        # Check if our agent is in the list
        agent_found = any(
            rate_config.approved_rate_hz == 29.97 
            for rate_config in agents_to_poll
        )
        assert agent_found, "Should find agent with 29.97 Hz rate"
        
        print(f"✅ Sensory polling finds {len(agents_to_poll)} registered agents")
    
    def test_rate_limiting_prevents_immediate_repolling(self):
        """Test that rate limiting prevents polling the same agent too quickly."""
        cap_manager = get_capability_rate_manager()
        
        specs = [
            CapabilityRateSpec(
                capability_type=CapabilityType.SENSORY,
                requested_rate_hz=10.0,  # Poll every 100ms
                required=True
            )
        ]
        cap_manager.register_agent_capabilities("rate-limited-agent", specs)
        
        current_time_ns = time.time_ns()
        
        # First poll should succeed
        agents = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns
        )
        assert len(agents) > 0, "First poll should find agent"
        
        # Immediate second poll should be rate-limited
        agents = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns + 1_000_000  # +1ms
        )
        assert len(agents) == 0, "Second poll should be rate-limited (too soon)"
        
        # Poll after interval should succeed
        agents = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns + 150_000_000  # +150ms (> 100ms interval)
        )
        assert len(agents) > 0, "Poll after interval should find agent again"
        
        print("✅ Rate limiting works correctly")
    
    def test_multiple_agents_with_different_rates(self):
        """Test that multiple agents with different rates are polled correctly."""
        cap_manager = get_capability_rate_manager()
        
        # Register fast agent (30 Hz)
        cap_manager.register_agent_capabilities(
            "fast-agent",
            [CapabilityRateSpec(CapabilityType.SENSORY, 30.0, True)]
        )
        
        # Register slow agent (5 Hz)
        cap_manager.register_agent_capabilities(
            "slow-agent",
            [CapabilityRateSpec(CapabilityType.SENSORY, 5.0, True)]
        )
        
        current_time_ns = time.time_ns()
        
        # First poll - both should be available
        agents = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns
        )
        assert len(agents) == 2, f"Should find 2 agents initially, found {len(agents)}"
        
        # After 50ms - only fast agent should be available (1/30s = 33ms)
        agents = cap_manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns + 50_000_000
        )
        
        # Fast agent (33ms interval) should be available
        # Slow agent (200ms interval) should not be available yet
        fast_available = any(a.approved_rate_hz == 30.0 for a in agents)
        
        print(f"✅ After 50ms: {len(agents)} agents available (fast agent: {fast_available})")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

