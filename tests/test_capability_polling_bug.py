"""
Test to reproduce the bug where get_agents_for_capability_polling() returns 0 agents
despite having 1 agent registered in _agent_registries.

From logs:
  [POLL-STATE] capability_manager has 1 agent in _agent_registries  
  [SENSORY-POLL] Polling 0 agents ❌  ← BUG!
"""
import pytest
import time
from feagi.core.capability_rate_manager import get_capability_rate_manager, reset_capability_rate_manager
from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec


class TestCapabilityPollingBug:
    
    def setup_method(self):
        """Reset capability manager before each test."""
        reset_capability_rate_manager()
    
    def test_registered_agent_appears_in_polling(self):
        """Test that a registered agent with SENSORY capability is returned by polling."""
        manager = get_capability_rate_manager()
        
        # Register video agent with 30 Hz sensory
        specs = [
            CapabilityRateSpec(
                capability_type=CapabilityType.SENSORY,
                requested_rate_hz=29.97,
                required=True
            )
        ]
        
        approved, rejected = manager.register_agent_capabilities("video-agent-1", specs)
        
        print(f"\n✅ Registration:")
        print(f"   - Approved: {len(approved)} capabilities")
        print(f"   - Rejected: {len(rejected)} capabilities")
        print(f"   - _agent_registries count: {len(manager._agent_registries)}")
        
        assert len(approved) == 1, "Should approve SENSORY capability"
        assert len(manager._agent_registries) == 1, "Should have 1 agent in registry"
        
        # Immediately poll for agents
        current_time_ns = time.time_ns()
        agents = manager.get_agents_for_capability_polling(
            CapabilityType.SENSORY,
            current_time_ns
        )
        
        print(f"\n🔍 First Poll (immediately after registration):")
        print(f"   - Current time: {current_time_ns}")
        print(f"   - Agents returned: {len(agents)}")
        for agent in agents:
            print(f"     • {agent.agent_id}: {agent.approved_rate_hz} Hz, "
                  f"last_poll={agent.last_poll_time_ns}, poll_count={agent.poll_count}")
        
        # THE BUG: This should return 1 agent but returns 0!
        assert len(agents) == 1, f"❌ BUG: Should return 1 agent, got {len(agents)}"
        assert agents[0].agent_id == "video-agent-1"
        assert agents[0].approved_rate_hz == 29.97
    
    def test_should_poll_now_logic(self):
        """Test the should_poll_now() logic in detail."""
        manager = get_capability_rate_manager()
        
        # Register agent
        specs = [CapabilityRateSpec(CapabilityType.SENSORY, 30.0, True)]
        manager.register_agent_capabilities("test-agent", specs)
        
        # Get the agent's rate config directly
        registry = manager._agent_registries["test-agent"]
        rate_config = registry.get_capability_rate(CapabilityType.SENSORY)
        
        print(f"\n📊 Rate Config Details:")
        print(f"   - Agent ID: {rate_config.agent_id}")
        print(f"   - Rate: {rate_config.approved_rate_hz} Hz")
        print(f"   - Poll interval: {rate_config.poll_interval_ns / 1_000_000:.1f}ms")
        print(f"   - Last poll time: {rate_config.last_poll_time_ns}")
        print(f"   - Poll count: {rate_config.poll_count}")
        
        # Test should_poll_now at different times
        base_time = time.time_ns()
        
        tests = [
            (base_time, "Immediately after registration"),
            (base_time + 10_000_000, "+10ms"),
            (base_time + 33_000_000, "+33ms (should be ready)"),
            (base_time + 100_000_000, "+100ms (definitely ready)"),
        ]
        
        for test_time, desc in tests:
            should_poll = rate_config.should_poll_now(test_time)
            elapsed_ms = (test_time - base_time) / 1_000_000
            print(f"   {desc} ({elapsed_ms:.1f}ms): should_poll={should_poll}")
        
        # First call should return True (never polled before)
        assert rate_config.should_poll_now(base_time), "First poll should always succeed"
    
    def test_polling_after_rate_limiting(self):
        """Test that polling respects rate limits correctly."""
        manager = get_capability_rate_manager()
        
        # Register agent at 10 Hz (100ms interval)
        specs = [CapabilityRateSpec(CapabilityType.SENSORY, 10.0, True)]
        manager.register_agent_capabilities("rate-limited-agent", specs)
        
        current_time_ns = time.time_ns()
        
        # First poll - should succeed
        agents = manager.get_agents_for_capability_polling(CapabilityType.SENSORY, current_time_ns)
        print(f"\n🔄 Poll 1 (t=0ms): {len(agents)} agents")
        assert len(agents) == 1, "First poll should succeed"
        
        # Second poll immediately - should fail (rate limited)
        agents = manager.get_agents_for_capability_polling(CapabilityType.SENSORY, current_time_ns + 1_000_000)  # +1ms
        print(f"   Poll 2 (t=1ms): {len(agents)} agents")
        assert len(agents) == 0, "Immediate second poll should be rate-limited"
        
        # Third poll after interval - should succeed
        agents = manager.get_agents_for_capability_polling(CapabilityType.SENSORY, current_time_ns + 110_000_000)  # +110ms
        print(f"   Poll 3 (t=110ms): {len(agents)} agents")
        assert len(agents) == 1, "Poll after interval should succeed"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

