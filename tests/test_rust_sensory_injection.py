"""
Test Rust Sensory Injection System

Tests the integration between Python registration manager and Rust sensory polling threads.
"""
import pytest
import time
import tempfile
import os
from pathlib import Path


class TestRustSensoryAPI:
    """Test Rust NPU sensory injection API."""
    
    def test_rust_npu_has_sensory_methods(self):
        """Verify Rust NPU exposes sensory registration methods."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(
            neuron_capacity=1000,
            synapse_capacity=10000,
            fire_ledger_window=20
        )
        
        # Check methods exist
        assert hasattr(npu, 'register_sensory_agent')
        assert hasattr(npu, 'deregister_sensory_agent')
        assert hasattr(npu, 'list_sensory_agents')
        assert hasattr(npu, 'start_burst_loop')
        assert hasattr(npu, 'stop_burst_loop')
        assert hasattr(npu, 'is_burst_loop_running')
    
    def test_burst_loop_required_for_sensory_registration(self):
        """Verify sensory registration requires running burst loop."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        # Should fail without burst loop running
        with pytest.raises(RuntimeError, match="Burst loop not started"):
            npu.register_sensory_agent(
                agent_id="test-agent",
                shm_path="/tmp/test-shm",
                rate_hz=30.0,
                area_mapping={}
            )
    
    def test_sensory_registration_with_burst_loop(self):
        """Test registering sensory agent with running burst loop."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        try:
            # Start burst loop
            npu.start_burst_loop(frequency_hz=15.0)
            time.sleep(0.2)  # Let it start
            assert npu.is_burst_loop_running()
            
            # Register agent (API succeeds, thread fails asynchronously on SHM open)
            # This is expected behavior - registration doesn't block on SHM availability
            npu.register_sensory_agent(
                agent_id="test-agent",
                shm_path="/tmp/nonexistent-shm",
                rate_hz=30.0,
                area_mapping={"area1": 0}
            )
            
            # Agent should be in list
            agents = npu.list_sensory_agents()
            assert "test-agent" in agents
            
            # Deregister
            npu.deregister_sensory_agent("test-agent")
            agents = npu.list_sensory_agents()
            assert "test-agent" not in agents
        finally:
            npu.stop_burst_loop()
            time.sleep(0.1)
            assert not npu.is_burst_loop_running()
    
    def test_list_sensory_agents_empty(self):
        """Test listing sensory agents when none registered."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        try:
            npu.start_burst_loop(frequency_hz=15.0)
            time.sleep(0.2)
            
            agents = npu.list_sensory_agents()
            assert isinstance(agents, list)
            assert len(agents) == 0
        finally:
            npu.stop_burst_loop()
    
    def test_deregister_nonexistent_agent(self):
        """Test deregistering agent that doesn't exist."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        try:
            npu.start_burst_loop(frequency_hz=15.0)
            time.sleep(0.2)
            
            # Should handle gracefully
            with pytest.raises(RuntimeError, match="not found|Failed"):
                npu.deregister_sensory_agent("nonexistent-agent")
        finally:
            npu.stop_burst_loop()


class TestBurstLoopWithSensory:
    """Test Rust burst loop integration with sensory manager."""
    
    def test_burst_loop_starts_and_stops(self):
        """Test basic burst loop lifecycle."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        # Not running initially
        assert not npu.is_burst_loop_running()
        assert npu.get_burst_loop_count() == 0
        
        # Start
        npu.start_burst_loop(frequency_hz=20.0)
        time.sleep(0.2)
        assert npu.is_burst_loop_running()
        
        # Let it run
        time.sleep(1.0)
        count = npu.get_burst_loop_count()
        assert count > 10, f"Expected >10 bursts in 1s @ 20Hz, got {count}"
        assert count < 30, f"Expected <30 bursts in 1s @ 20Hz, got {count}"
        
        # Stop
        npu.stop_burst_loop()
        time.sleep(0.2)
        assert not npu.is_burst_loop_running()
    
    def test_burst_loop_frequency_accuracy(self):
        """Test burst loop runs at correct frequency."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        try:
            target_freq = 15.0
            npu.start_burst_loop(frequency_hz=target_freq)
            time.sleep(0.5)  # Warmup
            
            # Measure actual frequency
            start_count = npu.get_burst_loop_count()
            start_time = time.time()
            time.sleep(2.0)
            end_count = npu.get_burst_loop_count()
            end_time = time.time()
            
            bursts = end_count - start_count
            duration = end_time - start_time
            actual_freq = bursts / duration
            
            # Allow 10% tolerance
            assert abs(actual_freq - target_freq) / target_freq < 0.1, \
                f"Target: {target_freq} Hz, Actual: {actual_freq:.1f} Hz"
        finally:
            npu.stop_burst_loop()


class TestPythonRegistrationIntegration:
    """Test Python registration manager calling Rust sensory registration."""
    
    @pytest.mark.skip(reason="Requires full FEAGI stack - manual test only")
    def test_registration_manager_calls_rust(self):
        """Test that registration_manager.py calls Rust NPU registration.
        
        This is a documentation test showing the expected flow.
        Run manually with full FEAGI stack.
        """
        # Expected flow:
        # 1. Agent registers with sensory capability + rate_hz
        # 2. registration_manager.py registers with capability_rate_manager
        # 3. registration_manager.py calls rust_npu.register_sensory_agent()
        # 4. Rust spawns polling thread for agent
        # 5. Thread polls SHM at rate_hz and injects into FCL
        pass
    
    @pytest.mark.skip(reason="Requires full FEAGI stack - manual test only")
    def test_sensory_data_flow_end_to_end(self):
        """End-to-end test: Agent writes SHM → Rust reads → NPU processes.
        
        This is a documentation test showing the expected data flow.
        Run manually with video agent + FEAGI + BV.
        """
        # Expected data flow:
        # 1. Video agent writes Type 11 cortical data to SHM
        # 2. Rust sensory thread polls SHM (30 Hz)
        # 3. Rust decodes Type 11 → neuron IDs
        # 4. Rust injects into FCL
        # 5. Burst loop processes FCL → Fire Queue
        # 6. BV reads Fire Queue and visualizes
        pass


class TestSensoryPerformance:
    """Performance benchmarks for Rust sensory injection."""
    
    @pytest.mark.benchmark
    def test_burst_loop_overhead(self):
        """Measure burst loop overhead (should be minimal)."""
        import feagi_rust
        
        npu = feagi_rust.RustNPU(1000, 10000, 20)
        
        try:
            npu.start_burst_loop(frequency_hz=50.0)
            time.sleep(0.5)  # Warmup
            
            start = time.time()
            start_count = npu.get_burst_loop_count()
            time.sleep(5.0)
            end_count = npu.get_burst_loop_count()
            duration = time.time() - start
            
            actual_freq = (end_count - start_count) / duration
            
            print(f"\n  Target: 50 Hz")
            print(f"  Actual: {actual_freq:.2f} Hz")
            print(f"  Bursts: {end_count - start_count} in {duration:.2f}s")
            
            # At 50Hz, overhead should be minimal
            assert actual_freq > 45, "Burst loop running too slow"
        finally:
            npu.stop_burst_loop()


if __name__ == "__main__":
    # Run with: pytest -v test_rust_sensory_injection.py
    pytest.main([__file__, "-v", "-s"])

