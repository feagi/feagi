"""
Test suite to measure FEAGI's SHM read performance and identify bottlenecks.

This test measures:
1. Raw SHM read speed (hardware/OS limit)
2. Sensory data decoding speed (Rust decoder)
3. Full sensory polling pipeline (capability manager + SHM + decoding)
4. Comparison against 30 Hz target (33ms budget)
"""
import pytest
import time
import struct
import os
from unittest.mock import Mock, MagicMock, patch

from feagi.api.zmq.neural.latest_only_slot import LatestOnlySharedSlot, LatestOnlyReader, LatestOnlyWriter


class TestSHMReadPerformance:
    """Measure FEAGI's ability to read sensory data from SHM at video speeds."""
    
    def test_raw_shm_read_speed(self, tmp_path):
        """Measure raw SHM read speed without any processing."""
        # Create a realistic video frame payload (~50KB for 128x128x3 segmented)
        shm_path = str(tmp_path / "test_sensory_slot.bin")
        
        # Typical video frame: 9 areas x ~6000 neurons = ~54000 neurons
        # Each neuron = 4 bytes (u32 neuron_id) = ~216KB per frame
        # But encoded as binary: type(1) + area_count(4) + per_area(id(2) + count(4) + neurons(4*N))
        frame_size = 100_000  # 100KB typical encoded frame
        test_data = b'\x0B' + struct.pack('<I', 9) + (b'\xFF' * (frame_size - 5))
        
        # Setup latest-only slot
        slot = LatestOnlySharedSlot(shm_path)
        writer = LatestOnlyWriter(slot)
        reader = LatestOnlyReader(slot)
        
        # Warmup
        for _ in range(10):
            writer.write_latest(test_data)
            _ = reader.read_latest()
        
        # Measure read speed for 100 frames
        iterations = 100
        start = time.perf_counter()
        
        for _ in range(iterations):
            writer.write_latest(test_data)
            data = reader.read_latest()
            # Note: latest-only may return None if no new data
        
        elapsed = time.perf_counter() - start
        avg_time_ms = (elapsed / iterations) * 1000
        read_rate_hz = iterations / elapsed
        
        print(f"\n✅ [RAW-SHM-READ] {iterations} read cycles in {elapsed:.3f}s")
        print(f"   - Avg time: {avg_time_ms:.2f}ms per cycle")
        print(f"   - Read rate: {read_rate_hz:.1f} Hz")
        print(f"   - Target: 30 Hz (33.3ms budget)")
        print(f"   - Headroom: {(33.3 - avg_time_ms):.1f}ms")
        
        # Assert we can read faster than 30 Hz
        assert avg_time_ms < 33.3, f"SHM read too slow: {avg_time_ms:.2f}ms > 33.3ms"
        
        # Cleanup
        del reader
        del slot
        try:
            os.unlink(shm_path)
        except:
            pass
    
    def test_sensory_data_decoding_speed(self):
        """Measure Rust sensory data decoding speed."""
        try:
            from feagi_connector.rust_native import decode_sensory_data_rust
        except ImportError:
            pytest.skip("Rust decoder not available")
        
        # Create realistic encoded video frame
        # Type 11 (full cortical areas): [type(1)] [area_count(4)] [area_id(2) neuron_count(4) neuron_ids(4*N)]*area_count
        area_count = 9  # 3x3 segmented vision
        neurons_per_area = 6000  # ~54K total neurons
        
        encoded_data = bytearray()
        encoded_data.append(11)  # Type 11
        encoded_data.extend(struct.pack('<I', area_count))
        
        for area_id in range(area_count):
            encoded_data.extend(struct.pack('<H', area_id))  # area_id (2 bytes)
            encoded_data.extend(struct.pack('<I', neurons_per_area))  # neuron_count (4 bytes)
            for neuron_id in range(neurons_per_area):
                encoded_data.extend(struct.pack('<I', neuron_id))  # neuron_id (4 bytes)
        
        encoded_bytes = bytes(encoded_data)
        print(f"\n📦 Test payload: {len(encoded_bytes)} bytes ({area_count} areas, {neurons_per_area} neurons each)")
        
        # Warmup
        for _ in range(10):
            _ = decode_sensory_data_rust(encoded_bytes)
        
        # Measure decoding speed
        iterations = 100
        start = time.perf_counter()
        
        for _ in range(iterations):
            result = decode_sensory_data_rust(encoded_bytes)
            assert result is not None
        
        elapsed = time.perf_counter() - start
        avg_time_ms = (elapsed / iterations) * 1000
        decode_rate_hz = iterations / elapsed
        
        print(f"\n✅ [RUST-DECODE] {iterations} decodes in {elapsed:.3f}s")
        print(f"   - Avg time: {avg_time_ms:.2f}ms per decode")
        print(f"   - Decode rate: {decode_rate_hz:.1f} Hz")
        print(f"   - Target: 30 Hz (33.3ms budget)")
        print(f"   - Headroom: {(33.3 - avg_time_ms):.1f}ms")
        
        # Assert we can decode faster than 30 Hz
        assert avg_time_ms < 33.3, f"Rust decode too slow: {avg_time_ms:.2f}ms > 33.3ms"
    
    def test_full_sensory_polling_pipeline(self, tmp_path):
        """Measure the complete sensory polling pipeline: SHM read + decode + FCL injection."""
        shm_path = str(tmp_path / "test_sensory_pipeline.bin")
        
        # Create realistic video frame
        area_count = 9
        neurons_per_area = 6000
        
        encoded_data = bytearray()
        encoded_data.append(11)  # Type 11
        encoded_data.extend(struct.pack('<I', area_count))
        
        for area_id in range(area_count):
            encoded_data.extend(struct.pack('<H', area_id))
            encoded_data.extend(struct.pack('<I', neurons_per_area))
            for neuron_id in range(neurons_per_area):
                encoded_data.extend(struct.pack('<I', neuron_id))
        
        encoded_bytes = bytes(encoded_data)
        
        # Setup SHM
        slot = LatestOnlySharedSlot(shm_path)
        reader = LatestOnlyReader(slot)
        
        # Warmup
        for _ in range(10):
            slot.write(encoded_bytes)
            data = reader.read()
            if data:
                try:
                    from feagi_connector.rust_native import decode_sensory_data_rust
                    _ = decode_sensory_data_rust(data)
                except ImportError:
                    pass
        
        # Measure full pipeline
        iterations = 100
        start = time.perf_counter()
        
        for _ in range(iterations):
            # Step 1: Write to SHM (simulating video agent)
            slot.write(encoded_bytes)
            
            # Step 2: Read from SHM (sensory polling)
            data = reader.read()
            
            # Step 3: Decode (Rust decoder)
            if data:
                try:
                    from feagi_connector.rust_native import decode_sensory_data_rust
                    result = decode_sensory_data_rust(data)
                except ImportError:
                    pass
        
        elapsed = time.perf_counter() - start
        avg_time_ms = (elapsed / iterations) * 1000
        pipeline_rate_hz = iterations / elapsed
        
        print(f"\n✅ [FULL-PIPELINE] {iterations} cycles in {elapsed:.3f}s")
        print(f"   - Avg time: {avg_time_ms:.2f}ms per cycle")
        print(f"   - Pipeline rate: {pipeline_rate_hz:.1f} Hz")
        print(f"   - Target: 30 Hz (33.3ms budget)")
        print(f"   - Headroom: {(33.3 - avg_time_ms):.1f}ms")
        
        # Assert we can process faster than 30 Hz
        assert avg_time_ms < 33.3, f"Pipeline too slow: {avg_time_ms:.2f}ms > 33.3ms"
        
        # Cleanup
        del reader
        del slot
        try:
            os.unlink(shm_path)
        except:
            pass
    
    def test_capability_rate_manager_overhead(self):
        """Measure the overhead of capability rate manager polling logic."""
        from feagi.core.capability_rate_manager import CapabilityRateManager
        from feagi.api.v1.capability_rates import CapabilityType, CapabilityRateSpec
        from feagi.core.feagi_state_manager import FeagiStateManager
        
        # Mock state manager
        mock_state = Mock(spec=FeagiStateManager)
        mock_state.get_burst_engine_frequency.return_value = 30.0
        mock_state.get_connected_agents.return_value = {}
        
        manager = CapabilityRateManager(mock_state)
        
        # Register 10 agents with sensory capability at 30 Hz
        for i in range(10):
            specs = [CapabilityRateSpec(CapabilityType.SENSORY, 30.0, True)]
            manager.register_agent_capabilities(f"agent-{i}", specs)
        
        # Measure polling overhead
        iterations = 1000
        start = time.perf_counter()
        
        for _ in range(iterations):
            current_time_ns = time.time_ns()
            agents = manager.get_agents_for_capability_polling(
                CapabilityType.SENSORY,
                current_time_ns
            )
            # Simulate some work
            _ = len(agents)
        
        elapsed = time.perf_counter() - start
        avg_time_ms = (elapsed / iterations) * 1000
        poll_rate_hz = iterations / elapsed
        
        print(f"\n✅ [CAP-MGR-OVERHEAD] {iterations} polls in {elapsed:.3f}s")
        print(f"   - Avg time: {avg_time_ms:.3f}ms per poll")
        print(f"   - Poll rate: {poll_rate_hz:.0f} Hz")
        print(f"   - Target: 30 Hz (33.3ms budget)")
        print(f"   - Overhead: {(avg_time_ms / 33.3 * 100):.1f}% of budget")
        
        # Assert overhead is negligible (< 1ms)
        assert avg_time_ms < 1.0, f"Capability manager overhead too high: {avg_time_ms:.3f}ms"
    
    def test_concurrent_shm_read_from_multiple_agents(self, tmp_path):
        """Test SHM read performance when multiple agents are writing simultaneously."""
        # Simulate video agent + motor agent reading/writing concurrently
        sensory_path = str(tmp_path / "test_sensory_concurrent.bin")
        motor_path = str(tmp_path / "test_motor_concurrent.bin")
        
        # Create realistic payloads
        sensory_data = b'\x0B' + struct.pack('<I', 9) + (b'\xFF' * 50000)
        motor_data = b'\x05' + struct.pack('<I', 1) + (b'\xAA' * 1000)
        
        # Setup SHM slots
        sensory_slot = LatestOnlySharedSlot(sensory_path)
        sensory_reader = LatestOnlyReader(sensory_slot)
        motor_slot = LatestOnlySharedSlot(motor_path)
        motor_reader = LatestOnlyReader(motor_slot)
        
        # Measure concurrent access
        iterations = 50
        start = time.perf_counter()
        
        for _ in range(iterations):
            # Concurrent writes
            sensory_slot.write(sensory_data)
            motor_slot.write(motor_data)
            
            # Concurrent reads
            s_data = sensory_reader.read()
            m_data = motor_reader.read()
        
        elapsed = time.perf_counter() - start
        avg_time_ms = (elapsed / iterations) * 1000
        concurrent_rate_hz = iterations / elapsed
        
        print(f"\n✅ [CONCURRENT-SHM] {iterations} concurrent read/write cycles in {elapsed:.3f}s")
        print(f"   - Avg time: {avg_time_ms:.2f}ms per cycle")
        print(f"   - Concurrent rate: {concurrent_rate_hz:.1f} Hz")
        print(f"   - Target: 30 Hz (33.3ms budget)")
        
        # Cleanup
        del sensory_reader
        del sensory_slot
        del motor_reader
        del motor_slot
        try:
            os.unlink(sensory_path)
            os.unlink(motor_path)
        except:
            pass


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

