"""
Test to measure lock contention between video agent (writer) and FEAGI (reader).

Hypothesis: The video agent writes at 26 Hz, but FEAGI only reads at 7 Hz because
the SHM file is locked during writes, causing FEAGI reads to block or timeout.
"""
import pytest
import time
import threading
import struct
from pathlib import Path

from feagi.api.zmq.neural.latest_only_slot import LatestOnlySharedSlot, LatestOnlyReader, LatestOnlyWriter


class TestSHMLockContention:
    """Test lock contention between concurrent writer and reader."""
    
    def test_lock_contention_at_video_speed(self, tmp_path):
        """Simulate video agent writing at 26 Hz while FEAGI reads continuously."""
        shm_path = str(tmp_path / "test_lock_contention.bin")
        
        # Realistic video frame
        frame_size = 100_000  # 100KB
        test_data = b'\x0B' + struct.pack('<I', 9) + (b'\xFF' * (frame_size - 5))
        
        # Setup SHM
        slot = LatestOnlySharedSlot(shm_path)
        writer = LatestOnlyWriter(slot)
        
        # Stats
        write_stats = {'count': 0, 'failed': 0, 'blocked_time': 0.0}
        read_stats = {'count': 0, 'successful': 0, 'none_count': 0, 'blocked_time': 0.0}
        
        stop_flag = threading.Event()
        
        def writer_thread():
            """Write at 26 Hz (video agent simulation)."""
            target_interval = 1.0 / 26.0  # 38.5ms
            
            while not stop_flag.is_set():
                write_start = time.perf_counter()
                try:
                    success = writer.write_latest(test_data)
                    write_stats['count'] += 1
                    if not success:
                        write_stats['failed'] += 1
                except Exception as e:
                    write_stats['failed'] += 1
                    print(f"Write error: {e}")
                
                write_time = time.perf_counter() - write_start
                write_stats['blocked_time'] += write_time
                
                # Sleep to maintain 26 Hz
                sleep_time = max(0, target_interval - write_time)
                time.sleep(sleep_time)
        
        def reader_thread():
            """Read as fast as possible (FEAGI sensory poll simulation)."""
            reader = LatestOnlyReader(slot)
            
            while not stop_flag.is_set():
                read_start = time.perf_counter()
                try:
                    data = reader.read_latest()
                    read_stats['count'] += 1
                    if data is not None:
                        read_stats['successful'] += 1
                    else:
                        read_stats['none_count'] += 1
                except Exception as e:
                    print(f"Read error: {e}")
                
                read_time = time.perf_counter() - read_start
                read_stats['blocked_time'] += read_time
                
                # No sleep - poll as fast as possible
        
        # Start threads
        write_thread = threading.Thread(target=writer_thread, daemon=True)
        read_thread = threading.Thread(target=reader_thread, daemon=True)
        
        write_thread.start()
        read_thread.start()
        
        # Run for 5 seconds
        time.sleep(5.0)
        stop_flag.set()
        
        write_thread.join(timeout=1.0)
        read_thread.join(timeout=1.0)
        
        # Calculate results
        write_rate = write_stats['count'] / 5.0
        read_rate = read_stats['count'] / 5.0
        read_success_rate = read_stats['successful'] / 5.0 if read_stats['successful'] > 0 else 0
        
        avg_write_time = (write_stats['blocked_time'] / write_stats['count'] * 1000) if write_stats['count'] > 0 else 0
        avg_read_time = (read_stats['blocked_time'] / read_stats['count'] * 1000) if read_stats['count'] > 0 else 0
        
        capture_efficiency = (read_stats['successful'] / write_stats['count'] * 100) if write_stats['count'] > 0 else 0
        
        print(f"\n📊 [LOCK-CONTENTION-TEST] 5 second concurrent access:")
        print(f"\n  Writer (Video Agent):")
        print(f"    - Total writes: {write_stats['count']}")
        print(f"    - Write rate: {write_rate:.1f} Hz (target: 26 Hz)")
        print(f"    - Failed writes: {write_stats['failed']}")
        print(f"    - Avg write time: {avg_write_time:.2f}ms")
        
        print(f"\n  Reader (FEAGI):")
        print(f"    - Total read attempts: {read_stats['count']}")
        print(f"    - Read attempt rate: {read_rate:.1f} Hz")
        print(f"    - Successful reads (new data): {read_stats['successful']}")
        print(f"    - Read success rate: {read_success_rate:.1f} Hz")
        print(f"    - None returns (no new data): {read_stats['none_count']}")
        print(f"    - Avg read time: {avg_read_time:.2f}ms")
        
        print(f"\n  Efficiency:")
        print(f"    - Capture rate: {capture_efficiency:.1f}% of writes captured")
        print(f"    - Expected: ~100% (reader faster than writer)")
        print(f"    - Observed gap: {write_stats['count'] - read_stats['successful']} frames lost")
        
        # Cleanup
        del writer
        slot.close()
        
        # Assertions
        assert write_rate >= 25.0, f"Writer too slow: {write_rate:.1f} Hz < 25 Hz"
        
        # KEY METRIC: If lock contention is the issue, capture_efficiency will be low (<50%)
        if capture_efficiency < 50:
            print(f"\n❌ [LOCK-CONTENTION] Severe lock contention detected!")
            print(f"   Reader only captured {capture_efficiency:.1f}% of frames")
            print(f"   This explains why FEAGI sees 7 Hz when agent sends 26 Hz")
        else:
            print(f"\n✅ [NO-LOCK-CONTENTION] Reader captured {capture_efficiency:.1f}% of frames")
            print(f"   Lock contention is NOT the bottleneck")
        
        # Document the finding regardless of pass/fail
        print(f"\n🔍 [DIAGNOSIS]:")
        if capture_efficiency < 30:
            print(f"   Lock contention is SEVERE - this is the bottleneck")
        elif capture_efficiency < 70:
            print(f"   Lock contention is MODERATE - contributing factor")
        else:
            print(f"   Lock contention is MINIMAL - bottleneck is elsewhere")
    
    def test_read_blocking_time_distribution(self, tmp_path):
        """Measure how long reads block when writer is active."""
        shm_path = str(tmp_path / "test_read_blocking.bin")
        
        frame_size = 100_000
        test_data = b'\x0B' + struct.pack('<I', 9) + (b'\xFF' * (frame_size - 5))
        
        slot = LatestOnlySharedSlot(shm_path)
        writer = LatestOnlyWriter(slot)
        reader = LatestOnlyReader(slot)
        
        # Measure read times with and without concurrent writes
        
        # Phase 1: Read without concurrent writes
        read_times_idle = []
        for _ in range(100):
            writer.write_latest(test_data)
            time.sleep(0.001)  # Let write complete
            
            read_start = time.perf_counter()
            _ = reader.read_latest()
            read_time = (time.perf_counter() - read_start) * 1000
            read_times_idle.append(read_time)
        
        avg_idle = sum(read_times_idle) / len(read_times_idle)
        max_idle = max(read_times_idle)
        
        # Phase 2: Read during concurrent writes
        stop_flag = threading.Event()
        read_times_contention = []
        
        def continuous_writer():
            while not stop_flag.is_set():
                writer.write_latest(test_data)
                time.sleep(0.038)  # 26 Hz
        
        write_thread = threading.Thread(target=continuous_writer, daemon=True)
        write_thread.start()
        
        time.sleep(0.1)  # Let writer stabilize
        
        for _ in range(100):
            read_start = time.perf_counter()
            _ = reader.read_latest()
            read_time = (time.perf_counter() - read_start) * 1000
            read_times_contention.append(read_time)
            time.sleep(0.001)
        
        stop_flag.set()
        write_thread.join(timeout=1.0)
        
        avg_contention = sum(read_times_contention) / len(read_times_contention)
        max_contention = max(read_times_contention)
        
        print(f"\n📊 [READ-BLOCKING-ANALYSIS]:")
        print(f"  Without concurrent writes:")
        print(f"    - Avg read time: {avg_idle:.3f}ms")
        print(f"    - Max read time: {max_idle:.3f}ms")
        print(f"  With concurrent writes (26 Hz):")
        print(f"    - Avg read time: {avg_contention:.3f}ms")
        print(f"    - Max read time: {max_contention:.3f}ms")
        print(f"  Blocking overhead:")
        print(f"    - Avg increase: {avg_contention - avg_idle:.3f}ms ({((avg_contention/avg_idle - 1) * 100):.1f}%)")
        print(f"    - Max increase: {max_contention - max_idle:.3f}ms")
        
        # Cleanup
        del reader
        del writer
        slot.close()
        
        if avg_contention > avg_idle * 2:
            print(f"\n❌ [LOCK-BLOCKING] Reads are {avg_contention/avg_idle:.1f}x slower during writes!")
        else:
            print(f"\n✅ [MINIMAL-BLOCKING] Lock contention adds only {avg_contention - avg_idle:.3f}ms overhead")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

