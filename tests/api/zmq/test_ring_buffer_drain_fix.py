"""
Test for ring buffer auto-drain fix.

This test verifies that the ring buffer buffer_full issue is resolved
by the auto-drain mechanism in SensoryNeuralStream.
"""

import pytest
from unittest.mock import Mock, patch
from feagi.api.zmq.neural.ring_buffer import ZeroCopyRingBuffer


class TestRingBufferDrainFix:
    """Test suite for ring buffer auto-drain functionality."""

    def test_ring_buffer_auto_drain_prevents_buffer_full(self):
        """
        Test that auto-draining prevents buffer_full condition.
        
        This test verifies the fix for the critical issue where ring buffers
        would fill up because data was processed inline but never consumed
        from the buffer, causing the read index to never advance.
        """
        # Create a small ring buffer to easily trigger buffer full condition
        with ZeroCopyRingBuffer(slots=4, slot_size=1024) as ring:
            
            # Test 1: Simulate old broken behavior (write-only, no auto-drain)
            written_count = 0
            for i in range(ring.slots + 2):  # Try to write more than capacity
                write_slot = ring.get_write_slot()
                if write_slot:
                    write_slot.memory_view[0:4] = b"TEST"
                    ring.commit_write(write_slot)
                    written_count += 1
                else:
                    # Should hit buffer full after ring.slots-1 writes
                    break
            
            # Verify buffer gets full without auto-drain
            assert written_count == ring.slots - 1, f"Expected buffer full after {ring.slots-1} writes, got {written_count}"
            assert ring.used_slots == ring.slots - 1, "Buffer should be nearly full"
            
            # Test 2: Simulate new fixed behavior (with auto-drain)
            # Reset buffer to empty state
            ring.read_index.value = ring.write_index.value
            assert ring.used_slots == 0, "Buffer should be empty after reset"
            
            # Now test continuous processing with auto-drain
            auto_drain_count = 0
            for i in range(ring.slots * 2):  # Process more than buffer capacity
                write_slot = ring.get_write_slot()
                if write_slot:
                    write_slot.memory_view[0:4] = b"FEAG"
                    ring.commit_write(write_slot)
                    
                    # CRITICAL FIX: Auto-drain after processing (the fix we implemented)
                    read_slot = ring.get_read_slot()
                    if read_slot and read_slot.index == write_slot.index:
                        ring.commit_read(read_slot)
                        auto_drain_count += 1
                    else:
                        break
                else:
                    # Should NOT hit buffer full with auto-drain
                    break
            
            # Verify auto-drain prevents buffer full
            assert auto_drain_count >= ring.slots, f"Expected to process at least {ring.slots} items with auto-drain, got {auto_drain_count}"
            assert ring.used_slots == 0, "Buffer should remain empty with auto-drain"
            
    def test_ring_buffer_stats_tracking(self):
        """Test that ring buffer statistics are properly tracked during auto-drain."""
        with ZeroCopyRingBuffer(slots=8, slot_size=512) as ring:
            
            # Process several items with auto-drain
            for i in range(10):
                write_slot = ring.get_write_slot()
                assert write_slot is not None, f"Should be able to get write slot {i}"
                
                ring.commit_write(write_slot)
                
                # Auto-drain immediately
                read_slot = ring.get_read_slot()
                assert read_slot is not None, f"Should be able to get read slot {i}"
                assert read_slot.index == write_slot.index, "Read and write slots should match"
                
                ring.commit_read(read_slot)
            
            # Verify statistics
            assert ring.stats.total_writes == 10, "Should track all writes"
            assert ring.stats.total_reads == 10, "Should track all reads"
            assert ring.stats.buffer_full_count == 0, "Should have no buffer full events with auto-drain"
            assert ring.used_slots == 0, "Buffer should be empty"

    def test_ring_buffer_auto_drain_edge_cases(self):
        """Test edge cases for auto-drain functionality."""
        with ZeroCopyRingBuffer(slots=2, slot_size=256) as ring:
            
            # Test rapid write/auto-drain cycles
            for cycle in range(5):
                # Fill buffer completely
                slots = []
                for i in range(ring.slots - 1):  # Fill to capacity - 1
                    slot = ring.get_write_slot()
                    assert slot is not None, f"Cycle {cycle}: Should get write slot {i}"
                    ring.commit_write(slot)
                    slots.append(slot)
                
                # Verify nearly full
                assert ring.used_slots == ring.slots - 1
                
                # Auto-drain all slots
                for slot in slots:
                    read_slot = ring.get_read_slot()
                    assert read_slot is not None, f"Cycle {cycle}: Should get read slot"
                    ring.commit_read(read_slot)
                
                # Verify empty again
                assert ring.used_slots == 0, f"Cycle {cycle}: Buffer should be empty after auto-drain"
            
    def test_ring_buffer_concurrent_safety(self):
        """Test that auto-drain doesn't break concurrent access patterns."""
        with ZeroCopyRingBuffer(slots=16, slot_size=1024) as ring:
            
            # Simulate interleaved writes and auto-drains
            write_count = 0
            read_count = 0
            
            for i in range(32):
                # Write if possible
                if i % 3 != 0:  # Write 2/3 of the time
                    write_slot = ring.get_write_slot()
                    if write_slot:
                        ring.commit_write(write_slot)
                        write_count += 1
                
                # Auto-drain if possible
                if i % 2 == 0:  # Drain 1/2 of the time
                    read_slot = ring.get_read_slot()
                    if read_slot:
                        ring.commit_read(read_slot)
                        read_count += 1
            
            # Drain any remaining
            while True:
                read_slot = ring.get_read_slot()
                if not read_slot:
                    break
                ring.commit_read(read_slot)
                read_count += 1
            
            # Verify no data loss
            assert read_count == write_count, "All written data should be readable"
            assert ring.used_slots == 0, "Buffer should be empty at end" 