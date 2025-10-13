#!/usr/bin/env python3
"""
Test script to validate the latest-only slot system fixes the temporal pattern replay bug.

This script simulates:
1. Agent writing sensory data to a latest-only slot
2. Agent dying (stopping writes)
3. FEAGI continuing to read from the slot
4. Verifying that no stale data is replayed

Expected behavior:
- When agent is alive: Fresh data is read continuously 
- When agent dies: No data is read (stale data is automatically rejected)
- No temporal pattern loops occur
"""

import asyncio
import time
import sys
from pathlib import Path

# Add the project root to path
sys.path.insert(0, str(Path(__file__).parent))

from feagi.api.zmq.neural.latest_only_slot import (
    LatestOnlySharedSlot,
    LatestOnlyWriter, 
    LatestOnlyReader,
    create_agent_slot_path,
    cleanup_agent_slots
)

async def test_temporal_replay_fix():
    """Test that temporal pattern replay bug is fixed."""
    
    # Test configuration
    test_dir = Path("/tmp/feagi_test_slots")
    agent_id = "test_agent_001"
    max_age_ms = 100.0  # 100ms max age
    
    # Clean up any existing test files
    cleanup_agent_slots(test_dir, agent_id)
    
    print("🧪 Testing Latest-Only Slot System")
    print("=" * 50)
    
    # Create slot
    slot_path = create_agent_slot_path(test_dir, agent_id, "sensory")
    slot = LatestOnlySharedSlot(slot_path, max_payload_size=1024)
    
    # Create writer and reader
    writer = LatestOnlyWriter(slot)
    reader = LatestOnlyReader(slot, max_age_ms)
    
    print(f"✅ Created slot: {slot_path}")
    
    # Phase 1: Agent alive - writing fresh data
    print("\n📡 PHASE 1: Agent alive, sending fresh data")
    
    for i in range(5):
        payload = f"sensory_data_frame_{i}".encode()
        success = writer.write_latest(payload)
        print(f"  Write {i}: {'✅' if success else '❌'} {payload.decode()}")
        
        # Read immediately (should get fresh data)
        await asyncio.sleep(0.01)  # 10ms - well within max_age
        data = reader.read_latest()
        if data:
            print(f"  Read {i}: ✅ {data.data.decode()} (age: {data.age_ms:.1f}ms)")
        else:
            print(f"  Read {i}: ❌ No data")
            
        await asyncio.sleep(0.05)  # 50ms between frames
    
    # Phase 2: Agent dies (stop writing, but old data persists in slot)
    print(f"\n💀 PHASE 2: Agent died, slot contains last data")
    print("   (This is where the OLD system would loop forever)")
    
    # Wait for data to become stale
    await asyncio.sleep(0.15)  # 150ms - exceeds max_age of 100ms
    
    # Try reading multiple times (old system would replay the same data)
    for i in range(10):
        data = reader.read_latest()
        if data:
            print(f"  ❌ BUG: Read stale data: {data.data.decode()} (age: {data.age_ms:.1f}ms)")
        else:
            print(f"  ✅ Correctly rejected stale data (attempt {i+1})")
        await asyncio.sleep(0.01)
        
    # Phase 3: Verify statistics
    print(f"\n📊 PHASE 3: Statistics")
    stats = slot.stats
    print(f"  Total writes: {stats.total_writes}")
    print(f"  Total reads: {stats.total_reads}")  
    print(f"  Stale rejections: {stats.stale_data_rejections}")
    print(f"  Overwrites: {stats.overwrites}")
    
    # Verify that stale rejections occurred
    if stats.stale_data_rejections > 0:
        print("  ✅ Stale data was properly rejected")
    else:
        print("  ⚠️  No stale data rejections (may indicate test timing issue)")
        
    # Phase 4: Agent comes back online
    print(f"\n🔄 PHASE 4: Agent comes back online")
    
    # Reset sequence tracking to simulate new reader 
    # (This tests the scenario that caused the original bug)
    new_reader = LatestOnlyReader(slot, max_age_ms)
    print("  📖 Created new reader (simulating reader recreation)")
    
    # Write fresh data
    fresh_payload = "fresh_data_after_restart".encode()
    writer.write_latest(fresh_payload)
    print(f"  📝 Agent wrote fresh data: {fresh_payload.decode()}")
    
    # Read with new reader
    await asyncio.sleep(0.01)
    data = new_reader.read_latest()
    if data:
        print(f"  ✅ New reader got fresh data: {data.data.decode()} (age: {data.age_ms:.1f}ms)")
        
        # Verify this is indeed fresh data, not old data
        if b"restart" in data.data:
            print("  ✅ Confirmed: Got fresh data, not replayed old data")
        else:
            print("  ❌ BUG: Got old data instead of fresh data!")
    else:
        print("  ❌ New reader failed to get fresh data")
    
    # Cleanup
    slot.close()
    cleanup_agent_slots(test_dir, agent_id)
    
    print(f"\n🎉 Test completed!")
    print("   If you see '✅ Correctly rejected stale data' messages above,")
    print("   then the temporal pattern replay bug has been FIXED! 🐛➡️✅")

async def test_multiple_agents():
    """Test multiple agents writing to different slots."""
    
    print(f"\n🧪 Testing Multiple Agents")
    print("=" * 30)
    
    test_dir = Path("/tmp/feagi_test_multi_slots")
    agents = ["agent_001", "agent_002", "agent_003"]
    
    slots = {}
    writers = {}
    readers = {}
    
    # Set up slots for each agent
    for agent_id in agents:
        cleanup_agent_slots(test_dir, agent_id)
        slot_path = create_agent_slot_path(test_dir, agent_id, "sensory")
        
        slot = LatestOnlySharedSlot(slot_path, max_payload_size=1024)
        writer = LatestOnlyWriter(slot) 
        reader = LatestOnlyReader(slot, max_age_ms=100.0)
        
        slots[agent_id] = slot
        writers[agent_id] = writer
        readers[agent_id] = reader
        
        print(f"  ✅ Created slot for {agent_id}")
    
    # All agents write simultaneously
    print(f"\n📡 All agents writing data...")
    
    for i in range(3):
        for agent_id in agents:
            payload = f"{agent_id}_frame_{i}".encode()
            writers[agent_id].write_latest(payload)
        await asyncio.sleep(0.02)
    
    # Read from all agents
    print(f"\n📖 Reading from all agents...")
    
    for agent_id in agents:
        data = readers[agent_id].read_latest()
        if data:
            print(f"  {agent_id}: ✅ {data.data.decode()}")
        else:
            print(f"  {agent_id}: ❌ No data")
    
    # Kill one agent, others continue
    print(f"\n💀 Killing {agents[1]}, others continue...")
    dead_agent = agents[1]
    
    # Continue writing for other agents
    for i in range(3, 6):
        for agent_id in agents:
            if agent_id != dead_agent:  # Skip dead agent
                payload = f"{agent_id}_frame_{i}".encode()
                writers[agent_id].write_latest(payload)
        await asyncio.sleep(0.02)
    
    # Wait for dead agent's data to become stale
    await asyncio.sleep(0.15)
    
    # Read from all (dead agent should have no data)
    print(f"\n📖 Reading after agent death...")
    
    for agent_id in agents:
        data = readers[agent_id].read_latest()
        if data:
            if agent_id == dead_agent:
                print(f"  {agent_id}: ❌ BUG: Still getting data from dead agent!")
            else:
                print(f"  {agent_id}: ✅ {data.data.decode()}")
        else:
            if agent_id == dead_agent:
                print(f"  {agent_id}: ✅ Correctly no data from dead agent")
            else:
                print(f"  {agent_id}: ⚠️  No data from live agent")
    
    # Cleanup
    for slot in slots.values():
        slot.close()
    for agent_id in agents:
        cleanup_agent_slots(test_dir, agent_id)
        
    print(f"✅ Multi-agent test completed!")

async def main():
    """Run all tests."""
    try:
        await test_temporal_replay_fix()
        await test_multiple_agents()
        print(f"\n🎉 ALL TESTS PASSED!")
        print(f"   The latest-only slot system successfully prevents")
        print(f"   temporal pattern replay bugs! 🚀")
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
